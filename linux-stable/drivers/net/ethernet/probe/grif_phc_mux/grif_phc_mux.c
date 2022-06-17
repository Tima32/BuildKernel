#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>

#include <linux/printk.h>
#include <linux/types.h>
#include <asm-generic/bug.h>

#include <grif/grif_fpga_mgr/grif_fpga.h>
#include <common_fpga/fpgafeat.h>

#include "export.h"

/* -1 is the default value when there is no PHC device handled by grif-phc.ko. It
 * must be exacly -1 because `ethtool -T` treats -1 as 'no device'.
 */
#define PHC_INDEX_NOT_SET (-1)

/* The following are global variables used by the single instance of
 * grif-phc-mux device. Note that this driver is not capable of handling more
 * that one platform device and you have to make sure that there is only one
 * such platform device is described in DT. */

/* This atomic counter is used to ensure that only one instance of phc_mux
 * device is registered. It is incremented on module probe and decremened on
 * module remove.
 */
static atomic_t phc_mux_cnt = ATOMIC_INIT(0);

/* This is the address of the single register provided by hardware to switch
 * multiplexing modes. Writing 1 to this register switches mux to single phc
 * mode, writing 0 switches to mode with two separate PHCs.
 */
#define PHC_MUX_CTRL_CR 0

/* grif_phc_index array maps stcmtk,port-number property of a DT node to the PTP clock
 * corresponding to that node's platform_device
 */

/* phc_mux_separate_mode bool is exported to sysfs and user can switch modes through it. */
struct phc_mux {
	struct platform_device *pdev;
	struct regmap *regmap;
	struct fpga_feature *phc_mux_feat;
        struct stcmtk_common_fpga *fpga;

	bool using_mux_feat;

	atomic_t grif_phc_index[2];
	atomic_t phc_mux_separate_mode;
};

static ssize_t separate_phc_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct phc_mux *data = dev_get_drvdata(dev);
	if(!data) {
		pr_err("Error getting driver data.");
		BUG();
	};
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&data->phc_mux_separate_mode));
}

static ssize_t separate_phc_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;
	int len;
	int err;
        struct phc_mux *data = dev_get_drvdata(dev);
	if(!data) {
		pr_err("Error getting driver data.");
		BUG();
	};

        struct fpga_feature *f = data->phc_mux_feat;

	err = sscanf(buf, "%d %n", &val, &len);
	if (err != 1)
		return -EINVAL;

	/* It is actually bool */
	val = !!val;
	/* Check situations when it's unable to turn separate mode on */
	if (data->using_mux_feat) {
		/* If we use PHC_MUX feature we have to turn FPGA's separate mode on */
		err = regmap_write(data->regmap, f->cr_base + PHC_MUX_CTRL_CR, !val);
		if (err) {
			dev_err(dev, "PHC mode writing error!\n");
			return -EINVAL;
		}
	}
	/* In case we dont use PHC_MUX feature and FPGA has one PHC instance,
	 * we cant turn separate mode on */
	else if (atomic_read(&data->grif_phc_index[0]) == -1
			|| atomic_read(&data->grif_phc_index[1]) == -1) {
		dev_err(dev, "PHC mode: unable to enable separate mode!\n");
		return -EINVAL;
	}
	/* We dont need `else` construction here because we havent other situations 
	 * when we can't enable separate mode */

	atomic_set(&data->phc_mux_separate_mode, val);
	dev_set_drvdata(dev, data);

	return (ssize_t)len;
}
DEVICE_ATTR_RW(separate_phc_mode);

/* get_grif_phc_index - get the index of PHC being handled by grif-phc.ko
 *
 * If there is a PHC device with specified value of port_num being handled by
 * girf-phc driver this fuction returns its phc_index. If there is no such
 * device returns -1.
 *

 * The main purpose of this function is to provide means for a network driver
 * (e.g grif-net) to find out the index of its assosiated PTP Hardware
 * Clock. Value returned by this function is always valid to be saved in field
 * `phc_index` of `struct ethtool_ts_info`
 *
 * See also: ethtool(8)
 */
int get_grif_phc_index(struct device *dev, ssize_t port_num)
{
	int index;
	struct phc_mux *data;

	if(!dev)
		return -1;

	data = dev_get_drvdata(dev);
	if(!data) {
		pr_err("Error getting driver data.");
		return -1;
	};

	/* If we are not is separate mode (i.e we are using the same PHC for
	 * both network ports) and we use PHC_MUX feature, then both ports get
	 * the phc of first port (having index 0).
	 * If we dont use PHC_MUX feature, we return any PHC.
	 */
	if (!atomic_read(&data->phc_mux_separate_mode)) {
		if (data->using_mux_feat) {
			port_num = 0;
			index = atomic_read(&data->grif_phc_index[port_num]);
		}
		else {
			/* We guarantee that grif_phc_index contains at least one available PHC
			 * So we just need to find availabe PHC in array */
			index = atomic_read(&data->grif_phc_index[0]);
			if (index == -1)
				index = atomic_read(&data->grif_phc_index[1]);
		}
	}
	else
		index = atomic_read(&data->grif_phc_index[port_num]);

	pr_debug("grif_phc_index = %d", index);

	return index;
}
EXPORT_SYMBOL(get_grif_phc_index);


/* set_grif_phc_index, unset_grif_phc_index - set or unset index of PHC handled by
 * grif_phc driver
 *
 * These functions are for private use by grif_phc.ko. They allow our module
 * (grif_phc) to set the index and provide availability status of PHC handled by
 * it for later retrieval by some network driver (e.g grif_net).
 *
 * As soon as its PHC device becomes available (registered, etc.) for userspace
 * the grif-phc must call `set_grif_phc_index`. In the same way
 * `unset_grif_phc_index` must be called right before the PHC becomes unavailable
 * (unregistered, etc.)
 *
 * At module load time the index is not set. Setting the index when it is being
 * set or unsetting when it is not being set are not allowed.
 */
void set_grif_phc_index(struct device *dev, ssize_t port_num, int new)
{
	int old;
	struct phc_mux *data;

	if(!dev)
		return;

	pr_debug("Setting `grif_phc_index` to %d", new);

	BUG_ON(new == PHC_INDEX_NOT_SET);
	BUG_ON(new < 0);

	data = dev_get_drvdata(dev);
	if(!data) {
		pr_err("Error getting driver data.");
		return;
	};

	old = atomic_xchg(&data->grif_phc_index[port_num], new);

	if (old != PHC_INDEX_NOT_SET && new != PHC_INDEX_NOT_SET) {
		pr_err("Setting `grif_phc_index` to %d when it is already set to %d. Have you forgotten to `unset_grif_phc_index` before setting it again?",
			new, old);
		BUG();
	}
	dev_set_drvdata(dev, data);
}
EXPORT_SYMBOL(set_grif_phc_index);

/* See comment for `set_grif_phc_index` above */
void unset_grif_phc_index(struct device *dev, ssize_t port_num)
{
	int old;
	struct phc_mux *data;

	if(!dev)
		return;

	data = dev_get_drvdata(dev);
	if(!data) {
		pr_err("Error getting driver data.");
		return;
	};

	old = atomic_xchg(&data->grif_phc_index[port_num], PHC_INDEX_NOT_SET);

	if (old == PHC_INDEX_NOT_SET) {
		pr_err("Unsetting `grif_phc_index` when it is not set yet (or already unset)");
		BUG();
	}

	pr_debug("Unsetting `grif_phc_index`. Old value was %d", old);
	dev_set_drvdata(dev, data);
}
EXPORT_SYMBOL(unset_grif_phc_index);


static int grif_phc_mux_probe(struct platform_device *pdev)
{
	int err = 0;
	struct phc_mux *data;
	struct stcmtk_common_fpga *fpga;
	struct device_node *np = pdev->dev.of_node;

	preempt_disable();
	if (atomic_read(&phc_mux_cnt)) {
		preempt_enable();
		dev_err(&pdev->dev,
			"There already exists another instance of phc_mux. Refusing to load.\n");
		err = -EBUSY;
		goto err_exists;
	}

	atomic_inc(&phc_mux_cnt);
	preempt_enable();

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	
	if (!data) {
		dev_err(&pdev->dev, "Couldn't allocate phc_mux data.\n");
		err = -ENOMEM;
                goto err_dec_cnt;
	}

	/* Initialize the values of grif_phc_mux variables */
	atomic_set(&data->grif_phc_index[0], PHC_INDEX_NOT_SET);
	atomic_set(&data->grif_phc_index[1], PHC_INDEX_NOT_SET);
	atomic_set(&data->phc_mux_separate_mode, 0);

	fpga = stcmtk_get_fpga(np);
	if (!fpga || IS_ERR_VALUE(fpga)) {
		dev_err(&pdev->dev, "Couldn't get FPGA manager.");
		err = -ENODEV;
                goto err_dec_cnt;
	}

        data->fpga = fpga;

	data->regmap = dev_get_regmap(fpga->dev, NULL);
	if (!data->regmap) {
		dev_warn(&pdev->dev, "Couldn't access FPGA regmap. Deferring...");
		err = -EPROBE_DEFER;
                goto err_release_fpga;
	}

	data->phc_mux_feat = stcmtk_fpga_get_feature(fpga,
                                                FPGA_FEATURE_PHC_MUX);
	if (!data->phc_mux_feat) {
		dev_err(&pdev->dev, "Couldn't get PHC_MUX feature.");
		data->using_mux_feat = false;
		dev_set_drvdata(&pdev->dev, data);
	}
	else {
		data->using_mux_feat = true;
		dev_set_drvdata(&pdev->dev, data);
		/* Immediately set initial mode */
		err = regmap_write(data->regmap, data->phc_mux_feat->cr_base + PHC_MUX_CTRL_CR,
				!atomic_read(&data->phc_mux_separate_mode));
		if (err) {
			dev_err(&pdev->dev, "PHC mode writing error!\n");
			err = -EINVAL;
			goto err_release_fpga;
		}
	}

	err = device_create_file(&pdev->dev, &dev_attr_separate_phc_mode);
	if (err) {
		dev_err(&pdev->dev, "Failed to device_create_file 'separate_phc_mode', err = %d\n", err);
		err = -ENODEV;
                goto err_release_fpga;
	}

	return 0;

err_release_fpga:
	stcmtk_put_fpga(fpga);
err_dec_cnt:
	atomic_dec(&phc_mux_cnt);
err_exists:
        return err;
}

static int grif_phc_mux_remove(struct platform_device *pdev)
{
        struct phc_mux *data = dev_get_drvdata(&pdev->dev);

	device_remove_file(&pdev->dev, &dev_attr_separate_phc_mode);
	stcmtk_put_fpga(data->fpga);
	atomic_dec(&phc_mux_cnt);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id phc_mux_of_match[] = {
	{ .compatible = "stcmtk,phc-mux", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, phc_mux_of_match);
#else
#error "Kernel is required to be compiled with CONFIG_OF set"
#endif

static struct platform_driver grif_phc_mux_drv = {
	.probe  = grif_phc_mux_probe,
	.remove = grif_phc_mux_remove,
	.driver = {
		.name	= "grif_phc_mux",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(phc_mux_of_match),
	},
};

module_platform_driver(grif_phc_mux_drv);

MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.3");
MODULE_AUTHOR("STC Metrotek Fpga Team <fpga@metrotek.ru>");
MODULE_DESCRIPTION("PHC clock multiplexer for Grif-based devices");
