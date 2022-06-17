#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>

#include <linux/printk.h>
#include <linux/types.h>
#include <asm-generic/bug.h>

#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

#include "export.h"

/* -1 is the default value when there is no PHC device handled by etn_phc.ko. It
 * must be exacly -1 because `ethtool -T` treats -1 as 'no device'.
 */
#define PHC_INDEX_NOT_SET (-1)

/* The following are global variables used by the single instance of
 * etn_phc_mux device. Note that this driver is not capable of handling more
 * that one platform device and you have to make sure that there is only one
 * such platform device is described in DT. */

/* This atomic counter is used to ensure that only one instance of phc_mux
 * device is registered. It is incremented on module probe and decremened on
 * module remove.
 */
static atomic_t phc_mux_cnt = ATOMIC_INIT(0);

/* This array maps stcmtk,port-number property of a DT node to the PTP clock
 * corresponding to that node's platform_device
 */
static atomic_t etn_phc_index[2];

/* This is the address of the single register provided by hardware to switch
 * multiplexing modes. Writing 1 to this register switches mux to single phc
 * mode, writing 0 switches to mode with two separate PHCs.
 */
static u32 __iomem *mux_reg;

/* This bool is exported to sysfs and user can switch modes through it. */
static atomic_t phc_mux_separate_mode;


static ssize_t separate_phc_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&phc_mux_separate_mode));
}

static ssize_t separate_phc_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;
	int len;
	int err;

	err = sscanf(buf, "%d %n", &val, &len);
	if (err != 1)
		return -EINVAL;

	/* It is actually bool */
	val = !!val;

	iowrite32(!val, mux_reg);

	atomic_set(&phc_mux_separate_mode, val);

	return (ssize_t)len;
}
DEVICE_ATTR_RW(separate_phc_mode);

/* get_etn_phc_index - get the index of PHC being handled by etn_phc.ko
 *
 * If there is a PHC device with specified value of port_num being handled by
 * etn_phc driver this fuction returns its phc_index. If there is no such
 * device returns -1.
 *
 * The main purpose of this function is to provide means for a network driver
 * (e.g etn_net) to find out the index of its assosiated PTP Hardware
 * Clock. Value returned by this function is always valid to be saved in field
 * `phc_index` of `struct ethtool_ts_info`
 *
 * See also: ethtool(8)
 */
int get_etn_phc_index(ssize_t port_num)
{
	int index;

	/* If we are not is separate mode (i.e we are using the same PHC for
	 * both network ports), then both ports get the phc of first port
	 * (having index 0)
	 */
	if (!atomic_read(&phc_mux_separate_mode))
		port_num = 0;

	index = atomic_read(&etn_phc_index[port_num]);

	pr_debug("etn_phc_index = %d", index);

	return index;
}
EXPORT_SYMBOL(get_etn_phc_index);


/* set_etn_phc_index, unset_etn_phc_index - set or unset index of PHC handled by
 * etn_phc driver
 *
 * These functions are for private use by etn_phc.ko. They allow our module
 * (etn_phc) to set the index and provide availability status of PHC handled by
 * it for later retrieval by some network driver (e.g etn_net).
 *
 * As soon as its PHC device becomes available (registered, etc.) for userspace
 * the etn_phc must call `set_etn_phc_index`. In the same way
 * `unset_etn_phc_index` must be called right before the PHC becomes unavailable
 * (unregistered, etc.)
 *
 * At module load time the index is not set. Setting the index when it is being
 * set or unsetting when it is not being set are not allowed.
 */
void set_etn_phc_index(ssize_t port_num, int new)
{
	int old;

	pr_debug("Setting `etn_phc_index` to %d", new);

	BUG_ON(new == PHC_INDEX_NOT_SET);
	BUG_ON(new < 0);

	old = atomic_xchg(&etn_phc_index[port_num], new);

	if (old != PHC_INDEX_NOT_SET && new != PHC_INDEX_NOT_SET) {
		pr_err("Setting `etn_phc_index` to %d when it is already set to %d. Have you forgotten to `unset_etn_phc_index` before setting it again?",
			new, old);
		BUG();
	}
}
EXPORT_SYMBOL(set_etn_phc_index);

/* See comment for `set_etn_phc_index` above */
void unset_etn_phc_index(ssize_t port_num)
{
	int old;

	old = atomic_xchg(&etn_phc_index[port_num], PHC_INDEX_NOT_SET);

	if (old == PHC_INDEX_NOT_SET) {
		pr_err("Unsetting `etn_phc_index` when it is not set yet (or already unset)");
		BUG();
	}

	pr_debug("Unsetting `etn_phc_index`. Old value was %d", old);
}
EXPORT_SYMBOL(unset_etn_phc_index);


static int etn_phc_mux_probe(struct platform_device *pdev)
{
	int err;
	struct stcmtk_common_fpga *f = stcmtk_get_fpga((&pdev->dev)->of_node);
	if (!f) {
		dev_err(&pdev->dev, "Can't get etn_fpga reference!\n");
		return -EIO;
	}

	preempt_disable();
	if (atomic_read(&phc_mux_cnt)) {
		preempt_enable();
		dev_err(&pdev->dev, "There already exists another instance of phc_mux. Refusing to load.");
		return -EBUSY;
	}

	atomic_inc(&phc_mux_cnt);
	preempt_enable();

	/* Initialize the values of local variables */
	atomic_set(&etn_phc_index[0], PHC_INDEX_NOT_SET);
	atomic_set(&etn_phc_index[1], PHC_INDEX_NOT_SET);
	atomic_set(&phc_mux_separate_mode, 0);

        /* Try to increase FPGA reference counter */
        if (etn_fpga_ref_get(f)) {
                pr_err("Failed to get reference on FPGA\n");
                goto err_dec_cnt;
        }

	mux_reg = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR_OR_NULL(mux_reg)) {
		dev_err(&pdev->dev, "Failed to ioremap registers. ERR = %d", (int) PTR_ERR(mux_reg));
                goto err_release_fpga;
	}

	/* Immediately set initial mode */
	iowrite32(!atomic_read(&phc_mux_separate_mode), mux_reg);

	dev_info(&pdev->dev, "my register is at %p", mux_reg);

	err = device_create_file(&pdev->dev, &dev_attr_separate_phc_mode);
	if (err) {
		dev_err(&pdev->dev, "Failed to device_create_file 'separate_phc_mode', err = %d\n", err);
                goto err_ummap;
	}

	return 0;

err_ummap:
        iounmap(mux_reg);
err_release_fpga:
        etn_fpga_ref_put(f);
err_dec_cnt:
	atomic_dec(&phc_mux_cnt);
        return -EINVAL;
}

static int etn_phc_mux_remove(struct platform_device *pdev)
{
	struct stcmtk_common_fpga *f = stcmtk_get_fpga((&pdev->dev)->of_node);
	if (!f) {
		dev_err(&pdev->dev, "Can't get etn_fpga reference!\n");
		return -EIO;
	}

	device_remove_file(&pdev->dev, &dev_attr_separate_phc_mode);
	iounmap(mux_reg);
	etn_fpga_ref_put(f);
	atomic_dec(&phc_mux_cnt);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id phc_mux_of_match[] = {
	{ .compatible = "etn,phc-mux", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, phc_mux_of_match);
#else
#error "Kernel is required to be compiled with CONFIG_OF set"
#endif

static struct platform_driver etn_phc_mux_drv = {
	.probe  = etn_phc_mux_probe,
	.remove = etn_phc_mux_remove,
	.driver = {
		.name	= "etn_phc_mux",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(phc_mux_of_match),
	},
};

module_platform_driver(etn_phc_mux_drv);

MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.6");
MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("PHC clock multiplexer for SoC-based devices");
