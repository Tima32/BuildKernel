//TODO: добавить/убрать
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/device.h>

#include <grif/grif_fpga_mgr/grif_fpga.h>
#include <common_fpga/fpgafeat.h>
#include "phc_regs.h"
#include <grif_phc_mux/export.h>

// Real frequency in FPGA is 62.5 MHz.
// But we increase PHC counter by 2 every clock cycle
// So we get "Virtual frequency" 125 MHz.
#define SCALE       (2)
#define PPM         (1000000)
#define PPM_IN_PPB  (1000)

// One HW tick (virtual frequency) is 8 ns
#define TICKS_NS_SHIFT         (3)

// Commonly we increment FPGA counter by 1 every cycle
// But if we want get frequency greater (less) than 125 MHz
// we increment counter by 2 (by 0) one time in specified period.
#define DRIFT_DISABLE          (0)
#define DRIFT_INC              (1)
#define DRIFT_DEC              (2)

// "Operation code" for time adjustment
#define OFFSET_DEC             (0)
#define OFFSET_INC             (1)

#define TIMESTAMP_SIZE  5
#define OFFSET_SIZE     4
#define PERIOD_SIZE     2

// Mutex to protect spi transactions between cpu and fpga (instead spin_lock)
static DEFINE_MUTEX(register_lock);

#ifndef DRV_NAME
#define DRV_NAME "grif_phc"
#endif

#ifndef DRV_VERSION
#define DRV_VERSION "0.0.3"
#endif

struct fpga_phc {
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info caps;
	int irq_enabled;
// TODO: надо ли?
	struct platform_device *pdev;

	/* We either use regmap or iomem. Structure is excessive a bit */
	struct regmap *regmap;
	void __iomem *regs_cr;
	void __iomem *regs_sr;

        struct stcmtk_common_fpga *fpga;
	struct fpga_feature *ptp_rtc_feat;

	ssize_t port_num;
	/* This interrupt occurs (if enabled) at the beginning of every second
	 */
	int irq;
	/* EIM or SPI */
	const char *bus_type;

	int (*write_cr_reg)(struct fpga_phc *, u16, u16);
	int (*write_sr_reg)(struct fpga_phc *, u16, u16);

	int (*read_cr_reg)(struct fpga_phc *, u16, void*);
	int (*read_sr_reg)(struct fpga_phc *, u16, void*);

	struct platform_device *grif_phc_mux_dev;
};


static inline int write_reg_cr_eim(struct fpga_phc *fpga_dev, u16 reg_addr, u16 val)
{
	u16 data[4] = { 0 };
	data[0] = val;

	memcpy_toio(fpga_dev->regs_cr + reg_addr*2, &data, 8);
	return 0;
}

static inline int write_reg_sr_eim(struct fpga_phc *fpga_dev, u16 reg_addr, u16 val)
{
	u16 data[4] = { 0 };
	data[0] = val;

	memcpy_toio(fpga_dev->regs_sr + reg_addr*2, &data, 8);
	return 0;
}

static inline int write_reg_cr_spi(struct fpga_phc *fpga_dev, u16 reg_addr, u16 val)
{
	int port = (int) fpga_dev->port_num;

	return regmap_write(fpga_dev->regmap, fpga_dev->ptp_rtc_feat->cr_base + port*fpga_dev->ptp_rtc_feat->cr_cnt + reg_addr, val);
}


static inline int write_reg_sr_spi(struct fpga_phc *fpga_dev, u16 reg_addr, u16 val)
{
	int port = (int) fpga_dev->port_num;

	return regmap_write(fpga_dev->regmap, fpga_dev->ptp_rtc_feat->sr_base + port*fpga_dev->ptp_rtc_feat->sr_cnt + reg_addr, val);
}


static inline int read_reg_cr_eim(struct fpga_phc *fpga_dev, u16 reg_addr, void *val)
{
	memcpy_fromio(val, fpga_dev->regs_cr + reg_addr*2, 8);
	return 0;
}

static inline int read_reg_sr_eim(struct fpga_phc *fpga_dev, u16 reg_addr, void *val)
{
	memcpy_fromio(val, fpga_dev->regs_sr + reg_addr*2, 8);
	return 0;
}

static inline int read_reg_cr_spi(struct fpga_phc *fpga_dev, u16 reg_addr, void *val)
{
	int port = (int) fpga_dev->port_num;

	return regmap_read(fpga_dev->regmap, fpga_dev->ptp_rtc_feat->cr_base + port*fpga_dev->ptp_rtc_feat->cr_cnt +reg_addr,val);
}

static inline int read_reg_sr_spi(struct fpga_phc *fpga_dev, u16 reg_addr, void *val)
{
	int port = (int) fpga_dev->port_num;

	return regmap_read(fpga_dev->regmap, fpga_dev->ptp_rtc_feat->sr_base + port*fpga_dev->ptp_rtc_feat->sr_cnt +reg_addr, val);
}

static irqreturn_t fpga_isr(int irq, void *priv)
{
	struct fpga_phc *phc = priv;

	struct ptp_clock_event event;

	event.type = PTP_CLOCK_PPS;
	ptp_clock_event(phc->ptp_clock, &event);

	return IRQ_HANDLED;
}

static u64 fpga_time_read(struct fpga_phc *data)
{
        int i;
	u64 hwticks = 0;
	u16 time_hw [TIMESTAMP_SIZE] = { 0 };
	u16 raw_data[4];

	/* Perform a strobe */
	data->write_cr_reg(data, PTP_RTC_TIME_CR,
					BIT(PTP_RTC_TIME_CR_RD_STB));
	data->write_cr_reg(data, PTP_RTC_TIME_CR, 0 );

	/* Read raw data */
	data->read_sr_reg(data, PTP_RTC_TIME_RD_W0_SR,
                                                raw_data);
	time_hw[0] = raw_data[0];

	data->read_sr_reg(data, PTP_RTC_TIME_RD_W1_SR,
                                                raw_data);
	time_hw[1] = raw_data[0];

	data->read_sr_reg(data, PTP_RTC_TIME_RD_W2_SR,
                                                raw_data);
	time_hw[2] = raw_data[0];

	data->read_sr_reg(data, PTP_RTC_TIME_RD_W3_SR,
                                                raw_data);
	time_hw[3] = raw_data[0];

	data->read_sr_reg(data, PTP_RTC_TIME_RD_W4_SR,
                                                raw_data);
	time_hw[4] = raw_data[0];

	for (i = 0; i < TIMESTAMP_SIZE; i++) {
		hwticks = (u64) time_hw[i] << (16*i) | hwticks;
	}

	return hwticks << TICKS_NS_SHIFT;
}


static void fpga_time_write(struct fpga_phc *data, u64 ns)
{
        int i;
	u64 hwticks = 0;
	u16 time_hw [TIMESTAMP_SIZE] = { 0 };

	hwticks = ns >> TICKS_NS_SHIFT;

        for (i = 0; i < TIMESTAMP_SIZE; i++) {
          //TODO: насколько это правильно
                time_hw[i] = hwticks >> (16*i) & 0xffff;
        }

	data->write_cr_reg(data, PTP_RTC_TIME_WR_W0_CR, time_hw[0] );
	data->write_cr_reg(data, PTP_RTC_TIME_WR_W1_CR, time_hw[1] );
	data->write_cr_reg(data, PTP_RTC_TIME_WR_W2_CR, time_hw[2] );
	data->write_cr_reg(data, PTP_RTC_TIME_WR_W3_CR, time_hw[3] );
	data->write_cr_reg(data, PTP_RTC_TIME_WR_W4_CR, time_hw[4] );
	/* Perform a strobe */
	data->write_cr_reg(data, PTP_RTC_TIME_CR,
						BIT(PTP_RTC_TIME_CR_WR_STB));
	data->write_cr_reg(data, PTP_RTC_TIME_CR, 0 );

}

static void fpga_time_offset(struct fpga_phc *data, u64 delta_ns, u8 dir)
{
	u64 delta_hwticks;
        int i;
	u16 offset [OFFSET_SIZE] = { 0 };
	unsigned raw_data = 0;

	delta_hwticks = delta_ns >> TICKS_NS_SHIFT;

        for (i = 0; i < OFFSET_SIZE; i++) {
                offset[i] = delta_hwticks >> (16*i) & 0xffff;
        }

	data->write_cr_reg(data,  PTP_RTC_OFFSET_W0_CR, offset[0] );
	data->write_cr_reg(data,  PTP_RTC_OFFSET_W1_CR, offset[1] );
	data->write_cr_reg(data,  PTP_RTC_OFFSET_W2_CR, offset[2] );
	data->write_cr_reg(data,  PTP_RTC_OFFSET_W3_CR, offset[3] );
        raw_data = dir << PTP_RTC_OFFSET_CR_INC_NDEC_TYPE;
	/* Perform a strobe */
	data->write_cr_reg(data,  PTP_RTC_OFFSET_CR,
						BIT(PTP_RTC_OFFSET_CR_WR_STB) | raw_data );
        //TODO: сработает ли с обнулением, или нужно оставить raw_data в регистре
	data->write_cr_reg(data,  PTP_RTC_OFFSET_CR, raw_data );
}


static void fpga_time_drift(struct fpga_phc *data, u32 period, u8 act)
{
        int i;
	u16 drift_period [PERIOD_SIZE] = { 0 };
	unsigned raw_data = 0;

        for (i = 0; i < PERIOD_SIZE; i++) {
                drift_period[i] = period >> (16*i) & 0xffff;
        }

	data->write_cr_reg(data,  PTP_RTC_DRIFT_PERIOD_W0_CR, drift_period[0] );
	data->write_cr_reg(data,  PTP_RTC_DRIFT_PERIOD_W1_CR, drift_period[1] );
        raw_data = act << PTP_RTC_DRIFT_CR_ACT_B0;
	/* Perform a strobe */
	data->write_cr_reg(data, PTP_RTC_DRIFT_CR,
						BIT(PTP_RTC_DRIFT_CR_WR_STB) | raw_data );
        //TODO: сработает ли с обнулением, или нужно оставить raw_data в регистре
	data->write_cr_reg(data,  PTP_RTC_DRIFT_CR, raw_data );
}


static void fpga_pps_enable(struct fpga_phc *data, int enable)
{

	data->write_cr_reg(data,  PTP_RTC_CTRL_CR, BIT(PTP_RTC_CTRL_CR_IRQ_EN) & enable );
}


static int ptp_fpga_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct fpga_phc *phc = container_of(ptp, struct fpga_phc, caps);
	u8 drift_act = DRIFT_INC;
	u32 drift = 0;

	if (ppb < 0) {
		drift_act = DRIFT_DEC;
		ppb = -ppb;
	}

	if (ppb == 0) {
		drift_act = DRIFT_DISABLE;
	} else {
		drift = ( PPM * PPM_IN_PPB / SCALE ) / ppb;
	}

	mutex_lock(&register_lock);

	fpga_time_drift(phc, drift, drift_act);

	mutex_unlock(&register_lock);

	return 0;
}



static int ptp_fpga_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct fpga_phc *phc = container_of(ptp, struct fpga_phc, caps);
	u8 offset_dir = OFFSET_INC;

	if (delta < 0) {
		offset_dir = OFFSET_DEC;
		delta = -delta;
	}

        // Use mutex instead spin_lock from soc driver, because spi process can sleep
	mutex_lock(&register_lock);

	fpga_time_offset(phc, delta, offset_dir);

	mutex_unlock(&register_lock);

	return 0;
}


static int ptp_fpga_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct fpga_phc *phc = container_of(ptp, struct fpga_phc, caps);
	u64 ns;
	u32 remainder;

        // Use mutex instead spin_lock from soc driver, because spi process can sleep
	mutex_lock(&register_lock);

	ns = fpga_time_read(phc);

	mutex_unlock(&register_lock);

	ts->tv_sec = div_u64_rem(ns, 1000000000, &remainder);
	ts->tv_nsec = remainder;

	return 0;
}


static int ptp_fpga_settime(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
	struct fpga_phc *phc = container_of(ptp, struct fpga_phc, caps);
	u64 ns;

	ns = ts->tv_sec * 1000000000ULL;
	ns += ts->tv_nsec;

        // Use mutex instead spin_lock from soc driver, because spi process can sleep
	mutex_lock(&register_lock);

	fpga_time_write(phc, ns);

	mutex_unlock(&register_lock);

	return 0;
}


static int ptp_fpga_enable(struct ptp_clock_info *ptp, struct ptp_clock_request *request, int on)
{
	struct fpga_phc *data = container_of(ptp, struct fpga_phc, caps);

	switch (request->type) {
	case PTP_CLK_REQ_PPS:
		fpga_pps_enable(data, on);
		return 0;

	default:
		break;
	}

	return -EOPNOTSUPP;
}


static struct ptp_clock_info ptp_fpga_caps = {
	.owner		= THIS_MODULE,
	.name		= "FPGA PTP Timer",
	.max_adj	= 100000000,
	.n_ext_ts	= 0,
	.n_pins		= 0,
	.pps		= 1,
	.adjfreq	= ptp_fpga_adjfreq,
	.adjtime	= ptp_fpga_adjtime,
	.gettime64	= ptp_fpga_gettime,
	.settime64	= ptp_fpga_settime,
	.enable		= ptp_fpga_enable
};


static int grif_phc_remove(struct platform_device *pdev)
{
	struct fpga_phc *data = dev_get_drvdata(&pdev->dev);

//	free_irq(data->irq, data);


	if(data->grif_phc_mux_dev)
		unset_grif_phc_index(&data->grif_phc_mux_dev->dev, data->port_num);
	ptp_clock_unregister(data->ptp_clock);
	stcmtk_put_fpga(data->fpga);
	return 0;
}


static int grif_phc_probe(struct platform_device *pdev)
{
	struct fpga_phc *data;
	struct stcmtk_common_fpga *fpga;
	struct device_node *np = pdev->dev.of_node;
	int err;
	struct device_node *grif_phc_mux_node = NULL;
	struct resource *res_cr;
	struct resource *res_sr;

        fpga = stcmtk_get_fpga(np);
	if (!fpga || IS_ERR_VALUE(fpga)) {
		dev_err(&pdev->dev, "Couldn't get FPGA manager.");
		return -ENODEV;
	}

        data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Couldn't allocate phc data.\n");
		err = -ENOMEM;
		goto ref_put;
	}

	err = of_property_read_u32(pdev->dev.of_node, "stcmtk,port-num", &data->port_num);
	if (err) {
		dev_err(&pdev->dev, "stcmtk,port-num DT property not found");
		goto ref_put;
	}

        data->fpga = fpga;

	err = of_property_read_string_index(pdev->dev.of_node, "bus-type", 0, &data->bus_type);
	if (err) {
		dev_err(&pdev->dev, "bus type not found\n");
		goto ref_put;
	}

	data->ptp_rtc_feat = stcmtk_fpga_get_feature(fpga,
                                                FPGA_FEATURE_PTP_RTC);
	if (!data->ptp_rtc_feat) {
		dev_err(&pdev->dev, "Couldn't get PTP_RTC feature.");
		err = -ENODEV;
		goto ref_put;
	}

	/* Check port mask (1 == use PHC port 0; 2 == use PHC port 1;
	 * 3 == use both PHC ports).
	 * We get data->port_num from DT node (available values = 0 or 1) and
	 * exit if mask doesnt support current PHC instance */
	if (data->ptp_rtc_feat->port_mask != (data->port_num + 1) &&
			data->ptp_rtc_feat->port_mask != 3) {
		err = -ENODEV;
		goto ref_put;
	}

	if (!strcmp("spi", data->bus_type)) {
		data->regmap = dev_get_regmap(fpga->dev, NULL);
		if (!data->regmap) {
			dev_warn(&pdev->dev, "Couldn't access FPGA regmap. Deferring...");
			err = -EPROBE_DEFER;
			goto ref_put;
		}
		data->write_sr_reg = write_reg_sr_spi;
		data->write_cr_reg = write_reg_cr_spi;
		data->read_sr_reg = read_reg_sr_spi;
		data->read_cr_reg = read_reg_cr_spi;
	} else { /* in case of EIM */
		/* If we dont use PHC_MUX feature (port_mask equls 1 or 2) and we have only one PHC,
		 * we have to use alternative CR/SR regs */
		if (data->ptp_rtc_feat->port_mask == 1 || data->ptp_rtc_feat->port_mask == 2) {
			/* Get alternative CR reg */
			res_cr = platform_get_resource_byname(pdev, IORESOURCE_MEM, "alt-reg-cr");
			if (!res_cr) {
				dev_err(&pdev->dev, "no memory resource defined for alternative CR\n");
				err = -ENODEV;
				goto ref_put;
			}

			/* Get alternative SR reg */
			res_sr = platform_get_resource_byname(pdev, IORESOURCE_MEM, "alt-reg-sr");
			if (!res_sr) {
				dev_err(&pdev->dev, "no memory resource defined for alternative SR\n");
				err = -ENODEV;
				goto ref_put;
			}
		}
		else {
			/* Get CR reg */
			res_cr = platform_get_resource_byname(pdev, IORESOURCE_MEM, "reg-cr");
			if (!res_cr) {
				dev_err(&pdev->dev, "no memory resource defined for CR\n");
				err = -ENODEV;
				goto ref_put;
			}

			/* Get SR reg */
			res_sr = platform_get_resource_byname(pdev, IORESOURCE_MEM, "reg-sr");
			if (!res_sr) {
				dev_err(&pdev->dev, "no memory resource defined for SR\n");
				err = -ENODEV;
				goto ref_put;
			}
		}

		data->regs_cr = devm_ioremap_resource(&pdev->dev, res_cr);
		data->regs_sr = devm_ioremap_resource(&pdev->dev, res_sr);

		data->write_sr_reg = write_reg_sr_eim;
		data->write_cr_reg = write_reg_cr_eim;
		data->read_sr_reg = read_reg_sr_eim;
		data->read_cr_reg = read_reg_cr_eim;
	}

	data->grif_phc_mux_dev = NULL;
        dev_set_drvdata(&pdev->dev, data);
/*
	data->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!data->irq) {
		err = -ENODEV;
		dev_err(&pdev->dev, "Failed to parse and map irq");
		goto ref_put;
	}
*/
	data->caps = ptp_fpga_caps;
	data->ptp_clock = ptp_clock_register(&data->caps, &pdev->dev);

	if (IS_ERR(data->ptp_clock)) {
		dev_err(&pdev->dev, "Error: ptp_clock_register failed.");
		err = PTR_ERR(data->ptp_clock);
		goto ref_put;
	}

	grif_phc_mux_node = of_find_node_by_name(NULL, "grif-phc-mux");
	if(!grif_phc_mux_node) {
		dev_err(&pdev->dev, "No grif-phc-mux device tree node\n");
		err = -ENODEV;
		goto unregister_ptp_clk;
	};

	data->grif_phc_mux_dev = of_find_device_by_node(grif_phc_mux_node);
	if(!data->grif_phc_mux_dev) {
		dev_err(&pdev->dev, "No grif-phc-mux device\n");
		err = -ENODEV;
		goto unregister_ptp_clk;
	};
	
	set_grif_phc_index(&data->grif_phc_mux_dev->dev, data->port_num, 
		ptp_clock_index(data->ptp_clock));
/*
	err = request_irq(data->irq, fpga_isr, IRQF_SHARED, "grif-phc", data);
	if (err) {
		dev_err(&pdev->dev, "Cannot allocate interrupt %d\n", data->irq");
		goto unset_phc_index;
	}
*/
	return 0;

unset_phc_index:
	if(data->grif_phc_mux_dev)
		unset_grif_phc_index(&data->grif_phc_mux_dev->dev, data->port_num);
unregister_ptp_clk:
	ptp_clock_unregister(data->ptp_clock);
ref_put:
	stcmtk_put_fpga(fpga);
ret:
	return err;
}

#ifdef CONFIG_OF
static const struct of_device_id phc_of_match[] = {
	{ .compatible = "stcmtk,phc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, phc_of_match);
#else
#error "Kernel is required to be compiled with CONFIG_OF set"
#endif

static struct platform_driver grif_phc_driver = {
	.probe  = grif_phc_probe,
	.remove = grif_phc_remove,
	.driver = {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(phc_of_match),
	},
};

static int __init ptp_fpga_init(void)
{
	/*
	 * We use platform_driver_register instead of platform_driver_probe here
	 * to allow this module to be loaded even when there are no devices that
	 * can be handled by it. This is done so because other modules (like
	 * `grif_net.ko`) may use symbols that we export (see export.{c,h}) thus
	 * depending on us.
	 */
	return platform_driver_register(&grif_phc_driver);
}

static void __exit ptp_fpga_exit(void)
{
	platform_driver_unregister(&grif_phc_driver);
}

module_init(ptp_fpga_init);
module_exit(ptp_fpga_exit);

MODULE_AUTHOR("STC Metrotek Fpga Team <fpga@metrotek.ru>");
MODULE_DESCRIPTION("FPGA PTP Hardware Clock for Grif-based projects");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

