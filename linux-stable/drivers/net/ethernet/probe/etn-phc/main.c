#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <etn_fpga_mgr/etn_fpga.h>
#include <etn_phc_mux/export.h>
#include <common_fpga/fpgafeat.h>

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

#ifndef DRV_NAME
#define DRV_NAME "etn_phc"
#endif

#ifndef DRV_VERSION
#define DRV_VERSION "0.0.8"
#endif


// This structure represent FPGA address space
struct fpga_phc_regs {

	// Current time.
	// XXX: low word must be read/written first
	u32 time_low;
	u32 time_med;
	u32 time_high;

	// Settings for adjusting frequency.
	// XXX: write only.
	// XXX: period must be write first.
	u32 drift_period;
	u32 drift_act;

	// Registers for adjusting time.
	// XXX: write only.
	// XXX: low word must be write first.
	u32 offset_low;
	u32 offset_high;
	u32 offset_dir;

	// Enable/disable PPS IRQ
	u32 irq_enable;
};


DEFINE_SPINLOCK(register_lock);


struct fpga_phc {
	struct fpga_phc_regs *regs;
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info caps;
	int irq_enabled;

	ssize_t port_num;
	uint32_t regs_base;
	/* This interrupt occurs (if enabled) at the beginning of every second
	 */
	int irq;
};


static irqreturn_t fpga_isr(int irq, void *priv)
{
	struct fpga_phc *phc = priv;

	struct ptp_clock_event event;

	event.type = PTP_CLOCK_PPS;
	ptp_clock_event(phc->ptp_clock, &event);

	return IRQ_HANDLED;
}


static u64 fpga_time_read(struct fpga_phc_regs *regs)
{
	u64 hwticks;
	u32 lo, hi, t;

	// We used raw function in order to eliminate pauses between transactions
	lo = __raw_readl(&regs->time_low);
	hi = __raw_readl(&regs->time_med);
	t  = __raw_readl(&regs->time_high);

	(void)t;

	hwticks = ((u64) hi) << 32;
	hwticks |= lo;

	return hwticks << TICKS_NS_SHIFT;
}


static void fpga_time_write(struct fpga_phc_regs *regs, u64 ns)
{
	u64 hwticks;
	u32 lo, hi;

	hwticks = ns >> TICKS_NS_SHIFT;

	hi = hwticks >> 32;
	lo = hwticks & 0xffffffff;

	// We used raw function in order to eliminate pauses between transactions
	__raw_writel(lo, &regs->time_low);
	__raw_writel(hi, &regs->time_med);
	__raw_writel(0,	 &regs->time_high);
}


static void fpga_time_offset(struct fpga_phc_regs *regs, u64 delta_ns, u8 dir)
{
	u64 delta_hwticks;
	u32 lo, hi;

	delta_hwticks = delta_ns >> TICKS_NS_SHIFT;

	hi = delta_hwticks >> 32;
	lo = delta_hwticks & 0xffffffff;

	__raw_writel(lo,  &regs->offset_low);
	__raw_writel(hi,  &regs->offset_high);
	__raw_writel(dir, &regs->offset_dir);
}


static void fpga_time_drift(struct fpga_phc_regs *regs, u32 period, u8 act)
{
	__raw_writel(period, &regs->drift_period);
	__raw_writel(act,    &regs->drift_act);

}


static void fpga_pps_enable(struct fpga_phc_regs *regs, int enable)
{
	__raw_writel(enable, &regs->irq_enable);
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

	fpga_time_drift(phc->regs, drift, drift_act);

	return 0;
}



static int ptp_fpga_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct fpga_phc *phc = container_of(ptp, struct fpga_phc, caps);
	unsigned long flags;
	u8 offset_dir = OFFSET_INC;

	if (delta < 0) {
		offset_dir = OFFSET_DEC;
		delta = -delta;
	}


	spin_lock_irqsave(&register_lock, flags);

	fpga_time_offset(phc->regs, delta, offset_dir);

	spin_unlock_irqrestore(&register_lock, flags);

	return 0;
}


static int ptp_fpga_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct fpga_phc *phc = container_of(ptp, struct fpga_phc, caps);
	unsigned long flags;
	u64 ns;
	u32 remainder;

	spin_lock_irqsave(&register_lock, flags);

	ns = fpga_time_read(phc->regs);

	spin_unlock_irqrestore(&register_lock, flags);

	ts->tv_sec = div_u64_rem(ns, 1000000000, &remainder);
	ts->tv_nsec = remainder;

	return 0;
}


static int ptp_fpga_settime(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
	struct fpga_phc *phc = container_of(ptp, struct fpga_phc, caps);
	unsigned long flags;
	u64 ns;

	ns = ts->tv_sec * 1000000000ULL;
	ns += ts->tv_nsec;

	spin_lock_irqsave(&register_lock, flags);

	fpga_time_write(phc->regs, ns);

	spin_unlock_irqrestore(&register_lock, flags);

	return 0;
}


static int ptp_fpga_enable(struct ptp_clock_info *ptp, struct ptp_clock_request *request, int on)
{
	struct fpga_phc *phc = container_of(ptp, struct fpga_phc, caps);

	switch (request->type) {
	case PTP_CLK_REQ_PPS:
		fpga_pps_enable(phc->regs, on);
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


static int etn_phc_remove(struct platform_device *pdev)
{
	struct fpga_phc *fpga_phc;
	struct stcmtk_common_fpga *f;

	printk("removing pdev at: %p\n", pdev);

	fpga_phc = platform_get_drvdata(pdev);
	f = stcmtk_get_fpga((&pdev->dev)->of_node);
	if (!f) {
		dev_err(&pdev->dev, "Can't get etn_fpga reference!\n");
		return -EIO;
	}

	free_irq(fpga_phc->irq, fpga_phc);
	unset_etn_phc_index(fpga_phc->port_num);
	ptp_clock_unregister(fpga_phc->ptp_clock);
	iounmap(fpga_phc->regs);
	kfree(fpga_phc);
	etn_fpga_ref_put(f);

	return 0;
}


static int etn_phc_probe(struct platform_device *pdev)
{
	struct fpga_phc *fpga_phc;
	int err;
	struct stcmtk_common_fpga *f = stcmtk_get_fpga((&pdev->dev)->of_node);
	if (!f) {
		dev_err(&pdev->dev, "Can't get etn_fpga reference!\n");
		return -EIO;
	}

	printk("probing pdev at: %p\n", pdev);

	if (etn_fpga_ref_get(f)) {
		printk("Failed to get ref on FPGA\n");
		return -EINVAL;
	}

	fpga_phc = kmalloc(sizeof(struct fpga_phc), GFP_KERNEL);
	if (!fpga_phc) {
		printk("Error: fpga_phc allocation failed");
		err = -ENOMEM;
		goto ref_put;
	}

	platform_set_drvdata(pdev, fpga_phc);

	err = of_property_read_u32(pdev->dev.of_node, "stcmtk,port-num", &fpga_phc->port_num);
	if (err) {
		dev_err(&pdev->dev, "stcmtk,port-num DT property not found");
		goto free_fpga_phc;
	}

	fpga_phc->regs = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(fpga_phc->regs)) {
		printk( "Error: can't map FPGA registers" );
		err = -ENOMEM;
		goto free_fpga_phc;
	}

	fpga_phc->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!fpga_phc->irq) {
		err = -ENODEV;
		dev_err(&pdev->dev, "Failed to parse and map irq");
		goto iounmap;
	}

	fpga_phc->caps = ptp_fpga_caps;
	fpga_phc->ptp_clock = ptp_clock_register(&fpga_phc->caps, &pdev->dev);

	if (IS_ERR(fpga_phc->ptp_clock)) {
		err = PTR_ERR(fpga_phc->ptp_clock);
		printk("Error: ptp_clock_register failed");
		goto iounmap;
	}

	set_etn_phc_index(fpga_phc->port_num, ptp_clock_index(fpga_phc->ptp_clock));

	err = request_irq(fpga_phc->irq, fpga_isr, IRQF_SHARED, "etn-phc", fpga_phc);
	if (err) {
		printk("Cannot allocate interrupt %d\n", fpga_phc->irq);
		goto ptp_unregister;
	}

	printk( "Probe done 0x%p\n", fpga_phc->ptp_clock );
	return 0;

ptp_unregister:
	ptp_clock_unregister(fpga_phc->ptp_clock);
iounmap:
	iounmap(fpga_phc->regs);
free_fpga_phc:
	kfree(fpga_phc);
ref_put:
	etn_fpga_ref_put(f);
	return err;
}

#ifdef CONFIG_OF
static const struct of_device_id phc_of_match[] = {
	{ .compatible = "etn,phc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, phc_of_match);
#else
#error "Kernel is required to be compiled with CONFIG_OF set"
#endif

static struct platform_driver etn_phc_driver = {
	.probe  = etn_phc_probe,
	.remove = etn_phc_remove,
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
	 * `etn_net.ko`) may use symbols that we export (see export.{c,h}) thus
	 * depending on us.
	 */
	return platform_driver_register(&etn_phc_driver);
}

static void __exit ptp_fpga_exit(void)
{
	platform_driver_unregister(&etn_phc_driver);
}

module_init(ptp_fpga_init);
module_exit(ptp_fpga_exit);

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("FPGA PTP Hardware Clock for SoC-based projects");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
