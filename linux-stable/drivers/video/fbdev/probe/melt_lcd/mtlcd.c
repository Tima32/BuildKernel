#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

/*
 * Debugging output
 * and
 * Much more debugging output
 */
//#define __BUILD_MTFB_DEBUG
//#define __BUILD_MTFB_DEBUG_EXCRUCIATING_DETAIL

/*
 * Read the LCD's memory instead of getting values from the framebuffer
 * when redrawing a word. It is currently disabled as there is no incentive
 * to ask the display when we already have all the data in the fb memory.
 * Hovewer, the protocol for 'asking' is implemented and one may choose to
 * use it by uncommenting the macro.
 */
//#define __BUILD_MTFB_SCREEN_MEM

#define NAME		"MTLCD"
#define DRV_VERSION	"1.0"

/*
 * Handy constants related to the MT-12232A LCD:
 * There are a total of BUS_WIDTH pins, CMD_WIDTH of which are used to issue
 * the commands (the other pins are reset, chip select and enable strobe - not
 * _technically_ command pins). COLSTART is where the LCD memory starts (see
 * docs).
 * DISP_SIZE_* and NR_* describe the display's geometry.
 */

#define CMD_WIDTH	10
#define BUS_WIDTH	13
#define DATA_BUS_WIDTH	(CMD_WIDTH - 2)
#define COLSTART	0x13

#define CRYSTAL_SIZE_X	61
#define PAGE_SIZE_Y	8
#define NR_CRYSTALS	2
#define NR_PAGES	4
#define DISP_SIZE_X	NR_CRYSTALS * CRYSTAL_SIZE_X
#define DISP_SIZE_Y	NR_PAGES * PAGE_SIZE_Y
#define BPP		16

#define DISP_SIZE_X_MM	56
#define DISP_SIZE_Y_MM	15

#define MULT	10
/*
 * The following defines are to uphold the timings as defined in the docs.
 * Delays are in ns, multiplied by a safety factor of MULT.
 */
#define TCYC	MULT * 2000
#define TAW	MULT * 100
#define TAH	MULT * 20
#define TDS	MULT * 160
#define TDH	MULT * 20
#define TEW	MULT * 300
#define TRES	MULT * 2	 // This one is in milliseconds, and uses udelay
#define TGEN	MULT * 500 // Generic delay to use when nothing
			 // is specified in the documentation

enum lcd_side {
	LEFT,
	RIGHT,
};

static int set_col(struct fb_info *info, int x, int y, int word);

struct lcddata {
	struct gpio_desc		*bus[BUS_WIDTH];
	struct gpio_desc		*res;
	struct gpio_desc		*cs;
	struct gpio_desc		*estrobe;
	struct fb_info			*info;
	struct fb_fix_screeninfo	mtfb_fix;
	struct fb_var_screeninfo	mtfb_var;
	struct fb_ops			mtfb_ops;
	struct fb_deferred_io		mtfb_defio;

	unsigned long int		*mem;
	u32				mtfb_pseudo_palette[16];
};

enum cmds {
	END		= 0xEE,
	STATDRVOFF	= 0xA5,
	STATDRVON	= 0xA4,
	DUTYSEL		= 0xA9,
	RESET		= 0xE2,
	DISPON		= 0xAF,
	PAGESEL		= 0xB8,
	READWD		= 0x300,
	WRITEWD		= 0x100,
	SETADDR		= 0x0,
	ADCSTRAIGHT	= 0xA0,
	ADCREVERSE	= 0xA1,
	DISPLINE	= 0xC0,
	STATREAD	= 0x512,
};

static int mtfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return vm_iomap_memory(vma, info->fix.smem_start, info->fix.smem_len);
}

static void mtfb_defer(struct fb_info *info, struct list_head *pagelist)
{
	int x, y, col, i, pixel_addr, bytes_per_px, curr_pixel;

#ifdef __BUILD_MTFB_DEBUG
	for (i = 0; i < info->fix.smem_len; i++)
		printk(KERN_CONT "%d", *(info->screen_base + i) ? 1 : 0);
		if (i % DISP_SIZE_X == 0)
			printk(KERN_CONT "\n");
#endif // __BUILD_MTFB_DEBUG

	/* These are the displayed pixels:
	 * `upper-left corner`		     <...>	`upper-right corner`
	 *  px1 px2 px3 px4 px5 px6 px7       ...	px(DISP_SIZE_X)
	 *  px(DISP_SIZE_X + 1) ...
	 * 				     <...>
	 * 				      ...	px(DISP_SIZE_X * DISP_SIZE_Y)
	 * `lower-left corner`		     <...>	`lower-right corner`
	 */

	/* The controller only allows access by 8-bit `col`s, which are 1x8 columns
	 * on the display. However, the framebuffer memory is organized such that
	 * pixels are presented linearly as `p1, p2, p3, ...`. The code below creates
	 * the `col`s to be sent to the device.
	 *
	 * x, y	are cartesian coordinates on the display
	 * col	is the column that's going to be sent to the LCD
	 */

	for (x = 0; x < DISP_SIZE_X; x++) {
		for (y = 0; y < DISP_SIZE_Y; y += PAGE_SIZE_Y) {
			col = 0;
			for (i = 0; i < PAGE_SIZE_Y; i++) {
				pixel_addr = DISP_SIZE_X * (y + i) + x;
				bytes_per_px= BPP / 8;
				curr_pixel = *(info->screen_base + pixel_addr * bytes_per_px) ? 1 : 0;
				col += curr_pixel << i;
				}
			set_col(info, x, y, col);
			}
		}
}

static ssize_t mtfb_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos) {
	int ret;

	ret = fb_sys_write(info, buf, count, ppos);
	mtfb_defer(info, NULL);

	return ret;
}

static void mtfb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	cfb_imageblit(info, image);
	mtfb_defer(info, NULL);
}

static void select_chip(struct fb_info *info, enum lcd_side chip)
{
	struct lcddata *priv = info->par;

	if (chip == RIGHT)
		gpiod_set_value(priv->cs, 0);
	else if (chip == LEFT)
		gpiod_set_value(priv->cs, 1);
	else
		printk(KERN_WARNING "MTFB %s @ %s:%d\n", "Couldn't select chip", __func__, __LINE__);

	ndelay(TGEN);
}

static int send_cmd(struct fb_info *info, enum cmds cmd, int param)
{
	int j;
	struct lcddata *priv = info->par;

	switch (cmd) {
#ifdef __BUILD_MTFB_SCREEN_MEM
		/* Reading this data from the display is pointless, as we have
		 * a copy in memory anyway, so let's just use that. In case anyone
		 * ever wants to actually read the pixels, here's the protocol code
		 */
		case READWD:
			send_cmd(info, READDATA, 0);
			ndelay(TAW);

			for (j = 2; j < CMD_WIDTH; j++)
				gpio_direction_input(priv->bus[j]);
			ndelay(TGEN);

			gpiod_set_value(priv->estrobe, 0);
			ndelay(TDS);

			ret = 0;
			for (j = 2; j < CMD_WIDTH; j++)
				ret |= gpio_get_value(priv->bus[j]) << j;

			gpiod_set_value(priv->estrobe, 1);

			for (j = 2; j < CMD_WIDTH; j++)
				gpio_direction_output(priv->bus[j], 0);

			ndelay(TDOH);

			break;
#endif // __BUILD_MTFB_SCREEN_MEM

		default:
#ifdef __BUILD_MTFB_DEBUG_EXCRUCIATING_DETAIL
// This will print everything that's going on on the bus
			printk(KERN_CONT "configuration:\n");
			printk(KERN_CONT "gpio: ");
			for (j = 0; j < CMD_WIDTH; j++)
				printk(KERN_CONT "%3d", j);
			printk(KERN_CONT "\nstate:");
			for (j = 0; j < CMD_WIDTH; j++)
				printk(KERN_CONT "%3d",
					((cmd | param) >>
					(CMD_WIDTH - j - 1)) & 0x1);
			printk(KERN_CONT "\n");
#endif // __BUILD_MTFB_DEBUG_EXCRUCIATING_DETAIL

			for (j = 0; j < CMD_WIDTH; j++)
				gpiod_set_value(priv->bus[j],
					((cmd | param) >>
					(CMD_WIDTH - j - 1)) & 0x1);

			gpiod_set_value(priv->estrobe, 0);
			ndelay(TDS);
			gpiod_set_value(priv->estrobe, 1);
			ndelay(TGEN);
			}

	return 0;
}

#ifdef __BUILD_MTFB_DEBUG
__maybe_unused static void printk_status(struct fb_info *info)
{
	int status = 0;

	send_cmd(info, STATREAD, 0);
	status = send_cmd(info, STATREAD, 0);

	printk(KERN_ERR "MTFB: %s %s %s %s (%d)\n",
		(status >> 4) & 0x1 ? "RESET"	: "NORMAL",
		(status >> 5) & 0x1 ? "ON"	: "OFF",
		(status >> 6) & 0x1 ? "STRAIGHT" : "REVERSE",
		(status >> 7) & 0x1 ? "BUSY"	: "READY",
		status);
}
#endif // __BUILD_MTFB_DEBUG

static int set_col(struct fb_info *info, int x, int y, int word)
{
	int page = (int)(y / 8);

	if ((x > DISP_SIZE_X) || (y > DISP_SIZE_Y) || (x < 0) || (y < 0)) {
		printk(KERN_WARNING "MTFB %s @ %s:%d\n", "Pixel out of bound!", __func__, __LINE__);
		return -1;
		}

	if (x < CRYSTAL_SIZE_X) {
		select_chip(info, LEFT);
	} else {
		x = x - CRYSTAL_SIZE_X - COLSTART;
		select_chip(info, RIGHT);
		}

	send_cmd(info, PAGESEL, page);
	send_cmd(info, SETADDR, (COLSTART + x));
	send_cmd(info, WRITEWD, word);

	return 0;
}

/* Initializing the LCD (each crystal separately).
 * Refer to the MT-12232A documentation for details.
 */
static void lcd_init(struct fb_info *info)
{
	struct lcddata *priv = info->par;

	// 1. Reset the display
	gpiod_set_value(priv->res, 1);
	mdelay(TRES);
	gpiod_set_value(priv->res, 0);
	mdelay(TRES);

	// 2. Reset the controller
	select_chip(info, LEFT);
	send_cmd(info, RESET, 0);
	select_chip(info, RIGHT);
	send_cmd(info, RESET, 0);

	// 3. Clear flags
	select_chip(info, LEFT);
	send_cmd(info, END, 0);
	select_chip(info, RIGHT);
	send_cmd(info, END, 0);

	// 4. Select working mode
	select_chip(info, LEFT);
	send_cmd(info, STATDRVON, 0);
	select_chip(info, RIGHT);
	send_cmd(info, STATDRVON, 0);

	// 5. Select multiplexing mode
	select_chip(info, LEFT);
	send_cmd(info, DUTYSEL, 0);
	select_chip(info, RIGHT);
	send_cmd(info, DUTYSEL, 0);

	// 6. Choose display start line
	select_chip(info, LEFT);
	send_cmd(info, DISPLINE, 0);
	select_chip(info, RIGHT);
	send_cmd(info, DISPLINE, 0);

	// 7. Choose display counter increment direction
	select_chip(info, LEFT);
	send_cmd(info, ADCREVERSE, 0);
	select_chip(info, RIGHT);
	send_cmd(info, ADCSTRAIGHT, 0);

	// 8. Turn the display on
	select_chip(info, LEFT);
	send_cmd(info, DISPON, 0);
	select_chip(info, RIGHT);
	send_cmd(info, DISPON, 0);

	ndelay(TGEN);
}

static void init_structs(struct lcddata *priv)
{
	priv->mtfb_ops.owner		= THIS_MODULE;
	priv->mtfb_ops.fb_read		= fb_sys_read;
	priv->mtfb_ops.fb_write		= mtfb_write;
	priv->mtfb_ops.fb_fillrect	= cfb_fillrect;
	priv->mtfb_ops.fb_copyarea	= cfb_copyarea;
	priv->mtfb_ops.fb_imageblit	= mtfb_imageblit;
	priv->mtfb_ops.fb_mmap		= mtfb_mmap;

	priv->mtfb_defio.delay		= HZ / 4;
	priv->mtfb_defio.deferred_io	= mtfb_defer;

	strcpy(priv->mtfb_fix.id, "mtfb");
	priv->mtfb_fix.type		= FB_TYPE_PACKED_PIXELS;
	priv->mtfb_fix.visual		= FB_VISUAL_TRUECOLOR;
	priv->mtfb_fix.accel		= FB_ACCEL_NONE;
	priv->mtfb_fix.line_length	= DISP_SIZE_X * BPP / 8;

	priv->mtfb_var.height		= DISP_SIZE_Y_MM;
	priv->mtfb_var.width		= DISP_SIZE_X_MM;
	priv->mtfb_var.yres		= DISP_SIZE_Y;
	priv->mtfb_var.yres_virtual	= DISP_SIZE_Y;
	priv->mtfb_var.xres		= DISP_SIZE_X;
	priv->mtfb_var.xres_virtual	= DISP_SIZE_X;
	priv->mtfb_var.activate		= FB_ACTIVATE_FORCE;
	priv->mtfb_var.vmode		= FB_VMODE_NONINTERLACED;
	priv->mtfb_var.red.offset	= 11;
	priv->mtfb_var.red.length	= 5;
	priv->mtfb_var.red.msb_right	= 0;
	priv->mtfb_var.green.offset	= 5;
	priv->mtfb_var.green.length	= 6;
	priv->mtfb_var.green.msb_right	= 0;
	priv->mtfb_var.blue.offset	= 0;
	priv->mtfb_var.blue.length	= 5;
	priv->mtfb_var.blue.msb_right	= 0;
	priv->mtfb_var.bits_per_pixel	= BPP;
};

static int mt_fb_probe(struct platform_device *pdev)
{
	struct lcddata *priv;

	int i;
	int ret, x, y;

#ifdef __BUILD_MTFB_DEBUG
	printk(KERN_INFO "MTFB %s @ %s:%d\n",
		"Probing the Melt MT-12232A LCD driver", __func__, __LINE__);
#endif // __BUILD_MTFB_DEBUG

	priv = devm_kzalloc(&pdev->dev, sizeof(struct lcddata), GFP_KERNEL);
	priv->mem = vmalloc(DISP_SIZE_X * DISP_SIZE_Y * (BPP / 8) * sizeof(u32));

	if (!priv || !priv->mem) {
		pr_err(NAME": Couldn't get memory to store internal data structures");
		goto err_out;
	}

	// {RDWR, A0, DB7, DB6, DB5, DB4, DB3, DB2, DB1, DB0};
	priv->bus[0]	= devm_gpiod_get(&pdev->dev, "rdwr", GPIOD_OUT_LOW);
	priv->bus[1]	= devm_gpiod_get(&pdev->dev, "a0", GPIOD_OUT_LOW);
	for (i = 0; i < DATA_BUS_WIDTH; i++)
		priv->bus[CMD_WIDTH - i - 1] = devm_gpiod_get_index(&pdev->dev,
						"data-bus", i, GPIOD_OUT_LOW);

	for (i = 0; i < CMD_WIDTH; i++) {
		if (IS_ERR(priv->bus[i])) {
			printk(KERN_ERR "MTFB %s %d @ %s:%d\n",
				"Failed at requesting GPIOs at GPIO", i, __func__, __LINE__);
			goto err_out_gpio;
		}
	}

	priv->estrobe	= devm_gpiod_get(&pdev->dev, "estr", GPIOD_OUT_LOW);
	priv->res	= devm_gpiod_get(&pdev->dev, "reset", GPIOD_OUT_LOW);
	priv->cs	= devm_gpiod_get(&pdev->dev, "cs", GPIOD_OUT_LOW);

	if (IS_ERR(priv->res) || IS_ERR(priv->cs) || IS_ERR(priv->estrobe)) {
		printk(KERN_ERR "MTFB %s @ %s:%d\n",
			"Failed at requesting GPIOs", __func__, __LINE__);
		goto err_out_gpio;
	}

#ifdef __BUILD_MTFB_DEBUG
	printk(KERN_INFO "MTFB %s @ %s:%d\n", "Registering a framebuffer",
		__func__, __LINE__);
#endif // __BUILD_MTFB_DEBUG

	priv->info = framebuffer_alloc(sizeof(priv), &pdev->dev);

	if (!priv->info) {
		pr_err(NAME": Couldn't get memory for the framebuffer");
		goto err_out_gpio;
	}

	priv->info->par			= priv;

	lcd_init(priv->info);
	init_structs(priv);

	priv->info->fix			= priv->mtfb_fix;
	priv->info->fix.smem_start	= (unsigned long) priv->mem;
	priv->info->fix.smem_len	= DISP_SIZE_X * DISP_SIZE_Y * BPP / 8;

	priv->info->screen_base		= (char __force __iomem *) priv->mem;
	priv->info->flags		= FBINFO_DEFAULT;

	priv->info->var			= priv->mtfb_var;
	priv->info->fbops		= &priv->mtfb_ops;

	priv->info->fbdefio		= &priv->mtfb_defio;
	fb_deferred_io_init(priv->info);

	priv->info->pseudo_palette	= &priv->mtfb_pseudo_palette;

	platform_set_drvdata(pdev, priv);
	ret = register_framebuffer(priv->info);
	if (ret) {
		printk(KERN_ERR "MTFB %s %d @ %s:%d\n", "Framebuffer registration failed, ret =", ret,
			__func__, __LINE__);
		goto err_out_fb;
	}

#ifdef __BUILD_MTFB_DEBUG
	printk(KERN_INFO "MTFB %s @ %s:%d\n", "Framebuffer registration DONE!",
		__func__, __LINE__);
#endif // __BUILD_MTFB_DEBUG

	if (!of_property_read_bool(pdev->dev.of_node, "no-clear-on-init")) {
		for (x = 0; x < DISP_SIZE_X; x++) {
			for (y = 0; y < DISP_SIZE_Y; y += PAGE_SIZE_Y) {
				set_col(priv->info, x, y, 0x0);
			}
		}
	}

	ret = fb_prepare_logo(priv->info, 0);
	ret = fb_show_logo(priv->info, 0);

	return 0;

err_out_fb:
	if (priv->info) {
		fb_deferred_io_cleanup(priv->info);
		unregister_framebuffer(priv->info);
		framebuffer_release(priv->info);
		printk(KERN_ERR "MTFB %s @ %s:%d\n", "ERROR; Freed FB", __func__, __LINE__);
	}
	vfree(&priv->mem);

err_out_gpio:
	for (i = 0; i < BUS_WIDTH; i++)
		if (priv->bus[i])
			devm_gpiod_put(&pdev->dev, priv->bus[i]);
	if (priv->res)
		devm_gpiod_put(&pdev->dev, priv->res);
	if (priv->cs)
		devm_gpiod_put(&pdev->dev, priv->cs);
	if (priv->estrobe)
		devm_gpiod_put(&pdev->dev, priv->estrobe);

	printk(KERN_ERR "MTFB %s @ %s:%d\n", "ERROR; Freed GPIOs", __func__, __LINE__);

err_out:
	pr_err(NAME": probe failed");
	return (-EBUSY);
}

static int mt_fb_remove(struct platform_device *pdev)
{
	struct lcddata *priv;
	int i;

	priv = platform_get_drvdata(pdev);
#ifdef __BUILD_MTFB_DEBUG
	printk(KERN_INFO "MTFB %s @ %s:%d\n", "Removing!", __func__, __LINE__);
#endif // __BUILD_MTFB_DEBUG

	fb_deferred_io_cleanup(priv->info);
	unregister_framebuffer(priv->info);
	framebuffer_release(priv->info);
	vfree((void *)priv->info->fix.smem_start);
#ifdef __BUILD_MTFB_DEBUG
	printk(KERN_INFO "MTFB %s @ %s:%d\n", "Freed FB", __func__, __LINE__);
#endif // __BUILD_MTFB_DEBUG

	for (i = 0; i < BUS_WIDTH; i++)
		if (priv->bus[i])
			devm_gpiod_put(&pdev->dev, priv->bus[i]);
	if (priv->res)
		devm_gpiod_put(&pdev->dev, priv->res);
	if (priv->cs)
		devm_gpiod_put(&pdev->dev, priv->cs);
	if (priv->estrobe)
		devm_gpiod_put(&pdev->dev, priv->estrobe);

#ifdef __BUILD_MTFB_DEBUG
	printk(KERN_INFO "MTFB %s @ %s:%d\n", "Freed GPIOs", __func__, __LINE__);
#endif // __BUILD_MTFB_DEBUG

	return 0;
}

static const struct of_device_id mt_of_match[] = {
	{ .compatible = "melt,mt-12232a", },
	{},
};
MODULE_DEVICE_TABLE(of, mt_of_match);

static struct platform_driver mt_fb_driver = {
	.remove = mt_fb_remove,
	.probe  = mt_fb_probe,
	.driver = {
		.name   = NAME,
		.of_match_table = of_match_ptr(mt_of_match),
	},
};

static int __init mt_fb_init(void)
{
	int ret;

	ret = platform_driver_register(&mt_fb_driver);
	if (ret)
		printk(KERN_ERR "MTFB %s @ %s:%d\n", "Error registering driver", __func__, __LINE__);

	return ret;
}

static void __exit mt_fb_exit(void)
{
	platform_driver_unregister(&mt_fb_driver);
}

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("Melt LCD framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(mt_fb_init);
module_exit(mt_fb_exit);
