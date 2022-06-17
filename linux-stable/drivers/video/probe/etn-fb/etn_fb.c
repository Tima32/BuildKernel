#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>

#include <linux/regmap.h>

#include <asm/page.h>

#include <etn_fpga_mgr/etn_fpga.h>
#include <common_fpga/fpgafeat.h>

#include "etn_fb.h"

struct lcd_regs {
    int cr;
    int sr;
};

struct et_lcd_fb {
    struct stcmtk_common_fpga	*fpga;
    struct fpga_feature		*lcd;
    struct platform_device	*pdev;
};

static int fps = 25;
static bool use_fpga2sdram = 1;

module_param(fps, int, S_IRUGO);
module_param(use_fpga2sdram, bool, S_IRUGO);

static u32 etn_fb_pseudo_palette[16];
static struct lcd_regs lcd_regs;

static void fpga_write_cr(struct et_lcd_fb *fbdev, int reg, u16 val)
{
	struct stcmtk_common_fpga *fpga = fbdev->fpga;
	regmap_write(fpga->regmap, lcd_regs.cr + reg, val);
}

static u16 fpga_read_cr(struct et_lcd_fb *fbdev, int reg)
{
	unsigned tmp;
	struct stcmtk_common_fpga *fpga = fbdev->fpga;

	regmap_read(fpga->regmap, lcd_regs.cr + reg, &tmp);
	return (u16) tmp;
}

static void fpga_set_cr_bit(struct et_lcd_fb *fbdev, int reg, int bit)
{
	unsigned long tmp = fpga_read_cr(fbdev, reg);

	set_bit(bit, &tmp);
	fpga_write_cr(fbdev, reg, tmp);
}

static void lcd_write_command(struct et_lcd_fb *fbdev, u16 val)
{
	/* Write command code */
	fpga_write_cr(fbdev, LCD_DATA_CR, val);

	/* WR and RS low, RD high */
	fpga_write_cr(fbdev, LCD_CTRL_CR, LCD_CTRL_CR_RD);
	ndelay(20);

	/* RS low, WR and RD high */
	fpga_write_cr(fbdev, LCD_CTRL_CR, LCD_CTRL_CR_RD | LCD_CTRL_CR_WR);
	ndelay(20);

	/* All control signals high */
	fpga_write_cr(fbdev, LCD_CTRL_CR, LCD_CTRL_CR_RD | LCD_CTRL_CR_WR |
                       LCD_CTRL_CR_RS);

	ndelay(20);
	fpga_write_cr(fbdev, LCD_DATA_CR, 0x00);
	fpga_write_cr(fbdev, LCD_CTRL_CR, 0x00);
}

static void lcd_write_data(struct et_lcd_fb *fbdev, u16 data)
{
	/* Write data */
	fpga_write_cr(fbdev, LCD_DATA_CR, data);
	ndelay(20);

	/* WR low, RD and RS high */
	fpga_write_cr(fbdev, LCD_CTRL_CR, LCD_CTRL_CR_RD | LCD_CTRL_CR_RS);
	ndelay(20);

	/* All control signals high */
	fpga_write_cr(fbdev, LCD_CTRL_CR, LCD_CTRL_CR_RD |
		       LCD_CTRL_CR_RS | LCD_CTRL_CR_WR);
}

/* See st7789vi Datasheet */
static void lcd_init(struct et_lcd_fb *fbdev)
{
	/* Clear data */
	fpga_write_cr(fbdev, LCD_DATA_CR, 0);

	/* All control signals high */
	fpga_write_cr(fbdev, LCD_CTRL_CR, LCD_CTRL_CR_RD |
		       LCD_CTRL_CR_RS | LCD_CTRL_CR_WR);
	mdelay(100);

	lcd_write_command(fbdev, 0x1);	// sw reset
	mdelay(200);

	lcd_write_command(fbdev, ILI9341_SLEEP_OUT);
	mdelay(200);

	lcd_write_command(fbdev, ILI9341_MEM_ACCESS_CTRL);
	lcd_write_data(fbdev, MY | MV);

	lcd_write_command(fbdev, ILI9341_PIXEL_FORMAT);
	lcd_write_data(fbdev, 0x0055);

	lcd_write_command(fbdev, ILI9341_COLUMN_ADDR);
	lcd_write_data(fbdev, 0x0000);
	lcd_write_data(fbdev, 0x0000);
	lcd_write_data(fbdev, (DISPLAY_WIDTH-1) >> 8);
	lcd_write_data(fbdev, (DISPLAY_WIDTH-1) & 0xFF);

	lcd_write_command(fbdev, ILI9341_PAGE_ADDR);
	lcd_write_data(fbdev, 0x0000);
	lcd_write_data(fbdev, 0x0000);
	lcd_write_data(fbdev, (DISPLAY_HEIGHT-1) >> 8);
	lcd_write_data(fbdev, (DISPLAY_HEIGHT-1) & 0xFF);

	lcd_write_command(fbdev, ILI9341_DISPLAY_ON);

	lcd_write_command(fbdev, ILI9341_MEM_WRITE);
}

#define CNVT_TOHW(val, width) ((((val)<<(width))+0x7FFF-(val))>>16)

static int etn_fb_setcolreg(unsigned regno,
			    unsigned red, unsigned green, unsigned blue,
			    unsigned transp, struct fb_info *info)
{
	int ret = 1;

	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				      7471 * blue) >> 16;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;
			u32 value;

			red = CNVT_TOHW(red, info->var.red.length);
			green = CNVT_TOHW(green, info->var.green.length);
			blue = CNVT_TOHW(blue, info->var.blue.length);
			transp = CNVT_TOHW(transp, info->var.transp.length);

			value = (red << info->var.red.offset) |
				(green << info->var.green.offset) |
				(blue << info->var.blue.offset) |
				(transp << info->var.transp.offset);

			pal[regno] = value;
			ret = 0;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

static struct fb_fix_screeninfo etn_fb_fix = {
	.id             = NAME,
	.type           = FB_TYPE_PACKED_PIXELS,
	.visual         = FB_VISUAL_TRUECOLOR,
	.accel          = FB_ACCEL_NONE,
	.line_length    = DISPLAY_WIDTH * DISPLAY_BPP / 8,
};

static struct fb_var_screeninfo etn_fb_var = {
	.width          = DISPLAY_WIDTH,
	.height         = DISPLAY_HEIGHT,
	.bits_per_pixel = DISPLAY_BPP,
	.xres           = DISPLAY_WIDTH,
	.yres           = DISPLAY_HEIGHT,
	.xres_virtual   = DISPLAY_WIDTH,
	.yres_virtual   = DISPLAY_HEIGHT,
	.activate       = FB_ACTIVATE_FORCE,
	.vmode          = FB_VMODE_NONINTERLACED,
	.red            = {11, 5, 0},
	.green          = {5, 6, 0},
	.blue           = {0, 5, 0},
};

static int etn_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return vm_iomap_memory(vma, info->fix.smem_start, info->fix.smem_len);
}

static struct fb_ops etn_fb_ops = {
	.owner          = THIS_MODULE,
	.fb_read        = fb_sys_read,
	.fb_write       = fb_sys_write,
	.fb_fillrect    = sys_fillrect,
	.fb_copyarea    = sys_copyarea,
	.fb_imageblit   = sys_imageblit,
	.fb_setcolreg   = etn_fb_setcolreg,
	.fb_mmap        = etn_fb_mmap,
};

static u64 platform_dma_mask = DMA_BIT_MASK(32);

static void set_fps(struct et_lcd_fb *fbdev, int fps)
{
	unsigned int fps_delay;

	if (fps < MIN_FPS)
		fps = MIN_FPS;
	if (fps > MAX_FPS)
		fps = MAX_FPS;

	/* We set delay to FPGA in ms */
	fps_delay = 1000 / fps;

	fpga_write_cr(fbdev, LCD_FPS_DELAY_CR, fps_delay);
}

static void set_dma_addr(struct et_lcd_fb *fbdev, dma_addr_t dma_addr)
{
	/* fpga2sdram interface has word address,
	 * but fpga2hps has byte address. */
	if (use_fpga2sdram)
		dma_addr = dma_addr / 8;

	/* Write address into FPGA-DMA */
	fpga_write_cr(fbdev, LCD_DMA_ADDR_CR0, dma_addr & 0xFFFF);
	fpga_write_cr(fbdev, LCD_DMA_ADDR_CR1, dma_addr >> 16);
}

static int etn_fb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct et_lcd_fb *fbdev;
	int ret;
	const __be32 *prop = NULL;
	u32 vmem_size;
	unsigned char *vmem;
	dma_addr_t dma_addr;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;

	fbdev = devm_kzalloc(&pdev->dev, sizeof(*fbdev), GFP_KERNEL);
	if (!fbdev)
		return -ENOMEM;

	fbdev->pdev = pdev;

	fbdev->fpga = stcmtk_get_fpga(np);
	if (!fbdev->fpga) {
		dev_err(dev, "Failed to get target FPGA\n");

		return -ENODEV;
	}

	ret = etn_fpga_ref_get(fbdev->fpga);
	if (ret) {
		dev_err(dev, "Failed to increment FPGA reference count\n");
		goto err_ref;
	}

	fbdev->lcd = stcmtk_fpga_get_feature(fbdev->fpga, FPGA_FEATURE_LCD);
	if (!fbdev->lcd) {
		dev_err(dev, "Failed to get FPGA_FEATURE_LCD\n");
		goto err_feat;
	}

	lcd_regs.cr = stcmtk_get_cr_base_on_port(fbdev->lcd, 0);
	lcd_regs.sr = stcmtk_get_sr_base_on_port(fbdev->lcd, 0);

	pdev->dev.dma_mask = &platform_dma_mask;
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	vmem_size = (etn_fb_var.width * etn_fb_var.height * etn_fb_var.bits_per_pixel) / 8;

	vmem = dmam_alloc_coherent(&pdev->dev, vmem_size, &dma_addr, GFP_KERNEL);
	if (!vmem) {
		dev_err(&pdev->dev, "FB: dma_alloc_coherent error\n");
		goto err_put;
	}

	memset(vmem, 0, vmem_size);

	info = framebuffer_alloc(0, &pdev->dev);
	if (!info)
		goto err_put;

	info->screen_base = vmem;
	info->fbops = &etn_fb_ops;
	info->fix = etn_fb_fix;
	info->fix.smem_start = dma_addr;
	info->fix.smem_len = vmem_size;
	info->var = etn_fb_var;
	info->flags = FBINFO_DEFAULT;
	info->pseudo_palette = &etn_fb_pseudo_palette;

	/* Disable refreshing */
	fpga_write_cr(fbdev, LCD_DMA_CR, 0);

	/* Get fps from dtb */
	prop = of_get_property(pdev->dev.of_node, "etn-fb,fps", NULL);
	if (prop) {
		fps = be32_to_cpup(prop);
	}

	/* Get use_fpga2sdram from dtb */
	use_fpga2sdram = of_property_read_bool(pdev->dev.of_node, "etn-fb,use_fpga2sdram");

	lcd_init(fbdev);

	set_dma_addr(fbdev, dma_addr);

	set_fps(fbdev, fps);

	/* Enable refreshing */
	fpga_set_cr_bit(fbdev, LCD_DMA_CR, LCD_DMA_CR_REDRAW_EN);
	dev_info(&pdev->dev, "DMA addr 0x%x\n", dma_addr);

	ret = register_framebuffer(info);
	if (ret < 0) {
		framebuffer_release(info);
		goto err_put;
	}

	platform_set_drvdata(pdev, info);

	return 0;

err_put:
err_feat:
	etn_fpga_ref_put(fbdev->fpga);
err_ref:
	return -EFAULT;
}

static int etn_fb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct et_lcd_fb *fbdev = platform_get_drvdata(pdev);

	/* Disable refreshing */
	fpga_write_cr(fbdev, LCD_DMA_CR, 0);

	if (info) {
		unregister_framebuffer(info);
		framebuffer_release(info);
	}
	if (fbdev) {
		etn_fpga_ref_put(fbdev->fpga);
	}

	return 0;
}

static const struct of_device_id etn_of_match[] = {
	{ .compatible = "etn,fb", },
	{},
};

MODULE_DEVICE_TABLE(of, etn_of_match);

static struct platform_driver etn_fb_driver = {
	.remove = etn_fb_remove,
	.driver = {
		.name   = NAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(etn_of_match),
	},
};

static int __init etn_fb_init(void)
{
	if (platform_driver_probe(&etn_fb_driver, etn_fb_probe)) {
		printk(KERN_ERR "Failed to probe ETN platform driver\n");
		return -ENXIO;
	}
	return 0;
}

static void __exit etn_fb_exit(void)
{
	platform_driver_unregister(&etn_fb_driver);
}

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("ETN LCD framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(etn_fb_init);
module_exit(etn_fb_exit);

