#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/of_address.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <asm/page.h>
#include <asm/atomic.h>
#include "mem_reserve.h"

/* Counter dev at this moment.
 * Used to prevent the creation of extra
 * device classes.
**/
static atomic_t dev_cnt = ATOMIC_INIT(0);

/* Common class for all char device */
static struct class *dev_class;

/* Show in sys_fs start physical address memory of file. */
static ssize_t phys_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev ); 
	struct mem_reserve_drv *mrd = platform_get_drvdata( pdev );
	return scnprintf(buf, PAGE_SIZE, "0x%0lX\n", mrd->dma_addr);
}

static DEVICE_ATTR( phys_addr, S_IRUSR, phys_addr_show, NULL );

/* Set pointer on srtruct file to driver information.
   Increment open file count.
 */
static int dev_open(struct inode *inodep, struct file *filep)
{
	struct mem_reserve_drv *mrd = container_of(inodep->i_cdev, struct mem_reserve_drv, c_dev);
	filep->private_data = mrd;
	mrd->open_cnt++;
	return 0;
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	struct mem_reserve_drv *mrd = filep->private_data;
	struct platform_device *pdev = mrd->pdev;

	if( *offset >= mrd->mem_size )
		return EOF;

	if( *offset + len > mrd->mem_size )
		len = mrd->mem_size - *offset;

	if(copy_from_user(mrd->vmem + *offset, buffer, len ) != 0 ){
		dev_err( &pdev->dev, "Copy from user faild" );
		return -EFAULT;
	}

	*offset = *offset + len;
	return len;
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	struct mem_reserve_drv *mrd = filep->private_data;
	struct platform_device *pdev = mrd->pdev;

	if( *offset >= mrd->mem_size )
		return EOF;

	if( *offset + len > mrd->mem_size )
		len = mrd->mem_size - *offset;

	if(copy_to_user(buffer, mrd->vmem + *offset, len ) != 0 ){
		dev_err( &pdev->dev, "Copy to user faild" );
		return -EFAULT;
	}

	*offset = *offset + len;
	return len;
}

static loff_t dev_lseek(struct file *filep, loff_t offset, int orig )
{
	struct mem_reserve_drv *mrd = filep->private_data;
	loff_t new_pos = 0;

	switch( orig ){
		case SEEK_SET:
			new_pos = offset;
			break;
		case SEEK_CUR:
			new_pos = filep->f_pos + offset;
			break;
		case SEEK_END:
			new_pos = mrd->mem_size - offset;
			break;
	}

	if( new_pos > mrd->mem_size )
		 new_pos = mrd->mem_size;
	if( new_pos < 0 )
		 new_pos = 0;
	filep->f_pos = new_pos;
	return new_pos;
}


static int dev_mmap(struct file *filep, struct vm_area_struct *vma)
{
	int ret = 0;
	struct mem_reserve_drv *mrd = filep->private_data;
	struct platform_device *pdev = mrd->pdev;
	unsigned long off = vma->vm_pgoff;
	unsigned long user_count = vma->vm_end - vma->vm_start;
	unsigned long pfn = PFN_DOWN(mrd->dma_addr) + off;

	if( vma->vm_pgoff + vma_pages(vma) > ( mrd->mem_size )){
		dev_err( &pdev->dev, "Offset more memory size" );
		return -EINVAL;
	}
	if( !(filep->f_flags & O_SYNC) ){
		dev_err( &pdev->dev, "For mmap flags must be O_SYNC" );
		return -EINVAL;
	}

	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached( vma->vm_page_prot );

	ret = remap_pfn_range( vma, vma->vm_start, pfn, user_count, vma->vm_page_prot);
	if( ret < 0 ){
		dev_err( &pdev->dev, "Error, remap_pfn_range" );
	}
	return ret;
}


/* Decrement open file count. */
static int dev_release(struct inode *inodep, struct file *filep)
{
	struct mem_reserve_drv *mrd = filep->private_data;
	mrd->open_cnt--;
	return 0;
}

static struct file_operations fops =
{
	.owner   = THIS_MODULE,
	.open    = dev_open,
	.read    = dev_read,
	.write   = dev_write,
	.release = dev_release,
	.llseek  = dev_lseek,
	.mmap    = dev_mmap,
};

static const struct of_device_id mem_reserve_of_match[] = {
	{ .compatible = "stcmtk,mem_reserve", },
	{},
};

MODULE_DEVICE_TABLE(of, mem_reserve_of_match);

/**
 * mem_reserve_probe() - probe for mem_reserve module
 *
 * This function parse device-tree, reserve memory region,
 * create char device, allocate memory.
 */
static int mem_reserve_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct mem_reserve_drv *mrd = devm_kzalloc( &pdev->dev, sizeof( struct mem_reserve_drv ), GFP_KERNEL );
	struct device_node *dev_node = NULL;
	struct resource resource;

	if( IS_ERR(mrd) ){
		dev_err(&pdev->dev, "Can't alloc mem for mem_reserve_drv struct.\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, mrd);
	mrd->pdev = pdev;

	retval = of_property_read_string(pdev->dev.of_node, "dev-name", &mrd->dev_name);
	if( retval < 0 ){
		dev_err(&pdev->dev, "Faild read property dev-name\n");
		return retval;
	}

	retval = of_property_read_u32(pdev->dev.of_node, "mem-size", &mrd->mem_size);
	if( retval < 0 ){
		dev_err(&pdev->dev, "Failed to read mem-size\n");
		return retval;
	}

/* Get reserved memory region from Device-tree */
	dev_node = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if( IS_ERR(dev_node) ) {
		dev_err(&pdev->dev, "No memory-region specified\n");
		return -EINVAL;
	}
  
	retval = of_address_to_resource(dev_node, 0, &resource);
	if(retval < 0){
		dev_err(&pdev->dev, "No memory address assigned to the region\n");
		return retval;
	}

	mrd->dma_addr = resource.start;
	mrd->vmem = devm_ioremap_resource(&pdev->dev, &resource);
	if( IS_ERR(mrd->vmem) ){
		dev_err(&pdev->dev, "IOremap memory-region error.\n");
		return -ENOMEM;
	}

	mrd->major_num = register_chrdev(0, mrd->dev_name, &fops);
	if( mrd->major_num < 0 ){
		dev_err(&pdev->dev, "Failed to register a major number\n");
		retval = mrd->major_num;
		goto unmap_mem;
	}

	cdev_init(&mrd->c_dev, &fops);

	if( atomic_inc_return(&dev_cnt) == 1 ){
		dev_class = class_create(THIS_MODULE, DEVICE_CLASS);
		if (IS_ERR(dev_class)){
			dev_err(&pdev->dev, "Failed to creat class\n");
			retval = PTR_ERR(dev_class);
				goto unreg_dev;
		}
	}

	mrd->dev = device_create( dev_class, NULL, MKDEV(mrd->major_num, mrd->port_num), NULL, mrd->dev_name);
	if(IS_ERR(mrd->dev)){
		dev_err(&pdev->dev, "Failed to create the device\n");
		retval = PTR_ERR(mrd->dev);
		goto class_dstr;
	}

	retval = cdev_add(&mrd->c_dev, MKDEV(mrd->major_num, mrd->port_num ), 1);
	if( retval < 0 ){
		goto unreg_dev;
	}

	dev_notice(&pdev->dev, "phys addr: 0x%0lX\n", mrd->dma_addr );

	retval = device_create_file(&pdev->dev, &dev_attr_phys_addr);
	if(retval){
		dev_err(&pdev->dev, "Failed to create file phys_addr" );
		goto dev_dstr;
	}

	dev_notice(&pdev->dev, "Device created correctly\n" );

	return 0;

dev_dstr:
	device_destroy(dev_class, MKDEV(mrd->major_num, mrd->port_num));
class_dstr:
	if( atomic_dec_return(&dev_cnt) == 0 ){
		class_destroy(dev_class);
	}
unreg_dev:
	unregister_chrdev( mrd->major_num, mrd->dev_name );
unmap_mem:
	devm_iounmap(&pdev->dev, mrd->vmem);
	return retval;
}

static int mem_reserve_remove( struct platform_device *pdev )
{
	struct mem_reserve_drv *mrd;
	mrd = platform_get_drvdata( pdev );
	if( mrd->open_cnt == 0 ){
		device_destroy( dev_class, MKDEV(mrd->major_num, mrd->port_num) );
		devm_iounmap(&pdev->dev, mrd->vmem);

		if( atomic_dec_return(&dev_cnt) == 0 ){
			class_destroy(dev_class);
		}

		unregister_chrdev(mrd->major_num, mrd->dev_name);
		cdev_del( &mrd->c_dev );
		device_remove_file(&pdev->dev, &dev_attr_phys_addr);

		dev_notice(&pdev->dev, "Remove device\n");
	} else {
		dev_err(&pdev->dev, "Device can't remove. Device is busy!\n");
		return -EBUSY;
	}
	return 0;
}

static struct platform_driver mem_reserve_driver = {
	.remove = mem_reserve_remove,
	.probe  = mem_reserve_probe,
	.driver = {
		.name   = DEVICE_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(mem_reserve_of_match),
	},
};

module_platform_driver( mem_reserve_driver );

MODULE_LICENSE("GPL");
MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("Driver for reserved memory.");
MODULE_VERSION("0.0.1");
