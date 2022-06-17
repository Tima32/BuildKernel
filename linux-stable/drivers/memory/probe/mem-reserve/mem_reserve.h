#define EOF          0
#define DEVICE_NAME  "mem_reserve_dev"
#define DEVICE_CLASS "mem_reserve_class"

struct mem_reserve_drv {
	const char             *dev_name;
	char                   *class_name;
	int                    major_num;
	int                    open_cnt;
	struct device          *dev;
	unsigned char          *vmem;
	dma_addr_t             dma_addr;
	struct cdev            c_dev;
	u32                    port_num;
	u32                    mem_size;
	struct platform_device *pdev;
};
