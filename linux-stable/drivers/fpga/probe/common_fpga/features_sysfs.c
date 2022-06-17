/*
 * Based on Linux/samples/kobject/kset-example.c
 *
 * Someone somewhen should review it and fix it to make it cleaner and simpler.
 * I am really not sure about all these custom structure for sysfs_ops and
 * attributes. Maybe we should just rewrite it with device_create_file?
 */

#define pr_fmt(fmt) "%s:%d: " fmt, __FUNCTION__, __LINE__

#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/printk.h>
#include <linux/slab.h>

#include "fpgafeat.h"
#include "features_sysfs.h"
#include "sysfs_attributes.h"

/*
 * The default show function that must be passed to sysfs.  This will be
 * called by sysfs for whenever a show function is called by the user on a
 * sysfs file associated with the kobjects we have registered.  We need to
 * transpose back from a "default" kobject to our custom struct fpga_feature 
 * and then call the show function for that specific object.
 */
static ssize_t feature_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf)
{
	struct feature_attribute *attribute;
	struct fpga_feature *feature;

	attribute = to_feature_attribute(attr);
	feature = to_fpga_feature(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(feature, attribute, buf);
}

/* Our custom sysfs_ops that we will associate with our ktype later on */
static const struct sysfs_ops feature_sysfs_ops = {
	.show = feature_attr_show,
};

/*
 * The release function for our object.  This is REQUIRED by the kernel to
 * have.  We free the memory held in our object here.
 *
 * NEVER try to get away with just a "blank" release function to try to be
 * smarter than the kernel.  Turns out, no one ever is...
 */
static void feature_release(struct kobject *kobj)
{
	struct fpga_feature *feature;

	feature = to_fpga_feature(kobj);
	pr_debug("Freeing feature %s\n", feature->name);
	/* Sorry, Greg, features are static, so we won't free it here */
}

static struct kobj_type feature_ktype = {
	.sysfs_ops = &feature_sysfs_ops,
	.release = feature_release,
	.default_attrs = feature_attrs,
};

static inline void firmware_version_parse(uint8_t result[4], uint16_t raw)
{
	int i;

	for (i = 0; i < 4; ++i)
		result[i] = (raw >> (i << 2)) & 0xf;
}

static ssize_t firmware_info_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stcmtk_common_fpga *grif_fpga = dev_get_drvdata(dev);
	struct fpga_feature_list *flist = grif_fpga->features;
	uint8_t version[4];

	BUG_ON(!flist);

	firmware_version_parse(version, flist->fpga_version);

	return scnprintf(buf, PAGE_SIZE,
                        "%s %hhu.%hhu.%hhu-%hhu 0x%hhx\n", flist->name,
                        version[3], version[2], version[1], version[0],
                        flist->rom_id);
	return 0;
}
DEVICE_ATTR(firmware_info, S_IRUGO, firmware_info_show, NULL);

static int create_feature_sysfs(struct fpga_feature *feature, struct kset *kset)
{
	int rc;

	/* We need to set kset before calling the kobject core */
	feature->kobj.kset = kset;

	/*
	 * Initialize and add the kobject to the kernel. 
	 * As kobject is part of the set, we don't have to set a parent.
	 */
	rc = kobject_init_and_add(&feature->kobj, &feature_ktype, NULL, 
			                  "%s", feature->name);
	if (rc) {
		kobject_put(&feature->kobj);
		return -1;
	}
	return 0;
}

int init_features_sysfs(struct device *dev, struct fpga_feature_list *list)
{
	int i, j;
	int err;

	err = device_create_file(dev, &dev_attr_firmware_info);
	if (err) {
		dev_err(dev, "Failed to device_create_file 'firmware_info', err = %d\n", err);
		BUG();
	}

	/* Initialize kset and add it as subdirectory of device */
	err = -ENOMEM;
	list->kset = kset_create_and_add("features", NULL, &dev->kobj);
	if (!list->kset) {
		dev_err(dev, "Failed to create kset for FPGA features\n");
		goto err_put_kobj;
	}

	/* Populate kset with features kobjects */
	for (i = 0; i < list->count; i++) {
		if (create_feature_sysfs(&(list->features_arr[i]), list->kset)) {
			dev_err(dev, "Failed to create sysfs entry for %s feature\n", 
					list->features_arr[i].name);
			goto err;
		}
	}

	return 0;

err:
	for (j = 0; j < i; j++)
		kobject_put(&(list->features_arr[j].kobj));

	kset_unregister(list->kset);
err_put_kobj:
	device_remove_file(dev, &dev_attr_firmware_info);
	return err;
}

void deinit_features_sysfs(struct device *dev, struct fpga_feature_list *list)
{
	int i;

	for (i = 0; i < list->count; i++) {
		kobject_put(&(list->features_arr[i].kobj));
	}

	kset_unregister(list->kset);
	device_remove_file(dev, &dev_attr_firmware_info);
}
