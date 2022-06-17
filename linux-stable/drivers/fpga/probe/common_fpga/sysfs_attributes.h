#ifndef __SYSFS_ATTRIBUTES_H
#define __SYSFS_ATTRIBUTES_H

/* a custom attribute that works just for a struct fpga_feature. */
struct feature_attribute {
	struct attribute attr;
	ssize_t (*show)(struct fpga_feature *feature, struct feature_attribute *attr, char *buf);
	ssize_t (*store)(struct fpga_feature *feature, struct feature_attribute *attr, const char *buf, size_t count);
};
#define to_feature_attribute(x) container_of(x, struct feature_attribute, attr)

static ssize_t type_show(struct fpga_feature *feature, 
		struct feature_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", feature->type);
}

static ssize_t name_show(struct fpga_feature *feature, 
		struct feature_attribute *attr, char *buf)
{
	return snprintf(buf, FEATURE_NAME_MAX, "%s\n", feature->name);
}

static ssize_t port_mask_show(struct fpga_feature *feature, 
		struct feature_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%08X\n", feature->port_mask);
}

static ssize_t cr_base_show(struct fpga_feature *feature, 
		struct feature_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", feature->cr_base);
}

static ssize_t cr_cnt_show(struct fpga_feature *feature, 
		struct feature_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", feature->cr_cnt);
}

static ssize_t sr_base_show(struct fpga_feature *feature, 
		struct feature_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", feature->sr_base);
}

static ssize_t sr_cnt_show(struct fpga_feature *feature, 
		struct feature_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", feature->sr_cnt);
}

static ssize_t version_show(struct fpga_feature *feature,
		struct feature_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", feature->version);
}

static struct feature_attribute type_attribute = __ATTR_RO(type);
static struct feature_attribute name_attribute = __ATTR_RO(name);
static struct feature_attribute port_mask_attribute = __ATTR_RO(port_mask);
static struct feature_attribute cr_base_attribute = __ATTR_RO(cr_base);
static struct feature_attribute cr_cnt_attribute = __ATTR_RO(cr_cnt);
static struct feature_attribute sr_base_attribute = __ATTR_RO(sr_base);
static struct feature_attribute sr_cnt_attribute = __ATTR_RO(sr_cnt);
static struct feature_attribute version_attribute = __ATTR_RO(version);

static struct attribute *feature_attrs[] = {
	&type_attribute.attr,
	&name_attribute.attr,
	&port_mask_attribute.attr,
	&cr_base_attribute.attr,
	&cr_cnt_attribute.attr,
	&sr_base_attribute.attr,
	&sr_cnt_attribute.attr,
	&version_attribute.attr,
	NULL,
};

#endif
