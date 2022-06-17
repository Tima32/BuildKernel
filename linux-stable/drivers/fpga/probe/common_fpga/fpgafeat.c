#define pr_fmt(fmt) "%s:%d: " fmt, __FUNCTION__, __LINE__

#include <linux/slab.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <asm-generic/bug.h>
#include <linux/regmap.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "fpgafeat.h"
#include "features_sysfs.h"

/* TO DO: добавить счетчик выделений структуры */
/* class for /sys/class/fpga */
static struct class *fpga_class = NULL;

DECLARE_COMPLETION(fpga_init_done);

void finish_completion()
{
	complete_all(&fpga_init_done);
}
EXPORT_SYMBOL(finish_completion);

struct device* stcmtk_device_create(struct device *dev,
				    dev_t devt,
				    struct stcmtk_common_fpga *fpga,
				    const char *str,
				    int id)
{
	return device_create(fpga_class, dev, devt, fpga, str, id);
}
EXPORT_SYMBOL(stcmtk_device_create);

struct device* stcmtk_class_find_device(struct device *start,
					struct device_node *node,
					int (*match)(struct device *, const void *))
{
	return class_find_device(fpga_class, start, node, match);
}
EXPORT_SYMBOL(stcmtk_class_find_device);

void stcmtk_class_destroy()
{
	class_destroy(fpga_class);
	fpga_class = NULL;
}
EXPORT_SYMBOL(stcmtk_class_destroy);

void stcmtk_device_destroy(dev_t devt)
{
	device_destroy(fpga_class, devt);
}
EXPORT_SYMBOL(stcmtk_device_destroy);

int stcmtk_class_create(struct module *owner, const char *name, const struct attribute_group *fpga_groups[])
{
	/* create if class does not exist */
	if(fpga_class == NULL) {
		fpga_class = class_create(owner, name);
		if (IS_ERR(fpga_class)) {
			return PTR_ERR(fpga_class);
		}

		if (fpga_groups)
			fpga_class->dev_groups = fpga_groups;
	}

	return 0;
}
EXPORT_SYMBOL(stcmtk_class_create);


/**
 * Add a feature to the struct fpga_feature.
 *
 * Function reallocates memory of features list, so user should call
 * free_features_list function to avoid memory leaks.
 *
 * \param list		 Pointer to features list.
 * \param feature Pointer to feature for adding.
 *
 * \return 0 if feature adding is successful, negative integer otherwise.
 */
static int add_feature_to_list(struct fpga_feature_list *list, struct fpga_feature *feature)
{
	void *tmp;
	void *ptr = (void *)list->features_arr;

	struct fpga_feature *feat = stcmtk_get_feature_by_name(feature->name, list);

	if (feat == NULL) {
		tmp = krealloc(ptr, sizeof(*feature) * (list->count + 1), GFP_KERNEL);
		if (tmp == NULL) {
			pr_err("Can't allocate memory to add feature\n");
			return -1;
		}
		ptr = tmp;

		list->features_arr = (struct fpga_feature *)ptr;
		memcpy(&list->features_arr[list->count], feature, sizeof(*feature));
		list->count++;
	} else {
		memcpy(feat, feature, sizeof(*feature));
	}

	return 0;
}

/**
 * Function frees memory allocated to feature list.
 *
 * \param list Pointer to struct fpga_feature variable to free.
 */
static void free_features_list(struct fpga_feature_list *list)
{
	list->count = 0;
	if (list->features_arr != NULL) {
		kfree(list->features_arr);
		list->features_arr = NULL;
	}
}

/**
 * Function reads a 4-byte word from memory contains feature info.
 *
 * \param regmap Pointer to struct regmap gives access to fpga registers.
 * \param addr Memory byte address to read from.
 * \param data Pointer to store read data.
 *
 * \return 0 if there are no errors, negative integer otherwise.
 */
static int read_features_memory_word(struct regmap *regmap, uint16_t word,
								uint32_t *data)
{
	int ret = 0;
	unsigned int tmp0 = 0, tmp1 = 0;

/* Control register with features */
#define FPGA_FEAT_WORD_LO_CR 0
#define FPGA_FEAT_WORD_HI_CR 1
#define FPGA_ID_CR 2

	/* FPGA features lies in memory with indirect address mode.
	 * To access it, we store word address in FPGA_ID_CR on H2F bus.
	 * And then read data from first two registers.*/
        /* FIXME: for EIM ONLY */
	// ret = regmap_write(regmap, TO_BYTE_ADDR(FPGA_ID_CR), word & 0xFF);
       
	ret = regmap_write(regmap, FPGA_ID_CR, word & 0xFF);
       
	if (ret) {
		pr_err("Failed to write feature ID word: %d, err %d\n", word, ret);
		return ret;
	}

	// ret = regmap_read(regmap, TO_BYTE_ADDR(FPGA_FEAT_WORD_LO_CR), &tmp0);
	ret = regmap_read(regmap, FPGA_FEAT_WORD_LO_CR, &tmp0);
	if (ret) {
		pr_err("Failed to read low feature %d part, err %d\n", word, ret);
		return ret;
	}

	// ret = regmap_read(regmap, TO_BYTE_ADDR(FPGA_FEAT_WORD_HI_CR), &tmp1);
	ret = regmap_read(regmap, FPGA_FEAT_WORD_HI_CR, &tmp1);
	if (ret) {
		pr_err("Failed to read high feature %d part, err %d\n", word, ret);
		return ret;
	}

	*data = (tmp1 << 16) | tmp0;

	return 0;
}

/**
 * Function reads a 2-byte word from memory contains feature version.
 *
 * \param feature Pointer to struct fpga_feature variable to store parsed data.
 * \param regmap Pointer to struct regmap gives access to fpga registers.
 *
 * \return 0 if there are no errors, negative integer otherwise.
 */
static int read_feature_version(struct fpga_feature *feature, struct regmap *regmap)
{
	int value;
	// int ret = regmap_read(regmap, TO_BYTE_ADDR(feature->sr_base), &value);
	int ret = regmap_read(regmap, feature->sr_base, &value);
	if (ret) {
		pr_err("Failed to read feature \"%s\" version\n", feature->name);
		return ret;
	}

	if (value == 0x00 || value == 0xFF) {
		pr_warn("%s: bad feature version: %d\n", feature->name, value);
		value = 0x00;
	}

	feature->version = value;

	return 0;
}

/**
 * Read features from FPGA.
 *
 * \param buf Pointer to buffer to store read date. It must have size at least
 * FEATURE_MEM_WORDS 4-bytes word.
 *
 * \return 0 if there are no errors, negative integer otherwise.
 */
static int read_features(uint32_t *buffer, struct regmap *regmap)
{
	uint16_t i;
	int err = 0;

	for (i = 0; i < FEATURE_MEM_WORDS; i++) {
		err = read_features_memory_word(regmap, i, &buffer[i]);
		if (err) {
			pr_err("Can't read features info from FPGA\n");
			goto exit;
		}
	}

exit:
	return err;
}

/**
 * Function parses an FPGA feature and form struct fpga_feature
 * structure that describes it.
 *
 * \param buffer  Input buffer to parse.
 * \param idx     Feature index.
 * \param feature Pointer to struct fpga_feature variable to store parsed data.
 *
 * \return 0 if there are no errors, negative integer otherwise.
 */
static int parse_feature(const uint32_t *buffer, unsigned int idx,
                         struct fpga_feature *feature)
{
	int i;
	char *tmp;
	const uint32_t *ptr = buffer;
	uint8_t individual_bit = *ptr & 0x1;
	uint8_t feat_width = (*ptr >> 24) & 0xff;
	ptr++;

	pr_debug("Feature %d: I_bit %d, length %d\n", idx, individual_bit, feat_width);

	feature->type = idx;

	tmp = (char *)ptr;

	for (i = 0; i < FEATURE_NAME_MAX; i++) {
		feature->name[i] = tmp[(FEATURE_NAME_MAX - 1) - i];
	}
	feature->name[FEATURE_NAME_MAX] = '\0';
	ptr += FEATURE_NAME_MAX / FEATURE_MEM_WORD_LEN;

	pr_debug("Feature %d: name %s\n", idx, feature->name);

	feature->sr_base = *ptr & 0xffff;
	feature->cr_base = (*ptr >> 16) & 0xffff;
	ptr++;

	feature->sr_cnt = *ptr & 0xffff;
	feature->cr_cnt = (*ptr >> 16) & 0xffff;
	ptr++;

	pr_debug("Feature %d: cr_base %d, cr_cnt %d\n", idx,
			feature->cr_base, feature->cr_cnt);
	pr_debug("Feature %d: sr_base %d, sr_cnt %d\n", idx,
			feature->sr_base, feature->sr_cnt);

	if (individual_bit) {
		feature->port_mask = *ptr;
		ptr++;
	}

	pr_debug("Feature %d: port mask 0x%08x\n", idx, feature->port_mask);

	return 0;
}

/**
 * Function calculates offset to feature descriptor for a feature.
 *
 * \param buffer Pointer to memory buffer. It doesn't point to whole memory
 * start but to feature offset base address.
 *
 * \param idx Feature index.
 *
 * \return offset address from in words from the beginning of ROM memory.
 */
static uint16_t get_feat_romid_addr(const uint32_t *buffer, unsigned int idx)
{
	uint16_t addr;
	unsigned int offset = idx / 2;
	if (idx % 2) {
		addr = (buffer[offset] >> 16) & 0xffff;
	} else {
		addr = buffer[offset] & 0xffff;
	}

	pr_debug("Feature %d address 0x%04x\n", idx, addr);

	return addr;
}

/**
 * Parse FPGA features from FPGA memory buffer into struct fpga_feature_list.
 *
 * \param buffer Input buffer to parse.
 * \param list   Pointer to fpga_feature_list struct to store parsed data.
 * \param regmap Pointer to struct regmap gives access to fpga registers.
 *
 * \return 0 if there are no errors, negative integer otherwise.
 */
static int parse_features(const uint32_t *buffer, struct fpga_feature_list *list, struct regmap *regmap)
{
	int err;
	uint8_t hdr_len;
	unsigned int feature_idx;
	uint64_t feat_bit_mask;
	uint16_t feat_romid_addr;
	const uint32_t *feat_base;
	const uint32_t *ptr = buffer;
	struct fpga_feature feature;
	const char *name;
	int i;

	list->fpga_version = *ptr & 0xffff;
	list->rom_id       = (*ptr >> 16) & 0xff;
	hdr_len	           = (*ptr >> 24) & 0xff;
	ptr++;

	name = ((const char *)buffer) + FEATURE_FIRMWARE_INFO_OFFSET *
							FEATURE_MEM_WORD_LEN;
	for (i = 0; i < FIRMWARE_NAME_MAX; ++i){
		list->name[i] = name[i];
	}
	list->name[FIRMWARE_NAME_MAX - 1] = 0;

	pr_info("FPGA version: 0x%04x\n", list->fpga_version);
	pr_info("ROM ID version: 0x%04x\n", list->rom_id);
	pr_info("Header length: %d\n", hdr_len);

	feat_bit_mask = ((uint64_t)*(ptr + 1) << 32) | *ptr;
	pr_debug("Features bitmask 0x%08llx\n", feat_bit_mask);
	ptr += 2;

	err = 0;
	feature_idx = 0;
	while (feat_bit_mask) {
		if ((feat_bit_mask & 0x1)) {
			feat_romid_addr = get_feat_romid_addr(ptr, feature_idx);
			feat_base = &buffer[feat_romid_addr];

			memset(&feature, 0, sizeof(feature));

			err = parse_feature(feat_base, feature_idx, &feature);
			if (err) {
				pr_err("Error during feature %d parsing\n", feature_idx);
				return -1;
			}

			err = read_feature_version(&feature, regmap);
			if (err) {
				pr_err("Error during feature %d version reading\n", feature_idx);
				return -1;
			}

			err = add_feature_to_list(list, &feature);
			if (err) {
				pr_err("Error during adding feature %s to list\n", feature.name);
				return -1;
			}
		}

		feat_bit_mask >>= 1;
		feature_idx++;
	}

	return 0;
}

static struct fpga_feature *get_feature(int type,
					struct fpga_feature_list *list)
{
	int i;
	for (i = 0; i < list->count; i++) {
		if (type == list->features_arr[i].type) {
			return &list->features_arr[i];
		}
	}

	return NULL;
}

/**
 * Get FPGA feature by name from features list if feature is present.
 *
 * \param name Feature name to get.
 * \param list Features list to search.
 *
 * \return Pointer to struct fpga_feature in case or success,
 *         NULL if feature is not found.
 */
struct fpga_feature *stcmtk_get_feature_by_name(char *name, struct fpga_feature_list *list)
{
	int i;
	for (i = 0; i < list->count; i++) {
		if (strcmp(name, list->features_arr[i].name) == 0) {
			return &list->features_arr[i];
		}
	}

	return NULL;
}
EXPORT_SYMBOL(stcmtk_get_feature_by_name);

/**
 * Return Control Registers base for given port.
 *
 * \param feat Feature to get base, it must be supported on the port.
 * \param port Port index, from 0.
 *
 * \return base address for the port.
 */
uint16_t stcmtk_get_cr_base_on_port(struct fpga_feature *feat, unsigned int port)
{
	return feat->cr_base + feat->cr_cnt * port;
}
EXPORT_SYMBOL(stcmtk_get_cr_base_on_port);

/**
 * Function returns Status Registers base for given port.
 *
 *	\param feat Feature to get base, it must be supported on the port.
 *	\param port Port index, from 0.
 *
 *	\return base address for the port.
 */
uint16_t stcmtk_get_sr_base_on_port(struct fpga_feature *feat, unsigned int port)
{
	return feat->sr_base + feat->sr_cnt * port;
}
EXPORT_SYMBOL(stcmtk_get_sr_base_on_port);



/** Function gets FPGA feature from features list if feature is presents
 *				 in the list on the certain port.
 *
 *	\note Control and status offsets for the returned feature are for zero
 *				port.
 *
 *	\param name Feature name to get.
 *	\param port Port number to check, from 0.
 *	\param list Features list to search.
 *
 *	\sa stcmtk_get_cr_base_on_port
 *	\sa stcmtk_get_sr_base_on_port
 *
 *	\return Pointer to struct fpga_feature in case or success, NULL if feature
 *					is not found.
 */
static struct fpga_feature *get_feature_on_port_by_name(char *name, unsigned int port,
		struct fpga_feature_list *list)
{
	struct fpga_feature *feat = stcmtk_get_feature_by_name(name, list);

	if (feat == NULL) {
		return NULL;
	}

	if ((feat->port_mask & (1L << port)) == 0) {
		return NULL;
	}

	return feat;
}

/** Function check if FPGA feature is presents in the features list.
 *
 *	\param name Feature name to get.
 *	\param list Features list to search.
 *
 *	\return 1 if feature is present, 0 if it is not.
 */
int stcmtk_is_fpga_feature_present(char *name, struct fpga_feature_list *list)
{
	return (stcmtk_get_feature_by_name(name, list) != NULL) ? 1 : 0;
}
EXPORT_SYMBOL(stcmtk_is_fpga_feature_present);

/** Function check if FPGA feature is presents in the features list on
 *				 the certain port
 *
 *	\param name Feature name to get.
 *	\param port Port number to check, from 0.
 *	\param list Features list to search.
 *
 *	\return 1 if feature is present, 0 if it is not.
 */
int is_fpga_feature_present_on_port(char *name, unsigned int port,
		struct fpga_feature_list *list)
{
	return (get_feature_on_port_by_name(name, port, list) != NULL) ? 1 : 0;
}

/**
 * Read features data from FPGA, parse it and return pointer to features list.
 */
static struct fpga_feature_list *fpgafeat_init(struct device *dev,
							struct regmap *regmap)
{
	struct fpga_feature_list *flist = NULL;
	uint32_t *mem_buffer = NULL;

	mem_buffer = kmalloc(FEATURE_MEM_WORDS * sizeof(uint32_t), GFP_KERNEL);
	if (mem_buffer == NULL) {
		pr_err("Failed to allocate memory\n");
		return NULL;
	}

	if (read_features(mem_buffer, regmap)) {
		pr_err("Error during reading FPGA features from memory\n");
		goto err_free_mem_buffer;
	}
	pr_debug("Features read from FPGA\n");

	flist = kzalloc(sizeof(*flist), GFP_KERNEL);
	if (!flist) {
		pr_err("Failed to allocate memory for FPGA features list\n");
		goto err_free_mem_buffer;
	}

	if (parse_features(mem_buffer, flist, regmap)) {
		pr_err("Error during parsing FPGA features\n");
		goto err_free_flist;
	}
	pr_debug("Features parsed %p\n", flist);

	if (init_features_sysfs(dev, flist)) {
		pr_err("Failed to create sysfs hierarchy for FPGA features\n");
		goto err_free_flist;
	}


	kfree(mem_buffer);

	return flist;

err_free_flist:
	kfree(flist);
err_free_mem_buffer:
	kfree(mem_buffer);
	return NULL;
}

/** Library deinitializer. It clears all memory that has been allocated
 *				 during library working.
 */
void stcmtk_fpgafeat_deinit(struct device *dev, struct fpga_feature_list **plist)
{
	struct fpga_feature_list *list = *plist;

	BUG_ON(!list);

	deinit_features_sysfs(dev, list);
	free_features_list(list);
	*plist = NULL;
}
EXPORT_SYMBOL(stcmtk_fpgafeat_deinit);


DECLARE_COMPLETION(fpga_parse_done);

/**
 * Fpga features parsing.
 * NOTE: it is expected that fpga has already been in proper state
 *       (FPGA_MGR_STATE_OPERATING) and regmap has been initialized
 */
struct fpga_feature_list *stcmtk_fpga_parse_features(struct stcmtk_common_fpga *fpga)
{
	struct fpga_feature_list *flist;

	reinit_completion(&fpga_parse_done);

	/* Get FPGA features */
	if (fpga->features != NULL) {
		dev_info(fpga->abstract_fpga,
		"Deinit previous features...\n");
	stcmtk_fpgafeat_deinit(fpga->abstract_fpga, &fpga->features);
	}

	flist = fpgafeat_init(fpga->abstract_fpga, fpga->regmap);
	fpga->features = flist;

	complete_all(&fpga_parse_done);

	return fpga->features;
}
EXPORT_SYMBOL(stcmtk_fpga_parse_features);

/**
 *     stcmtk_fpga_get_feature		Returns FPGA feature information.
 *     @g: Pointer to stcmtk_common_fpga that has features
 *     @type: Type of feature we need
 *
 *     Function returns pointer to FPGA feature of type in stcmtk_common_fpga
 *     or NULL if stcmtk_common_fpga doesn't contain target feature.
 */
struct fpga_feature *stcmtk_fpga_get_feature(struct stcmtk_common_fpga *g, int type)
{
	wait_for_completion_timeout(&fpga_parse_done,
			msecs_to_jiffies(WAIT_TIMEOUT));

	if(g && g->features)
		return get_feature(type, g->features);
	else
		return NULL;
}
EXPORT_SYMBOL(stcmtk_fpga_get_feature);

#define to_fpga_mgr(d) (struct stcmtk_common_fpga *)(d->driver_data)

static struct stcmtk_common_fpga *__fpga_mgr_get(struct device *dev)
{
	struct stcmtk_common_fpga *mgr;
	int ret = -ENODEV;

	mgr = to_fpga_mgr(dev);
	if (!mgr)
		goto err_dev;

	return mgr;

err_dev:
	put_device(dev);
	return ERR_PTR(ret);
}

static int fpga_of_node_match(struct device *dev, const void *data)
{
	return ((struct stcmtk_common_fpga *)dev->driver_data)->dev->of_node == data;
}

/*
 * stcmtk_put_fpga - release a reference to a fpga manager
 * @mgr:       fpga manager structure
 */
void stcmtk_put_fpga(struct stcmtk_common_fpga *mgr)
{
	put_device(mgr->abstract_fpga);
}
EXPORT_SYMBOL_GPL(stcmtk_put_fpga);


/**
 * stcmtk_get_fpga - get an exclusive reference to a fpga mgr
 * @node:		device node
 *
 * Given a device node, get an exclusive reference to a fpga mgr.
 *
 * Return: fpga manager struct or NULL condition containing error code.
 */

struct stcmtk_common_fpga* stcmtk_get_fpga(const struct device_node *np)
{
	struct device_node *fpga_node      = NULL;
	struct device *fpga_dev            = NULL;
	struct stcmtk_common_fpga *f       = NULL;


	wait_for_completion_timeout(&fpga_init_done,
			msecs_to_jiffies(WAIT_TIMEOUT));

	fpga_node = of_parse_phandle(np, "target-fpga", 0);
	if(!fpga_node) {
		pr_err("can't get fpga node from phandle\n");
		goto err_nonode;
	}

	fpga_dev = class_find_device(fpga_class, NULL, fpga_node,
				fpga_of_node_match);
	if (!fpga_dev) {
		pr_err("can't get fpga dev from node\n");
		goto err_nodev;
	}

	f = __fpga_mgr_get(fpga_dev);
	if(!f) {
		pr_err("can't get common fpga from pdev\n");
		goto err_nofpga;
	}

	of_node_put(fpga_node);
	return f;

err_nofpga:
err_nodev:
	of_node_put(fpga_node);
err_nonode:
	return NULL;
}
EXPORT_SYMBOL(stcmtk_get_fpga);

static int __init common_fpga_init(void)
{
	printk("Probe FPGA library module loading\n");
	return 0;
}

static void __exit common_fpga_exit(void)
{
}

module_init(common_fpga_init);
module_exit(common_fpga_exit);

MODULE_AUTHOR("STC Metrotek System Team <system@metrotek.ru>");
MODULE_DESCRIPTION("STC Metrotek Common FPGA Driver");
MODULE_LICENSE("GPL v2");
