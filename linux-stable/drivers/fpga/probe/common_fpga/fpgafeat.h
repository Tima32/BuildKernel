#ifndef COMMON_FPGA_FPGAFEAT__H
#define COMMON_FPGA_FPGAFEAT__H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/cdev.h>


/* Maximum length of feature's name in bytes, same as in FPGA memory. */
#define FEATURE_NAME_MAX (16)

/* Maximum length of FPGA firmware name, same as in FPGA memory */
#define FIRMWARE_NAME_MAX (64)

/* Maximum length of feature's description, bytes. */
#define FEATURE_DESCRIPTION_MAX (128)

/** Size of memory that stores memory information, 4-byte words */
#define FEATURE_MEM_WORDS (256)

/* Length of feature memory word, bytes */
#define FEATURE_MEM_WORD_LEN (sizeof(uint32_t))

/* Offset to firmware info string in words */
#define FEATURE_FIRMWARE_INFO_OFFSET (240)

/* 5 sec */
#define WAIT_TIMEOUT 5000

/** Enum describes all supported FPGA features */
enum fpga_features {
    FPGA_FEATURE_MX_FPGA_ID,
    FPGA_FEATURE_MX_PHY,
    FPGA_FEATURE_MX_STAT,
    FPGA_FEATURE_FLEX_FILT,
    FPGA_FEATURE_FLEX_FILT_PM,
    FPGA_FEATURE_FLEX_FILT_RTP,
    FPGA_FEATURE_FS_FILT,
    FPGA_FEATURE_FIELDS_RPL,
    FPGA_FEATURE_HTP_PACKER,
    FPGA_FEATURE_PKT_ENCAPS,
    FPGA_FEATURE_FLOW_GEN,
    FPGA_FEATURE_SIGN_ANLZ,
    FPGA_FEATURE_NIC_CPU_BUF,
    FPGA_FEATURE_RMON_STAT,
    FPGA_FEATURE_LB,
    FPGA_FEATURE_TWAMP_RX_FILT,
    FPGA_FEATURE_TWAMP_REFL,
    FPGA_FEATURE_NIC_CTRL,
    FPGA_FEATURE_BERT_GEN,
    FPGA_FEATURE_BERT_ANLZ,
    FPGA_FEATURE_LCD,
    FPGA_FEATURE_BEEPER,
    FPGA_FEATURE_SOM_HWTEST,
    FPGA_FEATURE_MPT_ANLZ,
    FPGA_FEATURE_MPT_TX,
    FPGA_FEATURE_FAN,
    FPGA_FEATURE_PTP_RTC,
    FPGA_FEATURE_TWAMP_CPU,
    FPGA_FEATURE_TX_CIR,
    FPGA_FEATURE_RX_RATE_LIMIT,
    FPGA_FEATURE_SFP_CTL,
    FPGA_FEATURE_LB_FILT,
    FPGA_FEATURE_INTERVAL_GEN,
    FPGA_FEATURE_INTERVAL_ANLZ,
    FPGA_FEATURE_LB_REPLACE,
    FPGA_FEATURE_LB_BASE,
    FPGA_FEATURE_I2C_MASTER,
    FPGA_FEATURE_ECP5_TEMP_MON,
    FPGA_FEATURE_CLK_CTL,
    FPGA_FEATURE_GEN_EXAM,
    FPGA_FEATURE_ANLZ_EXAM,
    FPGA_FEATURE_PKT_CAPTURE,
    FPGA_FEATURE_PAYLOAD_INS,
    FPGA_FEATURE_SDT_ANLZ,
    FPGA_FEATURE_GPIO_CTL,
    FPGA_FEATURE_LB_SELECTOR,
    FPGA_FEATURE_MEM,
    FPGA_FEATURE_DELAY_EMU,
    FPGA_FEATURE_SCI_CTL,
    FPGA_FEATURE_PHC_MUX,
    FPGA_FEATURE_SFP_DDMI,
    FPGA_FEATURE_FIELD_SPLIT,
    FPGA_FEATURE_RAND_GEN_TO_MEM,
    FPGA_FEATURE_ERSPAN,
    FPGA_FEATURE_PORT_STAT,
    FPGA_FEATURE_LB_STAT,
    FPGA_FEATURE_PT_STAT
};

struct fpga_feature {
    /* To present feature in sysfs */
    struct kobject kobj;

    /* Feature type as in enum fpga_features */
    int type;

    /** Feature name. */
    char name[ FEATURE_NAME_MAX + 1 ];

    /** Mask that shows which ports are supported the feature. '1' means feature
     *  is supported on corresponding port. */
    uint32_t port_mask;

    /** Control registers base address for least port. */
    uint16_t cr_base;

    /** Control registers count for each port. */
    uint16_t cr_cnt;

    /** Status registers base address for least port. */
    uint16_t sr_base;

    /** Status registers count for each port. */
    uint16_t sr_cnt;

    /** Feature version. */
    uint16_t version;
};
#define to_fpga_feature(x) container_of(x, struct fpga_feature, kobj)

struct fpga_feature_list {

    /* To organize feature in sysfs hierarchy */
    struct kset *kset;

    /** Version of FPGA firmware */
    uint16_t fpga_version;

    /** ROM memory ID version */
    uint8_t rom_id;

    /** Fpga firware name. Usually contains device name. */
    char name[FIRMWARE_NAME_MAX];

    /** Count of features that are supported in the FPGA firmware. */
    int count;

    /** Array of current features. */
    struct fpga_feature *features_arr;
};

/* Some description
 * @dev:		FPGA hardware device. pointer to struct device of platform
 * 			device or SPI device or etc.
 * @abstract_fpga:	FPGA abstract device belongs to special "fpga" class and
 * 			includes FPGA-specific information, like features list.
 * @features:		Pointer to parsed FPGA features list.
 * @regmap:		FPGA register map.
 * @cdev_minor:		Minor number for FPGA character device.
 * @cdev:		FPGA character device for abstract FPGA device.
 */
struct stcmtk_common_fpga {
	struct device *dev;
	struct device *abstract_fpga;
	struct fpga_feature_list *features;
	struct regmap *regmap;
	int cdev_minor;
	struct cdev cdev;

	/* this field either could point on
	 * struct grif_fpga or on struct etn_fpga */
	void *fpga_defined_fields;
};

struct fpga_feature_list *get_features_list(void);

struct fpga_feature *stcmtk_get_feature_by_name(char *name, struct fpga_feature_list *list);


uint16_t stcmtk_get_cr_base_on_port(struct fpga_feature *feat, unsigned int port);
uint16_t stcmtk_get_sr_base_on_port(struct fpga_feature *feat, unsigned int port);

int stcmtk_is_fpga_feature_present( char *name, struct fpga_feature_list *list );
void stcmtk_fpgafeat_deinit(struct device *dev, struct fpga_feature_list **plist);

struct fpga_feature *stcmtk_fpga_get_feature(struct stcmtk_common_fpga *g, int type);
struct fpga_feature_list *stcmtk_fpga_parse_features(struct stcmtk_common_fpga *fpga);

struct stcmtk_common_fpga* stcmtk_get_fpga(const struct device_node *np);
void stcmtk_put_fpga(struct stcmtk_common_fpga *mgr);

struct device* stcmtk_device_create(struct device *dev, dev_t devt,
									struct stcmtk_common_fpga *fpga, const char *str, int id);

struct device* stcmtk_class_find_device(struct device *start, struct device_node *node,
										int (*match)(struct device *, const void *));

void stcmtk_class_destroy(void);

void stcmtk_device_destroy(dev_t devt);

int stcmtk_class_create(struct module *owner, const char *name, const struct attribute_group *fpga_groups[]);

void finish_completion(void);

#endif // COMMON_FPGA_FPGAFEAT__H

