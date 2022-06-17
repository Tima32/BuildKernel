#ifndef COMMON_FPGA_FEATURES_SYSFS__H
#define COMMON_FPGA_FEATURES_SYSFS__H

int init_features_sysfs(struct device *dev, struct fpga_feature_list *list);
void deinit_features_sysfs(struct device *dev, struct fpga_feature_list *list);

#endif // COMMON_FPGA_FEATURES_SYSFS__H

