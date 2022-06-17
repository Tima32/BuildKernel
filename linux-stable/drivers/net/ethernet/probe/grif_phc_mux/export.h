
/*
 * These functions are used by:
 *
 * - grif-phc.ko module to report clock_index of its PHC device when it gets one
 *
 *TODO: что по ethtool в grif???

 * - etn_net.ko module to retrieve clock_index mentioned above when responding
 *   to SIOCETHTOOL (aka `ethtool -T`)
 *
 *   They use use atomic_t and can be called from any context.
 */

/* Return the value set by `set_grif_phc_index`
 */
int get_grif_phc_index(struct device *dev, ssize_t port_num);

/* Set PHC index to `index`. If you want to change previously set PHC index, do
 * not forget firstly to unset it with `unset_grif_phc_index`.
 */
void set_grif_phc_index(struct device *dev, ssize_t port_num, int index);

/* Set PHC index to -1
 */
void unset_grif_phc_index(struct device *dev, ssize_t port_num);
