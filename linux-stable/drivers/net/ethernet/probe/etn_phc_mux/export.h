
/*
 * These functions are used by:
 *
 * - etn_phc.ko module to report clock_index of its PHC device when it gets one
 *
 * - etn_net.ko module to retrieve clock_index mentioned above when responding
 *   to SIOCETHTOOL (aka `ethtool -T`)
 *
 *   They use use atomic_t and can be called from any context.
 */

/* Return the value set by `set_etn_phc_index`
 */
int get_etn_phc_index(ssize_t port_num);

/* Set PHC index to `index`. If you want to change previously set PHC index, do
 * not forget firstly to unset it with `unset_etn_phc_index`.
 */
void set_etn_phc_index(ssize_t port_num, int index);

/* Set PHC index to -1
 */
void unset_etn_phc_index(ssize_t port_num);
