#ifndef _ETN_FAN_H
#define _ETN_FAN_H

#define DRV_NAME "etn_fan"
#define DRV_VERSION "0.0.3"

#define MAX_PWM 255

#define FAN_MAIN_CR (0)

/* These are very handy numbers :) */
#define MODE_PWM_OFF     (3)
#define MODE_PWM_30PCT   (0)
#define MODE_PWM_60PCT   (1)
#define MODE_PWM_MAX     (2)


struct fan_regs {
	int cr;
	int sr;
};
#endif /* _ETN_FAN_H */
