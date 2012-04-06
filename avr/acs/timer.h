#ifndef LBL_BSD_TIMER_H
#define LBL_BSD_TIMER_H

#ifndef F_TICK
#define F_TICK 100
#endif /* F_TICK */

/* GPS polling rate */
#define F_GPS 1
#define GPS_PERIOD (F_TICK / F_GPS)

extern volatile uint8_t clock;
extern void timer_init(void);

#endif /* LBL_BSD_TIMER_H */
