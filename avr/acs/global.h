#ifndef LBL_BSD_GLOBAL_H
#define LBL_BSD_GLOBAL_H

#include <stdint.h>

#ifndef F_TICK
#define F_TICK 100
#endif /* F_TICK */

extern volatile uint8_t status;
#define STATUS_TICK 0x1
#define STATUS_I2C  0x2
#define STATUS_DCM  0x4

#endif /* LBL_BSD_GLOBAL_H */
