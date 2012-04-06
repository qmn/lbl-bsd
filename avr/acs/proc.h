#ifndef LBL_BSD_PROC_H
#define LBL_BSD_PROC_H

#include <stdint.h>

#define FLAG_TICK    0x01
#define FLAG_I2C     0x02
#define FLAG_GPS     0x04
#define FLAG_DCM     0x08
#define SEL_SEN_ZBUF 0x80

extern volatile uint8_t status;

#endif /* LBL_BSD_PROC_H */
