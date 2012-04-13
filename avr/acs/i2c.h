#ifndef LBL_BSD_I2C_H
#define LBL_BSD_I2C_H

#include <stdint.h>
#include <avr/io.h>

#ifndef F_SCL
#define F_SCL 100000UL
#endif /* F_SCL */

struct i2c_op {
	uint8_t sla;
	uint8_t reg;
	uint8_t len;
};

#define I2C_OP_TX(s,r,l) \
	{ .sla = ((s) << 1), .reg = (r), .len = (l) }
#define I2C_OP_RX(s,r,l) \
	{ .sla = (((s) << 1) | 0x1), .reg = (r), .len = (l) }
#define I2C_OP_HALT() { .sla = 0x01 }

extern struct i2c_op * volatile i2c_routine;
extern uint8_t * volatile i2c_data_rx;
extern uint8_t * volatile i2c_data_tx;

extern void i2c_init(void);
extern void i2c_send(const struct i2c_op *, const uint8_t *);

/* TWCR flags for transmitting start condition */
#define I2C_CTRL_START ((1 << TWINT) | (1 << TWEN) | (1 << TWSTA) | (1 << TWIE))

#endif /* LBL_BSD_I2C_H */
