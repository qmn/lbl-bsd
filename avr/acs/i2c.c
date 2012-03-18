#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "global.h"
#include "i2c.h"

#define I2C_PS 1
#define I2C_BR (((((F_CPU) / (F_SCL)) - 16) >> 1) / (I2C_PS))

void i2c_init(void) {
	TWBR = I2C_BR; /* bit rate */
	TWSR = (0 << TWPS1) | (0 << TWPS0); /* prescalar */
}

struct i2c_op * volatile i2c_routine;
uint8_t * volatile i2c_data_rx;
uint8_t * volatile i2c_data_tx;

#define I2C_SC_MASK (~((1 << TWPS1) | (1 << TWPS0)))
/** Status codes **/
#define I2C_SC_START   0x08
#define I2C_SC_RESTART 0x10
/* Master transmitter mode */
#define I2C_SC_MTX_SLA_ACK   0x18
#define I2C_SC_MTX_SLA_NACK  0x20
#define I2C_SC_MTX_DATA_ACK  0x28
#define I2C_SC_MTX_DATA_NACK 0x30
/* Master receiver mode */
#define I2C_SC_MRX_SLA_ACK   0x40
#define I2C_SC_MRX_SLA_NACK  0x48
#define I2C_SC_MRX_DATA_ACK  0x50
#define I2C_SC_MRX_DATA_NACK 0x58

#define I2C_WRITE 0x00
#define I2C_READ  0x01

#define I2C_SLA_MASK 0xFE
#define I2C_OPC_MASK 0x01
#define IS_HALT(op) ((op)->sla == 0x01)

#define I2C_RESUME  ((1 << TWINT) | (1 << TWEN))

ISR(TWI_vect) {
	static struct i2c_op *op;
	static uint8_t *rx;
	static uint8_t *tx;
	static uint8_t todo, count;

	switch (TWSR & I2C_SC_MASK) {

		case I2C_SC_START: /* start condition transmitted */
			/* Copy pointers */
			op = i2c_routine;
			rx = i2c_data_rx;
			tx = i2c_data_tx;
			todo = I2C_WRITE; /* next action */
			/* Fall-through */

		case I2C_SC_RESTART: /* start condition repeated */
			/* Transmit slave address */
			if (todo == I2C_WRITE) { /* new operation */
				todo = (op->sla & I2C_OPC_MASK);
				TWDR = (op->sla & I2C_SLA_MASK) | I2C_WRITE; /* SLA+W */
				/* Transition to I2C_SC_MTX_SLA_ACK */
			} else { /* ongoing RX operation */
				todo = I2C_WRITE;
				TWDR = (op->sla & I2C_SLA_MASK) | I2C_READ; /* SLA+R */
				/* Transition to I2C_SC_MRX_SLA_ACK */
			}
			TWCR = I2C_RESUME | (1 << TWIE);
			break;

		case I2C_SC_MTX_SLA_ACK: /* acknowledgement from slave */
			count = op->len;
			/* Transmit register address */
			TWDR = op->reg;
			TWCR = I2C_RESUME | (1 << TWIE);
			/* Transition to I2C_SC_MTX_DATA_ACK */
			break;

		case I2C_SC_MTX_DATA_ACK: /* acknowledgement of data */
			if (todo == I2C_WRITE) {
				if (count > 0) {
					count--;
					/* Transmit data byte */
					TWDR = *tx++;
					TWCR = I2C_RESUME | (1 << TWIE);
					/* Transition to I2C_SC_MTX_DATA_ACK */
					break;
				} else if (op++, IS_HALT(op)) {
					/* Transmit stop condition and halt */
					TWCR = I2C_RESUME | (1 << TWSTO);
					status |= STATUS_I2C;
					break;
				}
			}
			/* Retransmit start condition */
			TWCR = I2C_RESUME | (1 << TWSTA) | (1 << TWIE);
			/* Transition to I2C_SC_RESTART */
			break;

		case I2C_SC_MRX_SLA_ACK: /* acknowledgement from slave */
			/* TODO: handle (count == 0) */
			if (--count > 0) {
				/* Transmit ACK to initiate data read */
				TWCR = I2C_RESUME | (1 << TWIE) | (1 << TWEA);
				/* Transition to I2C_SC_MRX_DATA_ACK */
			} else {
				/* Transmit NACK to initiate a single-byte read */
				TWCR = I2C_RESUME | (1 << TWIE) | (0 << TWEA);
				/* Transition to I2C_SC_MRX_DATA_NACK */
			}
			break;

		case I2C_SC_MRX_DATA_ACK: /* ACK transmitted */
			*rx++ = TWDR; /* data byte */
			if (--count > 0) {
				/* Transmit ACK to continue data read */
				TWCR = I2C_RESUME | (1 << TWIE) | (1 << TWEA);
				/* Transition to I2C_SC_MRX_DATA_ACK */
			} else { /* Transmit NACK to indicate stop after next data byte */
				TWCR = I2C_RESUME | (1 << TWIE) | (0 << TWEA);
				/* Transition to I2C_SC_MRX_DATA_NACK */
			}
			break;

		case I2C_SC_MRX_DATA_NACK: /* NACK transmitted */
			*rx++ = TWDR; /* last data byte */
			if (op++, IS_HALT(op)) {
				/* Transmit stop condition and halt */
				TWCR = I2C_RESUME | (1 << TWSTO);
				status |= STATUS_I2C;
			} else {
				/* Retransmit start condition to continue
				   onto next operation in routine */
				TWCR = I2C_RESUME | (1 << TWSTA) | (1 << TWIE);
				/* Transition to I2C_SC_RESTART */
			}
			break;

		default:
			DDRB |= (1 << PB5);
			PORTB |= (1 << PB5);
	}
}

#define i2c_spin(b) do {} while ((TWCR & (1 << (b))) == 0)

void i2c_tx(const struct i2c_op *op, const uint8_t *tx) {
	for (; !(IS_HALT(op)); op++) {
		uint8_t i;
		/* Transmit start condition */
		TWCR = I2C_RESUME | (1 << TWSTA);
		i2c_spin(TWINT);
		/* Transmit SLA+W */
		TWDR = (op->sla & I2C_SLA_MASK) | I2C_WRITE;
		TWCR = I2C_RESUME;
		i2c_spin(TWINT);
		/* Transmit register address */
		TWDR = op->reg;
		TWCR = I2C_RESUME;
		i2c_spin(TWINT);
		/* Transmit data bytes */
		for (i = op->len; i > 0; i--) {
			TWDR = *tx++;
			TWCR = I2C_RESUME;
			i2c_spin(TWINT);
		}
	}
	/* Transmit stop condition */
	TWCR = I2C_RESUME | (1 << TWSTO);
	i2c_spin(TWSTO);
}
