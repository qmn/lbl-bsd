#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "i2c.h"
#include "imu.h"
#include "global.h"

volatile uint8_t status = STATUS_I2C;

static union imu_raw imu_b1;
static union imu_raw imu_b2;

#define TIMER0_PS 10
#define JIFFY (((F_CPU) >> (TIMER0_PS)) / (F_TICK))

static inline void _tick_init(void) {
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS02) | (1 << CS00);
	TIMSK0 |= (1 << OCIE0A);
	OCR0A = (JIFFY - 1);
}

#define FLAG_SEL 0x1
ISR(TIMER0_COMPA_vect) {
	static uint8_t state = 0;
	uint8_t flags;
	flags = status;
	
	if ((flags & STATUS_I2C) && !(TWCR & (1 << TWSTO))) {
		flags &= ~(STATUS_I2C);
		if (flags & STATUS_DCM) {
			if (state & FLAG_SEL) {
				imu_cur = &imu_b2;
				i2c_data_rx = imu_b1.buf;
			} else {
				imu_cur = &imu_b1;
				i2c_data_rx = imu_b2.buf;
			}
			state ^= FLAG_SEL;
		}
		TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA) | (1 << TWIE);
	}

	flags &= ~(STATUS_DCM);
	flags |= STATUS_TICK;
	status = flags;
}

static uint8_t hex[] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

static void _ascii(uint8_t *buf, union imu_raw *src) {
	uint8_t i;
	uint8_t *ptr;
	ptr = src->buf;
	for (i = 18; i > 0; i--) {
		uint8_t n;
		n = *ptr++;
		*buf++ = hex[(n >> 4)];
		*buf++ = hex[(n & 0xF)];
	}
}

static void _u16toa(uint8_t *buf, uint16_t n) {
	*buf++ = hex[(n >> 12)];
	*buf++ = hex[(n >> 8) & 0xF];
	*buf++ = hex[(n >> 4) & 0xF];
	*buf   = hex[(n & 0xF)];
}

int main(void) {
	uint8_t buf[38];
	_tick_init();
	i2c_init();
	imu_init();
	usart_init();
	i2c_data_rx = imu_b1.buf;
	DDRB |= (1 << PB5);
	buf[36] = '\r';
	buf[37] = '\n';
	sei();

	for (;;) {
		union imu_raw *imu_raw;

		do {} while (!(status & STATUS_TICK));
		status &= ~(STATUS_TICK);
		imu_raw = imu_cur;
		imu_normalize(imu_raw);
		_ascii(buf, imu_raw);	
		//_u16toa(buf, (uint16_t)imu_cur);
		usart_write(buf, sizeof(buf));
		status |= STATUS_DCM;
	}
	return 0;
}
