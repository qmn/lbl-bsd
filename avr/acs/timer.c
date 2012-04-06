#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "i2c.h"
#include "sensor.h"
#include "proc.h"
#include "timer.h"

#define TIMER0_PS 10
#define JIFFY (((F_CPU) >> (TIMER0_PS)) / (F_TICK))

void timer_init(void) {
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS02) | (1 << CS00);
	TIMSK0 |= (1 << OCIE0A);
	OCR0A = (JIFFY - 1);

	TCCR1B = (1 << CS10);
}

ISR(TIMER0_COMPA_vect) {
	static uint8_t gps_wait = 0;
	uint8_t _status;
	_status = status;

	if ((_status & FLAG_I2C) && !(TWCR & (1 << TWSTO))) {
		_status &= ~(FLAG_I2C | FLAG_GPS);
		/* Poll GPS */
		if (++gps_wait < GPS_PERIOD) {
			i2c_routine = sen_ins_read_op;
		} else {
			gps_wait = 0;
			i2c_routine = sen_gps_read_op;
		}
		/* Switch buffers */
		if (_status & FLAG_DCM) {
			_status &= ~(FLAG_DCM);
			_status ^= SEL_SEN_ZBUF;
			if (gps_wait == 1)
				_status |= FLAG_GPS;
			i2c_data_rx = (_status & SEL_SEN_ZBUF) ?
				sen_zbuf[1].raw : sen_zbuf[0].raw;
		}
		TWCR = I2C_CTRL_START;
	}
	
	_status |= FLAG_TICK;
	status = _status;
}
