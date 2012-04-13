#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "i2c.h"
#include "sensor.h"
#include "proc.h"
#include "timer.h"

/* 100 Hz system tick */
#define F_TICK 100
#define TIMER1_PS 3
#define TIMER2_PS 10
#define JIFFY (((F_CPU) >> (TIMER2_PS)) / (F_TICK))
#define TIMER0_PS 10

volatile uint16_t pwm_rud = (PWM_SRV_MID + 100);
volatile uint16_t pwm_elv = (PWM_SRV_MID);
volatile uint8_t  pwm_esc = (PWM_ESC_OFF);

static inline void _pwm_init(void) {
	/* Timer/Counter 1 Output Compare Register A (16.11.5) */
	OCR1A = 0;
	/* Timer/Counter 1 Output Compare Register B (16.11.6) */
	OCR1B = 0;
	/* Enable OC0A, OC1A, and OC1B output drivers */
	PORTD &= ~(1 << PD6);
	PORTB &= ~(1 << PB1);
	PORTB &= ~(1 << PB2);
	DDRD |= (1 << PD6);
	DDRB |= (1 << PB1);
	DDRB |= (1 << PB2);
}

static inline void _pwm_enable(void) {
	OCR1A = pwm_rud; /* rudder servo */
	OCR1B = pwm_elv; /* elevator servo */
	OCR0A = pwm_esc; /* ESC throttle */
	/* Enable Timer 0 */
	/* Timer/Counter 0 Control Register B (15.9.2)
	 * Clock select: prescalar 1024
	 *   CS02 = 1, CS01 = 0, CS00 = 1
	 */
	TCCR0B = (1 << CS02) | (1 << CS00);
}

static inline void _pwm_disable(void) {
	OCR1A = 0;
	OCR1B = 0;
}

ISR(TIMER0_COMPA_vect) {
	/* Disable and reset Timer 0 */
	TCCR0B = 0x00;
	TCNT0  = 0xFF;
}

void timer_init(void) {
	_pwm_init();

	/* Timer/Counter 0 Control Register A (15.9.1)
	 * Waveform generation mode: Fast PWM
	 *   TOP = 0xFF, update of OCR0x at 0x00, TOV0 flag set at 0xFF
	 *   WGM02 = 0 (TCCR0B), WGM01 = 1, WGM00 = 1
	 * Compare Output Mode:
	 *   Clear OC0A on Compare Match, set OC0A at 0x00
	 */
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	/* Disable Timer 0 */
	TCCR0B = 0x00;
	/* Timer/Counter 0 Interrupt Mask Register (15.9.6)
	 * Output Compare Match A Interrupt Enable: OCIE0A = 1
	 */
	TIMSK0 |= (1 << OCIE0A);

	/* Timer/Counter 1 Control Register A (16.11.1)
	 * Compare output mode:
	 *   Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting)
	 *   COM1A1 = 1, COM1A0 = 0, COM1B1 = 1, COM1B0 = 0
	 * Waveform generation mode: Fast PWM
	 *   TOP = ICR1, update OCR1x at BOTTOM, TOV1 flag set on TOP
	 *   WGM13 = 1 (TCCR1B), WGM12 = 1 (TCCR1B), WGM11 = 1, WGM10 = 0
	 */
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	/* Timer/Counter 1 Control Register B (16.11.2)
	 * Clock select: prescaler 8
	 *   CS12 = 0, CS11 = 1, CS10 = 0
	 */
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
	/* Timer/Counter 1 Input Capture Register 1 (16.11.7) */
	ICR1 = 40000;
	/* Timer/Counter 0 Interrupt Mask Register (16.11.8)
	 * Overflow Interrupt Enable: TOIE1 = 1
	 */
//	TIMSK1 |= (1 << TOIE1);

	TCCR2A = (1 << WGM21);
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
	TIMSK2 |= (1 << OCIE2A);
	OCR2A = (JIFFY);
}

#define GPS_PERIOD 100

ISR(TIMER2_COMPA_vect) {
	static uint8_t gps_wait = 0;
	uint8_t _status;
	_status = status;

	_status ^= FLAG_PWM;
	if (_status & FLAG_PWM) {
//		_pwm_disable();
	} else {
		_pwm_enable();
	}

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
