#include <avr/io.h>
#include <avr/interrupt.h>

#define ADC_SHIFT 6
#define ADC_CONVERSIONS (1 << ADC_SHIFT)

// 12.5 V / (5 V / 1024) = 640
#define PURTENTUR 640

#ifndef F_CPU
#define F_CPU 1000000
#endif

#define F_PWM 1000
#define PWM_TOP (F_CPU / F_PWM)

#define PWM_INITIAL 220
#define PWM_MIN 220
#define PWM_MAX 920

#define KP 1

volatile uint16_t adc_sum = 0;
volatile uint8_t  adc_convs = 0;
volatile uint16_t adc_ave = 0;
uint16_t pwm_set = PWM_INITIAL;

ISR(TIM0_OVF_vect) {
	// Need this for proper trigger of ADC
}

ISR(TIM1_OVF_vect) {

}

ISR(ADC_vect) {
	uint16_t adc = ADCL;
	adc |= (ADCH << 8);

	adc_sum += adc;
	adc_convs++;

	if (adc_convs >= ADC_CONVERSIONS) {
		adc_ave = (adc_sum >> ADC_SHIFT);
		adc_sum = 0;
		adc_convs = 0;

		if (adc_ave > PURTENTUR) {
			PORTB = (1 << PB0);
		} else {
			PORTB = (0 << PB0);
		}

		int16_t error = PURTENTUR - adc_ave;
		pwm_set += KP * error;

		if (pwm_set > PWM_MAX)
			pwm_set = PWM_MAX;
		if (pwm_set < PWM_MIN)
			pwm_set = PWM_MIN;

		OCR1B = pwm_set;
	}
}

int main(void) {
	PORTB |= (1 << PB0);
	DDRB |= (1 << PB0);
	PORTA &= ~(1 << PA5);
	DDRA |= (1 << PA5);

	// Turn on the ADC
	PRR &= ~(1 << PRADC);

	// Pick PA1 and GND for comparison
	ADMUX = (1 << MUX0);

	// And activate the interrupt
	// Set the prescaler to 8: 1MHz / 8 < 200 kHz
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) |
	         (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	// Set to trigger on Timer0 Overflow
	ADCSRB = (1 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);

	// Enable the timer, duh
	TCCR0B = (0 << CS02) | (1 << CS01) | (1 << CS00);
	TIMSK0 = (1 << TOIE0);

	// Setup PWM
	TCCR1A = (1 << COM1B1) | (0 << COM1B0) |
	         (1 << WGM11) | (1 << WGM10);
	TCCR1B = (1 << WGM13) | (1 << WGM12) |
	         (0 << CS12) | (0 << CS11) | (1 << CS10);
	OCR1A = PWM_TOP;
	OCR1B = PWM_INITIAL;
	
	sei();


	while (1) {	}

	return 0;
}
