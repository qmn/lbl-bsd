#include <avr/io.h>
#include <avr/interrupt.h>

#include "i2c_driver.h"

volatile int buf_ptr = 0;
volatile int buf_len = 16;
volatile int new_buf = 1;
static char buf[16];

ISR(USART_TX_vect) {
	if (buf_ptr >= buf_len) {
		buf_ptr = 0;
		new_buf = 1; // refill
		UCSR0B = (0 << TXEN0) | (0 << RXEN0) | (0 << TXCIE0); // disable
	} else {
		UDR0 = buf[buf_ptr];
		buf_ptr++;
		UCSR0B = (1 << TXEN0) | (0 << RXEN0) | (1 << TXCIE0);
	}
}

int main(void) {
	init_i2c();

	UBRR0L = 103;
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B = (1 << TXEN0);
	int idx = 0;

	while (1) {
		if (new_buf) {
			i2c_read_registers((44 << 1), 0, buf, 16);
			new_buf = 0;	
			buf_ptr = 1;
			UDR0 = buf[0];
			UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << TXCIE0);
		}
	}

	return 0;
}

