#include <stdarg.h>
#include <stdint.h>
#include "usart.h"

void print(uint16_t n) {
	static uint8_t hex[0x10] = {
		'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
	};
	uint8_t buf[4];
	buf[0] = hex[(n >> 12)];
	buf[1] = hex[(n >> 8) & 0xF];
	buf[2] = hex[(n >> 4) & 0xF];
	buf[3] = hex[(n & 0xF)];
	usart_write(buf, sizeof(buf));
}

void flush(void) {
	static uint8_t eol[] = { '\r', '\n' };
	usart_write(eol, sizeof(eol));
}
