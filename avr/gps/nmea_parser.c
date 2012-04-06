#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/*
 * nmea_parser.c
 * 
 * Parses NMEA strings from the GPS.
 *
 * @author Quan Nguyen
 *
 */

#include "nmea_consts.h"
#include "nmea_buffer.h"

volatile uint8_t rx_byte = '\0';

/* bitwise operators */
volatile int bbu_start = 0;
volatile int bbu_index = 0;

volatile static char uart_buffer[UART_BUFFER_LEN];
volatile int uart_idx = 0;
volatile uint8_t uart_done_flag = 0;

volatile static uint8_t output_buffer[OUTPUT_BUFFER_LEN];
volatile uint8_t output_idx = 0;

volatile static uint8_t temp_buffer[OUTPUT_BUFFER_LEN];

/* ================= BBU CODE =================================== */

ISR(PCINT0_vect) {
	if (bbu_start == BBUART_START && ~(PINA & (1 << PA7)) && uart_done_flag != 1) {
		bbu_start = BBUART_WAIT_HALF;
		PCMSK0 &= ~(1 << PCINT7);
		OCR0A = TIMER_WAIT_HALF;
		TCCR0A = (1 << COM0A1) | (1 << COM0A0) | (1 << WGM01);
		// (0, 1, 0) - prescale 8
		TCCR0B = (0 << CS02) | (1 << CS01) | (0<< CS00);
		TIMSK0 = (1 << OCIE0A);
		TCNT0 = 0;
	}
}

ISR(TIM0_COMPA_vect) {
	switch (bbu_start) {
		case BBUART_READ:
			if (bbu_index < 8) {
				/* Read next bit */
				rx_byte >>= 1;

				if (PINA & (1 << PA7)) {
					rx_byte |= 0x80;
				}
				bbu_index++;
			} else {
				/* Disable timer */
				char c = (char)(rx_byte);
				TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
	
				bbu_start = BBUART_START;

				// Ensures we have a GPRMC
				// If we copy a GGA, then parse, but find out we're not using it
				// we potentially waste a RMC
				if (uart_idx == 3 && c != 'R') {
					uart_idx = 0;
					PCMSK0 = (1 << PCINT7);
					return;
				}

				switch (c) {
					case '\r':
						uart_done_flag = 1;
						return;
					
					case '$':
						uart_idx = 0;

						// fall through
					default:
						uart_buffer[uart_idx] = c;
						uart_idx++;
						if (uart_idx >= UART_BUFFER_LEN)
							uart_idx = 0;
				}

				PCMSK0 = (1 << PCINT7);
			}
			break;

		case BBUART_WAIT_HALF:
			TCNT0 = 0;
			OCR0A = TIMER_WAIT;
			bbu_index = 0;
			rx_byte = '\0';
			bbu_start = BBUART_READ;
			break;

		case BBUART_START:
		default:
			TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
			break;
	}
}

/* ================= USI CODE =================================== */

/* I2C address, NOT SHIFTED. 0x58 = (44 << 1) */

volatile int usi_state = USI_START;
volatile int usi_reg = 0;

ISR(USI_STR_vect) {
	usi_state = USI_CHECK_ADDR;
	DDRA &= ~(1 << PA6);

	while ( (PINA & (1 << PA4)) && !(PINA & (1 << PA6))) {};
	// Now change the USI to start an actual receive
	USICR = (1 << USISIE) | (1 << USIOIE) |
	        (1 << USIWM1) | (1 << USIWM0) |
					(1 << USICS1) | (0 << USICS0) | 
					(0 << USICLK) | (0 << USITC);
	USISR = (1 << USISIF) | (1 << USIOIF) | 
	        (1 << USIPF)  | (1 << USIDC)  |
	         (0x0 << USICNT0);
}

/* We've received a byte! */
ISR(USI_OVF_vect) {
	switch (usi_state) {
		case USI_CHECK_ADDR:
			if ( (USIDR == 0) || (USIDR >> 1) == NP_SLA_ADDR) {
				if (USIDR & 0x01) {
					usi_state = USI_EMIT_DATA;
				} else {
					usi_state = USI_BEFORE_CAPTURE_REGISTER;
				}

				SEND_ACK();
			} else {
				usi_state = USI_START;
				THEN_WAIT_FOR_START();
			}
			break;

		case USI_BEFORE_CAPTURE_REGISTER:
			usi_state = USI_CAPTURE_REGISTER;
			THEN_USI_READ();
			break;

		case USI_CAPTURE_REGISTER:
			output_idx = USIDR;
			if (output_idx >= OUTPUT_BUFFER_LEN) {
				output_idx = 0;
			}
			usi_state = USI_REQUEST_CAPTURE;
			SEND_ACK();
			break;

		case USI_WRITE_REGISTER:
			usi_state = USI_REQUEST_CAPTURE;
			SEND_ACK();
			break;

		case USI_REQUEST_CAPTURE:
			usi_state = USI_WRITE_REGISTER;
			THEN_USI_READ();
			break;

		case USI_CHECK_EMIT_REPLY:
			if (USIDR) {
				THEN_WAIT_FOR_START();
				return;
			}

		// Fall through.
		case USI_EMIT_DATA:
			if (output_idx >= OUTPUT_BUFFER_LEN) {
				output_idx = 0;
				THEN_WAIT_FOR_START();
				return;
			}
			USIDR = output_buffer[output_idx];
			output_idx++;

			usi_state = USI_REQUEST_EMIT_REPLY;
			THEN_USI_SEND();
			break;

		case USI_REQUEST_EMIT_REPLY:
			usi_state = USI_CHECK_EMIT_REPLY;
			READ_ACK();
			break;
	}
}

void init(void) {
	int i = 0;
	for (i = 0; i < UART_BUFFER_LEN; i++) {
		uart_buffer[i] = '\0';
	}
	for (i = 0; i < OUTPUT_BUFFER_LEN; i++) {
		output_buffer[i] = '*';
	}

	// PORTB = (1 << PB0);
	// DDRB = (1 << PB0);
	PCMSK0 = (1 << PCINT7);
	GIMSK = (1 << PCIE0);

	PORTA |= (1 << PA4) | (1 << PA6) | (0 << PA7);
	DDRA |= (1 << PA4) | (0 << PA7);
	DDRA &= ~(1 << PA6);
	USICR = (1 << USISIE) | (0 << USIOIE) |
	        (1 << USIWM1) | (0 << USIWM0) |
					(1 << USICS1) | (0 << USICS0) | (0 << USICLK) |
					(0 << USITC);
	USISR = 0xF0;

	sei();
}

/*
 * 
 * $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx 
 * $GPGGA,231108.200,3752.5181,N,12215.4458,W,1,9,1.00,125.2,M,-24.9,M,,*6D
 * $GPRMC,231108.000,A,3752.5180,N,12215.4458,W,1.08,54.40,190312,,,A*46
 *
 */

uint8_t parser_status = 0;

// Parses GPRMC strings
#define DEC(i) (uart_buffer[i] - '0')
void parse(void) {
  parser_status = 0;
	int i;
	for (i = 0; i < OUTPUT_BUFFER_LEN; i++) {
		// clear the output buffer of old data
		temp_buffer[i] = 0;
	}

  if (!(uart_buffer[0] == '$' && uart_buffer[1] == 'G' && uart_buffer[2] == 'P' &&
      uart_buffer[3] == 'R' && uart_buffer[4] == 'M' && uart_buffer[5] == 'C')) { 
    parser_status |= NP_S_NO_RMC;
    return;
  }
 	
	// uint8_t hours =  DEC(7) * 10;
	//         hours += DEC(8);
	// uint8_t minutes =  DEC(9) * 10;
	//         minutes += DEC(10);
	// uint8_t seconds =  DEC(11) * 10;
	//         seconds += DEC(12);
  uint16_t hour_seconds =  DEC(9)  * 600;
           hour_seconds += DEC(10) * 60; 
           hour_seconds += DEC(11) * 10;
           hour_seconds += DEC(12);
	temp_buffer[NP_TIME_H] = (uint8_t)(hour_seconds >> 8);
	temp_buffer[NP_TIME_L] = (uint8_t)(hour_seconds & 0xFF);


  // A - okay, V - not okay
	switch (uart_buffer[18]) {
		case 'A':
			parser_status |= NP_S_A_FIX;
			break;
		case 'V':
			// fall through
		default:
			parser_status |= NP_S_V_NOFIX;
			// Force the buffer to update, especially if we've failed
			temp_buffer[NP_STATUS] = parser_status;
			return;
	}

  // latitude
  uint8_t lat_deg = DEC(20) * 10 + DEC(21);
  uint16_t lat_millimin = DEC(22) * 10000 +
                          DEC(23) * 1000  +
                          DEC(25) * 100   +
                          DEC(26) * 10    +
                          DEC(27);
	temp_buffer[NP_LAT_DEG] = lat_deg;
	temp_buffer[NP_LAT_H] = (uint8_t)(lat_millimin >> 8);
	temp_buffer[NP_LAT_L] = (uint8_t)(lat_millimin & 0xFF);

  uint8_t lon_deg = DEC(32) * 100 + DEC(33) * 10 + DEC(34);
  uint16_t lon_millimin = DEC(35) * 10000 +
                          DEC(36) * 1000  +
                          DEC(38) * 100   +
                          DEC(39) * 10    +
                          DEC(40);
	temp_buffer[NP_LON_DEG] = lon_deg;
	temp_buffer[NP_LON_H] = (uint8_t)(lon_millimin >> 8);
	temp_buffer[NP_LON_L] = (uint8_t)(lon_millimin & 0xFF);

	uint16_t speed = 0;
	i = 45; // first digit of the course
	for (; uart_buffer[i] != '.' && uart_buffer[i] != ',' && i < UART_BUFFER_LEN; i++) {
  	speed *= 10;
		speed += DEC(i);
	}
	i++;
	if (i < UART_BUFFER_LEN) {
		speed *= 10;
		speed += DEC(i);
		temp_buffer[NP_SPEED_H] = (uint8_t)(speed >> 8);
		temp_buffer[NP_SPEED_L] = (uint8_t)(speed & 0xFF);
	} else {
		parser_status |= NP_S_OVERFLOW;
		temp_buffer[NP_STATUS] = parser_status;
		return;
	}
	i++; // on the comma before course

	uint16_t course = 0;
	for (; uart_buffer[i] != ',' && i < UART_BUFFER_LEN; i++) {
	}
	i++; // on the first character of course
	for (; uart_buffer[i] != '.' && uart_buffer[i] != ',' && i < UART_BUFFER_LEN; i++) {
  	course *= 10;
		course += DEC(i);
	}
	i++;
	if (i < UART_BUFFER_LEN) {
		course *= 10;
		course += DEC(i);
		temp_buffer[NP_COURSE_H] = (uint8_t)(course >> 8);
		temp_buffer[NP_COURSE_L] = (uint8_t)(course & 0xFF);
	} else {
		parser_status |= NP_S_OVERFLOW;
		temp_buffer[NP_STATUS] = parser_status;
		return;
	}
	i++; // on the comma date course

	for (; uart_buffer[i] != ',' && i < UART_BUFFER_LEN; i++) {
	}
	i++; // on the first digit of the date
	if (i < UART_BUFFER_LEN) {
		uint8_t   day = DEC(i) * 10 + DEC(i+1);
		uint8_t month = DEC(i+2) * 10 + DEC(i+3);
		temp_buffer[NP_DATE_H] = month;
		temp_buffer[NP_DATE_L] = day;
	} else {
		parser_status |= NP_S_OVERFLOW;
		temp_buffer[NP_STATUS] = parser_status;
		return;
	}

	temp_buffer[NP_STATUS] = parser_status;
	PINB = (1 << PB0);
	return;
}

unsigned char ctoi(char c) {
	unsigned char r = 0;
	if (c - '0' < 10)
		r = c - '0';
	else if (c - 'A' < 6)
		r = c - 'A' + 10;
	return r;
}

// performs checksum by XOR-ing all of the bits
int checksum(void) {
	int8_t i = 0;
	uint8_t checksum = 0;

	if (uart_buffer[i] != '$')
		return 0;
	checksum = uart_buffer[1];

	for (i = 2; uart_buffer[i] != '*' && i < UART_BUFFER_LEN; i++) {
		checksum ^= uart_buffer[i];
	}

	if (uart_buffer[i] != '*')
		return 0;

	uint8_t check = ctoi(uart_buffer[i+1]) << 4;
	check |= ctoi(uart_buffer[i+2]);

	return (check == checksum);
}

int main(void) {
	// Turn on 8 MHz clock by setting prescaler to 0
	CLKPR = (1 << CLKPCE);
	CLKPR = (0 << CLKPCE) | (0x0 << CLKPS0);

	 DDRB = (1 << PB0);
	 PORTB = (1 << PB0);

	init();
	// PINB = (1 << PB0);

	while (1) {
		/*
		 * if we're ready to process the BBU
		 * place a lock on the uart buffer
		 * process the data here
		 * lock the registers as we're writing data
		 *
		 * unlock the uart buffer
		 */
		if (uart_done_flag) {
			if (checksum()) {
				parse();


				cli();
				int q = 0;
				for (q = 0; q < OUTPUT_BUFFER_LEN; q++) {
					output_buffer[q] = temp_buffer[q];
				}
				sei();
			}
			uart_done_flag = 0;
			PCMSK0 = (1 << PCINT7);
		}
	}

	return 0;
}
