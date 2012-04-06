/* Bit-banged UART interface */
#define BAUD 9600
#define CLOCK 1000000
#define PRESCALE 8
#define TIMER_WAIT 0x68
// ((BAUD / CLOCK) / PRESCALE)
#define TIMER_WAIT_HALF 0x08
//(TIMER_WAIT / 2)
#define UART_BUFFER_LEN 80
#define OUTPUT_BUFFER_LEN 16

#define BBUART_START      0
#define BBUART_WAIT_HALF  1
#define BBUART_READ       2

/* States for the USI TWI interface */
#define USI_START               0x1
#define USI_CHECK_ADDR          0x3
#define USI_CAPTURE_REGISTER    0x5
#define USI_WRITE_REGISTER      0x7
#define USI_REQUEST_CAPTURE     0x9
#define USI_CHECK_EMIT_REPLY    0xB
#define USI_EMIT_DATA           0xD
#define USI_REQUEST_EMIT_REPLY  0xF
#define USI_BEFORE_CAPTURE_REGISTER 0x11

/* Helper functions for the USI... */
#define SEND_ACK()                                                      \
{                                                                       \
	USIDR = 0;                                                            \
	DDRA |= (1 << PA6);                                                   \
  USISR = (0 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | \
	        (0xE << USICNT0);                                             \
}

#define READ_ACK()                                                      \
{                                                                       \
	DDRA &= ~(1 << PA6);                                                  \
	USIDR = 0;                                                            \
	USISR = (0 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | \
	        (0xE << USICNT0);                                             \
} 

#define THEN_USI_SEND()                                                 \
{                                                                       \
	DDRA |= (1 << PA6);                                                   \
	USISR = (0 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | \
	        (0x0 << USICNT0);                                             \
}

#define THEN_USI_READ()                                                 \
{                                                                       \
	DDRA &= ~(1 << PA6);                                                  \
	USISR = (0 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | \
	        (0x0 << USICNT0);                                             \
}

#define THEN_WAIT_FOR_START()                                           \
{                                                                       \
	USICR = (1 << USISIE) | (0 << USIOIE) |                               \
	 			  (1 << USIWM1) | (0 << USIWM0) |                               \
					(1 << USICS1) | (0 << USICS0) | (0 << USICLK) |               \
					(0 << USITC);                                                 \
	USISR = (0 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | \
	 			  (0x0 << USICNT0);                                             \
	usi_state = USI_START;                                                \
}
