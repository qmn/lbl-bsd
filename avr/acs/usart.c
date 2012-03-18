/**
 * \file usart.c
 */
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "usart.h"

/* Enable/disable non-blocking behavior */
#define USART_RX_NONBLOCK
#define USART_TX_NONBLOCK

/** Default size of receive queue in bytes */
#ifndef USART_RX_LEN
#define USART_RX_LEN 0x80
#endif /* USART_RX_LEN */
/** Default size of transmit queue in bytes */
#ifndef USART_TX_LEN
#define USART_TX_LEN 0x80
#endif /* USART_TX_LEN */

#define RING_RX_MASK ((USART_RX_LEN) - 1)
#define RING_TX_MASK ((USART_TX_LEN) - 1)

/* Validation */
#if (USART_RX_LEN & RING_RX_MASK)
#error "USART_RX_LEN value not a power of 2"
#elif (USART_RX_LEN < 0x1) || (USART_RX_LEN > 0x100)
#error "USART_RX_LEN value out of range; 1 <= USART_RX_LEN <= 256"
#endif /* USART_RX_LEN */
#if (USART_TX_LEN & RING_TX_MASK)
#error "USART_TX_LEN value not a power of 2"
#elif (USART_TX_LEN < 0x1) || (USART_TX_LEN > 0x100)
#error "USART_TX_LEN value out of range; 1 <= USART_TX_LEN <= 256"
#endif /* USART_TX_LEN */

static volatile uint8_t rx_data[USART_RX_LEN];
static volatile uint8_t tx_data[USART_TX_LEN];

static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;
#ifdef USART_RX_NONBLOCK
static volatile uint8_t rx_pend = 0;
#endif
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;
#ifdef USART_TX_NONBLOCK
static volatile uint8_t tx_free = (USART_TX_LEN - 1);
#endif

/* Baud rate generator (20.3.1) */
#define USART_UBRR ((((F_CPU) / (USART_BAUD)) >> 4) - 1)

void usart_init(void) {
	/* USART Baud Rate Registers (20.10.5) */
	UBRR0H = (uint8_t)(USART_UBRR >> 8);
	UBRR0L = (uint8_t)(USART_UBRR);
	/* USART Control and Status Register 0 C (20.10.4)
	 * Asynchronous USART: UMSEL01 = 0, UMSEL00 = 0
	 * Parity disable: UPM01 = 0, UPM00 = 0
	 * 1 stop bit: USBS0 = 0
	 * 8-bit character: UCSZ02 = 0 (UCSR0B), UCSZ01 = 1, UCSZ01 = 1
	 */
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	/* USART Control and Status Register 0 B (20.10.3)
	 * Receiver enable: RXEN0 = 1
	 * Transmitter enable: TXEN0 = 1
	 * RX Complete Interrupt enable: RXCIE0 = 1
	 */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

/**
 * \brief  USART RX Complete interrupt service routine
 */
ISR(USART_RX_vect) {
	uint8_t tail;
	tail = (rx_tail + 1);
#if (RING_RX_MASK != 0xFF)
	tail &= RING_RX_MASK;
#endif
	if (tail != rx_head) {
		rx_data[tail] = UDR0;
		rx_tail = tail;
#ifdef USART_RX_NONBLOCK
		rx_pend++;
#endif
	}
}

/**
 * \brief  USART Data Register Empty interrupt service routine
 */
ISR(USART_UDRE_vect) {
	uint8_t head;
	if ((head = tx_head) != tx_tail) {
		head++;
#if (RING_TX_MASK != 0xFF)
		head &= RING_TX_MASK;
#endif
		UDR0 = tx_data[head];
		tx_head = head;
#ifdef USART_TX_NONBLOCK
		tx_free--;
#endif
	} else {
		UCSR0B &= ~(1 << UDRIE0);
	}
}

uint8_t usart_read(void *buf, uint8_t len) {
	uint8_t *dst;
	uint8_t head;
#ifdef USART_RX_NONBLOCK
	if (len > rx_pend)
		return 1;
#endif
	for (dst = buf, head = rx_head; len > 0; len--) {
#ifndef USART_RX_NONBLOCK
		do {} while (head == rx_tail);
#endif
		head++;
#if (RING_RX_MASK != 0xFF)
		head &= RING_RX_MASK;
#endif
		*dst++ = rx_data[head];
		rx_head = head;
#ifdef USART_RX_NONBLOCK
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			rx_pend--;
		}
#endif
	}
	return 0;
}

uint8_t usart_write(const void *buf, uint8_t len) {
	const uint8_t *src;
	uint8_t tail;
#ifdef USART_TX_NONBLOCK
	if (len > tx_free)
		return 1;
#endif
	for (src = buf, tail = tx_tail; len > 0; len--) {
		tail++;
#if (RING_TX_MASK != 0xFF)
		tail &= RING_TX_MASK;
#endif
#ifndef USART_TX_NONBLOCK
		do {} while (tail == tx_head);
#endif
		tx_data[tail] = *src++;
		tx_tail = tail;
#ifdef USART_TX_NONBLOCK
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			tx_free++;
		}
#endif
	}
	UCSR0B |= (1 << UDRIE0);
	return 0;
}
