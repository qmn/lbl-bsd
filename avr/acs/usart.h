/**
 * \file  usart.h
 * \author  Albert Ou <a_ou@berkeley.edu>
 */
#ifndef LBL_BSD_USART_H
#define LBL_BSD_USART_H

#include <stdint.h>

/** Default baud rate */
#ifndef USART_BAUD
#define USART_BAUD 9600
#endif /* USART_BAUD */

/**
 * \brief  Initializes USART0 hardware.
 * \details
 * The baud rate is statically determined by the value of
 * \c USART_BAUD at compile time.
 */
extern void usart_init(void);

/**
 * \brief  Reads \a len bytes from the receive queue for USART0
 *         into the buffer pointed to by \a buf.
 * \details
 * If compiled with non-blocking behavior (\c USART_RX_NONBLOCK),
 * function will return immediately if the receive queue has fewer
 * than \a len bytes available.
 * Otherwise, the function will block until the receive queue has
 * been filled sufficiently to read all values.
 *
 * \param[out]  buf  pointer to the destination buffer
 * \param[in]   len  number of bytes to read
 *
 * \retval  0  Successful completion
 * \retval  1  Receive queue is insufficiently full and non-blocking
 *             behavior is enabled
 *
 * \pre  usart_init() must be called before this function.
 * \pre  Interrupts must be enabled through the global interrupt mask.
 */
extern uint8_t usart_read(void *buf, uint8_t len);

/**
 * \brief  Writes \a len bytes from the buffer pointed to by \a buf
 *         into the transmit queue for USART0.
 * \details
 * If compiled with non-blocking behavior (\c USART_TX_NONBLOCK),
 * the function will return immediately if \a len is greater than
 * the current free capacity of the transmit queue.
 * Otherwise, the function will block until the transmit queue has
 * been flushed sufficiently to write all values.
 *
 * \param[in]  buf  pointer to the source buffer
 * \param[in]  len  number of bytes to write
 *
 * \retval  0  Successful completion
 * \retval  1  Transmit queue has insufficient capacity and non-blocking
 *             behavior is enabled
 *
 * \pre  usart_init() must be called before this function.
 * \post  Interrupts will be enabled through the global interrupt mask
 *        in order to begin transmission.
 */
extern uint8_t usart_write(const void *buf, uint8_t len);

#endif /* LBL_BSD_USART_H */
