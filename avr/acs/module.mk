BIN := main.hex
SRCS := dcm.c fix.c timer.c usart.c i2c.c debug.c
CFLAGS := $(CFLAGS) -std=c99 -mmcu=atmega328 -DF_CPU=16000000UL -DUSART_BAUD=38400
