BIN := main.hex
SRCS := tick.c usart.c i2c.c imu.c
CFLAGS := $(CFLAGS) -std=c99 -mmcu=atmega328 -DF_CPU=16000000UL -DUSART_BAUD=38400
