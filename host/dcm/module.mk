BIN := dcm-host
SRCS := serial.c dcm.c
CFLAGS := $(CFLAGS) -std=c99 -pedantic -Wall -DUSE_FIXED
LDFLAGS := -lm
