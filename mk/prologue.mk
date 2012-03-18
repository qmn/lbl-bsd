# Reset variables
ifeq ($(strip $(ARCH)),avr)
CFLAGS := -g -Os -Wall
OBJCOPYFLAGS := -j .text -j .data -O ihex
else
CFLAGS := -g -O2
OBJCOPYFLAGS :=
endif
AFLAGS :=
LDFLAGS :=
INCDIR :=

BIN :=
SRCS :=

SUBDIR :=
