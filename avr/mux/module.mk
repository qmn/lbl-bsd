BIN := mux.hex
SRCS := mux.S sfr_t44a.inc
CFLAGS := $(CFLAGS) -mmcu=attiny44a
AFLAGS := -Wa,-I,$(CWD),--defsym,F_CPU=1000000
