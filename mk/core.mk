define _include_subdir
ifneq ($(strip $(CWD)),)
dir_stack := $(CWD) $(dir_stack)
CWD := $(CWD)/$(1)
else
dir_stack :=
CWD := $(1)
endif
include $(MK)/prologue.mk
include $(addsuffix /module.mk,$$(CWD))
include $(MK)/epilogue.mk
CWD := $$(firstword $$(dir_stack))
dir_stack := $$(wordlist 2,$$(words $$(dir_stack)),$$(dir_stack))
endef

.SUFFIXES:
.SUFFIXES: .c .o .elf .hex

##
# Toolchain configuration
##

ifeq ($(strip $(ARCH)),avr)
CC := avr-gcc
LD := avr-ld
OBJCOPY := avr-objcopy
else
CC := cc
LD := ld
endif

##
# Rules
##

%.o : %.c
	$(CC) $(CFLAGS) $(addprefix -I,$(INCDIR)) -c -o $@ $<
%.o : %.S
	$(CC) $(CFLAGS) $(AFLAGS) $(addprefix -I,$(INCDIR)) -c -o $@ $<
%:
	$(CC) $(LDFLAGS) -o $@ $^
%.hex: %
	$(OBJCOPY) $(OBJCOPYFLAGS) $< $@

.PHONY: all clean
CLEANFILES :=
clean :
	rm -f $(CLEANFILES)
