# Generate object list
OBJS := $(addsuffix .o,$(basename $(filter %.c %.cpp %.S,$(SRCS))))
OBJS := $(addprefix $(CWD)/,$(strip $(OBJS)))

BIN := $(addprefix $(CWD)/,$(BIN))

# Set target-specific variables
$(BIN) : CFLAGS := $(strip $(CFLAGS))
$(BIN) : AFLAGS := $(strip $(AFLAGS))
$(BIN) : LDFLAGS := $(strip $(LDFLAGS))
$(BIN) : INCDIR := $(addprefix $(CWD)/,$(strip $(INCDIR)))
$(BIN) : OBJCOPYFLAGS := $(strip $(OBJCOPYFLAGS))

# Set dependencies
OUT := $(strip $(basename $(filter %.hex,$(BIN))))
ifeq ($(OUT),)
$(BIN) : $(OBJS)
else
$(OUT) : $(OBJS)
$(BIN) : $(OUT)
endif

all :: $(BIN)

CLEANFILES += $(strip $(OBJS) $(OUT) $(BIN))

# Recurse into subdirectories
$(foreach sd,$(SUBDIR),$(eval $(call _include_subdir,$(sd))))

