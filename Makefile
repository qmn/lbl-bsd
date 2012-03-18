.DEFAULT_GOAL := all
SHELL := /bin/sh

define _find_parent
$(shell unset -v prev ; \
	while [ ! -e '$(1)' ] && [ "$${prev}" != "$${PWD}" ] ; do \
		prev="$${PWD}" ; cd .. ; \
	done ; [ -e '$(1)' ] && echo "$${PWD}")
endef

ifndef MK
MK := $(call _find_parent,mk/core.mk)/mk
endif
ifndef BASE
BASE := $(call _find_parent,config.mk)
endif

ifneq ($(strip $(BASE)),)
# global configuration
include $(BASE)/config.mk
endif

include $(MK)/core.mk
$(eval $(call _include_subdir,$(shell pwd)))

