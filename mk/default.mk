# platform definition
ifndef CONFIG_PLATFORM
	CONFIG_PLATFORM=pc
	CPPFLAGS+= -DPLATFORM_$(shell echo $(CONFIG_PLATFORM) | tr 'a-z' 'A-Z')=1
endif

include $(MKDIR)/platform-$(CONFIG_PLATFORM).mk

# compiler and linker flags
CFLAGS?= -Wall -Werror
LDFLAGS?=

# debug options
ifndef CONFIG_DEBUG
	CONFIG_DEBUG=0
endif
ifeq ($(CONFIG_DEBUG), 1)
	CFLAGS+= -O0 -g
	CPPFLAGS+= -DDEBUG
else
	CFLAGS+= -Os
endif

# toolchain
override CC= gcc
