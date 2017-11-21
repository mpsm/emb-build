# compiler and linker flags
CFLAGS?= -Wall -Werror

# platform definition
ifndef PLATFORM
	PLATFORM=pc
endif
include $(MKDIR)/platform-$(PLATFORM).mk
CPPFLAGS+= -DPLATFORM_$(shell echo $(PLATFORM) | tr 'a-z' 'A-Z')=1
CPPFLAGS+= -I $(SRCDIR)/platform/$(PLATFORM)
SRCDIRS+= $(SRCDIR)/platform/$(PLATFORM)

# add system files
ifndef SYSTEM
$(error "Specify SYSTEM variable for your platform")
endif
SRCDIRS+= $(SRCDIR)/system/$(SYSTEM)
CPPFLAGS+= -DSYSTEM_CONFIG_TYPE_$(shell echo $(SYSTEM) | tr 'a-z' 'A-Z')=1
include $(MKDIR)/system-$(SYSTEM).mk

# toolchain
ifndef TOOLCHAIN
TOOLCHAIN= 
endif
override CC= $(TOOLCHAIN)-gcc
override AS= $(CC) -x assembler-with-cpp

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
