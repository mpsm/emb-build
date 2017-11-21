include $(MKDIR)/mcu-kx.mk

SYSTEM= freertos
FREERTOS_PORT= kx
OBJS+= $(OBJDIR)/platform/$(CONFIG_PLATFORM)/startup_MK64F12.o
LDFLAGS+= -T $(SRCDIR)/platform/$(CONFIG_PLATFORM)/linker.ld
