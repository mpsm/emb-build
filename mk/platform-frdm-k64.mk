include $(MKDIR)/mcu-kx.mk

SYSTEM= freertos
FREERTOS_PORT= kx
OBJS+= $(TARGETOBJDIR)/platform/$(PLATFORM)/startup_MK64F12.o
LDFLAGS+= -T $(SRCDIR)/platform/$(PLATFORM)/linker.ld
