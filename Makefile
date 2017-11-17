TOPDIR:= $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
CURDIR:= $(shell pwd)

MKDIR:=  $(TOPDIR)/mk
SRCDIR:= $(TOPDIR)/src
SRCDIRS= $(SRCDIR)

include $(MKDIR)/default.mk

BINDIR:= $(TOPDIR)/bin
OBJDIR:= $(TOPDIR)/obj/$(ARCH)/$(CONFIG_PLATFORM)

SRCS+= $(foreach srcdir, $(SRCDIRS), $(wildcard $(srcdir)/*.c))
OBJS+= $(patsubst $(SRCDIR)/%, $(OBJDIR)/%, $(SRCS:.c=.o))
DEPS= $(OBJS:%.o=%.d)
OBJDIRS= $(shell echo $(foreach srcfile, $(OBJS), $(dir $(srcfile))) | tr ' ' '\n' | sort | uniq)

CPPFLAGS+= -I $(SRCDIR) -MMD -MP -MT $@ -MT $(@:.o=.d) -MF $(@:.o=.d)

TARGETNAME?= firmware
TARGET= $(BINDIR)/$(TARGETNAME)

# rules

all: $(TARGET)

$(TARGET): $(OBJS) | $(BINDIR)
	@echo "### Linking $@"
	$(CC) $^ $(LDFLAGS) -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.c | $(OBJDIRS)
	@echo "### Building $@"
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ -c $<

$(BINDIR) $(OBJDIRS):
	mkdir -p $@

-include $(DEPS)

clean:
	rm -f $(OBJS)

distclean:
	rm -rf $(TOPDIR)/obj $(TOPDIR)/bin

.PRECIOUS: $(DEPS)
.PHONY: depend distclean clean all

