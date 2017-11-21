include $(MKDIR)/default.mk

# add app files
ifndef APP
$(error "Specify application name.")
endif
SRCDIRS+= $(SRCDIR)/apps/$(APP)
CPPFLAGS+= -DAPP="$(APP)" -I $(SRCDIR)/apps/$(APP)

# flags
CPPFLAGS+= -I $(SRCDIR) -MMD -MP -MT $@ -MT $(@:.o=.d) -MF $(@:.o=.d)

# specify target
TARGET:= $(BINDIR)/$(APP)-$(PLATFORM)

# target working directory
TARGETOBJDIR:= $(OBJDIR)/$(PLATFORM)/$(APP)

# source, object and dep files
SRCS+= $(foreach srcdir, $(SRCDIRS), $(wildcard $(srcdir)/*.c))
OBJS+= $(patsubst $(SRCDIR)/%, $(TARGETOBJDIR)/%, $(SRCS:.c=.o))
DEPS= $(OBJS:%.o=%.d)

# object directories
OBJDIRS= $(shell echo $(foreach objfile, $(OBJS), $(dir $(objfile))) | tr ' ' '\n' | sort | uniq)

# build rules
all: $(TARGET)

$(TARGET): $(OBJS) | $(BINDIR)
	@echo "### Linking $@"
	$(CC) $(CCFLAGS) $^ $(LDFLAGS) -o $@

$(TARGETOBJDIR)/%.o: $(SRCDIR)/%.s | $(OBJDIRS)
	@echo "### Assembling $@"
	$(AS) $(CPPFLAGS) $(CCFLAGS) $(ASFLAGS) -o $@ -c $<

$(TARGETOBJDIR)/%.o: $(SRCDIR)/%.c | $(OBJDIRS)
	@echo "### Building $@"
	$(CC) $(CPPFLAGS) $(CCFLAGS) $(CFLAGS) -o $@ -c $<

$(BINDIR) $(OBJDIRS):
	mkdir -p $@

clean:
	rm -f $(OBJS) $(TARGET)

.PRECIOUS: $(DEPS)
.PHONY: clean all
