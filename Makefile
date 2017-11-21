# bootstrap paths
TOPDIR:= $(dir $(lastword $(MAKEFILE_LIST)))
include $(TOPDIR)/mk/paths.mk

# flags for sub make
MAKE?= make
MKFLAGS= --no-print-directory -r -R

# scan available apps
APPS:= $(patsubst app-%.mk,%,$(notdir $(wildcard $(MKDIR)/app-*.mk)))
ifeq ($(APPS),)
	$(error Apps not defined)
endif 
ALLAPSMAKE= @for app in $(APPS); do echo "make APP=$${app} $(1)";$(MAKE) $(MKFLAGS) -f $(MKDIR)/app-$${app}.mk APP=$${app} $(1); done
APPMAKE= @$(MAKE) $(MKFLAGS) -f $(MKDIR)/app-$(APP).mk $@

# export all variables to the submake
export

# first rule to call if no rule is specified
all:
	$(call ALLAPSMAKE,all)
	
force:
	@true

# in case the target is specified directly
$(OBJDIR) $(BINDIR):
	mkdir -p $@

$(BINDIR)/%: force | $(BINDIR)
	$(eval APP:=$(firstword $(subst -, ,$*)))
	$(call APPMAKE,$@)

$(OBJDIR)/%: force | $(OBJDIR)
	$(eval APP:=$(firstword $(subst /, ,$*)))
	$(call APPMAKE,$@)

# if APP is defined call app's rules - else call rule for each app
ifdef APP
%:
	$(call APPMAKE,$@)
else
%:
	$(call ALLAPSMAKE, $@)
endif

# general rules
distclean:
	rm -rf $(BINDIR) $(OBJDIR)

.PHONY: all distclean
