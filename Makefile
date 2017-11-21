# bootstrap paths
TOPDIR:= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
include $(TOPDIR)/mk/paths.mk

# scan available apps
APPS:= $(patsubst app-%.mk,%,$(notdir $(wildcard $(MKDIR)/app-*.mk)))
ifeq ($(APPS),)
	$(error Apps not defined)
endif 
$(info "Available apps: $(APPS)")

# export all variables to the submake
export

# in case the target is specified directly
$(OBJDIR):
	echo test
	make -f mk/app-$(APP).mk APP=$(firstword $(subst /, ,$*)) $@

# if APP is defined
#ifdef APP
#TODO: APP in APPS?
#%:
#	make -f $(MKDIR)/app-$(APP).mk $@
#else
#%:
#	for app in $(APPS); do make -f $(MKDIR)/app-$${app}.mk APP=$${app} $@; done
#endif



# general rules
distclean:
	rm -rf $(BINDIR) $(OBJDIR)


.PHONY: all	
