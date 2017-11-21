# in case mk file is called directly
ifndef TOPDIR
TOPDIR:= $(abspath $(dir $(lastword $(MAKEFILE_LIST)))../)
endif

# the TOPDIR's trailing '/' is intentional
SRCDIR:=  $(TOPDIR)src
MKDIR:=   $(TOPDIR)mk
BINDIR:=  $(TOPDIR)bin
OBJDIR:=  $(TOPDIR)obj

# level 2 directories
APPSDIR:= $(SRCDIR)/apps
