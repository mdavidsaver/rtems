##
## $Id$
##

ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am
include $(top_srcdir)/../../bsp.am

SUBDIRS = . bootloader

include_bspdir = $(includedir)/bsp

dist_project_lib_DATA = bsp_specs

include_HEADERS = include/bsp.h
include_HEADERS += ../../shared/include/tm27.h

nodist_include_HEADERS = include/bspopts.h
nodist_include_bsp_HEADERS = ../../shared/include/bootcard.h
DISTCLEANFILES = include/bspopts.h
noinst_PROGRAMS =

nodist_include_HEADERS += ../../shared/include/coverhd.h

noinst_LIBRARIES = libbspstart.a
libbspstart_a_SOURCES = start/start.S
project_lib_DATA = start.$(OBJEXT)

libbspstart_a_SOURCES += ../../powerpc/shared/start/rtems_crti.S
project_lib_DATA += rtems_crti.$(OBJEXT)

dist_project_lib_DATA += startup/linkcmds

# startup
libbsp_a_SOURCES = ../../shared/bspclean.c ../../shared/bsplibc.c \
    ../../shared/bsppredriverhook.c ../../shared/bsppretaskinghook.c \
    ../../shared/bspgetworkarea.c ../../shared/bsppost.c \
    ../../shared/bootcard.c ../../shared/sbrk.c \
    ../../shared/gnatinstallhandler.c \
    ../../powerpc/shared/startup/panic.c \
    startup/cmain.c startup/bspstart.c

# pclock
libbsp_a_SOURCES += ../shared/clock/clock.c

# ToD
include_HEADERS += ../../shared/tod.h
libbsp_a_SOURCES += ../../shared/tod.c ../../powerpc/shared/tod/todcfg.c

# console
include_bsp_HEADERS = ../../powerpc/shared/console/consoleIo.h \
	../../powerpc/shared/console/uart.h

# console
libbsp_a_SOURCES += \
    ../../powerpc/shared/console/uart.c \
    ../../powerpc/shared/console/console.c \
    ../../powerpc/shared/console/inch.c \
    ../../powerpc/shared/console/consoleIo.h \
    ../../powerpc/shared/console/keyboard.h \
    ../../powerpc/shared/console/uart.h \
    console/console-io.c

# irq
include_bsp_HEADERS += ../../powerpc/shared/irq/irq.h \
	../../../libcpu/@RTEMS_CPU@/@exceptions@/bspsupport/irq_supp.h \
    ../../shared/include/irq-generic.h

libbsp_a_SOURCES += irq/irq_init.c \
	../../powerpc/shared/irq/i8259.c \
	../../shared/src/irq-generic.c \
	../../shared/src/irq-legacy.c

include_bsp_HEADERS += ../../powerpc/shared/pci/pci.h
# pci
libbsp_a_SOURCES += ../../powerpc/shared/pci/pci.c \
    ../../powerpc/shared/pci/generic_clear_hberrs.c \
    ../../powerpc/shared/pci/pcifinddevice.c


# network
if HAS_NETWORKING
ne2000_CPPFLAGS = -D__INSIDE_RTEMS_BSD_TCPIP_STACK__
noinst_PROGRAMS += ne2000.rel
ne2000_rel_SOURCES = network/ne2000.c
ne2000_rel_CPPFLAGS = $(AM_CPPFLAGS) $(ne2000_CPPFLAGS)
ne2000_rel_LDFLAGS = $(RTEMS_RELLDFLAGS)

# This driver needs to be reworked for the BSD stack.
# We only install wd80x3.h if HAS_NETWORKING was defined
include_HEADERS += network/wd80x3.h

endif

libbsp_a_SOURCES += $(include_bsp_HEADERS)

noinst_LIBRARIES += libbsp.a

libbsp_a_LIBADD = ../../../libcpu/@RTEMS_CPU@/shared/cpuIdent.rel \
    ../../../libcpu/@RTEMS_CPU@/shared/cache.rel \
    ../../../libcpu/@RTEMS_CPU@/shared/stack.rel \
    ../../../libcpu/@RTEMS_CPU@/@exceptions@/rtems-cpu.rel \
    ../../../libcpu/@RTEMS_CPU@/@exceptions@/exc_bspsupport.rel \
    ../../../libcpu/@RTEMS_CPU@/mpc6xx/mmu.rel \
    ../../../libcpu/@RTEMS_CPU@/mpc6xx/timer.rel

if HAS_NETWORKING
libbsp_a_LIBADD += ne2000.rel
endif

EXTRA_DIST = times

include $(srcdir)/preinstall.am
include $(top_srcdir)/../../../../automake/local.am
