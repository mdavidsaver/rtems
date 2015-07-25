## Process this file with autoconf to produce a configure script.
##
## $Id$

AC_PREREQ([2.68])
AC_INIT([rtems-c-src-lib-libbsp-h8300-h8sim],[_RTEMS_VERSION],[http://www.rtems.org/bugzilla])
AC_CONFIG_SRCDIR([bsp_specs])
RTEMS_TOP(../../../../../..)

RTEMS_CANONICAL_TARGET_CPU
AM_INIT_AUTOMAKE([no-define nostdinc foreign 1.11.1])
RTEMS_BSP_CONFIGURE

RTEMS_PROG_CC_FOR_TARGET
RTEMS_CANONICALIZE_TOOLS
RTEMS_PROG_CCAS

RTEMS_CHECK_NETWORKING

RTEMS_BSPOPTS_SET([BSP_PRESS_KEY_FOR_RESET],[*],[0])
RTEMS_BSPOPTS_HELP([BSP_PRESS_KEY_FOR_RESET],
[If set to !0, print a message and wait until pressed before resetting
 board when application exits.])

RTEMS_BSPOPTS_SET([BSP_RESET_BOARD_AT_EXIT],[*],[1])
RTEMS_BSPOPTS_HELP([BSP_RESET_BOARD_AT_EXIT],
[If set to !0, reset the board when the application exits.])
RTEMS_BSP_CLEANUP_OPTIONS(0, 0)

AM_CONDITIONAL(HAS_NETWORKING,[test "$HAS_NETWORKING" = "yes"])

# Explicitly list all Makefiles here
AC_CONFIG_FILES([Makefile bootloader/Makefile])

RTEMS_PPC_EXCEPTIONS


AC_OUTPUT
