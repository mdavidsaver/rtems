/*
 *  This include file contains some definitions specific to the
 *  qemu powerpc Prep simulator
 *
 *  COPYRIGHT (c) 1989-2009.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */

#ifndef _BSP_H
#define _BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <bspopts.h>

/* Don't really have u-boot.
 * QEMU gives us some of the same information
 * via. an emulated NVRAM device
 */
#define HAS_UBOOT 1
#define HAS_QEMU_NVRAM 1

#include <rtems.h>
#include <rtems/iosupp.h>
#include <rtems/console.h>
#include <rtems/clockdrv.h>

#include <libcpu/io.h>

#define _IO_BASE PREP_ISA_IO_BASE
#define	_ISA_MEM_BASE		PREP_ISA_MEM_BASE
/* address of our ram on the PCI bus   */
#define	PCI_DRAM_OFFSET		0
/* offset of pci memory as seen from the CPU */
#define PCI_MEM_BASE		PREP_ISA_MEM_BASE
/* where (in CPU addr. space) does the PCI window start */
#define PCI_MEM_WIN0		0

#define outport_byte(port,value) outb(value,port)
#define outport_word(port,value) outw(value,port)
#define outport_long(port,value) outl(value,port)

#define inport_byte(port,value) (value = inb(port))
#define inport_word(port,value) (value = inw(port))
#define inport_long(port,value) (value = inl(port))

#define BSP_CONSOLE_PORT			BSP_UART_COM1
#define BSP_UART_BAUD_BASE			115200

#define BSP_UART_IOBASE_COM1 ((_IO_BASE)+0x3f8)
#define BSP_UART_IOBASE_COM2 ((_IO_BASE)+0x2f8)

/*
 *  Convert decrementer value to tenths of microseconds (used by shared timer
 *  driver).
 */
#define BSP_Convert_decrementer( _value ) \
  ((int) (((_value) * 10) / bsp_clicks_per_usec))

#if 0
/* support for simulated clock tick */
Thread clock_driver_sim_idle_body(uintptr_t);
#define BSP_IDLE_TASK_BODY clock_driver_sim_idle_body
#endif

/* Information from the NVRAM */
typedef struct {
    unsigned long	bi_memstart;	/* start of DRAM memory */
    unsigned long	bi_memsize;	/* size	 of DRAM memory in bytes */
} bd_t;

struct rtems_bsdnet_ifconfig;

int
rtems_ne_driver_attach (struct rtems_bsdnet_ifconfig *config, int attach);
#define RTEMS_BSP_NETWORK_DRIVER_NAME      "ne1"
#define RTEMS_BSP_NETWORK_DRIVER_ATTACH    rtems_ne_driver_attach

#ifdef __cplusplus
}
#endif

#endif
