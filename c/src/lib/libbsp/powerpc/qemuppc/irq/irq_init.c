/*===============================================================*\
| Project: RTEMS generic MPC83xx BSP                              |
+-----------------------------------------------------------------+
|                    Copyright (c) 2007                           |
|                    Embedded Brains GmbH                         |
|                    Obere Lagerstr. 30                           |
|                    D-82178 Puchheim                             |
|                    Germany                                      |
|                    rtems@embedded-brains.de                     |
+-----------------------------------------------------------------+
| The license and distribution terms for this file may be         |
| found in the file LICENSE in this distribution or at            |
|                                                                 |
| http://www.rtems.com/license/LICENSE.                           |
|                                                                 |
+-----------------------------------------------------------------+
| this file integrates the IPIC irq controller                    |
\*===============================================================*/

#include <rtems.h>

#include <libcpu/powerpc-utility.h>

#include <bsp.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>
#include <bsp/vectors.h>

//#define ISR_DEBUG

/* QEMU places the host bridge IACK register differently than other PREP hosts */
#define RAVEN_INTR_ACK_REG ((void*)0xbffffff0)

/* PIC IRQ connections
 *
 * Master PIC
 * 0 - timer
 * 1 - i8042 (keyboard)
 * 2 - Slave PIC
 * 3 - Serial1
 * 4 - Serial0
 * 5 -
 * 6 - FDC
 * 7 - Parallel
 *
 * Slave PIC
 * 8 - m48t59
 * 9 - NIC0, PCI pins A, C
 * 10- NIC1
 * 11- NIC2, PCI pins B, D
 * 12- i8042 (mouse)
 * 13- IDE
 * 14-
 * 15-
 *
 * Note: When comparing with openpic_i8259_irq.c, the QEMU PREP machine
 *       has the raven host bridge, a pair of i8259 PICs, but no openpic.
 */

#ifdef ISR_DEBUG
static
rtems_i8259_masks xBSP_irq_active_at_i8259s(void)
{
    rtems_i8259_masks isr, cache=i8259s_cache;

    /* Read PIC registers
     *  0x0a - IRR
     *  0x0b - ISR
     */
    outb(0x0a, PIC_MASTER_COMMAND_IO_PORT);
    isr = inb(PIC_MASTER_COMMAND_IO_PORT);

    if(isr&0x4) {
        rtems_i8259_masks slave;
        /* need to fetch for slave PIC as well */
        outb(0x0a, PIC_SLAVE_COMMAND_IO_PORT);
        slave = inb(PIC_SLAVE_COMMAND_IO_PORT);
        slave <<= 8;
        isr |= slave;
    }

    isr &= ~cache;

    return isr;
}
#endif /* ISR_DEBUG */

int C_dispatch_irq_handler( BSP_Exception_frame *frame, unsigned exception_number)
{
    uint8_t active;
#ifdef ISR_DEBUG
    rtems_i8259_masks isr = xBSP_irq_active_at_i8259s();
#endif /* ISR_DEBUG */

    /* the many ways of acknowledging interrupts. */
#ifdef RAVEN_INTR_ACK_REG
    /* ask the host bridge to do it for us. */
    active = in_8(RAVEN_INTR_ACK_REG);

#ifdef ISR_DEBUG
    /* cross check against value read from PICs */
    if(!((1<<active)&isr)) {
        printk("Mismatch %02x %02x\n", isr, (1<<active));
        BSP_panic("oops");
    }
    if(!isr)
        BSP_panic("PPC Ext exception will PIC reports nothing!\n");
#endif /* ISR_DEBUG */

#else
    /* poll the PICs to find out */
    /* OCW3 enable poll */
    outb(0x0c, PIC_MASTER_COMMAND_IO_PORT);
    active = inb(PIC_MASTER_COMMAND_IO_PORT);

    if(!(0x80&active)) {
        printk("spurious interrupt1 %02x\n", active);
        return 0;

    } else if(active==0x82) {
        outb(0x0c, PIC_SLAVE_COMMAND_IO_PORT);
        active = inb(PIC_SLAVE_COMMAND_IO_PORT);
        if(!(0x80&active)) {
            printk("spurious interrupt2 %02x\n", active);
            return 0;
        }
        active = (active&0x07) + 8;
    } else
        active &= 0x07;
#endif

    BSP_irq_disable_at_i8259s(active);
    BSP_irq_ack_at_i8259s(active);

    bsp_interrupt_handler_dispatch(active);

    BSP_irq_enable_at_i8259s(active);

    return 0;
}

/*
 * functions to enable/disable a source at the ipic
 */
rtems_status_code bsp_interrupt_vector_enable( rtems_vector_number irqnum)
{
#ifdef ISR_DEBUG
    printk("IV Enable %d\n", (int)irqnum);
#endif /* ISR_DEBUG */
    if(BSP_irq_enable_at_i8259s(irqnum))
        return RTEMS_IO_ERROR;
	return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_interrupt_vector_disable( rtems_vector_number irqnum)
{
#ifdef ISR_DEBUG
    printk("IV Disable %d\n", (int)irqnum);
#endif /* ISR_DEBUG */
    if(BSP_irq_disable_at_i8259s(irqnum)==-1)
        return RTEMS_IO_ERROR;
	return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_interrupt_facility_initialize(void)
{
#ifdef ISR_DEBUG
    printk("In bsp_interrupt_facility_initialize()\n");
#endif /* ISR_DEBUG */
    BSP_i8259s_init();

	/* Install exception handler */
    if (ppc_exc_set_handler( ASM_EXT_VECTOR, C_dispatch_irq_handler)) {
		return RTEMS_IO_ERROR;
	}

	return RTEMS_SUCCESSFUL;
}

void bsp_interrupt_handler_default( rtems_vector_number vector)
{
    printk( "Spurious interrupt: %d\n", (int)vector);
}
