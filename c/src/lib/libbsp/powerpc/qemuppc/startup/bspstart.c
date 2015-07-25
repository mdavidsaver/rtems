/*
 *  This set of routines starts the application.  It includes application,
 *  board, and monitor specific initialization and configuration.
 *  The generic CPU dependent initialization has been performed
 *  before any of these are invoked.
 *
 *  COPYRIGHT (c) 1989-2008.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */

#include <string.h>
#include <fcntl.h>

#include <libcpu/bat.h>
#include <libcpu/spr.h>
#include <libcpu/powerpc-utility.h>

#include <bsp.h>
#include <bsp/irq.h>
#include <bsp/pci.h>
#include <bsp/vectors.h>
#include <bsp/bootcard.h>
#include <bsp/irq-generic.h>

#define pci BSP_pci_configuration

extern const pci_config_access_functions pci_indirect_functions;

extern bd_t bsp_uboot_board_info;

static
void config_pci_devices(void);
static
void setup_watchdog(void);

/*
 * CPU Bus Frequency
 */
unsigned int BSP_bus_frequency;

/*
 *  Driver configuration parameters
 */
uint32_t   bsp_clicks_per_usec;

/*
 * Memory on this board.
 */
extern char bsp_interrupt_stack_start[];
extern char bsp_interrupt_stack_end[];
extern char bsp_interrupt_stack_size[];

/* Default decrementer exception handler */
static int default_decrementer_exception_handler( BSP_Exception_frame *frame, unsigned number)
{
  ppc_set_decrementer_register(UINT32_MAX);

  return 0;
}

/*
 *  bsp_start
 *
 *  This routine does the bulk of the system initialization.
 */

void bsp_start( void )
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  uintptr_t intrStackStart;
  uintptr_t intrStackSize;

  /*
   * Note we can not get CPU identification dynamically, so
   * force current_ppc_cpu.
   */
  current_ppc_cpu = PPC_PSIM;

  /*
   *  initialize the device driver parameters
   * assume we are running with 20MHz bus
   * this should speed up some tests :-)
   */
  BSP_bus_frequency        = 100;
  bsp_clicks_per_usec      = BSP_bus_frequency;

  /*
   * Initialize the interrupt related settings.
   */
  intrStackStart = (uintptr_t) bsp_interrupt_stack_start;
  intrStackSize =  (uintptr_t) bsp_interrupt_stack_size;

  /*
   * Setup BATs and enable MMU
   */
  /* Memory */
  setdbat(0, 0x0<<24, 0x0<<24, bsp_uboot_board_info.bi_memsize, _PAGE_RW);
  setibat(0, 0x0<<24, 0x0<<24, bsp_uboot_board_info.bi_memsize,        0);
  /* PCI I/O ports and memory mapped devices */
  setdbat(1, PREP_ISA_IO_BASE, PREP_ISA_IO_BASE, 0x20000,  IO_PAGE);
  setdbat(2, PREP_ISA_MEM_BASE, PREP_ISA_MEM_BASE, 0x4000000,  IO_PAGE);
  /* Host bridge IACK register is 0xbffffff0 */
  setdbat(3, 0xbffe0000, 0xbffe0000, 0x20000, IO_PAGE);

  _write_MSR(_read_MSR() | MSR_DR | MSR_IR);
  asm volatile("sync; isync");

  /*
   * Initialize default raw exception handlers.
   */
  sc = ppc_exc_initialize(
    PPC_INTERRUPT_DISABLE_MASK_DEFAULT,
    intrStackStart,
    intrStackSize
  );
  if (sc != RTEMS_SUCCESSFUL) {
    BSP_panic("cannot initialize exceptions");
  }

  /* Install default handler for the decrementer exception */
  sc = ppc_exc_set_handler( ASM_DEC_VECTOR, default_decrementer_exception_handler);
  if (sc != RTEMS_SUCCESSFUL) {
    BSP_panic("cannot install decrementer exception handler");
  }

  pci_initialize();
  config_pci_devices();

  /* Initalize interrupt support */
  sc = bsp_interrupt_initialize();
  if (sc != RTEMS_SUCCESSFUL) {
    BSP_panic("cannot intitialize interrupts");
  }

  setup_watchdog();
}

void detect_host_bridge(void)
{
    uint32_t id0;

    pci.pci_functions=&pci_indirect_functions;
    pci.pci_config_addr = (volatile unsigned char *)(_IO_BASE+0xcf8);
    pci.pci_config_data = (volatile unsigned char *)(_IO_BASE+0xcfc);

    pci_read_config_dword(0, 0, 0, 0, &id0);
    printk("idreg 0 = 0x%x\n",(unsigned)id0);
    if((id0 == PCI_VENDOR_ID_MOTOROLA +
        (PCI_DEVICE_ID_MOTOROLA_RAVEN<<16)))
    {
        printk("Found PCI Host bridge\n");
    }
}

struct config_info
{
    /* global */
    uint32_t next_mem, next_io;
    uint32_t last_mem, last_io;
    /* per device */
    unsigned int hasmem:1;
    unsigned int hasio:1;
};

static
int config_pdev_bar(struct config_info *info, int b, int d, int f, int bar)
{
    int isio;
    uint32_t mask, len;

    pci_read_config_dword(b,d,f, PCI_BASE_ADDRESS_0+4*bar, &mask);

    if((mask&PCI_BASE_ADDRESS_SPACE)==PCI_BASE_ADDRESS_SPACE_IO) {
        isio = 1;
        mask = PCI_BASE_ADDRESS_IO_MASK;
    } else {
        uint32_t mtype = mask&PCI_BASE_ADDRESS_MEM_TYPE_MASK;

        if(mtype==PCI_BASE_ADDRESS_MEM_TYPE_1M && info->next_mem>=0x100000) {
            printk("%d:%d.%d requires <1M addressing, which we can't do :( skipping\n",b,d,f);
            return 1;
        } else if(mtype==PCI_BASE_ADDRESS_MEM_TYPE_64) {
            printk("%d:%d.%d requires 64-bit addressing, which we don't do :( skipping\n",b,d,f);
            return 1;
        }

        isio = 0;
        mask = PCI_BASE_ADDRESS_MEM_MASK;
    }

    pci_write_config_dword(b,d,f, PCI_BASE_ADDRESS_0+4*bar, mask);
    pci_read_config_dword(b,d,f, PCI_BASE_ADDRESS_0+4*bar, &len);

    len &= mask;
    len = len&~(len-1);

    if(len==0) { /* unused bar */
        pci_write_config_dword(b,d,f, PCI_BASE_ADDRESS_0+4*bar, 0);
        return 0;
    }

    if(isio) {
        if(info->last_io-info->next_io < len) {
            printk("%d:%d.%d not enough space for %u byte IO BAR%d\n",b,d,f,(unsigned)len,bar);
            return 1;
        }
        pci_write_config_dword(b,d,f, PCI_BASE_ADDRESS_0+4*bar, info->next_io);
        printk("%d:%d.%d BAR%d assign IO  0x%08x len 0x%08x\n", b,d,f, bar, (unsigned)info->next_io, (unsigned)len);
        info->next_io += len;
        /* place next bar on adjecent page (no particular reason other than to space things out */
        info->next_io |= 0xfff;
        info->next_io += 1;
        info->hasio = 1;

    } else {
        if(info->last_mem-info->next_mem < len) {
            printk("%d:%d.%d not enough space for %d byte MEM BAR%d\n",b,d,f,(unsigned)len,bar);
            return 1;
        }
        pci_write_config_dword(b,d,f, PCI_BASE_ADDRESS_0+4*bar, info->next_mem);
        printk("%d:%d.%d BAR%d assign MEM 0x%08x len 0x%08x\n", b,d,f, bar, (unsigned)info->next_mem,(unsigned)len);
        info->next_mem += len;
        /* place next bar on adjecent page (no particular reason other than to space things out */
        info->next_mem |= 0xfff;
        info->next_mem += 1;
        info->hasmem = 1;
    }

    return 0;
}

static
int config_pdev(int b, int d, int f, void *uarg)
{
    struct config_info *info = uarg;
    int i;
    uint8_t val8;
    uint16_t cmd;
    uint32_t val32;

    pci_read_config_word(b,d,f, PCI_COMMAND, &cmd);
    if(cmd&(PCI_COMMAND_IO|PCI_COMMAND_MEMORY)) {
        /* disable before changing BARs */
        cmd &= ~(PCI_COMMAND_IO|PCI_COMMAND_MEMORY);
        pci_write_config_word(b,d,f, PCI_COMMAND, cmd);
    }

    pci_read_config_byte(b,d,f,PCI_HEADER_TYPE, &val8);
    if(val8!=PCI_HEADER_TYPE_NORMAL) {
        printk("%d:%d.%d Skipping setup of PCI-to-PCI bridge, not implemented\n",b,d,f);
        return 0; /* skip bridges */
    }

    info->hasio = info->hasmem = 0;

    for(i=0; i<6; i++) {
        int ret = config_pdev_bar(info, b,d,f, i);
        if(ret)
            return ret;
    }

    pci_read_config_byte(b,d,f,PCI_INTERRUPT_PIN, &val8);
    if(val8) {
        static const uint8_t irqmap[] = {9,11,9,11};
        /* 0 - no IRQ
         * 1 - pin A (IRQ 9)
         * 2 - pin B (IRQ 11)
         * 3 - pin C (IRQ 9)
         * 4 - pin D (IRQ 11)
         */
        /* like many boards, QEMU's raven rotates the irq lines from slot to slot */
        val8 += d-1;
        val8 &= 3;
        if(val8>=sizeof(irqmap)) {
            printk("%d:%d.%d IRQ pin out of range %u\n",b,d,f,val8+1);
            return 1;
        }
        val8 = irqmap[val8];
        printk("%d:%d.%d IRQ  assign %u\n",b,d,f,val8);
        pci_write_config_byte(b,d,f, PCI_INTERRUPT_LINE, val8);
    }

    cmd |= PCI_COMMAND_SERR|PCI_COMMAND_PARITY;
    if(info->hasio)
        cmd |= PCI_COMMAND_IO;
    if(info->hasmem)
        cmd |= PCI_COMMAND_MEMORY;

    pci_write_config_word(b,d,f, PCI_COMMAND, cmd);
    printk("%d:%d.%d COMMAND %04x\n",b,d,f,cmd);

    return 0;
}

static
void config_pci_devices(void)
{
    uint8_t dev;
    int ret = 0;
    struct config_info info;

    /* after ISA I/O ports */
    /* offset by PREP_ISA_IO_BASE */
    info.next_io  = 0x1000;
    info.last_io  = 0xf000;
    /* after VGA lowmem */
    /* offset by PREP_ISA_MEM_BASE */
    info.next_mem = 0x1000000;
    info.last_mem = 0x3effffff;

    /* configure devices on bus 0 only.
     * TODO, setup PCI-to-PCI bridges (PREP has none)
     * TODO, multifunction devices (PREP has none)
     */
    for(dev=0;dev<PCI_MAX_DEVICES;dev++)
    {
        uint32_t vd = 0xffffffff;
        pci_read_config_dword(0,dev,0, PCI_VENDOR_ID, &vd);
        if(vd==0xffffffff)
            continue; /* empty slot */
        ret = config_pdev(0,dev,0, &info);
        if(ret)
            break;
    }
    printk("Enum PCI %d\n", ret);
}

static struct {
    volatile void *mem;
    int b,d,f;
} wd_config;

/* setup i6300esb watchdog timer */
static
void setup_watchdog(void)
{
    uint32_t val;
    if(pci_find_device(PCI_VENDOR_ID_INTEL, 0x25ab, 0,
                       &wd_config.b,&wd_config.d,&wd_config.f))
        return; /* not found */
    else if(wd_config.b!=0) /* only bus 0 configured */
        return;

    pci_read_config_dword(wd_config.b,wd_config.d,wd_config.f,PCI_BASE_ADDRESS_0,&val);
    wd_config.mem = (void*)(PCI_MEM_BASE+val);
    printk("Setup watchdog %d:%d.%d\n",wd_config.b,wd_config.d,wd_config.f);
}

void bsp_reset(void)
{
    uint32_t val;
    if(wd_config.mem) {
        out_8(wd_config.mem+0x0c, 0x80);
        out_8(wd_config.mem+0x0c, 0x86);
        out_be32(wd_config.mem+0x00, 10); /* an arbitrary small number */

        out_8(wd_config.mem+0x0c, 0x80);
        out_8(wd_config.mem+0x0c, 0x86);
        out_be32(wd_config.mem+0x04, 10);

        out_8(wd_config.mem+0x0c, 0x80);
        out_8(wd_config.mem+0x0c, 0x86);
        out_be16(wd_config.mem+0x04, 0x10);

        val = 0x03; /* reboot on timeout, No IRQ or SMI, 1KHz scale */
        pci_write_config_word(wd_config.b,wd_config.d,wd_config.f,0x60,val);

        val = 0x02; /* Enable w/o locking */
        pci_write_config_byte(wd_config.b,wd_config.d,wd_config.f,0x68,val);
    } else {
        /* trigger CPU soft reset */
        /* this doesn't work with QEMU loading ELF as BIOS since
         * the data sections don't get reset.
        outb (0x01, 0x92);
         */
    }

    while(1) {}; /* spin forever */
}
