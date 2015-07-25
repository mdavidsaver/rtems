/*
 *  $Id$
 */

#include <bsp.h>
#include <bsp/vectors.h>
#include <libcpu/io.h>

/* QEMU emulates a simple non-volatile memory
 * device in order to provide some information
 * to the bootloader/OS
 *
 * cf. PPC_NVRAM_set_params() in the QEMU source
 */
uint8_t qemu_nvram_read(uint16_t addr)
{
    outb(addr, 0x74);
    outb(addr>>8, 0x75);
    asm volatile ("eieio" ::: "memory");
    return inb(0x77);
}

void qemu_nvram_get_string(uint16_t addr, char *buf, size_t len)
{
    len--;
    while(len--) {
        char c = qemu_nvram_read(addr++);
        if(c=='\0')
            break;
        *buf++ = c;
    }
    *buf = '\0';
}

uint32_t qemu_nvram_get_uint32(uint16_t addr)
{
    uint32_t ret;
    ret = qemu_nvram_read(addr)<<24;
    ret|= qemu_nvram_read(addr+1)<<16;
    ret|= qemu_nvram_read(addr+2)<<8;
    ret|= qemu_nvram_read(addr+3);
    return ret;
}

uint16_t qemu_nvram_get_uint16(uint16_t addr)
{
    uint16_t ret;
    ret = qemu_nvram_read(addr)<<8;
    ret|= qemu_nvram_read(addr+1);
    return ret;
}

static void __attribute__((unused))
__memcpy (unsigned char *d, unsigned char *s, int len)
{
  while (len--)
    *d++ = *s++;
}

static void __attribute__((unused))
__bzero (unsigned char *d, int len)
{
  while (len--)
    *d++ = 0;
}
/*
extern unsigned char __sdata2_load[], __sdata2_start[], __sdata2_end[];
extern unsigned char __data_load[], __data_start[], __data_end[];
extern unsigned char __sdata_load[], __sdata_start[], __sdata_end[];
*/
extern unsigned char __sbss2_start[], __sbss2_end[];
extern unsigned char __sbss_start[], __sbss_end[];
extern unsigned char __bss_start[], __bss_end[];

extern void boot_card(const char *);

/*  our pretend u-boot info for bsp_get_work_area() */
bd_t bsp_uboot_board_info;

uint32_t BSP_mem_size;

static char qemu_cmdline[256];

#ifdef __ALTIVEC__
# error altivec not supported
#endif

void cmain (void)
{
  /*
   * init variable sections
   */
    /*
  __memcpy (__sdata2_start, __sdata2_load, __sdata2_end - __sdata2_start);
  __memcpy (__sdata_start , __sdata_load , __sdata_end  - __sdata_start);
  __memcpy (__data_start  , __data_load  , __data_end   - __data_start);
  */
  __bzero (__sbss2_start  , __sbss2_end - __sbss2_start);
  __bzero (__sbss_start   , __sbss_end  - __sbss_start);
  __bzero (__bss_start    , __bss_end   - __bss_start);

  printk( "start of BSP\n");
  {
      char scratch[16];
      unsigned int ver, size;

      /* skip nvram checksum verification since
       * we assume this is -machine prep
       */

      qemu_nvram_get_string(0, scratch, sizeof(scratch));
      ver = qemu_nvram_get_uint32(0x10);
      size = qemu_nvram_get_uint16(0x14);
      printk("Running with '%s' version=%u nvsize=%u\n",
             scratch, ver, size);

      /* nvram format has been v2 since QEMU circa 2004
       * with the size extending.
       * So expect that any v3 would be incompatible.
       */
      if(ver!=2 || size<0x50) {
          printk("Warning: NVRAM incompatible, ignoring and assuming defaults\n");
          bsp_uboot_board_info.bi_memstart = 0;
          bsp_uboot_board_info.bi_memsize = BSP_mem_size = 16<<20; /* assume at least 16 MB */

      } else {
          const char *cmdaddr;
          uint32_t ramsize;
          qemu_nvram_get_string(0x20, scratch, sizeof(scratch));
          printk(" on '%s'\n", scratch);

          ramsize = qemu_nvram_get_uint16(0x30);
          ramsize <<= 16; /* in bytes */
          printk(" RAM size %u MB\n", (unsigned)(ramsize>>20));
          bsp_uboot_board_info.bi_memstart = 0;
          bsp_uboot_board_info.bi_memsize = BSP_mem_size = ramsize;

          size = qemu_nvram_get_uint32(0x44);
          if(size>sizeof(qemu_cmdline)-1) {
              printk("Warning: cmdline string too long, truncating to %lu\n",
                     (unsigned long)sizeof(qemu_cmdline)-1);
              size = sizeof(qemu_cmdline)-1;
          }

          cmdaddr = (const char*)qemu_nvram_get_uint32(0x40);
          memcpy(qemu_cmdline, cmdaddr, size);
          qemu_cmdline[sizeof(qemu_cmdline)-1] = '\0';

          printk(" bsp_boot_cmdline = '%s'\n", qemu_cmdline);
      }
  }
  boot_card(qemu_cmdline[0] ? qemu_cmdline : 0);
  printk( "end of BSP\n");
  while (1) {};
}
