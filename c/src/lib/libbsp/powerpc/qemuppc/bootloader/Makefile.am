
EXEEXT=.exe
DOWNEXT=.ralf

include $(top_srcdir)/../../../../automake/compile.am

OBJCOPY = ${target_alias}-objcopy

# "bios" stub bootloader

xinstdir = $(DESTDIR)${exec_prefix}/${RTEMS_BSP}/bin

all-local: bios$(DOWNEXT)

install-data-local: ${xinstdir}/bios$(DOWNEXT)

bios.o: $(srcdir)/bios.S
	$(AS) $< -o $@

bios$(EXEEXT): $(srcdir)/bios.ld bios.o
	$(LD) -o $@ -T$(srcdir)/bios.ld bios.o

bios$(DOWNEXT): bios$(EXEEXT)
	$(OBJCOPY) -O binary $< $@

${xinstdir}/bios$(DOWNEXT): bios$(DOWNEXT)
	$(MKDIR_P) ${xinstdir}
	$(INSTALL_DATA) $< $@

include $(top_srcdir)/../../../../automake/local.am
