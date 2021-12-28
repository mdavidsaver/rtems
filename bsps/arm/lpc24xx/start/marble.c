/*
 * Copyright (c) 2021 Michael Davidsaver  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <rtems/error.h>

#include <bsp/marble.h>
#include <bsp/io.h>
#include <bsp/i2c.h>
#include <bsp/ssp.h>
#include <bsp/eeprom.h>

static void init_err(int ret, const char* unit)
{
    if(ret) {
        fprintf(stderr, "ERROR: installing %s : %d\n", unit, ret);
    }
}
#define INIT_ERR(FUNC, ...) init_err(FUNC(__VA_ARGS__), #FUNC)

static void make_link(const char* target, const char* fname)
{
    int ret = symlink(target, fname);
    if(ret) {
        fprintf(stderr, "ERROR: linking %s <- %s : %d\n", target, fname, errno);
    }
}

static const lpc24xx_pin_range marble_iocon[] = {
    LPC24XX_PIN_I2C_1_SDA_P0_0,
    LPC24XX_PIN_I2C_1_SCL_P0_1,
    // UART0 pins selected in initialize_console()
    LPC24XX_PIN_SSP_1_SSEL_P0_7, // actually P0[6]
    LPC24XX_PIN_SSP_1_SCK_P0_6,  // actually P0[7]
    LPC24XX_PIN_SSP_1_MISO_P0_8,
    LPC24XX_PIN_SSP_1_MOSI_P0_9,
    LPC24XX_PIN_I2C_2_SDA_P0_10,
    LPC24XX_PIN_I2C_2_SCL_P0_11,
    LPC24XX_PIN_SSP_0_SCK_P0_15,
    LPC24XX_PIN_SSP_0_SSEL_P0_16,
    LPC24XX_PIN_SSP_0_MISO_P0_17,
    LPC24XX_PIN_SSP_0_MOSI_P0_18,
    LPC24XX_PIN_I2C_0_SDA,
    LPC24XX_PIN_I2C_0_SCL,
    LPC24XX_PIN_ETHERNET_MDC,
    LPC24XX_PIN_ETHERNET_MDIO,
    LPC24XX_PIN_TERMINAL
};

static void marble_gpio_setup(void)
{
#define GPIO(NAME) lpc24xx_gpio_config(MARBLE_ ## NAME, MARBLE_ ## NAME ## _SETUP)

    /* grep -o 'MARBLE_.*_SETUP' bsps/arm/lpc24xx/include/bsp/marble.h \
        | sed -e 's/MARBLE_\(.*\)_SETUP/GPIO(\1);/'
     */
    GPIO(PROG_B);
    GPIO(FPGA_DONE);
    GPIO(FMC1_FUSE);
    GPIO(FMC2_FUSE);
    GPIO(OVER_TEMP);
    GPIO(AT_DET);
    GPIO(EN_FMC1_P12V);
    GPIO(EN_PSU_CH);
    GPIO(Pmod3_5);
    GPIO(SDRAM_PGOOD);
    GPIO(EINT1);
    GPIO(BOOT_MODE0);
    GPIO(BOOT_MODE1);
    GPIO(BOOT_MODE2);
    GPIO(AMC_GA0);
    GPIO(AMC_GA1);
    GPIO(AMC_GA2);
    GPIO(FMC1_PRSNT_Override);
    GPIO(FMC2_PRSNT_Override);
    GPIO(FMC1_PG_C2M);
    GPIO(FMC2_PG_C2M);
    GPIO(HW_ID0);
    GPIO(HW_ID1);
    GPIO(HW_ID2);
    GPIO(HW_ID3);
    GPIO(FMC_TRST);
    GPIO(AMC_En);
    GPIO(FPGA_RESETn);
    GPIO(SW);
    GPIO(FMC_TDO);
    GPIO(FMC_TMS);
    GPIO(FMC_TDI);
    GPIO(FMC_TCK);
}

int marble_bsp_init(void)
{
    uint32_t rsid = LPC17XX_SCB.rsid;
    rtems_status_code ec = lpc24xx_pin_config(marble_iocon, LPC24XX_PIN_SET_FUNCTION);
    if(ec!=RTEMS_SUCCESSFUL) {
        rtems_error(ec, "Unable to complete pin mux. config\n");
        return -EINVAL;
    }

    INIT_ERR(lpc24xx_register_i2c_0);
    make_link(LPC24XX_I2C_0_BUS_PATH, "/dev/i2c-ipmb");

    INIT_ERR(lpc24xx_register_i2c_1);
    make_link(LPC24XX_I2C_1_BUS_PATH, "/dev/i2c-pm");

    INIT_ERR(lpc24xx_register_i2c_2);
    make_link(LPC24XX_I2C_2_BUS_PATH, "/dev/i2c-fpga");

    INIT_ERR(lpc24xx_register_ssp_0);
    make_link(LPC24XX_SSP_0_BUS_PATH, "/dev/spi-fpga");

    INIT_ERR(lpc24xx_register_ssp_1);
    make_link(LPC24XX_SSP_1_BUS_PATH, "/dev/spi-pmod3");

    INIT_ERR(lpc24xx_register_eeprom);

    marble_gpio_setup();

    printf("Marble Mini BSP\n"
           " RTEMS: %s\n"
           " Tools: %s\n"
           " Reset src:",
           rtems_get_version_string(), __VERSION__);
#define RSID(BIT) \
    if(rsid & LPC17XX_SCB_RSID_ ## BIT) \
        printf(" " #BIT)
    RSID(POR);
    RSID(EXTR);
    RSID(WDTR);
    RSID(BODR);
    RSID(SYSRESET);
    RSID(LOCKUP);
#undef RSID
    printf("\n");

    return 0;
}
