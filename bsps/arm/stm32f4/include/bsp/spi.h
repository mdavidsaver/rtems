/**
 * @file
 * @ingroup stm32f4_spi SPI Support
 * @brief SPI-module.
 */
/*
 * Copyright (c) 2022 Michael Davidsaver.  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_STM32F4_SPI_H
#define LIBBSP_ARM_STM32F4_SPI_H

#include <rtems.h>

#include <bsp/io.h>
#include <bsp/stm32f4.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @defgroup stm32f4_spi SPI Support
 * @ingroup RTEMSBSPsARMSTM32F4
 * @brief SPI Module
 * @{
 */

typedef struct {
    /** @brief Select pin to be used for a device.
     *
     * - Must be an Output.
     * - Must only be one pin.  (pin_first == pin_last)
     */
    stm32f4_gpio_config pin;
    /** @brief ss_active Active/Selected level
     *
     * false for active low, idle high.
     */
    bool active;
} stm32f4_spi_ss_config_t;

/// @brief Configuration for a single SPI device.
typedef struct {
    /// @brief SCK pin to be used for this unit.
    stm32f4_gpio_config sck;
    /// @brief MOSI pin to be used for this unit.
    stm32f4_gpio_config mosi;
    /// @brief MISO pin to be used for this unit.
    stm32f4_gpio_config miso;
    //! @brief Array of select pin configurations
    const stm32f4_spi_ss_config_t* selects;
    //! @brief length of selects array
    uint8_t num_selects;
} stm32f4_spi_config_t;

typedef struct stm32f4_spi_t stm32f4_spi_t;

extern stm32f4_spi_t stm32f4_spi1;
extern stm32f4_spi_t stm32f4_spi2;
extern stm32f4_spi_t stm32f4_spi3;

#define STM32F4_SPI1_BUS_PATH "/dev/spidev-1"
#define STM32F4_SPI2_BUS_PATH "/dev/spidev-2"
#define STM32F4_SPI3_BUS_PATH "/dev/spidev-3"

/** @brief Register SPI device
 * @param dev The address of one of the stm32f4_spi* globals.
 * @param conf I/O pin configuration
 * @return 0 on success or a negative errno code.
 *
 * It is the caller's responsibility to ensure that the
 * configured pins can actually be driven by the choosen
 * SPI core.  cf. the Alternate Function mapping table
 * in the appropriate ST part datasheet.
 *
 * The `conf` struct, and `conf->selects` array must
 * remain valid until the device is destroyed.
 */
int stm32f4_spi_register(stm32f4_spi_t *dev, const stm32f4_spi_config_t* conf);


/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_STM32F4_SPI_H */
