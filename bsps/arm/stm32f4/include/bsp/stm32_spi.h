/**
 * @file
 * @ingroup stm32_spi
 * @brief STM32 SPI support.
 */

/*
 * Copyright (c) 2022 Michael Davidsaver.  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_STM32F4_STM32_SPI_H
#define LIBBSP_ARM_STM32F4_STM32_SPI_H

#include <bsp/utility.h>

/**
 * @defgroup stm32_spi STM32 SPI Support
 * @ingroup stm32f4_spi
 * @brief STM32 SPI Support
 * @{
 */

typedef struct {
    uint32_t cr1;
#define STM32F4_SPI_CR1_BIDIMODE BSP_BIT32(15)
#define STM32F4_SPI_CR1_BIDIOE BSP_BIT32(14)
#define STM32F4_SPI_CR1_CRCEN BSP_BIT32(13)
#define STM32F4_SPI_CR1_CRCNEXT BSP_BIT32(12)
#define STM32F4_SPI_CR1_DFF BSP_BIT32(11)
#define STM32F4_SPI_CR1_RXONLY BSP_BIT32(10)
#define STM32F4_SPI_CR1_SSM BSP_BIT32(9)
#define STM32F4_SPI_CR1_SSI BSP_BIT32(8)
#define STM32F4_SPI_CR1_LSBFIRST BSP_BIT32(7)
#define STM32F4_SPI_CR1_SPE BSP_BIT32(6)
#define STM32F4_SPI_CR1_BR(val) BSP_FLD32(val, 3, 5)
#define STM32F4_SPI_CR1_BR_GET(reg) BSP_FLD32GET(reg, 3, 5)
#define STM32F4_SPI_CR1_BR_SET(reg, val) BSP_FLD32SET(reg, val, 3, 5)
#define STM32F4_SPI_CR1_MSTR BSP_BIT32(2)
#define STM32F4_SPI_CR1_CPOL BSP_BIT32(1)
#define STM32F4_SPI_CR1_CPHA BSP_BIT32(0)

    uint32_t cr2;
#define STM32F4_SPI_CR2_TXEIE BSP_BIT32(7)
#define STM32F4_SPI_CR2_RXNEIE BSP_BIT32(6)
#define STM32F4_SPI_CR2_ERRIE BSP_BIT32(5)
#define STM32F4_SPI_CR2_FRF BSP_BIT32(4)
#define STM32F4_SPI_CR2_SSOE BSP_BIT32(2)
#define STM32F4_SPI_CR2_TXDMAEN BSP_BIT32(3)
#define STM32F4_SPI_CR2_RXDMAEN BSP_BIT32(0)

    uint32_t sr;
#define STM32F4_SPI_SR_TIFRF BSP_BIT32(8)
#define STM32F4_SPI_SR_BSY BSP_BIT32(7)
#define STM32F4_SPI_SR_OVR BSP_BIT32(6)
#define STM32F4_SPI_SR_MODF BSP_BIT32(5)
#define STM32F4_SPI_SR_CRCERR BSP_BIT32(4)
#define STM32F4_SPI_SR_UDR BSP_BIT32(3)
#define STM32F4_SPI_SR_CHSIDE BSP_BIT32(2)
#define STM32F4_SPI_SR_TXE BSP_BIT32(1)
#define STM32F4_SPI_SR_RXNE BSP_BIT32(0)

    uint32_t dr;

    uint32_t crcpr;
    uint32_t rxcrcr;
    uint32_t txcrcr;
    uint32_t i2scfgr;
    uint32_t i2spr;
} stm32f4_spi;

/** @} */

#endif /* LIBBSP_ARM_STM32F4_STM32_SPI_H */
