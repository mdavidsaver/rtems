/**
 * @file
 * @ingroup stm32f4_flash
 * @brief Flash support.
 */

/*
 * Copyright (c) 2022 Michael Davidsaver  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_STM32F4_FLASH_H
#define LIBBSP_ARM_STM32F4_FLASH_H

#include <stdlib.h>
#include <stdint.h>
#include <bspopts.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @defgroup stm32f4_flash Flash Support
 * @ingroup RTEMSBSPsARMSTM32F4
 * @brief Flash Support
 * @{
 */

#ifdef STM32F4_FAMILY_F4XXXX

/** @brief Write to FLASH
 * @param addr Address of first byte to be written
 * @param value pointer to byte value to write
 * @param count Number of bytes
 * @return 0 on success, or -errno
 *
 * The caller is responsible for ensuring that addr is a FLASH address.
 *
 * A successful write updates the FLASH cache.  It is _not_ necessary
 * to stm32f4_flash_cache_flush_all()
 */
int stm32f4_flash_program(void *addr, const void* value, size_t count);

/** @brief Erase FLASH sector
 * @param sectorn Sector index
 * @return 0 on success, or -errno
 *
 * The caller is responsible for understanding the mapping between
 * sector index and address.
 *
 * @note Erasing invalidates FLASH cache.  Caller must
 *       stm32f4_flash_cache_flush_all() after successful erase(s).
 */
int stm32f4_flash_erase_sector(unsigned sectorn);

/** @brief Synchronize FLASH read cache after erase operations
 * @return
 */
void stm32f4_flash_cache_flush_all(void);

#endif /* STM32F4_FAMILY_F4XXXX */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_STM32F4_FLASH_H */
