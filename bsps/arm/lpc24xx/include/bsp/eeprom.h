/**
 * @file
 *
 * @ingroup RTEMSBSPsARMLPC24XXEEPROM
 */
/*
 * Copyright (c) 2021 Michael Davidsaver  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_LPC24XX_EEPROM_H
#define LIBBSP_ARM_LPC24XX_EEPROM_H

#include <bsp/io.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#define LPC24XX_EEPROM_PATH "/dev/eeprom"

/** Install /dev/eeprom which appears as a fixed size file.
 */
int lpc24xx_register_eeprom(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // LIBBSP_ARM_LPC24XX_EEPROM_H
