/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-12     18452       the first version
 */
#ifndef APPLICATIONS_MODBUS_MODBUS_CRC_H_
#define APPLICATIONS_MODBUS_MODBUS_CRC_H_

#include "bsp_sys.h"




#define MB_CRC_INIT_VOL     0xFFFF

uint16_t modbus_crc_cyc_cal(uint16_t init, const uint8_t *pdata, int len);
uint16_t modbus_crc_cal(const uint8_t *pdata, int len) ;

#endif /* APPLICATIONS_MODBUS_MODBUS_CRC_H_ */
