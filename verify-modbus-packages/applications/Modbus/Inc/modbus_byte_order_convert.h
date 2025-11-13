/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-12     18452       the first version
 */
#ifndef APPLICATIONS_MODBUS_BYTE_ORDER_CONVERT_H_
#define APPLICATIONS_MODBUS_BYTE_ORDER_CONVERT_H_

#include "bsp_sys.h"


int modbus_cvt_u8_put(uint8_t *buf, uint8_t val);
int modbus_cvt_u8_get(const uint8_t *buf, uint8_t *pval);
int modbus_cvt_u16_put(uint8_t *buf, uint16_t val);
int modbus_cvt_u16_get(const uint8_t *buf, uint16_t *pval);
int modbus_cvt_u32_put(uint8_t *buf, uint32_t val);
int modbus_cvt_u32_get(const uint8_t *buf, uint32_t *pval);
int modbus_cvt_f32_put(uint8_t *buf, float val);
int modbus_cvt_f32_get(const uint8_t *buf, float *pval);
uint8_t modbus_bitmap_get(const uint8_t *pbits, int idx);//从位表中读指定索引的位
void modbus_bitmap_set(uint8_t *pbits, int idx, uint8_t bit);//向位表中写指定索引的位








#endif /* APPLICATIONS_MODBUS_BYTE_ORDER_CONVERT_H_ */
