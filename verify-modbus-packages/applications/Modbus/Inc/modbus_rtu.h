/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-11     18452       the first version
 */
#ifndef APPLICATIONS_MODBUS_RTU_H_
#define APPLICATIONS_MODBUS_RTU_H_

#include "modbus_pdu.h"


#define MB_RTU_SADDR_SIZE       1   //RTU从机地址尺寸
#define MB_RTU_CRC_SIZE         2
#define MB_RTU_FRM_MIN          (MB_RTU_SADDR_SIZE + MB_RTU_CRC_SIZE + MB_PDU_SIZE_MIN)
#define MB_RTU_FRM_MAX          (MB_RTU_SADDR_SIZE + MB_RTU_CRC_SIZE + MB_PDU_SIZE_MAX)

#define MB_RTU_ADDR_BROADCAST   0   //广播地址
#define MB_RTU_ADDR_MIN         1   //最小地址
#define MB_RTU_ADDR_MAX         247 //最大地址
#define MB_RTU_ADDR_DEF         1   //默认地址

typedef struct{
    uint8_t  saddr;     // 从机地址
    mb_pdu_t pdu;       // PDU
}mb_rtu_frm_t;//RTU帧定义

int modbus_rtu_frame_make(uint8_t *buf, const mb_rtu_frm_t *frm, mb_pdu_type_t type);
int modbus_rtu_frame_parse(const uint8_t *buf, int len, mb_rtu_frm_t *frm, mb_pdu_type_t type);


#endif /* APPLICATIONS_MODBUS_RTU_H_ */
