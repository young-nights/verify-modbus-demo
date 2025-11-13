/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-12     18452       the first version
 */
#ifndef APPLICATIONS_MODBUS_INC_MODBUS_TCP_H_
#define APPLICATIONS_MODBUS_INC_MODBUS_TCP_H_

#include "modbus_backend.h"

#ifdef MB_USING_TCP_PROTOCOL

#define MB_TCP_MBAP_SIZE    7   //MBAP头尺寸
#define MB_TCP_FRM_MIN      (MB_TCP_MBAP_SIZE + MB_PDU_SIZE_MIN)
#define MB_TCP_FRM_MAX      (MB_TCP_MBAP_SIZE + MB_PDU_SIZE_MAX)

#define MB_TCP_MBAP_PID     0x0000//使用的协议类型代码

typedef struct{
    uint16_t tid;    //传输标识符, 响应须与请求一致
    uint16_t pid;    //协议类型, 响应须与请求一致
    uint16_t dlen;   //等于pdu数据长度加1, 打包时会自动计算处理不需人工赋值
    uint8_t  did;    //逻辑设备ID, 响应须与请求一致
}mb_tcp_mbap_t;//TCP协议MBAP头定义

typedef struct{
    mb_tcp_mbap_t mbap;//MBAP头
    mb_pdu_t pdu;//PDU数据
}mb_tcp_frm_t;//TCP帧定义

int modbus_tcp_frm_make(uint8_t *buf, const mb_tcp_frm_t *frm, mb_pdu_type_t type);//生成tcp帧, 返回帧长度
int modbus_tcp_frm_parse(const uint8_t *buf, int len, mb_tcp_frm_t *frm, mb_pdu_type_t type);//解析tcp帧, 返回pdu数据长度, 解析失败返回0, 功能码不支持返回-1

#endif




#endif /* APPLICATIONS_MODBUS_INC_MODBUS_TCP_H_ */
