/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-12     18452       the first version
 */

#include "bsp_sys.h"

#ifdef MB_USING_TCP_PROTOCOL


/**
 * @brief  生成 Modbus TCP 完整帧（MBAP + PDU）
 *
 * 将 mb_tcp_frm_t 结构体序列化为网络字节流。
 * 自动计算 MBAP 长度字段（dlen = pdu_len + 1）。
 *
 * @param[out] buf   输出缓冲区（建议 >= 260 字节）
 * @param[in]  frm   TCP 帧结构体
 * @param[in]  type  PDU 类型（MB_PDU_TYPE_REQ 或 MB_PDU_TYPE_RSP）
 *
 * @return int
 *   - >0 : 完整帧长度（7 + PDU 长度）
 *   -  0 : 生成失败（PDU 无效或长度超限）
 *
 * @note
 *   - 所有字段按大端序（网络序）写入
 *   - MBAP 头 7 字节：TID(2) + PID(2) + DLEN(2) + DID(1)
 *   - 调用 modbus_pdu_make() 生成 PDU 部分
 *   - 上层应确保 buf 足够大
 */
int modbus_tcp_frm_make(uint8_t *buf, const mb_tcp_frm_t *frm, mb_pdu_type_t type)//生成tcp帧, 返回帧长度
{
    int pdu_len = modbus_pdu_make(buf + MB_TCP_MBAP_SIZE, &(frm->pdu), type);

    uint8_t *p = buf;
    p += modbus_cvt_u16_put(p, frm->mbap.tid);
    p += modbus_cvt_u16_put(p, frm->mbap.pid);
    p += modbus_cvt_u16_put(p, pdu_len + 1);
    p += modbus_cvt_u8_put(p, frm->mbap.did);
    p += pdu_len;

    return((int)(p - buf));
}




/**
 * @brief  解析 Modbus TCP 完整帧（MBAP + PDU）
 *
 * 从网络字节流中提取 MBAP 头和 PDU，填充 mb_tcp_frm_t 结构体。
 * 严格校验长度和协议字段。
 *
 * @param[in]  buf   输入帧缓冲区
 * @param[in]  len   帧长度
 * @param[out] frm   解析结果结构体
 * @param[in]  type  预期类型（MB_PDU_TYPE_REQ 或 MB_PDU_TYPE_RSP）
 *
 * @return int
 *   - >0 : PDU 长度
 *   -  0 : 帧格式错误（长度不足、dlen 不符）
 *   - -1 : 功能码不支持
 *
 * @note
 *   - 自动校验 MBAP 长度字段（dlen == pdu_len + 1）
 *   - 自动校验协议标识符（pid == 0x0000）
 *   - 调用 modbus_pdu_parse() 解析 PDU
 */
int modbus_tcp_frm_parse(const uint8_t *buf, int len, mb_tcp_frm_t *frm, mb_pdu_type_t type)//解析tcp帧, 返回pdu数据长度, 解析失败返回0, 功能码不支持返回-1
{
    if (len < MB_TCP_FRM_MIN){
        return(0);
    }

    uint8_t *p = (uint8_t *)buf;
    p += modbus_cvt_u16_get(p, &(frm->mbap.tid));
    p += modbus_cvt_u16_get(p, &(frm->mbap.pid));
    p += modbus_cvt_u16_get(p, &(frm->mbap.dlen));
    p += modbus_cvt_u8_get(p, &(frm->mbap.did));

    int remain = len - (int)(p - buf);
    int pdu_len = modbus_pdu_parse(p, remain, &(frm->pdu), type);
    if (pdu_len <= 0){
        return(pdu_len);
    }

    if (remain < pdu_len){
        return(0);
    }

    return(pdu_len);
}

#endif









