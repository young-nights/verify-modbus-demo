/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-04     Administrator       the first version
 */
#include "modbus_crc.h"
#include "modbus_rtu.h"


#ifdef MB_USING_RTU_PROTOCOL


/**
 * @brief  生成 Modbus RTU 协议帧
 *
 * 根据输入的 mb_rtu_frm_t 结构体（从机地址 + PDU），构建完整的 RTU 帧：
 *   - 结构：地址 (1字节) + PDU + CRC (2字节)
 * 返回生成的帧总长度（字节数）。
 *
 * @param[out] buf   指向输出缓冲区的指针（必须足够大，至少 MB_RTU_FRM_MAX）
 * @param[in]  frm   指向 RTU 帧结构体的指针（包含 saddr 和 pdu）
 * @param[in]  type  PDU 类型（MB_PDU_TYPE_REQ 或 MB_PDU_TYPE_RSP）
 *
 * @return int  生成的帧总长度（>0），如果 PDU 生成失败可能返回无效值
 *
 * @note
 *   - 依赖 mb_cvt_u8_put() 写入地址、mb_pdu_make() 生成 PDU、mb_crc_cal() 计算 CRC
 *   - CRC 以低字节在前、高字节在后写入
 *   - 调用前确保 buf 足够大，否则可能溢出
 *
 * @warning
 *   - 无错误检查，假设输入有效；如果 pdu 无效，返回长度可能为 0
 */
int modbus_rtu_frame_make(uint8_t *buf, const mb_rtu_frm_t *frm, mb_pdu_type_t type)
{
    uint8_t *p = buf;
    p += modbus_cvt_u8_put(p, frm->saddr);
    p += modbus_pdu_make(p, &(frm->pdu), type);
    uint16_t crc = modbus_crc_cal(buf, (int)(p - buf));
    *p++ = crc;
    *p++ = (crc >> 8);
    return((int)(p - buf));
}

/**
 * @brief  解析 Modbus RTU 协议帧
 *
 * 从输入缓冲区解析 RTU 帧，提取地址和 PDU，并验证 CRC。
 * 返回 PDU 数据长度（成功时 >0）；帧错误返回 0；功能码不支持返回 -1。
 *
 * @param[in]  buf   指向输入缓冲区的指针（包含完整 RTU 帧）
 * @param[in]  len   输入缓冲区长度（必须 >= MB_RTU_FRM_MIN）
 * @param[out] frm   指向输出 RTU 帧结构体的指针（填充 saddr 和 pdu）
 * @param[in]  type  PDU 类型（MB_PDU_TYPE_REQ 或 MB_PDU_TYPE_RSP）
 *
 * @return int  PDU 数据长度（>0: 成功），0: 帧错误，-1: 功能码不支持
 *
 * @note
 *   - 依赖 mb_pdu_parse() 解析 PDU、mb_crc_cal() 验证 CRC
 *   - CRC 验证失败或长度不足返回 0
 *   - 不处理地址过滤（上层逻辑）
 *
 * @warning
 *   - 输入 buf 必须完整；部分帧可能导致 CRC 通过但 PDU 错位
 */
int modbus_rtu_frame_parse(const uint8_t *buf, int len, mb_rtu_frm_t *frm, mb_pdu_type_t type)
{
    if (len < MB_RTU_FRM_MIN){
        return(0);
    }

    frm->saddr = *buf;
    int remain = len - (MB_RTU_SADDR_SIZE + MB_RTU_CRC_SIZE);
    int pdu_len = modbus_pdu_parse(buf + 1, remain, &(frm->pdu), type);
    if (pdu_len <= 0){
        return(pdu_len);
    }

    if (remain < pdu_len){
        return(0);
    }

    int flen = pdu_len + (MB_RTU_SADDR_SIZE + MB_RTU_CRC_SIZE);
    uint16_t cal_crc = modbus_crc_cal(buf, flen);
    if (cal_crc != 0)
    {
        return(0);
    }

    return(pdu_len);
}

#endif


