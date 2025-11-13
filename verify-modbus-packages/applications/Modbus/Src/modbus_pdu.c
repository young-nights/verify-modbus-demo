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



/**
 * @brief  生成 Modbus PDU 异常响应帧
 *
 * 将异常结构体（功能码 + 异常码）转换为 PDU 字节流。
 * 异常响应结构：功能码 (原码 | 0x80) + 异常码 (1字节)。
 * 返回生成的 PDU 长度（始终为 2）。
 *
 * @param[out] buf  目标缓冲区指针（必须至少 2 字节）
 * @param[in]  exc  指向异常结构体的指针（包含 fc 和 ec）
 *
 * @return int  生成的 PDU 长度（2），表示成功
 *
 * @note
 *   - 依赖 modbus_cvt_u8_put() 写入字节
 *   - 用于从站响应异常情况，如非法功能码、地址错误
 *   - 上层需确保 fc 已设置异常位 (0x80)
 */
static int modbus_pdu_except_make(uint8_t *buf, const mb_pdu_except_t *exc)
{
    uint8_t *p = buf;

    p += modbus_cvt_u8_put(p, exc->fc);
    p += modbus_cvt_u8_put(p, exc->ec);

    return((int)(p - buf));
}


/**
 * @brief  解析 Modbus PDU 异常响应帧
 *
 * 将异常 PDU 字节流转换为 mb_pdu_except_t 结构体。
 * 异常响应结构：异常功能码 (原码 | 0x80) + 异常码 (1字节)。
 *
 * @param[in]  buf  源缓冲区指针（包含至少 2 字节）
 * @param[in]  len  缓冲区长度（>= 2）
 * @param[out] exc  指向异常结构体的指针（填充 fc 和 ec）
 *
 * @return int
 *   - 2 : 解析成功
 *   - 0 : 帧太短（len < 2）
 *
 * @note
 *   - 仅解析格式，不验证功能码是否合法（上层通过 MODBUS_FC_EXCEPT_CHK 检查）
 *   - 常用于主站处理从站异常响应
 */
static int modbus_pdu_except_parse(const uint8_t *buf, int len, mb_pdu_except_t *exc)
{
    if (len < 2)
    {
        return(0);
    }

    uint8_t *p = (uint8_t *)buf;

    p += modbus_cvt_u8_get(p, &(exc->fc));
    p += modbus_cvt_u8_get(p, &(exc->ec));

    return((int)(p - buf));
}


/**
 * @brief  生成 Modbus PDU 读请求帧
 *
 * 将读请求结构体转换为 5 字节 PDU（大端序）：
 *   - 结构：功能码(1) + 起始地址(2) + 数量(2)
 * 支持功能码：0x01, 0x02, 0x03, 0x04
 *
 * @param[out] buf     目标缓冲区指针（至少 5 字节）
 * @param[in]  rd_req  指向读请求结构体的指针
 *                     - fc:   功能码（0x01~0x04）
 *                     - addr: 起始地址（0~65535）
 *                     - nb:   数量（1~最大值）
 *
 * @return int  生成的 PDU 长度（5），表示成功
 *
 * @note
 *   - 常用于主站发起读操作
 */
static int modbus_pdu_rd_req_make(uint8_t *buf, const mb_pdu_rd_req_t *rd_req)
{
    // 工作指针
    uint8_t *p = buf;

    // 写入功能码（1字节）
    p += modbus_cvt_u8_put(p, rd_req->fc);
    // 写入起始地址（2字节，大端）
    p += modbus_cvt_u16_put(p, rd_req->addr);
    // 写入数量（2字节，大端）
    p += modbus_cvt_u16_put(p, rd_req->nb);

    return((int)(p - buf));
}



/**
 * @brief  解析 Modbus PDU 读请求帧
 *
 * 将 5 字节读请求 PDU（大端序）转换为 mb_pdu_rd_req_t 结构体：
 *   - 结构：功能码(1) + 起始地址(2) + 数量(2)
 * 支持功能码：0x01, 0x02, 0x03, 0x04
 *
 * @param[in]  buf     源缓冲区指针（包含至少 5 字节）
 * @param[in]  len     缓冲区长度（>= 5）
 * @param[out] rd_req  指向读请求结构体的指针（填充 fc, addr, nb）
 *
 * @return int
 *   - 5 : 解析成功
 *   - 0 : 帧太短（len < 5）
 *
 * @note
 *   - 常用于从站处理主站读请求
 */
static int modbus_pdu_rd_req_parse(const uint8_t *buf, int len, mb_pdu_rd_req_t *rd_req)
{
    if (len < 5)
    {
        return(0);
    }

    uint8_t *p = (uint8_t *)buf;

    p += modbus_cvt_u8_get(p, &(rd_req->fc));
    p += modbus_cvt_u16_get(p, &(rd_req->addr));
    p += modbus_cvt_u16_get(p, &(rd_req->nb));

    return((int)(p - buf));
}


/**
 * @brief  生成 Modbus PDU 读响应帧
 *
 * 将读响应结构体转换为变长 PDU（大端序）：
 *   - 结构：功能码(1) + 数据字节数(1) + 数据(dlen)
 * 支持功能码：0x01, 0x02, 0x03, 0x04
 *
 * @param[out] buf     目标缓冲区指针（至少 2 + dlen 字节）
 * @param[in]  rd_rsp  指向读响应结构体的指针
 *                     - fc:    功能码（0x01~0x04）
 *                     - dlen:  数据字节数（由 nb 计算）
 *                     - pdata: 数据缓冲区指针（不可为 NULL）
 *
 * @return int  生成的 PDU 总长度（2 + dlen）
 *
 * @note
 *   - 数据 pdata 必须为大端序（由上层转换）
 *   - 常用于从站响应主站读请求
 */
static int modbus_pdu_rd_rsp_make(uint8_t *buf, const mb_pdu_rd_rsp_t *rd_rsp)
{
    uint8_t *p = buf;

    // 写入功能码（1字节）
    p += modbus_cvt_u8_put(p, rd_rsp->fc);
    // 写入数据字节数（1字节）
    p += modbus_cvt_u8_put(p, rd_rsp->dlen);
    // 复制实际数据
    memcpy(p, rd_rsp->pdata, rd_rsp->dlen);
    // 指针前移
    p += rd_rsp->dlen;

    return((int)(p - buf));
}


/**
 * @brief  解析 Modbus PDU 读响应帧（变长）
 *
 * 将读响应 PDU 转换为 mb_pdu_rd_rsp_t 结构体（零拷贝）：
 *   - 结构：功能码(1) + 数据字节数(1) + 数据(dlen)
 * 支持功能码：0x01, 0x02, 0x03, 0x04
 *
 * @param[in]  buf     源缓冲区指针（包含完整 PDU）
 * @param[in]  len     缓冲区长度
 * @param[out] rd_rsp  指向读响应结构体的指针
 *                     - fc:    功能码
 *                     - dlen:  数据字节数
 *                     - pdata: 指向 buf 中数据区的指针（零拷贝）
 *
 * @return int
 *   - (2 + dlen) : 解析成功
 *   - 0          : 帧太短或数据不完整
 *
 * @note
 *   - 采用零拷贝设计：pdata 直接指向 buf，减少内存操作
 *   - pdata 生命周期与 buf 绑定，buf 释放前不可使用 pdata
 *   - 常用于主站解析从站响应
 */
static int modbus_pdu_rd_rsp_parse(const uint8_t *buf, int len, mb_pdu_rd_rsp_t *rd_rsp)
{
    // 最小：fc(1) + dlen(1) + 数据(至少1)
    if (len < 3){
        return(0);
    }
    // 工作指针
    uint8_t *p = (uint8_t *)buf;
    // 读取功能码
    p += modbus_cvt_u8_get(p, &(rd_rsp->fc));
    // 读取数据长度
    p += modbus_cvt_u8_get(p, &(rd_rsp->dlen));
    // pdata 指向数据区（零拷贝）
    rd_rsp->pdata = p;
    // 跳过数据
    p += rd_rsp->dlen;

    return((int)(p - buf));
}

static int modbus_pdu_wr_single_make(uint8_t *buf, const mb_pdu_wr_single_t *wr_single)
{
    uint8_t *p = buf;

    p += modbus_cvt_u8_put(p, wr_single->fc);
    p += modbus_cvt_u16_put(p, wr_single->addr);
    p += modbus_cvt_u16_put(p, wr_single->val);

    return((int)(p - buf));
}

static int modbus_pdu_wr_single_parse(const uint8_t *buf, int len, mb_pdu_wr_single_t *wr_single)
{
    if (len < 5)
    {
        return(0);
    }

    uint8_t *p = (uint8_t *)buf;

    p += modbus_cvt_u8_get(p, &(wr_single->fc));
    p += modbus_cvt_u16_get(p, &(wr_single->addr));
    p += modbus_cvt_u16_get(p, &(wr_single->val));

    return((int)(p - buf));
}

static int modbus_pdu_wr_req_make(uint8_t *buf, const mb_pdu_wr_req_t *wr_req)
{
    uint8_t *p = buf;

    p += modbus_cvt_u8_put(p, wr_req->fc);
    p += modbus_cvt_u16_put(p, wr_req->addr);
    p += modbus_cvt_u16_put(p, wr_req->nb);
    p += modbus_cvt_u8_put(p, wr_req->dlen);
    memcpy(p, wr_req->pdata, wr_req->dlen);
    p += wr_req->dlen;

    return((int)(p - buf));
}

static int modbus_pdu_wr_req_parse(const uint8_t *buf, int len, mb_pdu_wr_req_t *wr_req)
{
    if (len < 7)
    {
        return(0);
    }

    uint8_t *p = (uint8_t *)buf;

    p += modbus_cvt_u8_get(p, &(wr_req->fc));
    p += modbus_cvt_u16_get(p, &(wr_req->addr));
    p += modbus_cvt_u16_get(p, &(wr_req->nb));
    p += modbus_cvt_u8_get(p, &(wr_req->dlen));
    wr_req->pdata = p;
    p += wr_req->dlen;

    return((int)(p - buf));
}

static int modbus_pdu_wr_rsp_make(uint8_t *buf, const mb_pdu_wr_rsp_t *wr_rsp)
{
    uint8_t *p = buf;

    p += modbus_cvt_u8_put(p, wr_rsp->fc);
    p += modbus_cvt_u16_put(p, wr_rsp->addr);
    p += modbus_cvt_u16_put(p, wr_rsp->nb);

    return((int)(p - buf));
}

static int modbus_pdu_wr_rsp_parse(const uint8_t *buf, int len, mb_pdu_wr_rsp_t *wr_rsp)
{
    if (len < 5)
    {
        return(0);
    }

    uint8_t *p = (uint8_t *)buf;

    p += modbus_cvt_u8_get(p, &(wr_rsp->fc));
    p += modbus_cvt_u16_get(p, &(wr_rsp->addr));
    p += modbus_cvt_u16_get(p, &(wr_rsp->nb));

    return((int)(p - buf));
}

static int modbus_pdu_mask_wr_make(uint8_t *buf, const mb_pdu_mask_wr_t *mask_wr)
{
    uint8_t *p = buf;

    p += modbus_cvt_u8_put(p, mask_wr->fc);
    p += modbus_cvt_u16_put(p, mask_wr->addr);
    p += modbus_cvt_u16_put(p, mask_wr->val_and);
    p += modbus_cvt_u16_put(p, mask_wr->val_or);

    return((int)(p - buf));
}

static int modbus_pdu_mask_wr_parse(const uint8_t *buf, int len, mb_pdu_mask_wr_t *mask_wr)
{
    if (len < 7)
    {
        return(0);
    }

    uint8_t *p = (uint8_t *)buf;

    p += modbus_cvt_u8_get(p, &(mask_wr->fc));
    p += modbus_cvt_u16_get(p, &(mask_wr->addr));
    p += modbus_cvt_u16_get(p, &(mask_wr->val_and));
    p += modbus_cvt_u16_get(p, &(mask_wr->val_or));

    return((int)(p - buf));
}

static int modbus_pdu_wr_rd_req_make(uint8_t *buf, const mb_pdu_wr_rd_req_t *wr_rd_req)
{
    uint8_t *p = buf;

    p += modbus_cvt_u8_put(p, wr_rd_req->fc);
    p += modbus_cvt_u16_put(p, wr_rd_req->rd_addr);
    p += modbus_cvt_u16_put(p, wr_rd_req->rd_nb);
    p += modbus_cvt_u16_put(p, wr_rd_req->wr_addr);
    p += modbus_cvt_u16_put(p, wr_rd_req->wr_nb);
    p += modbus_cvt_u8_put(p, wr_rd_req->dlen);
    memcpy(p, wr_rd_req->pdata, wr_rd_req->dlen);
    p += wr_rd_req->dlen;

    return((int)(p - buf));
}

static int modbus_pdu_wr_rd_req_parse(const uint8_t *buf, int len, mb_pdu_wr_rd_req_t *wr_rd_req)
{
    if (len < 11)
    {
        return(0);
    }

    uint8_t *p = (uint8_t *)buf;

    p += modbus_cvt_u8_get(p, &(wr_rd_req->fc));
    p += modbus_cvt_u16_get(p, &(wr_rd_req->rd_addr));
    p += modbus_cvt_u16_get(p, &(wr_rd_req->rd_nb));
    p += modbus_cvt_u16_get(p, &(wr_rd_req->wr_addr));
    p += modbus_cvt_u16_get(p, &(wr_rd_req->wr_nb));
    p += modbus_cvt_u8_get(p, &(wr_rd_req->dlen));
    wr_rd_req->pdata = p;
    p += wr_rd_req->dlen;

    return((int)(p - buf));
}

static int modbus_pdu_req_make(uint8_t *buf, const mb_pdu_t *pdu)//生成pdu请求帧, 返回帧长度, 失败返回0
{
    switch(pdu->fc)
    {
    case MODBUS_FC_READ_COILS :
        return(modbus_pdu_rd_req_make(buf, (mb_pdu_rd_req_t *)pdu));
    case MODBUS_FC_READ_DISCRETE_INPUTS :
        return(modbus_pdu_rd_req_make(buf, (mb_pdu_rd_req_t *)pdu));
    case MODBUS_FC_READ_HOLDING_REGISTERS :
        return(modbus_pdu_rd_req_make(buf, (mb_pdu_rd_req_t *)pdu));
    case MODBUS_FC_READ_INPUT_REGISTERS :
        return(modbus_pdu_rd_req_make(buf, (mb_pdu_rd_req_t *)pdu));
    case MODBUS_FC_WRITE_SINGLE_COIL :
        return(modbus_pdu_wr_single_make(buf, (mb_pdu_wr_single_t *)pdu));
    case MODBUS_FC_WRITE_SINGLE_REGISTER :
        return(modbus_pdu_wr_single_make(buf, (mb_pdu_wr_single_t *)pdu));
    case MODBUS_FC_READ_EXCEPTION_STATUS :
        break;
    case MODBUS_FC_WRITE_MULTIPLE_COILS :
        return(modbus_pdu_wr_req_make(buf, (mb_pdu_wr_req_t *)pdu));
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS :
        return(modbus_pdu_wr_req_make(buf, (mb_pdu_wr_req_t *)pdu));
    case MODBUS_FC_REPORT_SLAVE_ID :
        break;
    case MODBUS_FC_MASK_WRITE_REGISTER :
        return(modbus_pdu_mask_wr_make(buf, (mb_pdu_mask_wr_t *)pdu));
    case MODBUS_FC_WRITE_AND_READ_REGISTERS :
        return(modbus_pdu_wr_rd_req_make(buf, (mb_pdu_wr_rd_req_t *)pdu));
    default:
        break;
    }

    return(0);
}

static int modbus_pdu_req_parse(const uint8_t *buf, int len, mb_pdu_t *pdu)//解析pdu请求帧, 成功返回帧长度, 失败返回-1
{
    uint8_t fc = *buf;
    switch(fc)
    {
    case MODBUS_FC_READ_COILS :
        return(modbus_pdu_rd_req_parse(buf, len, (mb_pdu_rd_req_t *)pdu));
    case MODBUS_FC_READ_DISCRETE_INPUTS :
        return(modbus_pdu_rd_req_parse(buf, len, (mb_pdu_rd_req_t *)pdu));
    case MODBUS_FC_READ_HOLDING_REGISTERS :
        return(modbus_pdu_rd_req_parse(buf, len, (mb_pdu_rd_req_t *)pdu));
    case MODBUS_FC_READ_INPUT_REGISTERS :
        return(modbus_pdu_rd_req_parse(buf, len, (mb_pdu_rd_req_t *)pdu));
    case MODBUS_FC_WRITE_SINGLE_COIL :
        return(modbus_pdu_wr_single_parse(buf, len, (mb_pdu_wr_single_t *)pdu));
    case MODBUS_FC_WRITE_SINGLE_REGISTER :
        return(modbus_pdu_wr_single_parse(buf, len, (mb_pdu_wr_single_t *)pdu));
    case MODBUS_FC_READ_EXCEPTION_STATUS :
        break;
    case MODBUS_FC_WRITE_MULTIPLE_COILS :
        return(modbus_pdu_wr_req_parse(buf, len, (mb_pdu_wr_req_t *)pdu));
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS :
        return(modbus_pdu_wr_req_parse(buf, len, (mb_pdu_wr_req_t *)pdu));
    case MODBUS_FC_REPORT_SLAVE_ID :
        break;
    case MODBUS_FC_MASK_WRITE_REGISTER :
        return(modbus_pdu_mask_wr_parse(buf, len, (mb_pdu_mask_wr_t *)pdu));
    case MODBUS_FC_WRITE_AND_READ_REGISTERS :
        return(modbus_pdu_wr_rd_req_parse(buf, len, (mb_pdu_wr_rd_req_t *)pdu));
    default:
        break;
    }

    return(-1);
}

static int modbus_pdu_rsp_make(uint8_t *buf, const mb_pdu_t *pdu)//生成pdu响应帧, 返回帧长度, 失败返回0
{
    if (MODBUS_FC_EXCEPT_CHK(pdu->fc))
    {
        return(modbus_pdu_except_make(buf, (mb_pdu_except_t *)pdu));
    }

    switch(pdu->fc)
    {
    case MODBUS_FC_READ_COILS :
        return(modbus_pdu_rd_rsp_make(buf, (mb_pdu_rd_rsp_t *)pdu));
    case MODBUS_FC_READ_DISCRETE_INPUTS :
        return(modbus_pdu_rd_rsp_make(buf, (mb_pdu_rd_rsp_t *)pdu));
    case MODBUS_FC_READ_HOLDING_REGISTERS :
        return(modbus_pdu_rd_rsp_make(buf, (mb_pdu_rd_rsp_t *)pdu));
    case MODBUS_FC_READ_INPUT_REGISTERS :
        return(modbus_pdu_rd_rsp_make(buf, (mb_pdu_rd_rsp_t *)pdu));
    case MODBUS_FC_WRITE_SINGLE_COIL :
        return(modbus_pdu_wr_single_make(buf, (mb_pdu_wr_single_t *)pdu));
    case MODBUS_FC_WRITE_SINGLE_REGISTER :
        return(modbus_pdu_wr_single_make(buf, (mb_pdu_wr_single_t *)pdu));
    case MODBUS_FC_READ_EXCEPTION_STATUS :
        break;
    case MODBUS_FC_WRITE_MULTIPLE_COILS :
        return(modbus_pdu_wr_rsp_make(buf, (mb_pdu_wr_rsp_t *)pdu));
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS :
        return(modbus_pdu_wr_rsp_make(buf, (mb_pdu_wr_rsp_t *)pdu));
    case MODBUS_FC_REPORT_SLAVE_ID :
        break;
    case MODBUS_FC_MASK_WRITE_REGISTER :
        return(modbus_pdu_mask_wr_make(buf, (mb_pdu_mask_wr_t *)pdu));
    case MODBUS_FC_WRITE_AND_READ_REGISTERS :
        return(modbus_pdu_rd_rsp_make(buf, (mb_pdu_rd_rsp_t *)pdu));
    default:
        break;
    }

    return(0);
}

static int modbus_pdu_rsp_parse(const uint8_t *buf, int len, mb_pdu_t *pdu)//解析pdu响应帧, 成功返回帧长度, 失败返回-1
{
    uint8_t fc = *buf;
    if (MODBUS_FC_EXCEPT_CHK(fc))
    {
        return(modbus_pdu_except_parse(buf, len, (mb_pdu_except_t *)pdu));
    }

    switch(fc)
    {
    case MODBUS_FC_READ_COILS :
        return(modbus_pdu_rd_rsp_parse(buf, len, (mb_pdu_rd_rsp_t *)pdu));
    case MODBUS_FC_READ_DISCRETE_INPUTS :
        return(modbus_pdu_rd_rsp_parse(buf, len, (mb_pdu_rd_rsp_t *)pdu));
    case MODBUS_FC_READ_HOLDING_REGISTERS :
        return(modbus_pdu_rd_rsp_parse(buf, len, (mb_pdu_rd_rsp_t *)pdu));
    case MODBUS_FC_READ_INPUT_REGISTERS :
        return(modbus_pdu_rd_rsp_parse(buf, len, (mb_pdu_rd_rsp_t *)pdu));
    case MODBUS_FC_WRITE_SINGLE_COIL :
        return(modbus_pdu_wr_single_parse(buf, len, (mb_pdu_wr_single_t *)pdu));
    case MODBUS_FC_WRITE_SINGLE_REGISTER :
        return(modbus_pdu_wr_single_parse(buf, len, (mb_pdu_wr_single_t *)pdu));
    case MODBUS_FC_READ_EXCEPTION_STATUS :
        break;
    case MODBUS_FC_WRITE_MULTIPLE_COILS :
        return(modbus_pdu_wr_rsp_parse(buf, len, (mb_pdu_wr_rsp_t *)pdu));
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS :
        return(modbus_pdu_wr_rsp_parse(buf, len, (mb_pdu_wr_rsp_t *)pdu));
    case MODBUS_FC_REPORT_SLAVE_ID :
        break;
    case MODBUS_FC_MASK_WRITE_REGISTER :
        return(modbus_pdu_mask_wr_parse(buf, len, (mb_pdu_mask_wr_t *)pdu));
    case MODBUS_FC_WRITE_AND_READ_REGISTERS :
        return(modbus_pdu_rd_rsp_parse(buf, len, (mb_pdu_rd_rsp_t *)pdu));
    default:
        break;
    }

    return(-1);
}

int modbus_pdu_make(uint8_t *buf, const mb_pdu_t *pdu, mb_pdu_type_t type)//生成pdu帧, 返回帧长度, 错误返回0
{
    switch(type)
    {
        case MB_PDU_TYPE_REQ:
            return(modbus_pdu_req_make(buf, pdu));
        case MB_PDU_TYPE_RSP:
            return(modbus_pdu_rsp_make(buf, pdu));
        default:
            break;
    }
    return(0);
}

int modbus_pdu_parse(const uint8_t *buf, int len, mb_pdu_t *pdu, mb_pdu_type_t type)
{
    switch(type)
    {
        case MB_PDU_TYPE_REQ:
            return(modbus_pdu_req_parse(buf, len, pdu));
        case MB_PDU_TYPE_RSP:
            return(modbus_pdu_rsp_parse(buf, len, pdu));
        default:
            break;
    }
    return(0);
}




