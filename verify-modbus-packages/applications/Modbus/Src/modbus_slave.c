/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-13     18452       the first version
 */
#include "bsp_sys.h"



#ifdef MB_USING_SLAVE


MB_WEAK int modbus_port_read_disc(uint16_t addr, uint8_t *pbit)//读离散量输入, 返回 : 0-成功, -2-地址错误
{
    MB_ASSERT(pbit != NULL);

    return(-2);
}

MB_WEAK int modbus_port_read_coil(uint16_t addr, uint8_t *pbit)//读线圈, 返回 : 0-成功, -2-地址错误
{
    MB_ASSERT(pbit != NULL);

    return(-2);
}

MB_WEAK int modbus_port_write_coil(uint16_t addr, uint8_t bit)//写线圈, 返回 : 0-成功, -2-地址错误, -4-设备故障
{
    return(-2);
}

MB_WEAK int modbus_port_read_input(uint16_t addr, uint16_t *preg)//读输入寄存器, 返回 : 0-成功, -2-地址错误
{
    MB_ASSERT(preg != NULL);

    return(-2);
}

MB_WEAK int modbus_port_read_hold(uint16_t addr, uint16_t *preg)//读保持寄存器, 返回 : 0-成功, -2-地址错误
{
    MB_ASSERT(preg != NULL);

    return(-2);
}

MB_WEAK int modbus_port_write_hold(uint16_t addr, uint16_t reg)//写保持寄存器, 返回 : 0-成功, -2-地址错误, -3-值非法, -4-设备故障
{
    return(-2);
}




/**
 * @brief  处理 Modbus 从站读线圈请求（功能码 0x01）
 *
 * 本函数运行于 Modbus 从站（Slave），用于响应主站发送的“读线圈状态”（Read Coils）请求。
 * 它从用户注册的回调函数中逐个读取线圈状态，构造位图响应数据，并填充到 PDU 响应结构中。
 *
 * @param[in,out] hinst  Modbus 从站实例指针，包含回调函数表和数据缓冲区
 * @param[in,out] pdu    PDU 结构体指针
 *                       - 输入：包含 rd_req（请求：起始地址、数量）
 *                       - 输出：填充 rd_rsp（响应：字节计数、位图数据）或 exc（异常响应）
 *
 * @return 无返回值
 *   - 成功：填充 pdu->rd_rsp.dlen 和 pdu->rd_rsp.pdata
 *   - 失败：设置 pdu->exc.ec 和 pdu->exc.fc，触发异常响应
 *
 * @note
 *   - 依赖用户注册的回调函数 hinst->cb->read_coil(addr, &bit)
 *   - 响应数据使用 hinst->datas 作为临时位图缓冲区（低位在前，符合 Modbus 规范）
 *   - 位图字节数 = (nb + 7) / 8，向上取整
 *   - 上层调用 modbus_rtu_frame_make() 会自动添加地址字段和 CRC
 *
 * @warning
 *   - 若未注册 read_coil 回调，返回异常码 0x04（从站设备故障）
 *   - 用户回调返回负值时，自动转换为 Modbus 标准异常码（如 -2 → 0x02）
 *   - hinst->datas 必须足够大（至少 256 字节），否则可能溢出
 */
static void modbus_slave_pdu_deal_read_coils(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    // 1. 检查是否有注册读线圈回调函数
    if ((hinst->cb == NULL) || (hinst->cb->read_coil == NULL))
    {
        pdu->exc.ec = MODBUS_EC_SLAVE_OR_SERVER_FAILURE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    // 2. 起始线圈地址（如 0）
    uint16_t addr = pdu->rd_req.addr;
    // 3. 要读取的线圈数量（如 8）
    int nb = pdu->rd_req.nb;
    // 4. 清空临时缓冲区
    memset(hinst->datas, 0, sizeof(hinst->datas));
    for (int i=0; i<nb; i++)
    {
        uint8_t bit;
        int rst = hinst->cb->read_coil(addr + i, &bit);
        if (rst < 0)
        {
            pdu->exc.ec = -rst;
            pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
            return;
        }
        modbus_bitmap_set(hinst->datas, i, bit);
    }

    pdu->rd_rsp.dlen = (nb + 7) / 8;
    pdu->rd_rsp.pdata = hinst->datas;
}


/**
 * @brief  处理 Modbus 从站读离散输入请求（功能码 0x02）
 *
 * 本函数运行于 Modbus 从站（Slave），用于响应主站发送的“读离散输入”（Read Discrete Inputs）请求。
 * 离散输入通常对应物理输入信号（如开关、传感器），只读不可写。
 * 函数通过用户注册的回调函数逐个读取输入状态，构造位图响应数据，并填充到 PDU 响应结构中。
 *
 * @param[in,out] hinst  Modbus 从站实例指针，包含回调函数表和数据缓冲区
 * @param[in,out] pdu    PDU 结构体指针
 *                       - 输入：包含 rd_req（请求：起始地址、数量）
 *                       - 输出：填充 rd_rsp（响应：字节计数、位图数据）或 exc（异常响应）
 *
 * @return 无返回值
 *   - 成功：填充 pdu->rd_rsp.dlen 和 pdu->rd_rsp.pdata
 *   - 失败：设置 pdu->exc.ec 和 pdu->exc.fc，触发异常响应
 *
 * @note
 *   - 依赖用户注册的回调函数 hinst->cb->read_disc(addr, &bit)
 *   - 响应数据使用 hinst->datas 作为临时位图缓冲区（低位在前，符合 Modbus 规范）
 *   - 位图字节数 = (nb + 7) / 8，向上取整
 *   - 与读线圈（0x01）逻辑完全相同，仅功能码和回调函数不同
 *   - 上层调用 modbus_rtu_frame_make() 会自动添加地址字段和 CRC
 *
 * @warning
 *   - 若未注册 read_disc 回调，返回异常码 0x04（从站设备故障）
 *   - 用户回调返回负值时，自动转换为 Modbus 标准异常码（如 -2 → 0x02）
 *   - hinst->datas 必须足够大（至少 256 字节），否则可能溢出
 *   - 离散输入地址空间独立于线圈地址空间
 */
static void modbus_slave_pdu_deal_read_discs(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    /* 1. 检查是否注册了读离散输入回调函数 */
    if ((hinst->cb == NULL) || (hinst->cb->read_disc == NULL))
    {
        /* 未实现读离散输入功能 → 返回异常响应 0x04（从站设备故障） */
        pdu->exc.ec = MODBUS_EC_SLAVE_OR_SERVER_FAILURE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    /* 2. 提取请求参数 */
    uint16_t addr = pdu->rd_req.addr;   // 起始离散输入地址（0 开始）
    int nb = pdu->rd_req.nb;            // 要读取的输入数量
    /* 3. 清空临时数据缓冲区，防止旧数据干扰 */
    memset(hinst->datas, 0, sizeof(hinst->datas));
    /* 4. 遍历每个离散输入，调用用户回调获取状态 */
    for (int i=0; i<nb; i++)
    {
        // 单个输入状态（0 或 1）
        uint8_t bit;
        // 用户实现逻辑（如读 GPIO）
        int rst = hinst->cb->read_disc(addr + i, &bit);
        /* 用户回调失败 → 转换为 Modbus 异常响应 */
        if (rst < 0)
        {
            pdu->exc.ec = -rst;
            pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
            return;
        }
        /* 将 bit 写入位图缓冲区（低位在前） */
        modbus_bitmap_set(hinst->datas, i, bit);
    }
    /* 5. 构造响应字段 */
    pdu->rd_rsp.dlen = (nb + 7) / 8;    // 位图字节数：每 8 位占 1 字节，向上取整
    pdu->rd_rsp.pdata = hinst->datas;   // 指向位图数据首地址

    /* 响应格式示例（nb=10）：
         *   [FC=0x02][字节计数=2][位图数据 2 字节]
         *   位图：bit0 在最低位，bit9 在第 2 字节次低位
         */
}


/**
 * @brief  处理 Modbus 从站读保持寄存器请求（功能码 0x03）
 *
 * 本函数运行于 Modbus 从站（Slave），用于响应主站发送的“读保持寄存器”（Read Holding Registers）请求。
 * 保持寄存器为 **可读可写** 的 16 位寄存器，常用于存储配置参数、设定值、运行状态等。
 * 函数通过用户注册的回调函数逐个读取寄存器值，转换为大端字节序（网络序），并填充到响应缓冲区。
 *
 * @param[in,out] hinst  Modbus 从站实例指针，包含回调函数表和数据缓冲区
 * @param[in,out] pdu    PDU 结构体指针
 *                       - 输入：包含 rd_req（请求：起始地址、数量）
 *                       - 输出：填充 rd_rsp（响应：字节计数、寄存器数据）或 exc（异常响应）
 *
 * @return 无返回值
 *   - 成功：填充 pdu->rd_rsp.dlen 和 pdu->rd_rsp.pdata
 *   - 失败：设置 pdu->exc.ec 和 pdu->exc.fc，触发异常响应
 *
 * @note
 *   - 依赖用户注册的回调函数 hinst->cb->read_hold(addr, &val)
 *   - 每个寄存器占 2 字节，大端序（高字节在前，低字节在后），符合 Modbus 协议
 *   - 响应数据使用 hinst->datas 作为临时缓冲区
 *   - 总字节数 = nb × 2
 *   - 上层调用 modbus_rtu_frame_make() 会自动添加地址字段和 CRC
 *
 * @warning
 *   - 若未注册 read_hold 回调，返回异常码 0x04（从站设备故障）
 *   - 用户回调返回负值时，自动转换为 Modbus 标准异常码（如 -2 → 0x02）
 *   - hinst->datas 必须足够大（至少 256 字节），否则可能溢出
 *   - 保持寄存器地址空间独立于输入寄存器（0x04）
 */
static void modbus_slave_pdu_deal_read_holds(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    /* 1. 检查是否注册了读保持寄存器回调函数 */
    if ((hinst->cb == NULL) || (hinst->cb->read_hold == NULL))
    {
        /* 未实现读保持寄存器功能 → 返回异常响应 0x04（从站设备故障） */
        pdu->exc.ec = MODBUS_EC_SLAVE_OR_SERVER_FAILURE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    /* 2. 提取请求参数 */
    uint16_t addr = pdu->rd_req.addr;
    int nb = pdu->rd_req.nb;
    /* 3. 初始化数据指针，指向从站通用缓冲区 */
    uint8_t *p = hinst->datas;
    /* 4. 遍历每个寄存器，调用用户回调获取值并写入缓冲区（大端序） */
    for (int i=0; i<nb; i++)
    {
        uint16_t val;
        int rst = hinst->cb->read_hold(addr + i, &val);
        if (rst < 0)
        {
            pdu->exc.ec = -rst;
            pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
            return;
        }
        /* 将 16 位值以大端序写入缓冲区：高字节在前 */
        p += modbus_cvt_u16_put(p, val);
    }

    /* 5. 构造响应字段 */
    pdu->rd_rsp.dlen = 2 * nb;          // 总字节数 = 寄存器数 × 2
    pdu->rd_rsp.pdata = hinst->datas;   // 指向寄存器数据首地址

    /* 响应格式示例（nb=2, 值=0x1234, 0x5678）
     *   [FC=0x03][字节计数=4][12 34 56 78]
     */
}

static void modbus_slave_pdu_deal_read_inputs(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    if ((hinst->cb == NULL) || (hinst->cb->read_input == NULL))
    {
        pdu->exc.ec = MODBUS_EC_SLAVE_OR_SERVER_FAILURE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    uint16_t addr = pdu->rd_req.addr;
    int nb = pdu->rd_req.nb;
    uint8_t *p = hinst->datas;
    for (int i=0; i<nb; i++)
    {
        uint16_t val;
        int rst = hinst->cb->read_input(addr + i, &val);
        if (rst < 0)
        {
            pdu->exc.ec = -rst;
            pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
            return;
        }
        p += modbus_cvt_u16_put(p, val);
    }

    pdu->rd_rsp.dlen = 2 * nb;
    pdu->rd_rsp.pdata = hinst->datas;
}


/**
 * @brief  处理 Modbus 从站写单个线圈请求（功能码 0x05）
 *
 * 本函数运行于 Modbus 从站（Slave），用于响应主站发送的“写单个线圈”（Write Single Coil）请求。
 * 线圈为 **可读可写** 的 1 位输出，常用于控制继电器、LED、阀门等。
 * 函数验证写入值合法性后，调用用户注册的回调函数执行实际写入操作。
 *
 * @param[in,out] hinst  Modbus 从站实例指针，包含回调函数表
 * @param[in,out] pdu    PDU 结构体指针
 *                       - 输入：包含 wr_single（请求：地址、值）
 *                       - 输出：成功时保持原请求字段（响应格式与请求相同）
 *                       - 输出：失败时填充 exc（异常响应）
 *
 * @return 无返回值
 *   - 成功：响应数据 = 请求数据（地址 + 值），由上层封装
 *   - 失败：设置 pdu->exc.ec 和 pdu->exc.fc，触发异常响应
 *
 * @note
 *   - 依赖用户注册的回调函数 hinst->cb->write_coil(addr, bit)
 *   - Modbus 协议规定：写线圈值必须为 0xFF00（ON）或 0x0000（OFF）
 *   - 其他值（如 0x0100）视为非法 → 返回异常码 0x03
 *   - 成功响应格式与请求完全相同（回显）
 *   - 上层调用 modbus_rtu_frame_make() 会自动添加地址字段和 CRC
 *
 * @warning
 *   - 若未注册 write_coil 回调，返回异常码 0x04（从站设备故障）
 *   - 用户回调返回负值时，自动转换为 Modbus 标准异常码（如 -2 → 0x02）
 *   - 不使用 hinst->datas 缓冲区（响应直接回显请求）
 */
static void modbus_slave_pdu_deal_write_coil(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    /* 1. 检查是否注册了写线圈回调函数 */
    if ((hinst->cb == NULL) || (hinst->cb->write_coil == NULL))
    {
        pdu->exc.ec = MODBUS_EC_SLAVE_OR_SERVER_FAILURE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    /* 2. 提取请求参数 */
    uint16_t addr = pdu->wr_single.addr;    // 目标线圈地址
    uint16_t val = pdu->wr_single.val;      // 写入值（必须为 0xFF00 或 0x0000）
    /* 3. 验证写入值是否合法（Modbus 协议强制要求） */
    if ((val != 0xFF00) && (val != 0x0000))
    {
        /* 非法数据值 → 返回异常码 0x03 */
        pdu->exc.ec = MODBUS_EC_ILLEGAL_DATA_VALUE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }
    /* 4. 转换为 1 位值并调用用户回调执行实际写入 */
    int rst = hinst->cb->write_coil(addr, (val ? 1 : 0));
    /* 5. 用户回调失败 → 转换为 Modbus 异常响应 */
    if (rst < 0)
    {
        pdu->exc.ec = -rst;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    /* 6. 成功：响应与请求完全相同（回显） */
    /* 不需要填充 rd_rsp，上层会直接使用 wr_single 字段作为响应 */
}



/**
 * @brief  处理 Modbus 从站写单个保持寄存器请求（功能码 0x06）
 *
 * 本函数运行于 Modbus 从站（Slave），用于响应主站发送的“写单个保持寄存器”（Write Single Register）请求。
 * 保持寄存器为 **可读可写** 的 16 位寄存器，常用于设置参数、阈值、模式等。
 * 函数直接调用用户注册的回调函数执行写入操作，成功时回显请求数据。
 *
 * @param[in,out] hinst  Modbus 从站实例指针，包含回调函数表
 * @param[in,out] pdu    PDU 结构体指针
 *                       - 输入：包含 wr_single（请求：地址、值）
 *                       - 输出：成功时保持原请求字段（响应格式与请求相同）
 *                       - 输出：失败时填充 exc（异常响应）
 *
 * @return 无返回值
 *   - 成功：响应数据 = 请求数据（地址 + 值），由上层封装
 *   - 失败：设置 pdu->exc.ec 和 pdu->exc.fc，触发异常响应
 *
 * @note
 *   - 依赖用户注册的回调函数 hinst->cb->write_hold(addr, val)
 *   - 写入值 `val` 为 16 位无符号整数（0 ~ 65535），大端序传输
 *   - 成功响应格式与请求完全相同（回显）
 *   - 上层调用 modbus_rtu_frame_make() 会自动添加地址字段和 CRC
 *   - 与功能码 0x10（写多个寄存器）配套使用
 *
 * @warning
 *   - 若未注册 write_hold 回调，返回异常码 0x04（从站设备故障）
 *   - 用户回调返回负值时，自动转换为 Modbus 标准异常码（如 -2 → 0x02）
 *   - 不使用 hinst->datas 缓冲区（响应直接回显请求）
 *   - 保持寄存器地址空间独立于输入寄存器（0x04）
 */
static void modbus_slave_pdu_deal_write_reg(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    /* 1. 检查是否注册了写保持寄存器回调函数 */
    if ((hinst->cb == NULL) || (hinst->cb->write_hold == NULL))
    {
        pdu->exc.ec = MODBUS_EC_SLAVE_OR_SERVER_FAILURE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    /* 2. 提取请求参数 */
    uint16_t addr = pdu->wr_single.addr;    // 目标保持寄存器地址
    uint16_t val = pdu->wr_single.val;      // 写入值（16 位，大端序）
    /* 3. 调用用户回调执行实际写入操作 */
    int rst = hinst->cb->write_hold(addr, val);
    /* 4. 用户回调失败 → 转换为 Modbus 异常响应 */
    if (rst < 0)
    {
        pdu->exc.ec = -rst;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }
    /* 5. 成功：响应与请求完全相同（回显） */
    /* 不需要填充 rd_rsp，上层会直接使用 wr_single 字段作为响应 */
}


/**
 * @brief  处理 Modbus 从站写多个线圈请求（功能码 0x0F）
 *
 * 本函数运行于 Modbus 从站（Slave），用于响应主站发送的“写多个线圈”（Write Multiple Coils）请求。
 * 支持 **批量写入 1~1968 个线圈**，常用于同步控制多个继电器、阀门、LED 等输出设备。
 * 函数从请求数据中提取位图（低位在前），逐位调用用户回调执行写入。
 *
 * @param[in,out] hinst  Modbus 从站实例指针，包含回调函数表
 * @param[in,out] pdu    PDU 结构体指针
 *                       - 输入：包含 wr_req（请求：起始地址、数量、位图数据）
 *                       - 输出：成功时填充 wr_rsp（响应：地址 + 数量）
 *                       - 输出：失败时填充 exc（异常响应）
 *
 * @return 无返回值
 *   - 成功：填充 pdu->wr_rsp.addr 和 pdu->wr_rsp.nb，上层封装响应
 *   - 失败：设置 pdu->exc.ec 和 pdu->exc.fc，触发异常响应
 *
 * @note
 *   - 依赖用户注册的回调函数 hinst->cb->write_coil(addr, bit)
 *   - 位图数据 `pdu->wr_req.pdata` 采用 **低位在前**（bit0 在最低位），符合 Modbus 规范
 *   - 位图字节数 = (nb + 7) / 8，向上取整
 *   - 成功响应格式：`[FC][地址H][地址L][数量H][数量L]`
 *   - 上层调用 modbus_rtu_frame_make() 会自动添加地址字段和 CRC
 *
 * @warning
 *   - 若未注册 write_coil 回调，返回异常码 0x04（从站设备故障）
 *   - 用户回调返回负值时，自动转换为 Modbus 标准异常码（如 -2 → 0x02）
 *   - 不使用 hinst->datas 缓冲区（请求数据直接来自 pdu->wr_req.pdata）
 *   - 必须确保 pdu->wr_req.pdata 指向有效位图数据（由上层解析保证）
 */
static void modbus_slave_pdu_deal_write_coils(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    if ((hinst->cb == NULL) || (hinst->cb->write_coil == NULL))
    {
        pdu->exc.ec = MODBUS_EC_SLAVE_OR_SERVER_FAILURE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    uint16_t addr = pdu->wr_req.addr;   // 起始线圈地址
    int nb = pdu->wr_req.nb;            // 要写入的线圈数量
    uint8_t *pbits = pdu->wr_req.pdata; // 指向位图数据（低位在前）

    /* 3. 遍历每个线圈，提取位图中的 bit 并写入 */
    for (int i=0; i<nb; i++)
    {
        /* 从位图中读取第 i 位的值（bit0 在最低位） */
        uint8_t bit = modbus_bitmap_get(pbits, i);
        /* 调用用户回调执行实际写入 */
        int rst = hinst->cb->write_coil(addr + i, bit);
        if (rst < 0)
        {
            pdu->exc.ec = -rst;
            pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
            return;
        }
    }
}


/**
 * @brief  处理 Modbus 从站写多个保持寄存器请求（功能码 0x10）
 *
 * 本函数运行于 Modbus 从站（Slave），用于响应主站发送的“写多个保持寄存器”（Write Multiple Registers）请求。
 * 支持 **批量写入 1~123 个 16 位保持寄存器**，常用于同步设置参数、阈值、控制字等。
 * 函数从请求数据流中解析大端序的寄存器值，逐个调用用户回调执行写入。
 *
 * @param[in,out] hinst  Modbus 从站实例指针，包含回调函数表
 * @param[in,out] pdu    PDU 结构体指针
 *                       - 输入：包含 wr_req（请求：起始地址、数量、字节计数、数据流）
 *                       - 输出：成功时填充 wr_rsp（响应：地址 + 数量）
 *                       - 输出：失败时填充 exc（异常响应）
 *
 * @return 无返回值
 *   - 成功：填充 pdu->wr_rsp.addr 和 pdu->wr_rsp.nb，上层封装响应
 *   - 失败：设置 pdu->exc.ec 和 pdu->exc.fc，触发异常响应
 *
 * @note
 *   - 依赖用户注册的回调函数 hinst->cb->write_hold(addr, val)
 *   - 寄存器数据采用 **大端字节序**（高字节在前），由 modbus_cvt_u16_get() 解析
 *   - 总字节数 = nb × 2，必须与 pdu->wr_req.dlen 一致（由上层验证）
 *   - 成功响应格式：`[FC][地址H][地址L][数量H][数量L]`
 *   - 上层调用 modbus_rtu_frame_make() 会自动添加地址字段和 CRC
 *
 * @warning
 *   - 若未注册 write_hold 回调，返回异常码 0x04（从站设备故障）
 *   - 用户回调返回负值时，自动转换为 Modbus 标准异常码（如 -2 → 0x02）
 *   - 不使用 hinst->datas 缓冲区（请求数据直接来自 pdu->wr_req.pdata）
 *   - 必须确保 pdu->wr_req.pdata 指向有效数据流且长度正确（由上层保证）
 */
static void modbus_slave_pdu_deal_write_regs(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    if ((hinst->cb == NULL) || (hinst->cb->write_hold == NULL))
    {
        pdu->exc.ec = MODBUS_EC_SLAVE_OR_SERVER_FAILURE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    uint16_t addr = pdu->wr_req.addr;
    int nb = pdu->wr_req.nb;
    uint8_t *p = pdu->wr_req.pdata;
    for (int i=0; i<nb; i++)
    {
        uint16_t val;
        p += modbus_cvt_u16_get(p, &val);
        int rst = hinst->cb->write_hold(addr + i, val);
        if (rst < 0)
        {
            pdu->exc.ec = -rst;
            pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
            return;
        }
    }
}


/**
 * @brief  处理 Modbus 从站掩码写寄存器请求（功能码 0x16）
 *
 * 本函数运行于 Modbus 从站（Slave），用于响应主站发送的“掩码写寄存器”（Mask Write Register）请求。
 * 该功能码用于 **对单个保持寄存器的某些位进行原子性修改**，而不影响其他位。
 * 常用于 **位操作控制**，如：设置/清除标志位、切换模式、控制状态机等。
 *
 * 掩码逻辑公式：
 *      new_value = (current_value & AND_Mask) | (OR_Mask & ~AND_Mask)
 *
 *  - AND_Mask 为 1 的位：保持原值
 *  - AND_Mask 为 0 的位：由 OR_Mask 决定（0=清零，1=置位）
 *
 * @param[in,out] hinst  Modbus 从站实例指针，包含回调函数表
 * @param[in,out] pdu    PDU 结构体指针
 *                       - 输入：包含 mask_wr（请求：地址、AND掩码、OR掩码）
 *                       - 输出：成功时保持原请求字段（响应格式与请求相同）
 *                       - 输出：失败时填充 exc（异常响应）
 *
 * @return 无返回值
 *   - 成功：响应数据 = 请求数据（地址 + AND + OR），由上层封装
 *   - 失败：设置 pdu->exc.ec 和 pdu->exc.fc，触发异常响应
 *
 * @note
 *   - 依赖用户注册的回调函数：
 *       - hinst->cb->read_hold(addr, &val)  // 读取当前值
 *       - hinst->cb->write_hold(addr, val) // 写回新值
 *   - 所有值均为 **16 位大端序** 传输
 *   - 成功响应格式与请求完全相同（回显）
 *   - 上层调用 modbus_rtu_frame_make() 会自动添加地址字段和 CRC
 *
 * @warning
 *   - 若未注册 read_hold 或 write_hold 回调，返回异常码 0x04（从站设备故障）
 *   - 用户回调返回负值时，自动转换为 Modbus 标准异常码（如 -2 → 0x02）
 *   - 不使用 hinst->datas 缓冲区（响应直接回显请求）
 *   - 保持寄存器地址空间独立于输入寄存器
 */
static void modbus_slave_pdu_deal_mask_write_reg(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    if ((hinst->cb == NULL) || (hinst->cb->read_hold == NULL) || (hinst->cb->write_hold == NULL))
    {
        pdu->exc.ec = MODBUS_EC_SLAVE_OR_SERVER_FAILURE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    uint16_t addr = pdu->mask_wr.addr;
    uint16_t val_and = pdu->mask_wr.val_and;
    uint16_t val_or = pdu->mask_wr.val_or;
    uint16_t val;
    int rst = hinst->cb->read_hold(addr, &val);
    if (rst < 0)
    {
        pdu->exc.ec = -rst;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }
    val = ((val & val_and) | (val_or & ~val_and));
    rst = hinst->cb->write_hold(addr, val);
    if (rst < 0)
    {
        pdu->exc.ec = -rst;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }
}


/**
 * @brief  处理 Modbus 从站读写多个寄存器请求（功能码 0x17）
 *
 * 本函数运行于 Modbus 从站（Slave），用于响应主站发送的“读写多个寄存器”（Read/Write Multiple Registers）请求。
 * 该功能码 **原子性地先写多个保持寄存器，再读多个保持寄存器**，常用于：
 *   - 批量设置参数后立即读取状态
 *   - 实现“写后读验证”
 *   - 高效配置 + 反馈场景（如 PLC 控制）
 *
 * 执行顺序：**先写（Write） → 后读（Read）**
 *
 * @param[in,out] hinst  Modbus 从站实例指针，包含回调函数表和数据缓冲区
 * @param[in,out] pdu    PDU 结构体指针
 *                       - 输入：包含 wr_rd_req（请求：读地址、读数量、写地址、写数量、写数据流）
 *                       - 输出：成功时填充 rd_rsp（响应：字节计数 + 读出数据）
 *                       - 输出：失败时填充 exc（异常响应）
 *
 * @return 无返回值
 *   - 成功：填充 pdu->rd_rsp.dlen 和 pdu->rd_rsp.pdata（仅包含读出数据）
 *   - 失败：设置 pdu->exc.ec 和 pdu->exc.fc，触发异常响应
 *
 * @note
 *   - 依赖用户注册的回调函数：
 *       - hinst->cb->write_hold(addr, val)  // 写操作
 *       - hinst->cb->read_hold(addr, &val)  // 读操作
 *   - 写数据：从 pdu->wr_rd_req.pdata 解析，大端序
 *   - 读数据：写入 hinst->datas 缓冲区，大端序
 *   - 响应格式：`[FC][字节计数][读出数据流]`，**不包含写相关信息**
 *   - 上层调用 modbus_rtu_frame_make() 会自动添加地址字段和 CRC
 *
 * @warning
 *   - 若未注册 read_hold 或 write_hold 回调，返回异常码 0x04（从站设备故障）
 *   - 用户回调返回负值时，自动转换为 Modbus 标准异常码（如 -2 → 0x02）
 *   - 写操作失败时 **立即中止**，不执行读操作
 *   - hinst->datas 必须足够大（至少 252 字节），支持最大 125 个寄存器
 *   - 读写地址可相同或不同，协议允许重叠
 */
static void modbus_slave_pdu_deal_write_and_read_regs(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    if ((hinst->cb == NULL) || (hinst->cb->read_hold == NULL) || (hinst->cb->write_hold == NULL))
    {
        pdu->exc.ec = MODBUS_EC_SLAVE_OR_SERVER_FAILURE;
        pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
        return;
    }

    uint16_t rd_addr = pdu->wr_rd_req.rd_addr;
    int rd_nb = pdu->wr_rd_req.rd_nb;
    uint16_t wr_addr = pdu->wr_rd_req.wr_addr;
    int wr_nb = pdu->wr_rd_req.wr_nb;
    uint8_t *p = pdu->wr_rd_req.pdata;
    for (int i=0; i<wr_nb; i++)
    {
        uint16_t val;
        p += modbus_cvt_u16_get(p, &val);
        int rst = hinst->cb->write_hold(wr_addr + i, val);
        if (rst < 0)
        {
            pdu->exc.ec = -rst;
            pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
            return;
        }
    }

    p = hinst->datas;
    for (int i=0; i<rd_nb; i++)
    {
        uint16_t val;
        int rst = hinst->cb->read_hold(rd_addr + i, &val);
        if (rst < 0)
        {
            pdu->exc.ec = -rst;
            pdu->exc.fc = MODBUS_FC_EXCEPT_MAKE(pdu->exc.fc);
            return;
        }
        p += modbus_cvt_u16_put(p, val);
    }

    pdu->rd_rsp.dlen = rd_nb * 2;
    pdu->rd_rsp.pdata = hinst->datas;
}

static void modbus_slave_pdu_deal(mb_inst_t *hinst, mb_pdu_t *pdu)
{
    switch(pdu->fc)
    {
    case MODBUS_FC_READ_COILS :
        modbus_slave_pdu_deal_read_coils(hinst, pdu);
        break;
    case MODBUS_FC_READ_DISCRETE_INPUTS :
        modbus_slave_pdu_deal_read_discs(hinst, pdu);
        break;
    case MODBUS_FC_READ_HOLDING_REGISTERS :
        modbus_slave_pdu_deal_read_holds(hinst, pdu);
        break;
    case MODBUS_FC_READ_INPUT_REGISTERS :
        modbus_slave_pdu_deal_read_inputs(hinst, pdu);
        break;
    case MODBUS_FC_WRITE_SINGLE_COIL :
        modbus_slave_pdu_deal_write_coil(hinst, pdu);
        break;
    case MODBUS_FC_WRITE_SINGLE_REGISTER :
        modbus_slave_pdu_deal_write_reg(hinst, pdu);
        break;
    case MODBUS_FC_READ_EXCEPTION_STATUS :
        break;
    case MODBUS_FC_WRITE_MULTIPLE_COILS :
        modbus_slave_pdu_deal_write_coils(hinst, pdu);
        break;
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS :
        modbus_slave_pdu_deal_write_regs(hinst, pdu);
        break;
    case MODBUS_FC_REPORT_SLAVE_ID :
        break;
    case MODBUS_FC_MASK_WRITE_REGISTER :
        modbus_slave_pdu_deal_mask_write_reg(hinst, pdu);
        break;
    case MODBUS_FC_WRITE_AND_READ_REGISTERS :
        modbus_slave_pdu_deal_write_and_read_regs(hinst, pdu);
        break;
    default:
        break;
    }
}

#ifdef MB_USING_RTU_PROTOCOL
static void modbus_slave_recv_deal_rtu(mb_inst_t *hinst, uint8_t *buf, int len)
{
    mb_rtu_frm_t frm;
    int pdu_len = modbus_rtu_frame_parse(buf, len, &frm, MB_PDU_TYPE_REQ);
    if (pdu_len == 0)//帧错误, 不处理
    {
        return;
    }

    #ifdef MB_USING_ADDR_CHK
    if (frm.saddr != hinst->saddr)//地址不同, 不处理
    {
        return;
    }
    #endif

    if (pdu_len < 0)//功能码不支持, 响应异常帧
    {
        frm.pdu.exc.ec = MODBUS_EC_ILLEGAL_FUNCTION;
        frm.pdu.exc.fc = MODBUS_FC_EXCEPT_MAKE(frm.pdu.exc.fc);
    }
    else
    {
        modbus_slave_pdu_deal(hinst, &(frm.pdu));
    }

    int flen = modbus_rtu_frame_make(hinst->buf, &frm, MB_PDU_TYPE_RSP);
    modbus_send(hinst, hinst->buf, flen);
}
#endif

#ifdef MB_USING_TCP_PROTOCOL
static void modbus_slave_recv_deal_tcp(mb_inst_t *hinst, uint8_t *buf, int len)
{
    mb_tcp_frm_t frm;
    int pdu_len = modbus_tcp_frm_parse(buf, len, &frm, MB_PDU_TYPE_REQ);
    if (pdu_len == 0)//帧错误, 不处理
    {
        return;
    }

    #ifdef MB_USING_ADDR_CHK
    if ((frm.mbap.did != 0xFF) && (frm.mbap.did != hinst->saddr))//使用地址且地址不同, 不处理
    {
        return;
    }
    #endif

    #ifdef MB_USING_MBAP_CHK
    if (frm.mbap.pid != MB_TCP_MBAP_PID)//协议标识错误, 不处理
    {
        return;
    }
    #endif

    if (pdu_len < 0)//功能码不支持, 响应异常帧
    {
        frm.pdu.exc.ec = MODBUS_EC_ILLEGAL_FUNCTION;
        frm.pdu.exc.fc = MODBUS_FC_EXCEPT_MAKE(frm.pdu.exc.fc);
    }
    else
    {
        modbus_slave_pdu_deal(hinst, &(frm.pdu));
    }

    int flen = modbus_tcp_frm_make(hinst->buf, &frm, MB_PDU_TYPE_RSP);
    modbus_send(hinst, hinst->buf, flen);
}
#endif

static void modbus_slave_recv_deal(mb_inst_t *hinst, uint8_t *buf, int len)
{
    switch(hinst->prototype)
    {
    #ifdef MB_USING_RTU_PROTOCOL
    case MB_PROT_RTU :
        modbus_slave_recv_deal_rtu(hinst, buf, len);
        break;
    #endif
    #ifdef MB_USING_TCP_PROTOCOL
    case MB_PROT_TCP :
        modbus_slave_recv_deal_tcp(hinst, buf, len);
        break;
    #endif
    default:
        break;
    }
}

const mb_cb_table_t mb_cb_table = {
    .read_disc = modbus_port_read_disc,      //读离散量输入
    .read_coil = modbus_port_read_coil,      //读线圈
    .write_coil = modbus_port_write_coil,    //写线圈
    .read_input = modbus_port_read_input,    //读输入寄存器
    .read_hold = modbus_port_read_hold,      //读保持寄存器
    .write_hold = modbus_port_write_hold,    //写保持寄存器
};

//修改从机回调函数表, 默认使用modbus_port中接口函数做回调函数
void modbus_set_cb_table(mb_inst_t *hinst, const mb_cb_table_t *cb)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(cb != NULL);

    hinst->cb = (mb_cb_table_t *)cb;
}

//从机状态机处理, 在线程中循环调用即可
void modbus_slave_fsm(mb_inst_t *hinst)
{
    MB_ASSERT(hinst != NULL);

    if (modbus_connect(hinst) < 0)//连接失败, 延时返回
    {
        modbus_port_delay_ms(1000);
        return;
    }

    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0)
    {
        return;
    }

    modbus_slave_recv_deal(hinst, hinst->buf, rlen);
}

#endif















