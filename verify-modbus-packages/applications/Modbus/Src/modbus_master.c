/*
 * modbus_master.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2024-04-02     qiyongzhong       first version
 */

#include "bsp_sys.h"


#ifdef MB_USING_MASTER

#ifdef MB_USING_RTU_PROTOCOL

/**
 * @brief  RTU 主站执行读操作（功能码 0x01~0x04）
 *
 * 封装发送请求、接收响应、解析帧、异常处理、数据提取的完整流程。
 * 适用于读线圈、离散输入、保持寄存器、输入寄存器。
 *
 * @param[in]  hinst  实例指针
 * @param[in]  func   功能码 (0x01, 0x02, 0x03, 0x04)
 * @param[in]  addr   起始地址
 * @param[in]  nb     数量（位或寄存器）
 * @param[out] pdata  输出数据缓冲区（字节流）
 *
 * @return int
 *   - >0 : 成功读取的数据字节数
 *   -  0 : 通信失败（超时、CRC错、地址错）
 *   - <0 : 异常响应（-异常码）
 *
 * @note
 *   - 使用 hinst->buf 作为发送和接收缓冲区
 *   - 自动处理 CRC、地址、异常响应
 *   - 上层需确保 pdata 足够大（nb <= 125 时，max 250 字节）
 */
static int modbus_read_req_rtu(mb_inst_t *hinst, uint8_t func, uint16_t addr, int nb, uint8_t *pdata)
{
    // 1. 构建请求帧
    mb_rtu_frm_t frm;
    frm.saddr = hinst->saddr;
    frm.pdu.rd_req.fc = func;
    frm.pdu.rd_req.addr = addr;
    frm.pdu.rd_req.nb = nb;
    // 2. 数据帧打包
    int flen = modbus_rtu_frame_make(hinst->buf, (void *)&frm, MB_PDU_TYPE_REQ);
    // 3. 通过rs485串口发送
    int slen = modbus_send(hinst, hinst->buf, flen);
    if (slen != flen){
        return(0);
    }
    // 4. 接收从机应答数据包
    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0){
        return(0);
    }
    // 5. 解析RTU帧
    int pdu_len = modbus_rtu_frame_parse(hinst->buf, rlen, &frm, MB_PDU_TYPE_RSP);
    if (pdu_len <= 0)
    {
        return(0);
    }
    
    #ifdef MB_USING_ADDR_CHK
    if (frm.saddr != hinst->saddr)
    {
        return(0);
    }
    #endif

    // 6. 检测异常应答
    if (MODBUS_FC_EXCEPT_CHK(frm.pdu.fc))
    {
        return(-(int)frm.pdu.exc.ec);
    }

    int dlen = frm.pdu.rd_rsp.dlen;
    memcpy(pdata, frm.pdu.rd_rsp.pdata, dlen);

    return(dlen);
}
#endif



#ifdef MB_USING_TCP_PROTOCOL
static int modbus_read_req_tcp(mb_inst_t *hinst, uint8_t func, uint16_t addr, int nb, uint8_t *pdata)
{
    mb_tcp_frm_t frm;
    hinst->tsid++;
    frm.mbap.tid = hinst->tsid;
    frm.mbap.pid = MB_TCP_MBAP_PID;
    frm.mbap.did = hinst->saddr;
    frm.pdu.rd_req.fc = func;
    frm.pdu.rd_req.addr = addr;
    frm.pdu.rd_req.nb = nb;
    
    int flen = modbus_tcp_frm_make(hinst->buf, (void *)&frm, MB_PDU_TYPE_REQ);
    int slen = modbus_send(hinst, hinst->buf, flen);
    if (slen != flen)
    {
        return(0);
    }

    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0)
    {
        return(0);
    }
    
    int pdu_len = modbus_tcp_frm_parse(hinst->buf, rlen, &frm, MB_PDU_TYPE_RSP);
    if (pdu_len < 0)
    {
        return(0);
    }

    #ifdef MB_USING_ADDR_CHK
    if (frm.mbap.did != hinst->saddr)
    {
        return(0);
    }
    #endif

    #ifdef MB_USING_MBAP_CHK
    if ((frm.mbap.tid != hinst->tid) || (frm.mbap.pid != MB_TCP_MBAP_PID) || (frm.mbap.dlen != (pdu_len + 1)))
    {
        return(0);
    }
    #endif

    if (MODBUS_FC_EXCEPT_CHK(frm.pdu.fc))//是异常应答
    {
        return(-(int)frm.pdu.exc.ec);
    }

    int dlen = frm.pdu.rd_rsp.dlen;
    memcpy(pdata, frm.pdu.rd_rsp.pdata, dlen);

    return(dlen);
}
#endif



/**
 * @brief  统一读操作接口（支持 RTU 和 TCP）
 *
 * 根据实例协议类型自动选择 RTU 或 TCP 实现。
 * 封装发送请求、接收响应、解析、异常处理、数据提取全流程。
 *
 * @param[in]  hinst  Modbus 实例指针
 * @param[in]  func   功能码
 *                    - 0x01: 读线圈
 *                    - 0x02: 读离散输入
 *                    - 0x03: 读保持寄存器
 *                    - 0x04: 读输入寄存器
 * @param[in]  addr   起始地址（0 开始）
 * @param[in]  nb     数量（位或寄存器）
 * @param[out] pdata  输出数据缓冲区（字节流，大端序）
 *
 * @return int
 *   - >0 : 成功读取的字节数（位：(nb+7)/8，寄存器：nb×2）
 *   -  0 : 通信失败（超时、CRC错、无响应）
 *   - <0 : 异常响应（-异常码）
 */
int modbus_read_req(mb_inst_t *hinst, uint8_t func, uint16_t addr, int nb, uint8_t *pdata)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(pdata != NULL);
    MB_ASSERT(nb > 0);

    switch (hinst->prototype)
    {
        #ifdef MB_USING_RTU_PROTOCOL
        case MB_PROT_RTU :
            return(modbus_read_req_rtu(hinst, func, addr, nb, pdata));
        #endif
        #ifdef MB_USING_TCP_PROTOCOL
        case MB_PROT_TCP :
            return(modbus_read_req_tcp(hinst, func, addr, nb, pdata));
        #endif
        default:
            break;
    }
    
    return(0);
}

#ifdef MB_USING_RTU_PROTOCOL

/**
 * @brief  RTU 主站执行写多个操作（功能码 0x0F / 0x10）
 *
 * 封装发送写请求、接收响应、解析、异常处理全流程。
 * 支持写多个线圈（0x0F）和写多个保持寄存器（0x10）。
 *
 * @param[in] hinst  实例指针
 * @param[in] func   功能码 (0x0F: 写线圈, 0x10: 写寄存器)
 * @param[in] addr   起始地址
 * @param[in] nb     数量（位或寄存器）
 * @param[in] pdata  输入数据（字节流，大端序）
 * @param[in] dlen   数据字节数（nb×1 或 nb×2）
 *
 * @return int
 *   - >0 : 成功写入的数量
 *   -  0 : 通信失败
 *   - <0 : 异常响应（-异常码）
 */
static int modbus_write_req_rtu(mb_inst_t *hinst, uint8_t func, uint16_t addr, int nb, const uint8_t *pdata, int dlen)
{
    mb_rtu_frm_t frm;
    frm.saddr = hinst->saddr;
    frm.pdu.wr_req.fc = func;
    frm.pdu.wr_req.addr = addr;
    frm.pdu.wr_req.nb = nb;
    frm.pdu.wr_req.dlen = dlen;
    frm.pdu.wr_req.pdata = (uint8_t *)pdata;
    
    int flen = modbus_rtu_frame_make(hinst->buf, (void *)&frm, MB_PDU_TYPE_REQ);
    int slen = modbus_send(hinst, hinst->buf, flen);
    if (slen != flen){
        return(0);
    }

    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0){
        return(0);
    }
    
    int pdu_len = modbus_rtu_frame_parse(hinst->buf, rlen, &frm, MB_PDU_TYPE_RSP);
    if (pdu_len <= 0){
        return(0);
    }
    
    #ifdef MB_USING_ADDR_CHK
    if (frm.saddr != hinst->saddr)
    {
        return(0);
    }
    #endif

    if (MODBUS_FC_EXCEPT_CHK(frm.pdu.fc))//是异常应答
    {
        return(-(int)frm.pdu.exc.ec);
    }

    return(frm.pdu.wr_rsp.nb);
}
#endif

#ifdef MB_USING_TCP_PROTOCOL
static int modbus_write_req_tcp(mb_inst_t *hinst, uint8_t func, uint16_t addr, int nb, const uint8_t *pdata, int dlen)
{
    mb_tcp_frm_t frm;
    hinst->tsid++;
    frm.mbap.tid = hinst->tsid;
    frm.mbap.pid = MB_TCP_MBAP_PID;
    frm.mbap.did = hinst->saddr;
    frm.pdu.wr_req.fc = func;
    frm.pdu.wr_req.addr = addr;
    frm.pdu.wr_req.nb = nb;
    frm.pdu.wr_req.dlen = dlen;
    frm.pdu.wr_req.pdata = (uint8_t *)pdata;
    
    int flen = modbus_tcp_frm_make(hinst->buf, (void *)&frm, MB_PDU_TYPE_REQ);
    int slen = modbus_send(hinst, hinst->buf, flen);
    if (slen != flen)
    {
        return(0);
    }

    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0)
    {
        return(0);
    }
    
    int pdu_len = modbus_tcp_frm_parse(hinst->buf, rlen, &frm, MB_PDU_TYPE_RSP);
    if (pdu_len < 0)
    {
        return(0);
    }

    #ifdef MB_USING_ADDR_CHK
    if (frm.mbap.did != hinst->saddr)
    {
        return(0);
    }
    #endif

    #ifdef MB_USING_MBAP_CHK
    if ((frm.mbap.tid != hinst->tid) || (frm.mbap.pid != MB_TCP_MBAP_PID) || (frm.mbap.dlen != (pdu_len + 1)))
    {
        return(0);
    }
    #endif

    if (MODBUS_FC_EXCEPT_CHK(frm.pdu.fc))//是异常应答
    {
        return(-(int)frm.pdu.exc.ec);
    }

    return(frm.pdu.wr_rsp.nb);
}
#endif



/**
 * @brief  统一写多个操作接口（支持 RTU 和 TCP）
 *
 * 根据实例协议类型自动选择 RTU 或 TCP 实现。
 * 封装发送写请求、接收响应、解析、异常处理全流程。
 *
 * @param[in] hinst  Modbus 实例指针
 * @param[in] func   功能码
 *                    - 0x0F: 写多个线圈
 *                    - 0x10: 写多个保持寄存器
 * @param[in] addr   起始地址（0 开始）
 * @param[in] nb     数量（位或寄存器）
 * @param[in] pdata  输入数据缓冲区（字节流，大端序）
 * @param[in] dlen   数据字节数（nb×1 或 nb×2）
 *
 * @return int
 *   - >0 : 成功写入的数量
 *   -  0 : 通信失败（超时、CRC错、无响应）
 *   - <0 : 异常响应（-异常码）
 */
int modbus_write_req(mb_inst_t *hinst, uint8_t func, uint16_t addr, int nb, const uint8_t *pdata, int dlen)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(pdata != NULL);
    MB_ASSERT(nb > 0);
    MB_ASSERT(dlen > 0);

    switch (hinst->prototype)
    {
        #ifdef MB_USING_RTU_PROTOCOL
        case MB_PROT_RTU :
            return(modbus_write_req_rtu(hinst, func, addr, nb, pdata, dlen));
        #endif
        #ifdef MB_USING_TCP_PROTOCOL
        case MB_PROT_TCP :
            return(modbus_write_req_tcp(hinst, func, addr, nb, pdata, dlen));
        #endif
        default:
            break;
    }
    
    return(0);
}

/**
 * @brief  读线圈（功能码 0x01）
 *
 * 高层接口，自动处理位→字节转换。
 * 成功时返回读取的位数，失败时返回 0 或负数异常码。
 *
 * @param[in]  hinst  Modbus 实例指针
 * @param[in]  addr   起始线圈地址（0 开始）
 * @param[in]  nb     读取位数（1 ~ 2000）
 * @param[out] pbits  输出位图缓冲区（LSB first）
 *
 * @return int
 *   - >0 : 成功读取的位数
 *   -  0 : 通信失败（超时、CRC错）
 *   - <0 : 异常响应（-异常码）
 */
int modbus_read_bits(mb_inst_t *hinst, uint16_t addr, int nb, uint8_t *pbits)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(pbits != NULL);
    MB_ASSERT(nb > 0);
    
    int dlen = modbus_read_req(hinst, MODBUS_FC_READ_COILS, addr, nb, pbits);
    return((dlen != (nb + 7) / 8) ? dlen : nb);
}



/**
 * @brief  读取离散输入（功能码 0x02）
 *
 * 高层接口，自动处理位→字节转换。
 * 成功返回读取的位数，失败返回 0 或负数异常码。
 *
 * @param[in]  hinst  Modbus 实例指针
 * @param[in]  addr   起始地址（0 开始，对应实际地址 10001+）
 * @param[in]  nb     读取位数（1 ~ 2000）
 * @param[out] pbits  输出位图缓冲区（LSB first）
 *
 * @return int
 *   - >0 : 成功读取的位数
 *   -  0 : 通信失败（超时、CRC错、无响应）
 *   - <0 : 异常响应（-异常码）
 */
int modbus_read_input_bits(mb_inst_t *hinst, uint16_t addr, int nb, uint8_t *pbits)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(pbits != NULL);
    MB_ASSERT(nb > 0);
    
    int dlen = modbus_read_req(hinst, MODBUS_FC_READ_DISCRETE_INPUTS, addr, nb, pbits);
    return((dlen != (nb + 7) / 8) ? dlen : nb);
}



/**
 * @brief  读取保持寄存器（功能码 0x03）
 *
 * 高层接口，自动处理大端序转换。
 * 成功返回读取的寄存器数量，失败返回 0 或负数异常码。
 *
 * @param[in]  hinst  Modbus 实例指针
 * @param[in]  addr   起始地址（0 开始，对应实际地址 40001+）
 * @param[in]  nb     读取寄存器数（1 ~ 125）
 * @param[out] pregs  输出寄存器数组（本地字节序）
 *
 * @return int
 *   - >0 : 成功读取的寄存器数量
 *   -  0 : 通信失败（超时、CRC错、长度不符）
 *   - <0 : 异常响应（-异常码）
 */
int modbus_read_regs(mb_inst_t *hinst, uint16_t addr, int nb, uint16_t *pregs)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(pregs != NULL);
    MB_ASSERT(nb > 0);
    
    int dlen = modbus_read_req(hinst, MODBUS_FC_READ_HOLDING_REGISTERS, addr, nb, hinst->datas);
    if (dlen <= 0){
        return(dlen);
    }
    
    if (dlen != (nb * 2)){
        return(0);
    }

    uint8_t *p = hinst->datas;
    for (int i=0; i<nb; i++){
        p += modbus_cvt_u16_get(p, pregs + i);
    }

    return(nb);
}

/**
 * @brief  读取输入寄存器（功能码 0x04）
 *
 * 高层接口，自动处理大端序转换。
 * 成功返回读取的寄存器数量，失败返回 0 或负数异常码。
 *
 * @param[in]  hinst  Modbus 实例指针
 * @param[in]  addr   起始地址（0 开始，对应实际地址 30001+）
 * @param[in]  nb     读取寄存器数（1 ~ 125）
 * @param[out] pregs  输出寄存器数组（本地字节序）
 *
 * @return int
 *   - >0 : 成功读取的寄存器数量
 *   -  0 : 通信失败（超时、CRC错、长度不符）
 *   - <0 : 异常响应（-异常码）
 *
 * @note
 *   - 输入寄存器为只读，常用于采集传感器数据
 *   - 寄存器为 16 位大端序传输
 *   - 使用 hinst->datas 作为临时字节缓冲
 *   - 缓冲区大小建议 >= 250 字节
 */
int modbus_read_input_regs(mb_inst_t *hinst, uint16_t addr, int nb, uint16_t *pregs)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(pregs != NULL);
    MB_ASSERT(nb > 0);
    
    int dlen = modbus_read_req(hinst, MODBUS_FC_READ_INPUT_REGISTERS, addr, nb, hinst->datas);
    if (dlen <= 0){
        return(dlen);
    }
    
    if (dlen != (nb * 2)){
        return(0);
    }

    uint8_t *p = hinst->datas;
    for (int i=0; i<nb; i++){
        p += modbus_cvt_u16_get(p, pregs + i);
    }

    return(nb);
}

#ifdef MB_USING_RTU_PROTOCOL

/**
 * @brief  RTU 主站写单个线圈或寄存器（功能码 0x05 / 0x06）
 *
 * 封装发送请求、接收响应、解析、异常处理、回响校验全流程。
 * 响应必须与请求完全一致（回响）。
 *
 * @param[in] hinst  实例指针
 * @param[in] func   功能码
 *                    - 0x05: 写单个线圈
 *                    - 0x06: 写单个保持寄存器
 * @param[in] addr   目标地址
 * @param[in] val    写入值
 *                    - 线圈: 0x0000(OFF) 或 0xFF00(ON)
 *                    - 寄存器: 0~65535
 *
 * @return int
 *   -  1 : 成功
 *   -  0 : 通信失败
 *   - <0 : 异常响应（-异常码）
 */
static int modbus_write_single_rtu(mb_inst_t *hinst, uint8_t func, uint16_t addr, uint16_t val)
{
    mb_rtu_frm_t frm;
    frm.saddr = hinst->saddr;
    frm.pdu.wr_single.fc = func;
    frm.pdu.wr_single.addr = addr;
    frm.pdu.wr_single.val = val;
    
    int flen = modbus_rtu_frame_make(hinst->buf, (void *)&frm, MB_PDU_TYPE_REQ);
    int slen = modbus_send(hinst, hinst->buf, flen);
    if (slen != flen){
        return(0);
    }

    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0){
        return(0);
    }
    
    int pdu_len = modbus_rtu_frame_parse(hinst->buf, rlen, &frm, MB_PDU_TYPE_RSP);
    if (pdu_len <= 0){
        return(0);
    }
    
    #ifdef MB_USING_ADDR_CHK
    if (frm.saddr != hinst->saddr)
    {
        return(0);
    }
    #endif

    if (MODBUS_FC_EXCEPT_CHK(frm.pdu.fc)){
        return(-(int)frm.pdu.exc.ec);
    }

    return(1);
}
#endif

#ifdef MB_USING_TCP_PROTOCOL
static int modbus_write_single_tcp(mb_inst_t *hinst, uint8_t func, uint16_t addr, uint16_t val)
{
    mb_tcp_frm_t frm;
    hinst->tsid++;
    frm.mbap.tid = hinst->tsid;
    frm.mbap.pid = MB_TCP_MBAP_PID;
    frm.mbap.did = hinst->saddr;
    frm.pdu.wr_single.fc = func;
    frm.pdu.wr_single.addr = addr;
    frm.pdu.wr_single.val = val;
    
    int flen = modbus_tcp_frm_make(hinst->buf, (void *)&frm, MB_PDU_TYPE_REQ);
    int slen = modbus_send(hinst, hinst->buf, flen);
    if (slen != flen)
    {
        return(0);
    }

    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0)
    {
        return(0);
    }
    
    int pdu_len = modbus_tcp_frm_parse(hinst->buf, rlen, &frm, MB_PDU_TYPE_RSP);
    if (pdu_len < 0)
    {
        return(0);
    }

    #ifdef MB_USING_ADDR_CHK
    if (frm.mbap.did != hinst->saddr)
    {
        return(0);
    }
    #endif

    #ifdef MB_USING_MBAP_CHK
    if ((frm.mbap.tid != hinst->tid) || (frm.mbap.pid != MB_TCP_MBAP_PID) || (frm.mbap.dlen != (pdu_len + 1)))
    {
        return(0);
    }
    #endif

    if (MODBUS_FC_EXCEPT_CHK(frm.pdu.fc))//是异常应答
    {
        return(-(int)frm.pdu.exc.ec);
    }

    return(1);
}
#endif


/**
 * @brief  统一写单个线圈或寄存器接口（支持 RTU 和 TCP）
 *
 * 根据实例协议类型自动选择 RTU 或 TCP 实现。
 * 封装发送请求、接收响应、解析、异常处理、回响校验全流程。
 *
 * @param[in] hinst  Modbus 实例指针
 * @param[in] func   功能码
 *                    - 0x05: 写单个线圈
 *                    - 0x06: 写单个保持寄存器
 * @param[in] addr   目标地址
 * @param[in] val    写入值
 *                    - 线圈: 0x0000(OFF) 或 0xFF00(ON)
 *                    - 寄存器: 0~65535
 *
 * @return int
 *   -  1 : 成功
 *   -  0 : 通信失败（超时、CRC错、无响应）
 *   - <0 : 异常响应（-异常码）
 */
static int modbus_write_single(mb_inst_t *hinst, uint8_t func, uint16_t addr, uint16_t val)
{
    switch (hinst->prototype)
    {
        #ifdef MB_USING_RTU_PROTOCOL
        case MB_PROT_RTU :
            return(modbus_write_single_rtu(hinst, func, addr, val));
        #endif
        #ifdef MB_USING_TCP_PROTOCOL
        case MB_PROT_TCP :
            return(modbus_write_single_tcp(hinst, func, addr, val));
        #endif
        default:
            break;
    }
    
    return(0);
}

//写单个线圈, 功能码-0x05, 成功返回1, 异常应答返回负值错误码, 其它错误返回0
int modbus_write_bit(mb_inst_t *hinst, uint16_t addr, uint8_t bit)
{
    MB_ASSERT(hinst != NULL);
    
    uint16_t val = bit ? 0xFF00 : 0x0000;
    return(modbus_write_single(hinst, MODBUS_FC_WRITE_SINGLE_COIL, addr, val));
}

//写单个寄存器, 功能码-0x06, 成功返回1, 异常应答返回负值错误码, 其它错误返回0
int modbus_write_reg(mb_inst_t *hinst, uint16_t addr, uint16_t val)
{
    MB_ASSERT(hinst != NULL);
    
    return(modbus_write_single(hinst, MODBUS_FC_WRITE_SINGLE_REGISTER, addr, val));
}

//写多个线圈, 功能码-0x0F, 成功返回写位数量, 异常应答返回负值错误码, 其它错误返回0
int modbus_write_bits(mb_inst_t *hinst, uint16_t addr, int nb, const uint8_t *pbits)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(pbits != NULL);
    MB_ASSERT(nb > 0);
    
    int dlen = (nb + 7) / 8;
    return(modbus_write_req(hinst, MODBUS_FC_WRITE_MULTIPLE_COILS, addr, nb, pbits, dlen));
}

//写多个寄存器, 功能码-0x10, 成功返回写寄存器数量, 异常应答返回负值错误码, 其它错误返回0
int modbus_write_regs(mb_inst_t *hinst, uint16_t addr, int nb, const uint16_t *pregs)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(pregs != NULL);
    MB_ASSERT(nb > 0);
    
    uint8_t *p = hinst->datas;
    for (int i=0; i<nb; i++)
    {
        p += modbus_cvt_u16_put(p, pregs[i]);
    }
    int dlen = nb * 2;
    return(modbus_write_req(hinst, MODBUS_FC_WRITE_MULTIPLE_REGISTERS, addr, nb, hinst->datas, dlen));
}

#ifdef MB_USING_RTU_PROTOCOL
static int modbus_mask_write_rtu(mb_inst_t *hinst, uint16_t addr, uint16_t val_and, uint16_t val_or)
{
    mb_rtu_frm_t frm;
    frm.saddr = hinst->saddr;
    frm.pdu.mask_wr.fc = MODBUS_FC_MASK_WRITE_REGISTER;
    frm.pdu.mask_wr.addr = addr;
    frm.pdu.mask_wr.val_and = val_and;
    frm.pdu.mask_wr.val_or = val_or;
    
    int flen = modbus_rtu_frame_make(hinst->buf, (void *)&frm, MB_PDU_TYPE_REQ);
    int slen = modbus_send(hinst, hinst->buf, flen);
    if (slen != flen)
    {
        return(0);
    }

    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0)
    {
        return(0);
    }
    
    int pdu_len = modbus_rtu_frame_parse(hinst->buf, rlen, &frm, MB_PDU_TYPE_RSP);
    if (pdu_len <= 0)
    {
        return(0);
    }
    
    #ifdef MB_USING_ADDR_CHK
    if (frm.saddr != hinst->saddr)
    {
        return(0);
    }
    #endif

    if (MODBUS_FC_EXCEPT_CHK(frm.pdu.fc))//是异常应答
    {
        return(-(int)frm.pdu.exc.ec);
    }

    return(1);
}
#endif

#ifdef MB_USING_TCP_PROTOCOL
static int modbus_mask_write_tcp(mb_inst_t *hinst, uint16_t addr, uint16_t val_and, uint16_t val_or)
{
    mb_tcp_frm_t frm;
    hinst->tsid++;
    frm.mbap.tid = hinst->tsid;
    frm.mbap.pid = MB_TCP_MBAP_PID;
    frm.mbap.did = hinst->saddr;
    frm.pdu.mask_wr.fc = MODBUS_FC_MASK_WRITE_REGISTER;
    frm.pdu.mask_wr.addr = addr;
    frm.pdu.mask_wr.val_and = val_and;
    frm.pdu.mask_wr.val_or = val_or;
    
    int flen = modbus_tcp_frm_make(hinst->buf, (void *)&frm, MB_PDU_TYPE_REQ);
    int slen = modbus_send(hinst, hinst->buf, flen);
    if (slen != flen)
    {
        return(0);
    }

    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0)
    {
        return(0);
    }
    
    int pdu_len = modbus_tcp_frm_parse(hinst->buf, rlen, &frm, MB_PDU_TYPE_RSP);
    if (pdu_len < 0)
    {
        return(0);
    }

    #ifdef MB_USING_ADDR_CHK
    if (frm.mbap.did != hinst->saddr)
    {
        return(0);
    }
    #endif

    #ifdef MB_USING_MBAP_CHK
    if ((frm.mbap.tid != hinst->tid) || (frm.mbap.pid != MB_TCP_MBAP_PID) || (frm.mbap.dlen != (pdu_len + 1)))
    {
        return(0);
    }
    #endif

    if (MODBUS_FC_EXCEPT_CHK(frm.pdu.fc))//是异常应答
    {
        return(-(int)frm.pdu.exc.ec);
    }

    return(1);
}
#endif

//屏蔽写寄存器, 功能码-0x16, 成功返回1, 异常应答返回负值错误码, 其它错误返回0
int modbus_mask_write_reg(mb_inst_t *hinst, uint16_t addr, uint16_t mask_and, uint16_t mask_or)
{
    MB_ASSERT(hinst != NULL);

    switch (hinst->prototype)
    {
    #ifdef MB_USING_RTU_PROTOCOL
    case MB_PROT_RTU :
        return(modbus_mask_write_rtu(hinst, addr, mask_and, mask_or));
    #endif
    #ifdef MB_USING_TCP_PROTOCOL
    case MB_PROT_TCP :
        return(modbus_mask_write_tcp(hinst, addr, mask_and, mask_or));
    #endif
    default:
        break;
    }
    
    return(0);
}

#ifdef MB_USING_RTU_PROTOCOL
static int modbus_write_and_read_regs_rtu(mb_inst_t *hinst, uint16_t wr_addr, int wr_nb, const uint16_t *p_wr_regs,uint16_t rd_addr, int rd_nb, uint16_t *p_rd_regs)
{
    uint8_t *p = hinst->datas;
    for (int i=0; i<wr_nb; i++)
    {
        p += modbus_cvt_u16_put(p, p_wr_regs[i]);
    }
    
    mb_rtu_frm_t frm;
    frm.saddr = hinst->saddr;
    frm.pdu.wr_rd_req.fc = MODBUS_FC_WRITE_AND_READ_REGISTERS;
    frm.pdu.wr_rd_req.rd_addr = rd_addr;
    frm.pdu.wr_rd_req.rd_nb = rd_nb;
    frm.pdu.wr_rd_req.wr_addr = wr_addr;
    frm.pdu.wr_rd_req.wr_nb = wr_nb;
    frm.pdu.wr_rd_req.dlen = wr_nb * 2;
    frm.pdu.wr_rd_req.pdata = hinst->datas;
    
    int flen = modbus_rtu_frame_make(hinst->buf, (void *)&frm, MB_PDU_TYPE_REQ);
    int slen = modbus_send(hinst, hinst->buf, flen);
    if (slen != flen)
    {
        return(0);
    }

    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0)
    {
        return(0);
    }
    
    int pdu_len = modbus_rtu_frame_parse(hinst->buf, rlen, &frm, MB_PDU_TYPE_RSP);
    if (pdu_len <= 0)
    {
        return(0);
    }
    
    #ifdef MB_USING_ADDR_CHK
    if (frm.saddr != hinst->saddr)
    {
        return(0);
    }
    #endif

    if (MODBUS_FC_EXCEPT_CHK(frm.pdu.fc))//是异常应答
    {
        return(-(int)frm.pdu.exc.ec);
    }

    int dlen = frm.pdu.rd_rsp.dlen;
    if (dlen != (rd_nb * 2))
    {
        return(0);
    }

    p = frm.pdu.rd_rsp.pdata;
    for (int i=0; i<rd_nb; i++)
    {
        p += modbus_cvt_u16_get(p, p_rd_regs + i);
    }

    return(rd_nb);
}
#endif

#ifdef MB_USING_TCP_PROTOCOL
static int modbus_write_and_read_regs_tcp(mb_inst_t *hinst, uint16_t wr_addr, int wr_nb, const uint16_t *p_wr_regs,uint16_t rd_addr, int rd_nb, uint16_t *p_rd_regs)
{
    uint8_t *p = hinst->datas;
    for (int i=0; i<wr_nb; i++)
    {
        p += modbus_cvt_u16_put(p, p_wr_regs[i]);
    }
    
    mb_tcp_frm_t frm;
    hinst->tsid++;
    frm.mbap.tid = hinst->tsid;
    frm.mbap.pid = MB_TCP_MBAP_PID;
    frm.mbap.did = hinst->saddr;
    frm.pdu.wr_rd_req.fc = MODBUS_FC_WRITE_AND_READ_REGISTERS;
    frm.pdu.wr_rd_req.rd_addr = rd_addr;
    frm.pdu.wr_rd_req.rd_nb = rd_nb;
    frm.pdu.wr_rd_req.wr_addr = wr_addr;
    frm.pdu.wr_rd_req.wr_nb = wr_nb;
    frm.pdu.wr_rd_req.dlen = wr_nb * 2;
    frm.pdu.wr_rd_req.pdata = hinst->datas;
    
    int flen = modbus_tcp_frm_make(hinst->buf, (void *)&frm, MB_PDU_TYPE_REQ);
    int slen = modbus_send(hinst, hinst->buf, flen);
    if (slen != flen)
    {
        return(0);
    }

    int rlen = modbus_recv(hinst, hinst->buf, sizeof(hinst->buf));
    if (rlen <= 0)
    {
        return(0);
    }
    
    int pdu_len = modbus_tcp_frm_parse(hinst->buf, rlen, &frm, MB_PDU_TYPE_RSP);
    if (pdu_len < 0)
    {
        return(0);
    }

    #ifdef MB_USING_ADDR_CHK
    if (frm.mbap.did != hinst->saddr)
    {
        return(0);
    }
    #endif

    #ifdef MB_USING_MBAP_CHK
    if ((frm.mbap.tid != hinst->tid) || (frm.mbap.pid != MB_TCP_MBAP_PID) || (frm.mbap.dlen != (pdu_len + 1)))
    {
        return(0);
    }
    #endif

    if (MODBUS_FC_EXCEPT_CHK(frm.pdu.fc))//是异常应答
    {
        return(-(int)frm.pdu.exc.ec);
    }

    int dlen = frm.pdu.rd_rsp.dlen;
    if (dlen != (rd_nb * 2))
    {
        return(0);
    }

    p = frm.pdu.rd_rsp.pdata;
    for (int i=0; i<rd_nb; i++)
    {
        p += modbus_cvt_u16_get(p, p_rd_regs + i);
    }

    return(rd_nb);

}
#endif

//读/写多个寄存器, 功能码-0x17, 成功返回读取寄存器数量, 异常应答返回负值错误码, 其它错误返回0
int mb_write_and_read_regs(mb_inst_t *hinst, uint16_t wr_addr, int wr_nb, const uint16_t *p_wr_regs,uint16_t rd_addr, int rd_nb, uint16_t *p_rd_regs)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(p_wr_regs != NULL);
    MB_ASSERT(p_rd_regs != NULL);
    MB_ASSERT(wr_nb > 0);
    MB_ASSERT(rd_nb > 0);

    switch (hinst->prototype)
    {
    #ifdef MB_USING_RTU_PROTOCOL
    case MB_PROT_RTU :
        return(modbus_write_and_read_regs_rtu(hinst, wr_addr, wr_nb, p_wr_regs, rd_addr, rd_nb, p_rd_regs));
    #endif
    #ifdef MB_USING_TCP_PROTOCOL
    case MB_PROT_TCP :
        return(modbus_write_and_read_regs_tcp(hinst, wr_addr, wr_nb, p_wr_regs, rd_addr, rd_nb, p_rd_regs));
    #endif
    default:
        break;
    }
    
    return(0);
}
                                     
#endif
