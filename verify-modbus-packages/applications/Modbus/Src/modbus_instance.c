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




#ifdef MB_USING_RAW_PRT
/**
 * @brief  打印 Modbus 通信的原始字节数据（调试用）
 *
 * 该函数在调试阶段用于将发送或接收的 Modbus 原始数据以十六进制格式输出，
 * 便于查看帧结构、校验码、功能码等信息。仅在宏 #MB_USING_RAW_PRT 被定义时生效。
 *
 * @param[in] is_send  数据方向标识
 *                     - true  : 发送数据（打印前缀 ">>"）
 *                     - false : 接收数据（打印前缀 "<<"）
 * @param[in] pdata    指向待打印数据的缓冲区首地址
 * @param[in] dlen     数据长度（字节数），必须大于 0
 *
 * @return 无返回值
 *
 * @note
 *   - 依赖宏 #MB_PRINTF 实现实际输出（通常映射为 printf、串口打印或日志系统）
 *   - 若 #MB_USING_RAW_PRT 未定义，函数将不会被编译，节省资源
 *   - 调用前应确保 pdata 不为 NULL 且 dlen 合法，否则可能引发未定义行为
 */
static void modbus_raw_printf(bool is_send, const u8 *pdata, int dlen)
{
    MB_PRINTF("%s", is_send ? ">>" : "<<");
    for (int i=0; i<dlen; i++)
    {
        MB_PRINTF("%02X ", pdata[i]);
    }
    MB_PRINTF("\n");
}
#endif





/**
 * @brief  创建并初始化 Modbus 通信实例（主站或从站）
 *
 * 该函数是 Modbus 协议栈的入口点，用于动态分配并初始化一个 Modbus 实例
 * 它会根据指定的后端类型（RTU 或 TCP）创建底层通信后端，并设置默认配置
 * 成功返回实例句柄，失败返回 NULL
 *
 * @param[in] type   后端通信类型
 *                   - MB_BACKEND_TYPE_RTU : 串口 RTU 模式
 *                   - MB_BACKEND_TYPE_TCP : TCP 客户端模式
 * @param[in] param  指向后端配置参数的指针（不可为 NULL）
 *                   - RTU: 包含串口设备名、波特率、校验位、RS485 控制引脚等
 *                   - TCP: 包含服务器 IP 地址和端口号
 *
 * @return mb_inst_t*  成功：指向新创建的 Modbus 实例
 * @return NULL        失败：参数错误、内存不足或后端创建失败
 *
 * @note
 *   - 调用前必须确保 param 有效，否则触发 MB_ASSERT 断言（调试模式下崩溃）
 *   - 资源安全：任何一步失败都会释放已分配的资源（backend），防止内存泄漏
 *   - 默认配置：
 *       - 从机地址：MB_RTU_ADDR_DEF（通常为 1）
 *       - 协议类型：与后端类型自动匹配（RTU → RTU 协议，TCP → TCP 协议）
 *       - 事务 ID：0（仅 TCP 模式使用）
 *   - 从站模式下自动绑定默认回调函数表（mb_cb_table），用于读写寄存器/线圈
 *
 * @warning
 *   - 必须成对调用 mb_destory() 释放实例，避免内存泄漏
 *   - MB_USING_SLAVE 宏控制是否启用从站功能
 */
mb_inst_t * modbus_create(mb_backend_type_t type, const mb_backend_param_t *param)
{
    // 1.参数合法性检查
    MB_ASSERT(param != NULL);

    // 2.创建底层通信后端句柄（串口 / TCP 客户端）
    mb_backend_t *backend = modbus_backend_create(type, param);
    if (backend == NULL){
        return(NULL);
    }

    // 3.为实例结构体动态分配内存
    mb_inst_t *hinst = malloc(sizeof(mb_inst_t));
    if (hinst == NULL){
        modbus_backend_destory(backend);
        return(NULL);
    }

    // 4.初始化实例字段
    // 4.1 设置Modbus-RTU的默认地址
    hinst->saddr = MB_RTU_ADDR_DEF;
    // 4.2 选择设备类型
    hinst->prototype = (type == MB_BACKEND_TYPE_RTU) ? MB_PROT_RTU : MB_PROT_TCP;
    // 4.3
    hinst->tsid = 0;
    // 4.4 后端指针
    hinst->backend = backend;

    #ifdef MB_USING_SLAVE
    extern const mb_cb_table_t mb_cb_table;
    hinst->cb = (mb_cb_table_t *)&mb_cb_table;
    #else
    hinst->cb = NULL;
    #endif

    return(hinst);
}




/**
 * @brief  销毁 Modbus 通信实例，释放所有相关资源
 *
 * 该函数是 mb_create() 的配套释放函数，用于：
 * 1. 关闭底层通信后端（串口、TCP 连接）
 * 2. 释放实例结构体内存
 *
 * @warning
 *   - 必须与 mb_create() 成对调用，否则会导致内存泄漏
 *   - 调用后 hinst 指针立即失效，禁止再次使用
 *   - 不要对同一个实例重复调用（虽有 NULL 保护，但行为未定义）
 *
 * @param[in,out] hinst  指向待销毁的 Modbus 实例指针
 *                       - 调用前必须有效（非 NULL）
 *                       - 调用后指针失效
 *
 * @return 无返回值
 *
 * @note
 *   - 内部会调用 mb_backend_destory() 关闭串口或 socket
 *   - 使用 MB_ASSERT 防止传入 NULL 指针（调试模式下崩溃）
 *   - 后端置为 NULL 防止野指针重复释放
 */
void modbus_destroy(mb_inst_t *hinst)
{
    MB_ASSERT(hinst != NULL);

    if (hinst->backend != NULL){
        modbus_backend_destory(hinst->backend);
        hinst->backend = NULL;
    }

    free(hinst);
}



/**
 * @brief  修改 Modbus 从机地址（默认地址为 1）
 *
 * 用于设置目标从机的通信地址。在 RTU 模式下为从机地址字段；
 * 在 TCP 模式下为 MBAP 头部中的 Unit Identifier（did）。
 *
 * @param[in,out] hinst  Modbus 实例指针
 * @param[in]     saddr  新的从机地址（1~247，0xFF 为广播地址，视协议而定）
 *
 * @return 无返回值
 *
 * @note
 *   - 必须在 mb_connect() 之前调用
 *   - 默认值由 MB_RTU_ADDR_DEF 定义（通常为 1）
 */
void modbus_set_slave_addr(mb_inst_t *hinst, uint8_t saddr)
{
    MB_ASSERT(hinst != NULL);

    hinst->saddr = saddr;
}


/**
 * @brief  修改 Modbus 协议类型（RTU 或 TCP）
 *
 * 允许用户强制指定协议类型，覆盖 mb_create() 中的自动匹配逻辑。
 * 通常用于：RTU 后端使用 ASCII 协议，或 TCP 后端使用自定义封装。
 *
 * @param[in,out] hinst  Modbus 实例指针
 * @param[in]     prot   协议类型
 *                       - MB_PROT_RTU : Modbus RTU
 *                       - MB_PROT_TCP : Modbus TCP
 *
 * @return 无返回值
 *
 * @note
 *   - 默认协议与后端类型一致（RTU → RTU，TCP → TCP）
 *   - 必须在通信前设置，运行中修改无效
 *
 * @warning
 *   - 协议与后端不匹配可能导致通信失败（如 TCP 后端使用 RTU 帧）
 */
void modbus_set_prototype(mb_inst_t *hinst, mb_prot_t prot)
{
    MB_ASSERT(hinst != NULL);

    hinst->prototype = prot;
}


/**
 * @brief  设置通信超时时间
 *
 * 配置主站模式下的应答超时和字节间超时，用于控制通信可靠性。
 *
 * @param[in,out] hinst        Modbus 实例指针
 * @param[in]     ack_tmo_ms   应答超时时间（毫秒），建议 100~5000ms
 * @param[in]     byte_tmo_ms  字节间超时时间（毫秒），建议 10~100ms
 *
 * @return 无返回值
 *
 * @note
 *   - 默认值：应答超时 300ms，字节超时 32ms
 *   - 超时时间由底层后端（RTU/TCP）实现
 *   - 仅对主站模式有效
 */
void modbus_set_tmo(mb_inst_t *hinst, int ack_tmo_ms, int byte_tmo_ms)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(hinst->backend != NULL);

    modbus_backend_config(hinst->backend, ack_tmo_ms, byte_tmo_ms);
}


/**
 * @brief  建立底层通信连接（打开串口或连接 TCP 服务器）
 *
 * 对于 RTU：打开串口设备并配置波特率、校验位等；
 * 对于 TCP：创建 socket 并连接到远程服务器。
 *
 * @param[in,out] hinst  Modbus 实例指针
 *
 * @retval  0  连接成功
 * @retval -1  连接失败（设备不存在、权限不足、连接超时等）
 *
 * @note
 *   - 必须在通信前调用
 *   - 可重复调用（断线重连）
 */
int modbus_connect(mb_inst_t *hinst)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(hinst->backend != NULL);

    return(modbus_backend_open(hinst->backend));
}


/**
 * @brief  断开底层通信连接（关闭串口或 socket）
 *
 * 释放通信资源，建议在程序退出或切换设备时调用。
 *
 * @param[in,out] hinst  Modbus 实例指针
 *
 * @retval  0  断开成功
 * @retval -1  断开失败（资源已释放或底层错误）
 *
 * @note
 *   - 不释放实例内存，仅关闭通信通道
 *   - 可在 mb_connect() 后重复调用
 */
int modbus_disconn(mb_inst_t *hinst)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(hinst->backend != NULL);

    return(modbus_backend_close(hinst->backend));
}


/**
 * @brief  从底层接收原始数据（阻塞或非阻塞，取决于后端）
 *
 * 读取串口或 socket 中的数据到用户缓冲区。
 * 接收错误时自动关闭连接。
 *
 * @param[in,out] hinst    Modbus 实例指针
 * @param[out]    buf      接收数据缓冲区
 * @param[in]     bufsize  缓冲区大小（字节）
 *
 * @retval >0  成功接收的字节数
 * @retval  0  超时（无数据）
 * @retval -1  接收错误（连接断开、硬件故障等）
 *
 * @note
 *   - 错误时自动调用 mb_backend_close()
 *   - 启用 MB_USING_RAW_PRT 时会打印接收数据
 */
int modbus_recv(mb_inst_t *hinst, uint8_t *buf, int bufsize)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(hinst->backend != NULL);
    MB_ASSERT(buf != NULL);
    MB_ASSERT(bufsize > 0);

    int len = modbus_backend_read(hinst->backend, buf, bufsize);
    if (len < 0)//发生错误, 关闭后端
    {
        modbus_backend_close(hinst->backend);
    }

    #ifdef MB_USING_RAW_PRT
    if (len > 0)
    {
        modbus_raw_prt(false, buf, len);
    }
    #endif

    return(len);
}


/**
 * @brief  向底层发送原始数据
 *
 * 将用户数据通过串口或 socket 发送出去。
 * 发送错误时自动关闭连接。
 *
 * @param[in,out] hinst  Modbus 实例指针
 * @param[in]     buf    待发送数据缓冲区
 * @param[in]     size   待发送数据长度（字节）
 *
 * @retval >0  成功发送的字节数
 * @retval -1  发送失败（连接断开、硬件故障等）
 *
 * @note
 *   - 错误时自动调用 mb_backend_close()
 *   - 启用 MB_USING_RAW_PRT 时会打印发送数据
 */
int modbus_send(mb_inst_t *hinst, uint8_t *buf, int size)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(hinst->backend != NULL);
    MB_ASSERT(buf != NULL);
    MB_ASSERT(size > 0);

    int len = modbus_backend_write(hinst->backend, buf, size);
    if (len < 0)//发生错误, 关闭后端
    {
        modbus_backend_close(hinst->backend);
    }

    #ifdef MB_USING_RAW_PRT
    if (len > 0)
    {
        modbus_raw_prt(true, buf, len);
    }
    #endif

    return(len);
}

/**
 * @brief  清空底层接收缓冲区
 *
 * 丢弃串口或 socket 中所有待读取的数据。
 * 常用于同步通信、清除垃圾数据。
 *
 * @param[in,out] hinst  Modbus 实例指针
 *
 * @retval  0  清空成功
 * @retval -1  清空失败（设备未打开等）
 *
 * @note
 *   - 常在发送请求前调用，确保接收的是最新应答
 */
int modbus_flush(mb_inst_t *hinst)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(hinst->backend != NULL);

    return(modbus_backend_flush(hinst->backend));
}


