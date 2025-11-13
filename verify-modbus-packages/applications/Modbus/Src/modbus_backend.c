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
#include "modbus_backend.h"



/**
 * @brief  获取当前系统时间（毫秒）
 *
 * 返回从系统启动以来的毫秒数，用于超时判断。
 * 使用 long long 避免 32 位溢出（约 49 天）。
 *
 * @return long long  毫秒时间戳
 *
 * @note
 *   - 依赖 RT-Thread 的 rt_tick_get_millisecond()
 *   - 精度取决于系统 tick 频率（通常 1ms）
 *   - 可被用户重载（MB_WEAK）
 */
MB_WEAK long long modbus_port_get_ms(void)
{
    return(rt_tick_get_millisecond());
}


/**
 * @brief  毫秒级线程延时
 *
 * 阻塞当前线程指定毫秒数。
 * 精度取决于系统 tick，通常为 1ms。
 *
 * @param[in] tmo_ms  延时毫秒数（>= 0）
 *
 * @note
 *   - 使用 rt_thread_mdelay()，会让出 CPU
 *   - 适用于非关键实时路径
 *   - tmo_ms = 0 时立即返回
 *
 * @warning
 *   - 不要在中断或高优先级线程中使用
 *   - 可被用户重载（MB_WEAK）
 */
MB_WEAK void modbus_port_delay_ms(int tmo_ms)
{
    rt_thread_mdelay(tmo_ms);
}

// RTU 后端--------------------------------------------------------------------------------------------------------------
#ifdef MB_USING_RTU_BACKEND



/**
 * @brief  打开并初始化 RTU 串口设备（RS485/RS232）
 *
 * 支持波特率、奇偶校验、RS485 DE 引脚控制。
 * 返回 RT-Thread 设备句柄，供 read/write 使用。
 *
 * @param[in] param  指向配置结构体的指针
 *   - dev:       串口设备名（如 "uart1"）
 *   - baudrate:  波特率（如 9600）
 *   - parity:    奇偶校验（RT_SERIAL_PARITY_*）
 *   - pin:       RS485 DE 引脚，<0 表示无
 *   - lvl:       DE 高电平有效？1=高，0=低
 *
 * @return void*
 *   - 非 NULL : 设备句柄（rt_device_t）
 *   - NULL    : 打开失败
 *
 * @note
 *   - 使用中断接收（RT_DEVICE_FLAG_INT_RX）
 *   - RS485 模式下，DE 引脚信息存入 dev->user_data
 *   - 编码格式：0xABCD0000 | (pin << 1) | lvl
 *   - 可被用户重载（MB_WEAK）
 *
 * @warning
 *   - 必须在 mb_port_rtu_write() 中解析 user_data 切换 DE
 *   - 不要重复打开设备
 */
MB_WEAK void * modbus_port_rtu_open(const mb_backend_param_t *param)
{
    // 1. 参数非空
    MB_ASSERT(param != NULL);
    // 2. 设备名非空
    MB_ASSERT(param->rtu.dev != NULL);
    // 3. 获取设备名
    char *name = param->rtu.dev;
    // 4. 查找设备
    rt_device_t dev = rt_device_find(name);
    if (dev == RT_NULL){
        LOG_E("device (%s) not found.", name);
        return(NULL);
    }

    // 5. 默认配置
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;
    cfg.baud_rate = param->rtu.baudrate;
    cfg.parity = param->rtu.parity;
    if (rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &cfg) < 0){
        LOG_E("device (%s) config fail.", name);
        return(NULL);
    }
    // 6. 开启串口设备
    if ( rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) < 0){
        LOG_E("device (%s)  open fail.", name);
        return(NULL);
    }
    // 7. RS485 DE 引脚
    int pin = param->rtu.pin;
    // 8. 电平极性
    int lvl = param->rtu.lvl;

    if (pin >= 0){
        uint32_t ud = 0xABCD0000 + ((pin << 1) & 0xFFFF)  + (lvl ? 1 : 0);
        dev->user_data = (void *)ud;
        rt_pin_mode(pin, PIN_MODE_OUTPUT);
        rt_pin_write(pin, ! lvl);
    }

    LOG_D("device (%s) open suceess.", name);

    return((void *)dev);
}


/**
 * @brief  关闭 RTU 串口设备
 *
 * 释放 modbus_port_rtu_open() 打开的设备资源。
 * 关闭后，hinst 失效，不可再用于 read/write。
 *
 * @param[in] hinst  设备句柄（由 modbus_port_rtu_open() 返回）
 *
 * @return int
 *   - 0  : 关闭成功
 *   - <0 : 关闭失败（RT-Thread 返回值）
 */
MB_WEAK int modbus_port_rtu_close(void *hinst)
{
    MB_ASSERT(hinst != NULL);

    rt_device_t dev = (rt_device_t)hinst;
    return(rt_device_close(dev));
}


/**
 * @brief  从 RTU 串口非阻塞读取数据
 *
 * 使用 RT-Thread 设备框架读取串口数据。
 * 配合 RT_DEVICE_FLAG_INT_RX 实现中断驱动接收。
 *
 * @param[in]  hinst    设备句柄（由 modbus_port_rtu_open() 返回）
 * @param[out] buf      接收缓冲区
 * @param[in]  bufsize  缓冲区大小（>0）
 *
 * @return int
 *   - >0 : 成功读取的字节数
 *   -  0 : 无数据可用（非错误，需继续轮询）
 *   - -1 : 读取错误（设备关闭或故障）
 *
 * @note
 *   - 必须在中断接收模式下使用（RT_DEVICE_FLAG_INT_RX）
 *   - 返回 0 时表示当前无新数据，应继续轮询或延时
 *   - 上层协议栈依赖此函数构建帧超时（t3.5）
 *   - 可被用户重载（MB_WEAK）
 */
MB_WEAK int modbus_port_rtu_read(void *hinst, uint8_t *buf, int bufsize)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(buf != NULL);

    rt_device_t dev = (rt_device_t)hinst;
    int len = rt_device_read(dev, -1, buf, bufsize);
    if (len < 0)
    {
        LOG_E("device read error.");
        return(-1);
    }

    return(len);
}


/**
 * @brief  向 RTU 串口发送数据（支持 RS485 半双工）
 *
 * 自动控制 DE 引脚切换发送/接收模式。
 * 发送完成后立即恢复接收状态。
 *
 * @param[in] hinst  设备句柄
 * @param[in] buf    发送缓冲区
 * @param[in] size   发送字节数（1~256）
 *
 * @return int
 *   - >0 : 实际发送字节数
 *   - -1 : 发送失败
 *
 * @note
 *   - DE 引脚信息来自 modbus_port_rtu_open() 的 user_data
 *   - 魔数 0xABCD0000 标识有效配置
 *   - 非阻塞写入（offset = -1）
 *   - 发送后立即切回接收，防止总线冲突
 */
MB_WEAK int modbus_port_rtu_write(void *hinst, uint8_t *buf, int size)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(buf != NULL);

    // 1. 类型转换
    rt_device_t dev = (rt_device_t)hinst;
    // 2. 读取 user_data
    uint32_t ud = (uint32_t)(dev->user_data);
    // 3. 魔术字段匹配 -> 是 -> 提取pin
    int pin = ((ud & 0xFFFF0000) == 0xABCD0000) ? ((ud & 0xFFFF) >> 1) : -1;
    // 4. 提取电平极性
    int lvl = (ud & 0x01);
    // 5. 发送模式（DE 高）
    if (pin >= 0) rt_pin_write(pin, lvl);
    // 6. 非阻塞发送
    int len = rt_device_write(dev, -1, buf, size);
    // 7. 接收模式（DE 低）
    if (pin >= 0) rt_pin_write(pin, ! lvl);

    if (len < 0){
        LOG_E("device read error.");
        return(-1);
    }

    return(len);
}


/**
 * @brief  清空 RTU 串口接收缓冲区
 *
 * 丢弃所有残留数据，确保下次接收从干净状态开始。
 * 常用于初始化、错误恢复、帧同步。
 *
 * @param[in] hinst  设备句柄（由 modbus_port_rtu_open() 返回）
 *
 * @return int
 *   -  0 : 清空成功
 *   - -1 : 设备错误（读取失败）
 *
 * @note
 *   - 使用非阻塞读取（offset = -1）
 *   - 循环直到缓冲区为空（len == 0）
 *   - 不影响发送缓冲区
 *   - 可被用户重载（MB_WEAK）
 *
 * @warning
 *   - 必须在接收状态（DE=低）下调用
 *   - 避免在高频任务中调用（可能影响实时性）
 */
MB_WEAK int modbus_port_rtu_flush(void *hinst)
{
    MB_ASSERT(hinst != NULL);

    uint8_t c;
    rt_device_t dev = (rt_device_t)hinst;
    while(1)
    {
        int len = rt_device_read(dev, -1, &c, 1);
        if (len < 0){
            return(-1);
        }
        if (len == 0){
            break;
        }
    }

    return(0);
}




/**
 * @brief  RTU 后端操作函数表（虚函数表）
 *
 * 定义了串口 RTU 模式的底层操作实现：
 * - 打开/关闭串口
 * - 读写数据（含 RS485 方向控制）
 * - 清空缓存
 *
 * @note 使用静态常量，避免重复初始化
 */
static const mb_backend_ops_t mb_port_rtu_ops =
{
    .open  = modbus_port_rtu_open,
    .close = modbus_port_rtu_close,
    .read  = modbus_port_rtu_read,
    .write = modbus_port_rtu_write,
    .flush = modbus_port_rtu_flush
};



/**
 * @brief  创建 RTU 通信后端实例
 *
 * 分配并初始化 mb_backend_t 结构体，用于串口 RTU 通信。
 * 成功返回后端指针，失败返回 NULL。
 *
 * @param[in] rtu  RTU 配置参数
 *                 - dev: 串口设备名（如 "uart1"）
 *                 - baudrate: 波特率
 *                 - parity: 校验位
 *                 - pin: RS485 DE 引脚（-1 表示不使用）
 *                 - lvl: DE 电平逻辑
 *
 * @return mb_backend_t*  成功：RTU 后端指针
 * @return NULL           失败：内存不足、strdup 失败
 *
 * @note
 *   - 使用 strdup 深拷贝设备名，确保后端生命周期独立
 *   - 默认超时：ack=300ms, byte=32ms
 *   - 底层句柄 hinst 初始为 NULL，需调用 mb_backend_open() 打开
 *
 * @warning
 *   - 当前未检查 strdup 返回值，可能导致空指针解引用
 *   - 必须与 mb_backend_destory() 成对使用
 */
static mb_backend_t * modbus_backend_create_rtu(const mb_backend_param_rtu_t *rtu)
{
    extern char *strdup(const char *str);
    mb_backend_t *backend = malloc(sizeof(mb_backend_t));
    if (backend)
    {
        backend->type = MB_BACKEND_TYPE_RTU;
        backend->param.rtu = *rtu;
        backend->param.rtu.dev = strdup(rtu->dev);
        backend->ops = &mb_port_rtu_ops;
        backend->ack_tmo_ms = MB_BKD_ACK_TMO_MS_DEF;
        backend->byte_tmo_ms = MB_BKD_BYTE_TMO_MS_DEF;
        backend->hinst = NULL;
    }

    return(backend);
}
#endif











// TCP & SOCK 后端 port 配置----------------------------------------------------------------------------------------------------------
#if (defined(MB_USING_TCP_BACKEND) || defined(MB_USING_SOCK_BACKEND))

#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>
#include <unistd.h>


/**
 * @brief  创建并连接 Modbus TCP 客户端
 *
 * 建立到指定服务器的 TCP 连接（阻塞模式）
 * 支持 IP 地址或域名解析。
 *
 * @param[in] param  指向配置结构体的指针
 *   - host: 目标 IP 或域名（如 "192.168.1.10"）
 *   - port: 目标端口（通常 502）
 *
 * @return void*
 *   - 非 NULL : socket 句柄（int）
 *   - NULL    : 连接失败
 *
 * @note
 *   - 使用阻塞 connect()，建议设置超时
 *   - 返回的句柄可用于 mb_port_tcp_read/write/close
 *   - 支持 IPv4，仅客户端模式
 *   - 可被用户重载（MB_WEAK）
 *
 * @warning
 *   - gethostbyname() 可能阻塞数秒
 *   - 建议连接后设置为非阻塞模式
 * @example
 *   mb_backend_param_t p = {
 *       .tcp.host = "10.0.0.50",
 *       .tcp.port = 502
 *   };
 *   void *h = modbus_port_tcp_open(&p);
 *   if (!h) LOG_E("connect failed");
 */
MB_WEAK void * modbus_port_tcp_open(const mb_backend_param_t *param)
{
    MB_ASSERT(param != NULL);

    // 1. 创建 TCP socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0){
        LOG_E("socket create fail.");
        return(NULL);
    }
    LOG_D("socket create success, fd = %d.", sock);

    // 2. 域名解析
    struct hostent *host = gethostbyname(param->tcp.host);
    if (host == NULL){
        closesocket(sock);
        LOG_E("host get error.");
        return(NULL);
    }

    // 3. 构造地址
    struct sockaddr_in srv_addr;
    memset(&srv_addr, 0, sizeof(srv_addr));
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_port = htons(param->tcp.port);
    srv_addr.sin_addr = *((struct in_addr *)host->h_addr);
    if (connect(sock, (struct sockaddr *)&srv_addr, sizeof(struct sockaddr)) < 0)
    {
        closesocket(sock);
        LOG_E("socket connect fail.");
        return(NULL);
    }

    return((void *)sock);
}


/**
 * @brief  关闭 Modbus TCP 客户端连接
 *
 * 释放 modbus_port_tcp_open() 创建的 socket 资源
 * 关闭后，hinst 失效，不可再用于 read/write。
 *
 * @param[in] hinst  socket 句柄（由 modbus_port_tcp_open() 返回）
 *
 * @return int
 *   -  0 : 关闭成功
 *   - -1 : 关闭失败
 *
 * @note
 *   - 使用 closesocket()，适配 RT-Thread (lwIP)
 *   - 必须在 mb_master_deinit() 之后调用
 *   - 关闭后底层 TCP 连接进入 FIN-WAIT 状态
 *   - 可被用户重载（MB_WEAK）
 */
MB_WEAK int modbus_port_tcp_close(void *hinst)
{
    MB_ASSERT(hinst != NULL);

    int sock = (int)hinst;
    return((closesocket(sock) == 0) ? 0 : -1);
}


/**
 * @brief  从 TCP socket 非阻塞读取数据
 *
 * 使用 MSG_DONTWAIT 实现非阻塞接收。
 * 返回值与 RTU 版一致，便于协议栈统一处理。
 *
 * @param[in]  hinst    socket 句柄（由 modbus_port_tcp_open() 返回）
 * @param[out] buf      接收缓冲区
 * @param[in]  bufsize  缓冲区大小（>0）
 *
 * @return int
 *   - >0 : 成功读取的字节数
 *   -  0 : 无数据可用（继续轮询）
 *   - -1 : 连接已关闭或严重错误
 *
 * @note
 *   - 必须在非阻塞模式下使用
 *   - len < 0 且 errno == EAGAIN → 正常无数据
 *   - 上层应使用超时轮询
 */
MB_WEAK int modbus_port_tcp_read(void *hinst, uint8_t *buf, int bufsize)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(buf != NULL);

    int sock = (int)hinst;
    int len = recv(sock, buf, bufsize, MSG_DONTWAIT);
    if (len == 0)//socket已关闭
    {
        LOG_E("TCP read error.");
        return(-1);
    }
    if (len < 0)//超时或其它错误
    {
        return(0);
    }

    return(len);
}


/**
 * @brief  向 TCP socket 发送数据（阻塞模式）
 *
 * 使用阻塞 send() 确保数据完整发送。
 * 返回实际发送字节数，与 RTU 版接口一致。
 *
 * @param[in] hinst  socket 句柄
 * @param[in] buf    发送缓冲区
 * @param[in] size   发送字节数（1~260）
 *
 * @return int
 *   - >0 : 实际发送字节数
 *   - -1 : 发送失败或连接断开
 *
 * @note
 *   - 阻塞模式下，send() 会自动重试直到全部发送
 *   - 若返回 len < size，应循环发送剩余部分
 *   - 上层应检查返回值是否等于 size
 *   - 可被用户重载（MB_WEAK）
 */
MB_WEAK int modbus_port_tcp_write(void *hinst, uint8_t *buf, size_t size)
{
    MB_ASSERT(hinst != NULL);
    MB_ASSERT(buf != NULL);

    int sock = (int)hinst;
    int len = send(sock, buf, size, 0);
    if (len <= 0)//socket已关闭
    {
        LOG_E("TCP write error.");
        return(-1);
    }

    return(len);
}


/**
 * @brief  清空 TCP socket 接收缓冲区
 *
 * 丢弃所有残留数据，确保下次接收从干净状态开始。
 * 常用于连接初始化、错误恢复、重连后同步。
 *
 * @param[in] hinst  socket 句柄（由 modbus_port_tcp_open() 返回）
 *
 * @return int
 *   -  0 : 清空成功（或无数据）
 *   - -1 : 连接已关闭（对端发送 FIN）
 *
 * @note
 *   - 使用非阻塞读取（MSG_DONTWAIT）
 *   - 循环直到缓冲区为空（len < 0 且 errno == EAGAIN）
 *   - 不影响发送缓冲区
 *   - 可被用户重载（MB_WEAK）
 */
MB_WEAK int modbus_port_tcp_flush(void *hinst)
{
    MB_ASSERT(hinst != NULL);

    uint8_t c;
    int sock = (int)hinst;
    while(1)
    {
        int len = recv(sock, &c, 1, MSG_DONTWAIT);
        if (len == 0)//socket已关闭
        {
            return(-1);
        }
        if (len < 0)//超时或其它错误
        {
            break;
        }
    }
    return(0);
}

#endif


// TCP 后端----------------------------------------------------------------------------------------------------------
#ifdef MB_USING_TCP_BACKEND

/**
 * @brief  TCP 客户端后端操作函数表（虚函数表）
 *
 * 定义了 Modbus TCP 客户端的底层 socket 操作：
 * - 连接服务器
 * - 读写数据（非阻塞）
 * - 清空接收缓冲区
 *
 * @note 使用静态常量，节省内存
 */
static const mb_backend_ops_t mb_port_tcp_ops =
{
    modbus_port_tcp_open,
    modbus_port_tcp_close,
    modbus_port_tcp_read,
    modbus_port_tcp_write,
    modbus_port_tcp_flush
};


/**
 * @brief  创建 TCP 客户端通信后端实例
 *
 * 分配并初始化 mb_backend_t 结构体，用于 Modbus TCP 客户端通信。
 * 成功返回后端指针，失败返回 NULL。
 *
 * @param[in] tcp  TCP 配置参数
 *                 - host: 服务器 IP 地址或域名（如 "192.168.1.100"）
 *                 - port: 服务器端口（通常 502）
 *
 * @return modbus_backend_t*  成功：TCP 后端指针
 * @return NULL               失败：内存不足、strdup 失败
 *
 * @note
 *   - 使用 calloc 分配并清零内存
 *   - 使用 strdup 深拷贝 host 字符串，确保后端生命周期独立
 *   - 默认超时：ack=300ms, byte=32ms
 *   - 底层 socket 句柄 hinst 初始为 NULL，需调用 mb_backend_open() 连接
 *
 * @warning
 *   - 当前未检查 strdup 返回值，可能导致空指针解引用
 *   - 必须与 mb_backend_destory() 成对使用
 *   - 不支持服务器模式（仅客户端）
 */
static mb_backend_t *modbus_backend_create_tcp(const mb_backend_param_tcp_t *tcp)
{
    extern char *strdup(const char *str);
    mb_backend_t *backend = calloc(1, sizeof(mb_backend_t));
    if (backend)
    {
        backend->type = MB_BACKEND_TYPE_TCP;
        backend->param.tcp.host = strdup(tcp->host);
        backend->param.tcp.port = tcp->port;
        backend->ops = &mb_port_tcp_ops;
        backend->ack_tmo_ms = MB_BKD_ACK_TMO_MS_DEF;
        backend->byte_tmo_ms = MB_BKD_BYTE_TMO_MS_DEF;
        backend->hinst = NULL;
    }

    return(backend);
}
#endif



// SOCK 后端-----------------------------------------------------------------------------------------------------------
#ifdef MB_USING_SOCK_BACKEND
/**
 * @brief  SOCK 后端操作函数表（虚函数表）
 *
 * 用于接管外部传入的已连接 socket 文件描述符。
 * 特点：
 * - open = NULL：表示无需连接，已就绪
 * - 其他操作复用 TCP 实现
 *
 * @note 适用于 Modbus TCP 服务器、WebSocket 代理、隧道等场景
 */
static const mb_backend_ops_t mb_port_sock_ops =
{
    NULL,
    modbus_port_tcp_close,
    modbus_port_tcp_read,
    modbus_port_tcp_write,
    modbus_port_tcp_flush
};


/**
 * @brief  创建 SOCK 通信后端实例（接管外部 socket fd）
 *
 * 分配并初始化 mb_backend_t 结构体，用于复用已连接的 socket。
 * 成功返回后端指针，失败返回 NULL。
 *
 * @param[in] sock  SOCK 配置参数
 *                  - fd: 已连接的 socket 文件描述符（必须有效）
 *
 * @return mb_backend_t*  成功：SOCK 后端指针
 * @return NULL           失败：内存不足、fd 无效
 *
 * @note
 *   - 使用 calloc 分配并清零内存
 *   - 不进行 connect()，直接使用传入的 fd
 *   - hinst 直接指向 fd，mb_backend_open() 会自动返回成功
 *   - 默认超时：ack=300ms, byte=32ms
 *
 * @warning
 *   - 调用者必须保证 fd 有效且已连接
 *   - fd 的生命周期由调用者管理，mb_backend_destory() 会关闭 fd
 *   - 必须与 modbus_backend_destory() 成对使用
 */
static mb_backend_t *modbus_backend_create_sock(const mb_backend_param_sock_t *sock)
{
    mb_backend_t *backend = calloc(1, sizeof(mb_backend_t));
    if (backend)
    {
        backend->type = MB_BACKEND_TYPE_SOCK;
        backend->param.sock.fd = sock->fd;
        backend->ops = &mb_port_sock_ops;
        backend->ack_tmo_ms = MB_BKD_ACK_TMO_MS_DEF;
        backend->byte_tmo_ms = MB_BKD_BYTE_TMO_MS_DEF;
        backend->hinst = (void *)(sock->fd);
    }

    return(backend);
}
#endif


/**
 * @brief  创建 Modbus 通信后端（工厂函数）
 *
 * 根据指定的后端类型（RTU/TCP/SOCK）创建并初始化 mb_backend_t 实例。
 * 成功返回后端指针，失败返回 NULL。
 *
 * @param[in] type   后端类型
 *                   - MB_BACKEND_TYPE_RTU  : 串口 RTU 模式
 *                   - MB_BACKEND_TYPE_TCP  : TCP 客户端模式
 *                   - MB_BACKEND_TYPE_SOCK : 已有 socket fd
 * @param[in] param  后端配置参数（联合体）
 *                   - RTU:  包含 dev, baudrate, parity, pin, lvl
 *                   - TCP:  包含 host, port
 *                   - SOCK: 包含 fd
 *
 * @return mb_backend_t*  成功：指向新创建的后端实例
 * @return NULL           失败：内存不足、strdup 失败、类型不支持
 *
 * @note
 *   - 使用条件编译（#ifdef）裁剪未启用的后端，节省 Flash
 *   - 所有子创建函数（create_rtu/tcp/sock）负责：
 *       • 内存分配
 *       • 字符串深拷贝（dev/host）
 *       • 绑定 ops 虚函数表
 *       • 设置默认超时
 *   - 调用者无需关心底层实现差异
 *
 * @warning
 *   - param 必须非 NULL，否则行为未定义
 *   - 必须与 mb_backend_destory() 成对使用
 *   - 不支持的 type 会返回 NULL
 */
mb_backend_t *modbus_backend_create(mb_backend_type_t type, const mb_backend_param_t *param)//创建后端, 成功返回后端指针, 失败返回NULL
{
    mb_backend_t *backend = NULL;
    switch(type)
    {
    #ifdef MB_USING_RTU_BACKEND
    case MB_BACKEND_TYPE_RTU :
        backend = modbus_backend_create_rtu(&(param->rtu));
        break;
    #endif

    #ifdef MB_USING_TCP_BACKEND
    case MB_BACKEND_TYPE_TCP :
        backend = modbus_backend_create_tcp(&(param->tcp));
        break;
    #endif

    #ifdef MB_USING_SOCK_BACKEND
    case MB_BACKEND_TYPE_SOCK :
        backend = modbus_backend_create_sock(&(param->sock));
        break;
    #endif

    default:
        break;
    }

    return(backend);
}


/**
 * @brief  销毁 Modbus 通信后端，释放所有资源
 *
 * 安全地释放由 modbus_backend_create() 创建的后端实例。
 * 资源释放顺序：
 *   1. 关闭底层通信通道（串口/socket）
 *   2. 释放动态分配的字符串（dev/host）
 *   3. 释放后端结构体
 *
 * @param[in,out] backend  待销毁的后端指针
 *                         - 可为 NULL（安全返回）
 *                         - 调用后指针失效
 *
 * @return 无返回值
 *
 * @note
 *   - 必须与 modbus_backend_create() 成对调用
 *   - RTU 后端：释放 dev 字符串（strdup 分配）
 *   - TCP 后端：释放 host 字符串
 *   - SOCK 后端：无字符串，仅关闭 fd
 *   - 内部调用 modbus_backend_close() 关闭连接
 *
 * @warning
 *   - 函数名拼写错误：应为 modbus_backend_destroy
 *   - 调用后 backend 指针立即失效，禁止再次使用
 *   - 不要对已销毁的指针重复调用
 */
void modbus_backend_destory(mb_backend_t *backend)
{
    if (backend == NULL){
        return;
    }

    modbus_backend_close(backend);

    switch(backend->type)
    {
    #ifdef MB_USING_RTU_BACKEND
    case MB_BACKEND_TYPE_RTU :
        if (backend->param.rtu.dev)
        {
            free(backend->param.rtu.dev);
            backend->param.rtu.dev = NULL;
        }
        break;
    #endif

    #ifdef MB_USING_TCP_BACKEND
    case MB_BACKEND_TYPE_TCP :
        if (backend->param.tcp.host)
        {
            free(backend->param.tcp.host);
            backend->param.tcp.host = NULL;
        }
        break;
    #endif

    default:
        break;
    }

    free(backend);
}


/**
 * @brief  打开底层通信通道（串口或 socket）
 *
 * 调用后端虚函数表中的 open() 函数，建立物理连接。
 * 支持幂等调用：已打开时直接返回成功。
 *
 * @param[in,out] backend  后端实例指针
 *
 * @retval  0  打开成功（或已打开）
 * @retval -1  打开失败（参数错误、ops 损坏、底层失败）
 *
 * @note
 *   - RTU 后端：打开串口设备
 *   - TCP 后端：创建 socket 并连接服务器
 *   - SOCK 后端：open = NULL，跳过打开，直接返回成功（已连接）
 *   - 重复调用不会重复打开
 *
 * @warning
 *   - 当前实现对 SOCK 后端不友好（open == NULL 返回 -1）
 *   - 建议修改为：仅当 open != NULL 时才调用
 */
int modbus_backend_open(mb_backend_t *backend)//打开后端, 成功返回0, 错误返回-1
{
    if (backend == NULL){
        return(-1);
    }

    if (backend->hinst != NULL){
        return(0);
    }

    if ((backend->ops == NULL) || (backend->ops->open == NULL)){
        return(-1);
    }

    backend->hinst = backend->ops->open((void *)&(backend->param));
    if (backend->hinst == NULL){
        return(-1);
    }
    return(0);
}


/**
 * @brief  关闭底层通信通道（串口或 socket）
 *
 * 调用后端虚函数表中的 close() 函数，释放底层句柄。
 * 支持幂等调用：已关闭时直接返回成功。
 *
 * @param[in,out] backend  后端实例指针
 *
 * @retval  0  关闭成功（或已关闭）
 * @retval -1  关闭失败（参数错误、ops 损坏、底层失败）
 *
 * @note
 *   - RTU 后端：关闭串口设备
 *   - TCP/SOCK 后端：关闭 socket
 *   - 重复调用不会重复关闭
 *   - 关闭后 hinst 被置为 NULL，防止野指针
 *
 * @warning
 *   - 必须在通信结束或程序退出前调用
 *   - 关闭失败可能导致资源泄露
 */
int modbus_backend_close(mb_backend_t *backend)
{
    if (backend == NULL)
    {
        return(-1);
    }
    if (backend->hinst == NULL)//已关闭
    {
        return(0);
    }
    if ((backend->ops == NULL) || (backend->ops->close == NULL))
    {
        return(-1);
    }
    if (backend->ops->close(backend->hinst) != 0)//关闭失败
    {
        return(-1);
    }
    backend->hinst = NULL;
    return(0);
}


/**
 * @brief  配置 Modbus 通信超时参数
 *
 * 设置主站模式下的两个关键超时：
 *   - 应答超时（ack_tmo_ms）：从站未响应最大等待时间
 *   - 字节间超时（byte_tmo_ms）：帧内字节最大间隔，用于判断帧结束
 *
 * @param[in,out] backend       后端实例指针
 * @param[in]     ack_tmo_ms    应答超时时间（毫秒），建议 100~5000
 * @param[in]     byte_tmo_ms   字节间超时时间（毫秒），建议 10~100
 *
 * @retval  0  配置成功
 * @retval -1  backend 为 NULL
 *
 * @note
 *   - 立即生效，下次 mb_backend_read() 使用新值
 *   - 所有后端（RTU/TCP/SOCK）共用此配置
 *   - 默认值由 MB_BKD_ACK_TMO_MS_DEF / MB_BKD_BYTE_TMO_MS_DEF 定义
 *
 * @warning
 *   - 不建议在通信过程中频繁修改，可能导致帧错位
 *   - 过小的 byte_tmo_ms 可能导致帧被截断
 */
int modbus_backend_timeout_config(mb_backend_t *backend, int ack_tmo_ms, int byte_tmo_ms)//配置后端超时参数, 成功返回0, 错误返回-1
{
    if (backend == NULL)
    {
        return(-1);
    }
    backend->ack_tmo_ms = ack_tmo_ms;
    backend->byte_tmo_ms = byte_tmo_ms;
    return(0);
}


/**
 * @brief  从后端读取数据（支持应答超时 + 字节间超时）
 *
 * 实现 Modbus 协议帧同步的核心逻辑：
 *   - 第一次字节：使用应答超时（ack_tmo_ms）
 *   - 后续字节：使用字节间超时（byte_tmo_ms）
 *
 * @param[in,out] backend  后端实例指针
 * @param[out]    buf      接收缓冲区
 * @param[in]     bufsize  缓冲区大小（>0）
 *
 * @retval >0  成功接收的字节数（可能是一完整帧）
 * @retval  0  超时（应答超时或字节间超时）
 * @retval -1  错误（参数错误、未打开、ops 错误、底层 read 失败）
 *
 * @note
 *   - 每次收到字节会重置计时器
 *   - 每轮循环延时 2ms，避免 CPU 100%
 *   - 底层 read() 应为非阻塞模式
 *
 * @warning
 *   - 必须先调用 mb_backend_open()
 *   - 超时值由 modbus_backend_timeout_config() 设置
 */
int modbus_backend_read(mb_backend_t *backend, uint8_t *buf, int bufsize)
{
    if ((backend == NULL) || (buf == NULL) || (bufsize <= 0))
    {
        return(-1);
    }
    if (backend->hinst == NULL)//未打开
    {
        return(-1);
    }
    if ((backend->ops == NULL) || (backend->ops->read == NULL))
    {
        return(-1);
    }
    int pos = 0;
    long long told_ms = modbus_port_get_ms();
    while(pos < bufsize)
    {
        int len = backend->ops->read(backend->hinst, buf + pos, bufsize - pos);
        if (len < 0)//发生错误
        {
            return(-1);
        }
        if (len > 0)//读到数据
        {
            told_ms = modbus_port_get_ms();
            pos += len;
            continue;
        }
        int tmo_ms = modbus_port_get_ms() - told_ms;
        if (pos)//已有数据接收到, 则检查字节超时
        {
            if (tmo_ms > backend->byte_tmo_ms)//字节超时了
            {
                break;
            }
        }
        else//未收到过数据, 则检查应答超时
        {
            if (tmo_ms > backend->ack_tmo_ms)//应答超时了
            {
                break;
            }
        }
        modbus_port_delay_ms(2);
    }
    return(pos);
}


/**
 * @brief  向后端写入数据
 *
 * 直接调用底层 write() 函数发送数据。
 * 不进行分片或超时处理。
 *
 * @param[in,out] backend  后端实例指针
 * @param[in]     buf      待发送数据缓冲区
 * @param[in]     size     待发送数据长度（>0）
 *
 * @retval >0  成功发送的字节数
 * @retval -1  错误（参数错误、未打开、ops 错误）
 *
 * @note
 *   - RTU 模式：write() 内部会自动切换 RS485 方向
 *   - TCP/SOCK：直接 send()
 *
 * @warning
 *   - 必须先调用 mb_backend_open()
 */
int modbus_backend_write(mb_backend_t *backend, uint8_t *buf, int size)
{
    if ((backend == NULL) || (buf == NULL) || (size <= 0))
    {
        return(-1);
    }
    if (backend->hinst == NULL)//未打开
    {
        return(-1);
    }
    if ((backend->ops == NULL) || (backend->ops->write == NULL))
    {
        return(-1);
    }
    return(backend->ops->write(backend->hinst, buf, size));
}


/**
 * @brief  清空后端接收缓冲区
 *
 * 丢弃底层设备中所有待读取的数据。
 * 常用于：
 *   - 发送请求前清除残留应答
 *   - 通信异常后重同步
 *
 * @param[in,out] backend  后端实例指针
 *
 * @retval  0  清空成功
 * @retval -1  错误（参数错误、未打开、ops 错误）
 *
 * @note
 *   - 串口/TCP：循环非阻塞 read 直到无数据
 *
 * @warning
 *   - 必须先调用 mb_backend_open()
 */
int modbus_backend_flush(mb_backend_t *backend)
{
    if (backend == NULL)
    {
        return(-1);
    }
    if (backend->hinst == NULL)//未打开
    {
        return(-1);
    }
    if ((backend->ops == NULL) || (backend->ops->flush == NULL))
    {
        return(-1);
    }
    return(backend->ops->flush(backend->hinst));
}






