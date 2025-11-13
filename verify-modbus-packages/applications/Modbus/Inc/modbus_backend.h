/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-12     18452       the first version
 */
#ifndef APPLICATIONS_MODBUS_BACKEND_H_
#define APPLICATIONS_MODBUS_BACKEND_H_

#include "modbus_config.h"

//-----------------------------------------------------------------------------
#define MB_ASSERT(x)        //ASSERT(x)
#define MB_WEAK             __attribute__((weak))

//-----------------------------------------------------------------------------
#define MB_BKD_ACK_TMO_MS_DEF       300 // 响应超时时间
#define MB_BKD_BYTE_TMO_MS_DEF      32  // 字节间隔超时时间

//-----------------------------------------------------------------------------
#define MB_USING_PORT_RTT
#if defined(MB_USING_PORT_RTT)
#include "rtthread.h"
#define MB_PRINTF           rt_kprintf
#else
#define MB_PRINTF
#endif


//-----------------------------------------------------------------------------
/**
 * @brief 后端类型定义
 * @param [0]MB_BACKEND_TYPE_RTU : RTU后端
 *        [1]MB_BACKEND_TYPE_TCP : TCP后端
 *        [2]MB_BACKEND_TYPE_SOCK: SOCK后端
 */
typedef enum{
    MB_BACKEND_TYPE_RTU = 0,
    MB_BACKEND_TYPE_TCP,
    MB_BACKEND_TYPE_SOCK
}mb_backend_type_t;


/**
 * @brief RTU后端参数定义
 * @param [0]*dev     : 设备名称
 *        [1]baudrate : 波特率
 *        [2]parity   : 校验位
 *        [3]pin      ：收发控制引脚, <0 表示不使用
 *        [4]lvl      : 发送控制电平
 */
typedef struct{
    char *dev;
    int baudrate;
    int parity;
    int pin;
    int lvl;
}mb_backend_param_rtu_t;


/**
 * @brief TCP后端参数定义
 * @param [0]*host    : ip地址或域名
 *        [1]port     : 端口
 */
typedef struct{
    char *host;
    int port;
}mb_backend_param_tcp_t;


/**
 * @brief SOCK后端参数定义
 * @param fd : socket连接标识符
 */
typedef struct{
    int fd;
}mb_backend_param_sock_t;


/**
 * @brief 后端参数联合体定义
 */
typedef union{
    mb_backend_param_rtu_t  rtu; //RTU后端参数
    mb_backend_param_tcp_t  tcp; //TCP后端参数
    mb_backend_param_sock_t sock; //SOCK后端参数
}mb_backend_param_t;

//打开, 成功返回实例指针或文件标识, 错误返回NULL
typedef void * (* modbus_bkd_ops_open_t)(const mb_backend_param_t *param);
//关闭, 成功返回0, 错误返回-1
typedef int (* modbus_bkd_ops_close_t)(void *hinst);
//接收数据, 返回接收到的数据长度, 0表示超时, 错误返回-1
typedef int (* modbus_bkd_ops_read_t)(void *hinst, uint8_t *buf, int bufsize);
//发送数据, , 返回成功发送的数据长度, 错误返回-1
typedef int (* modbus_bkd_ops_write_t)(void *hinst, uint8_t *buf, int size);
//清空接收缓存, 成功返回0, 错误返回-1
typedef int (* modbus_bkd_ops_flush_t)(void *hinst);

typedef struct{
    modbus_bkd_ops_open_t open;
    modbus_bkd_ops_close_t close;
    modbus_bkd_ops_read_t read;
    modbus_bkd_ops_write_t write;
    modbus_bkd_ops_flush_t flush;
}mb_backend_ops_t;


/**
 * @brief Modbus 通信后端实例结构体
 *
 * 统一抽象串口（RTU）、TCP 客户端、已有 socket 的通信通道。
 * 提供配置、操作接口、超时控制和底层句柄管理。
 *
 * @note
 *   - 由 modbus_backend_create() 分配，modbus_backend_destory() 释放
 *   - 所有成员生命周期由后端模块管理，上层无需直接访问
 *   - 支持运行时切换超时参数
 *
 * @warning
 *   - 不要手动修改成员，除非你清楚后果
 *   - hinst 在 close 后必须为 NULL
 */
typedef struct{
    /** @brief 后端类型 */
    mb_backend_type_t type;
    /**
     * @brief 配置参数（联合体）
     * - RTU:  串口设备名、波特率、校验位、RS485 引脚
     * - TCP:  服务器 IP 和端口
     * - SOCK: 已连接的 socket fd
     */
    mb_backend_param_t param;
    /** @brief 操作函数集（虚函数表） */
    const mb_backend_ops_t *ops;
    /** @brief 应答超时（毫秒），主站等待从站响应 */
    int ack_tmo_ms;
    /** @brief 字节间超时（毫秒），用于帧结束判断 */
    int byte_tmo_ms;
    /**
     * @brief 底层句柄
     * - RTU:  rt_device_t*
     * - TCP/SOCK: int socket fd
     * - 未打开时为 NULL
     */
    void *hinst;
}mb_backend_t;




//-----------------------------------------------------------------------------
long long modbus_port_get_ms(void);//获取毫秒时间
void modbus_port_delay_ms(int tmo_ms);//系统延时

#ifdef MB_USING_RTU_BACKEND
void * modbus_port_rtu_open(const mb_backend_param_t *param);//打开, 成功返回实例指针或文件标识, 错误返回NULL
int modbus_port_rtu_close(void *hinst);//关闭, 成功返回0, 错误返回-1
int modbus_port_rtu_read(void *hinst, uint8_t *buf, int bufsize);//接收数据, 返回接收到的数据长度, 0表示超时, 错误返回-1
int modbus_port_rtu_write(void *hinst, uint8_t *buf, int size);//发送数据, , 返回成功发送的数据长度, 错误返回-1
int modbus_port_rtu_flush(void *hinst);//清空接收缓存, 成功返回0, 错误返回-1
#endif

#if (defined(MB_USING_TCP_BACKEND) || defined(MB_USING_SOCK_BACKEND))
void * modbus_port_tcp_open(const mb_backend_param_t *param);//打开, 成功返回实例指针或文件标识, 错误返回NULL
int modbus_port_tcp_close(void *hinst);//关闭, 成功返回0, 错误返回-1
int modbus_port_tcp_read(void *hinst, uint8_t *buf, int bufsize);//接收数据, 返回接收到的数据长度, 0表示超时, 错误返回-1
int modbus_port_tcp_write(void *hinst, uint8_t *buf, size_t size);//发送数据, , 返回成功发送的数据长度, 错误返回-1
int modbus_port_tcp_flush(void *hinst);//清空接收缓存, 成功返回0, 错误返回-1
#endif


mb_backend_t *modbus_backend_create(mb_backend_type_t type, const mb_backend_param_t *param);//创建后端, 成功返回后端指针, 失败返回NULL
void modbus_backend_destory(mb_backend_t *backend);//销毁后端
int modbus_backend_open(mb_backend_t *backend);//打开后端, 成功返回0, 错误返回-1
int modbus_backend_close(mb_backend_t *backend);//关闭后端, 成功返回0, 错误返回-1
int modbus_backend_config(mb_backend_t *backend, int ack_tmo_ms, int byte_tmo_ms);//配置后端超时参数, 成功返回0, 错误返回-1
int modbus_backend_read(mb_backend_t *backend, uint8_t *buf, int bufsize);//从后端读数据, 返回读取到数据长度, 0表示超时, 错误返回-1
int modbus_backend_write(mb_backend_t *backend, uint8_t *buf, int size);//向后端写数据, 返回已发送数据长度, 错误返回-1
int modbus_backend_flush(mb_backend_t *backend);//清空后端接收缓存, 成功返回0, 错误返回-1







#endif /* APPLICATIONS_MODBUS_BACKEND_H_ */
