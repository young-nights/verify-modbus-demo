/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-08-05     18452       the first version
 */

#ifndef __MODBUS_INSTANCE_H
#define __MODBUS_INSTANCE_H

#include "modbus_rtu.h"
#include "modbus_backend.h"


#ifdef MB_USING_RAW_PRT
/**
 * @brief 调试用：打印 Modbus 原始数据帧
 * @see modbus_raw_printf 实现细节
 */
void modbus_raw_printf(bool is_send, const u8 *pdata, int dlen);
#endif



#ifdef MB_TCP_FRM_MAX
#define MB_BUF_SIZE MB_TCP_FRM_MAX
#else
#define MB_BUF_SIZE MB_RTU_FRM_MAX
#endif


/**
 * 通讯协议类型
 */
typedef enum{
    MB_PROT_RTU = 0,    //MODBUS-RTU通信协议
    MB_PROT_TCP         //MODBUS-TCP通信协议
}mb_prot_t;

typedef int (*mb_read_bit_t)(uint16_t addr, uint8_t *pbit);//读bit位, 返回: 0-成功, -2-地址错误
typedef int (*mb_write_bit_t)(uint16_t addr, uint8_t bit);//写bit位, 返回: 0-成功, -2-地址错误, -4-设备故障
typedef int (*mb_read_reg_t)(uint16_t addr, uint16_t *pval);//读16位寄存器, 返回 : 0-成功, -2-地址错误
typedef int (*mb_write_reg_t)(uint16_t addr, uint16_t val);//写16位寄存器, 返回 : 0-成功, -2-地址错误, -3-值非法, -4-设备故障
typedef int (*mb_mask_write_t)(uint16_t addr, uint16_t mask_and, uint16_t mask_or);//屏蔽写寄存器, 返回 : 0-成功, -2-地址错误, -3-值非法, -4-设备故障


/**
 * 从机回调函数表定义
 */
typedef struct{
    mb_read_bit_t   read_disc;  //读离散量输入
    mb_read_bit_t   read_coil;  //读线圈
    mb_write_bit_t  write_coil; //写线圈
    mb_read_reg_t   read_input; //读输入寄存器
    mb_read_reg_t   read_hold;  //读保持寄存器
    mb_write_reg_t  write_hold; //写保持寄存器
}mb_cb_table_t;



/**
 * @brief MODBUS实例结构体参数
 */
typedef struct{
    rt_uint8_t  saddr;          // 从机地址
    mb_prot_t   prototype;      // 设备类型(RTU or TCP)
    rt_uint16_t tsid;           // TCP传输标识计数
    mb_backend_t *backend;      // 后端指针
    mb_cb_table_t *cb;          // 从机回调函数表
    uint8_t datas[256];         // 读写数据缓冲区
    uint8_t buf[MB_BUF_SIZE];   // 收发缓冲区
}mb_inst_t;

//创建modbus实例, 成功返回实例指针, 失败返回NULL
mb_inst_t * modbus_create(mb_backend_type_t type, const mb_backend_param_t *param);
//销毁modbus实例
void modbus_destory(mb_inst_t *hinst);
//修改从机地址, 默认地址为1
void modbus_set_slave(mb_inst_t *hinst, uint8_t saddr);
//修改协议, 默认使用与后端类型一致的协议类型
void modbus_set_prot(mb_inst_t *hinst, mb_prot_t prot);
//修改超时时间, 默认应答超时300ms, 字节超时32ms
void modbus_set_tmo(mb_inst_t *hinst, int ack_tmo_ms, int byte_tmo_ms);
//建立连接, 成功返回0, 失败返回-1, 如已建立连接则立即返回
int modbus_connect(mb_inst_t *hinst);
//断开连接, 成功返回0, 失败返回-1, 如已断开连接则立即返回
int modbus_disconn(mb_inst_t *hinst);
//接收数据, 返回收到数据长度, 超时返回0, 错误返回-1, 发生错误时会自动关闭后端
int modbus_recv(mb_inst_t *hinst, uint8_t *buf, int bufsize);
//发送数据, 返回发送数据长度, 错误返回-1, 发生错误时会自动关闭后端
int modbus_send(mb_inst_t *hinst, uint8_t *buf, int size);
//清空接收缓存, 成功返回0, 失败返回-1
int modbus_flush(mb_inst_t *hinst);

#ifdef MB_USING_MASTER
//读请求, 功能码和数据由用户确定, 成功返回应答数据长度, 异常应答返回负值错误码, 其它错误返回0
int mb_read_req(mb_inst_t *hinst, uint8_t func, uint16_t addr, int nb, uint8_t *pdata);
//写请求, 功能码和数据由用户确定, 成功返回请求数量, 异常应答返回负值错误码, 其它错误返回0
int mb_write_req(mb_inst_t *hinst, uint8_t func, uint16_t addr, int nb, const uint8_t *pdata, int dlen);
//读多个线圈, 功能码-0x01, 成功返回读取位数量, 异常应答返回负值错误码, 其它错误返回0
int mb_read_bits(mb_inst_t *hinst, uint16_t addr, int nb, uint8_t *pbits);
//读多个离散量输入, 功能码-0x02, 成功返回读取位数量, 异常应答返回负值错误码, 其它错误返回0
int mb_read_input_bits(mb_inst_t *hinst, uint16_t addr, int nb, uint8_t *pbits);
//读多个保持寄存器, 功能码-0x03, 成功返回读取寄存器数量, 异常应答返回负值错误码, 其它错误返回0
int mb_read_regs(mb_inst_t *hinst, uint16_t addr, int nb, uint16_t *pregs);
//读多个输入寄存器, 功能码-0x04, 成功返回读取寄存器数量, 异常应答返回负值错误码, 其它错误返回0
int mb_read_input_regs(mb_inst_t *hinst, uint16_t addr, int nb, uint16_t *pregs);
//写单个线圈, 功能码-0x05, 成功返回1, 异常应答返回负值错误码, 其它错误返回0
int mb_write_bit(mb_inst_t *hinst, uint16_t addr, uint8_t bit);
//写单个保持寄存器, 功能码-0x06, 成功返回1, 异常应答返回负值错误码, 其它错误返回0
int mb_write_reg(mb_inst_t *hinst, uint16_t addr, uint16_t val);
//写多个线圈, 功能码-0x0F, 成功返回写位数量, 异常应答返回负值错误码, 其它错误返回0
int mb_write_bits(mb_inst_t *hinst, uint16_t addr, int nb, const uint8_t *pbits);
//写多个保持寄存器, 功能码-0x10, 成功返回写寄存器数量, 异常应答返回负值错误码, 其它错误返回0
int mb_write_regs(mb_inst_t *hinst, uint16_t addr, int nb, const uint16_t *pregs);
//屏蔽写保持寄存器, 功能码-0x16, 成功返回1, 异常应答返回负值错误码, 其它错误返回0
int mb_mask_write_reg(mb_inst_t *hinst, uint16_t addr, uint16_t mask_and, uint16_t mask_or);
//读/写多个保持寄存器, 功能码-0x17, 成功返回读取寄存器数量, 异常应答返回负值错误码, 其它错误返回0
int mb_write_and_read_regs(mb_inst_t *hinst, uint16_t wr_addr, int wr_nb, const uint16_t *p_wr_regs,uint16_t rd_addr, int rd_nb, uint16_t *p_rd_regs);
#endif

#ifdef MB_USING_SLAVE
//修改从机回调函数表, 默认使用modbus_port中接口函数做回调函数
void mb_set_cb_table(mb_inst_t *hinst, const mb_cb_table_t *cb);
//从机状态机处理, 在线程中循环调用即可
void mb_slave_fsm(mb_inst_t *hinst);
#endif



#endif /* __MODBUS_INSTANCE_H */
