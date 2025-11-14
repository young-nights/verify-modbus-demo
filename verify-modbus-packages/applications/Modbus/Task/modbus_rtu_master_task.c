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


#ifdef MB_USING_RTU_MASTER

#define DBG_TAG "mb.rtu.master"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static const mb_backend_param_t mb_bkd_prm = {
    .rtu.dev = "uart3",     //设备名
    .rtu.baudrate = 115200, //波特率
    .rtu.parity = 0,        //校验位, 0-无, 1-奇, 2-偶
    .rtu.pin = 79,          //控制引脚, <0 表示不使用
    .rtu.lvl = 1            //控制发送电平
};


// 以写入指令为例：01 06 00 6B 00 01 39 D6
static void modbus_message_thread_entry(void *args)
{
    mb_inst_t *modbus_hinst = modbus_create(MB_BACKEND_TYPE_RTU, &mb_bkd_prm);
    if (modbus_hinst == NULL) {
        LOG_E("Failed to create Modbus instance.");
        return;
    }
    // 设置从机地址（默认1，可修改）
    modbus_set_slave_addr(modbus_hinst, 1);

    // 连接设备（打开串口）
    if (modbus_connect(modbus_hinst) < 0) {
        LOG_E("Modbus connect failed.");
        modbus_destroy(modbus_hinst);  // 释放实例
        return;
    }

    // 4. 准备要写入的数据（3个寄存器）
    uint16_t write_value = 0x0001; // 要写入的值
    int start_addr = 0x006B;       // 起始地址

    // 5. 执行写入操作（功能码 0x10）
    modbus_write_reg(modbus_hinst, start_addr, write_value);



    while(1)
    {
        modbus_write_reg(modbus_hinst, start_addr, write_value);
        rt_thread_mdelay(1000);
    }
}

static int modbus_rtu_master_startup(void)
{
    rt_thread_t Modbus_Thread_Handle = rt_thread_create("mb-rtu-master", modbus_message_thread_entry, NULL, 2048, 6, 20);
    RT_ASSERT(Modbus_Thread_Handle != NULL);
    rt_thread_startup(Modbus_Thread_Handle);
    return(0);
}
INIT_APP_EXPORT(modbus_rtu_master_startup);

#endif /* MB_USING_MASTER */















