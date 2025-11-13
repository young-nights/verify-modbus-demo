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

static void mb_sample_read_regs(mb_inst_t *hinst)
{

    // 连接失败
    if (modbus_connect(hinst) < 0){
        LOG_E("modbus connect fail.");
        return;
    }

    uint16_t regs[64];
    int addr = 4000;
    int nb = 29;
    int total = modbus_read_regs(hinst, addr, nb, regs);
    if (total <= 0)
    {
        LOG_E("modbus read register fail.");
        return;
    }

    LOG_D("modbus read register success.");

    for (int i=0; i<total; i++)
    {
        LOG_D("addr : %d, value : %d", addr + i, regs[i]);
    }
}



static void modbus_message_thread_entry(void *args)
{
    mb_inst_t *modbus_hinst = modbus_create(MB_BACKEND_TYPE_RTU, &mb_bkd_prm);
    RT_ASSERT(modbus_hinst != NULL);

    while(1)
    {
//        mb_sample_read_regs(modbus_hinst);
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

















