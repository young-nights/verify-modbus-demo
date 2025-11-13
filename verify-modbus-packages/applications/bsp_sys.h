/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-08-05     18452       the first version
 */
#ifndef __BSP_SYS_H
#define __BSP_SYS_H



/* RTT实时操作系统的头文件 */
#include <rtthread.h>
#include <drv_common.h>
#include <board.h>
#include <rtdevice.h>
#include <rthw.h>
#include <rtdbg.h>
#include <stdio.h>
#include <rtconfig.h>
#include <string.h>
#include <stdlib.h>


/* 该头文件包含了所有CubeMX自动生成的初始化引脚 */
#include "main.h"

/* macBSP文件 */
#include "bsp_rs485_drv.h"
#include "bsp_rs485_dev.h"

#include "modbus_backend.h"
#include "modbus_byte_order_convert.h"
#include "modbus_crc.h"
#include "modbus_instance.h"
#include "modbus_pdu.h"
#include "modbus_rtu.h"
#include "modbus_tcp.h"
#include "modbus_config.h"



#endif /* __BSP_SYS_H */
