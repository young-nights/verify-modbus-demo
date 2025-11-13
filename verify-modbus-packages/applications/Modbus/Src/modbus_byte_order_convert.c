/*
 * @file modbus_cvt.h
 * @brief Modbus 数据转换工具（字节序 + 位操作）
 *
 * 提供主机序 ↔ 网络序（大端）转换，以及位图操作。
 * 所有函数均为内联友好，适用于 RTU/TCP 协议栈。
 */
#include "bsp_sys.h"



/**
 * @brief  写入 8 位无符号整数到缓冲区
 *
 * 将一个 uint8_t 值写入指定内存位置。
 * 由于是单字节，无需字节序转换。
 *
 * @param[out] buf  目标缓冲区指针（不可为 NULL）
 * @param[in]  val  待写入的 8 位值（0~255）
 *
 * @return 1  始终返回 1，表示写入 1 字节
 *
 * @note
 *   - 与 modbus_cvt_u16_put/u32_put 保持接口一致
 *   - 常用于写入 Modbus 帧中的地址、功能码、字节计数等
 *   - 建议与指针偏移结合使用：p += modbus_cvt_u8_put(p, val);
 *
 * @warning
 *   - 不检查 buf 是否越界，由调用者保证缓冲区足够
 *   - 生产环境建议启用 MB_ASSERT 进行 NULL 检查
 */
int modbus_cvt_u8_put(uint8_t *buf, uint8_t val)
{
    *buf = val;
    return(1);
}




/**
 * @brief  从缓冲区读取 8 位无符号整数
 *
 * 从指定内存位置读取一个 uint8_t 值。
 * 由于是单字节，无需字节序转换。
 *
 * @param[in]  buf   源缓冲区指针（不可为 NULL）
 * @param[out] pval  输出值指针（不可为 NULL）
 *
 * @return 1  始终返回 1，表示读取 1 字节
 *
 * @note
 *   - 与 modbus_cvt_u16_get/u32_get 保持接口一致
 *   - 常用于解析 Modbus 帧中的地址、功能码、字节计数等
 *   - 建议与指针偏移结合使用：p += modbus_cvt_u8_get(p, &val);
 */
int modbus_cvt_u8_get(const uint8_t *buf, uint8_t *pval)
{
    *pval = *buf;
    return(1);
}


/**
 * @brief  写入 16 位无符号整数（大端序）
 *
 * 将一个 uint16_t 值以 Modbus 协议要求的**大端字节序**写入缓冲区：
 *   - 高字节在前，低字节在后
 *
 * @param[out] buf  目标缓冲区指针（不可为 NULL，至少 2 字节空间）
 * @param[in]  val  待写入的 16 位值（0~65535）
 *
 * @return 2  始终返回 2，表示写入 2 字节
 *
 * @note
 *   - 符合 Modbus 协议规范（网络字节序）
 *   - 常用于写入寄存器地址、数量、值等
 *   - 建议与指针偏移结合使用：p += modbus_cvt_u16_put(p, val);
 *
 * @example
 *   u8 buf[4];
 *   u8 *p = buf;
 *   p += modbus_cvt_u16_put(p, 0x1234);  // buf = [0x12, 0x34, ?, ?]
 *   p += modbus_cvt_u16_put(p, 0x5678);  // buf = [0x12, 0x34, 0x56, 0x78]
 */
int modbus_cvt_u16_put(uint8_t *buf, uint16_t val)
{
    uint8_t *p = buf;
    *p++ = (val >> 8);
    *p++ = val;
    return(2);
}


/**
 * @brief  从缓冲区读取 16 位无符号整数（大端序）
 *
 * 从指定内存位置读取两个字节，组合为一个 uint16_t 值。
 * 符合 Modbus 协议的大端字节序规范。
 *
 * @param[in]  buf   源缓冲区指针（不可为 NULL，至少 2 字节）
 * @param[out] pval  输出值指针（不可为 NULL）
 *
 * @return 2  始终返回 2，表示读取 2 字节
 *
 * @note
 *   - 高字节在前，低字节在后
 *   - 常用于解析寄存器地址、数量、值等
 *   - 建议与指针偏移结合：p += modbus_cvt_u16_get(p, &val);
 */
int modbus_cvt_u16_get(const uint8_t *buf, uint16_t *pval)
{
    uint8_t *p = (uint8_t *)buf;
    uint16_t uval = *p++;
    uval <<= 8;
    uval += *p++;
    *pval = uval;
    return(2);
}


/**
 * @brief  写入 32 位无符号整数（大端序）
 *
 * 将一个 uint32_t 值以大端字节序写入缓冲区。
 *
 * @param[out] buf  目标缓冲区指针（至少 4 字节）
 * @param[in]  val  待写入的 32 位值
 *
 * @return 4  始终返回 4
 *
 * @see modbus_cvt_u32_get()
 */
int modbus_cvt_u32_put(uint8_t *buf, uint32_t val)
{
    uint8_t *p = buf;
    *p++ = (val >> 24);
    *p++ = (val >> 16);
    *p++ = (val >> 8);
    *p++ = val;
    return(4);
}

/**
 * @brief  从缓冲区读取 32 位无符号整数（大端序）
 *
 * @param[in]  buf   源缓冲区指针（至少 4 字节）
 * @param[out] pval  输出值指针
 *
 * @return 4  始终返回 4
 */
int modbus_cvt_u32_get(const uint8_t *buf, uint32_t *pval)
{
    uint8_t *p = (uint8_t *)buf;
    uint32_t uval = *p++;
    uval <<= 8;
    uval += *p++;
    uval <<= 8;
    uval += *p++;
    uval <<= 8;
    uval += *p++;
    *pval = uval;
    return(4);
}

/**
 * @brief  写入 32 位浮点数（IEEE 754 + 大端）
 *
 * 将 float 值按位转换为 uint32_t，再以大端序写入。
 *
 * @param[out] buf  目标缓冲区
 * @param[in]  val  浮点值
 *
 * @return 4  占用 4 字节
 *
 * @warning
 *   - 仅在 IEEE 754 平台有效
 *   - 浮点数在不同平台可能不一致
 */
int modbus_cvt_f32_put(uint8_t *buf, float val)
{
    float fval = val;
    uint32_t *puv = (uint32_t *)&fval;
    return(modbus_cvt_u32_put(buf, *puv));
}


/**
 * @brief  读取 32 位浮点数（IEEE 754 + 大端）
 *
 * @param[in]  buf   源缓冲区
 * @param[out] pval  输出浮点值指针
 *
 * @return 4
 */
int modbus_cvt_f32_get(const uint8_t *buf, float *pval)
{
    return(modbus_cvt_u32_get(buf, (uint32_t *)pval));
}


/**
 * @brief  从位图中读取指定位的值
 *
 * @param[in] pbits  位图数组首地址
 * @param[in] idx    位索引（从 0 开始）
 *
 * @return uint8_t  0 或 1
 *
 * @note 低位先传，符合 Modbus 功能码 01/02/0F/15
 */
uint8_t modbus_bitmap_get(const uint8_t *pbits, int idx)//从位表中读指定索引的位
{
    return(((pbits[idx/8] & (1 << (idx % 8))) != 0) ? 1 : 0);
}


/**
 * @brief  向位图中写入指定位的值
 *
 * @param[in,out] pbits  位图数组首地址
 * @param[in]     idx    位索引
 * @param[in]     bit    要写入的值（0 或 1）
 */
void modbus_bitmap_set(uint8_t *pbits, int idx, uint8_t bit)//向位表中写指定索引的位
{
    if (bit)
    {
        pbits[idx/8] |= (1 << (idx % 8));
    }
    else
    {
        pbits[idx/8] &= ~(1 << (idx % 8));
    }
}













