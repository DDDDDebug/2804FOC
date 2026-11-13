/**
 * @file    angle_as5600.h
 * @brief   AS5600 磁编码器实现
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * AS5600：12 位 I2C 磁编码器，地址 0x6C，角度寄存器 0x0C。
 * 实现 angle_sensor.h 接口，封装 I2C 读取和角度换算。
 */

#ifndef __ANGLE_AS5600_H__
#define __ANGLE_AS5600_H__

#include "i2c.h"
#include "angle_sensor.h"
#include <stdint.h>

/**
 * @brief  创建 AS5600 传感器实例（填充 AngleSensor_t 接口）
 * @param  hi2c : I2C 句柄
 * @return 已填充 AngleSensor_t 结构体
 */
AngleSensor_t AS5600_Create(I2C_HandleTypeDef *hi2c);

/**
 * @brief  在非中断上下文刷新角度缓存。
 * @note   仅应从主循环调用；FOC 定时中断通过 AS5600_ReadAngle() 读取缓存。
 */
void AS5600_Poll(I2C_HandleTypeDef *hi2c);

/* Live diagnostics, intended for SWD inspection during bench bring-up. */
extern volatile uint32_t g_as5600_poll_ok;
extern volatile uint32_t g_as5600_poll_error;
extern volatile uint32_t g_as5600_last_error;
extern volatile uint16_t g_as5600_last_raw;
extern volatile uint32_t g_as5600_last_success_ms;

/**
 * @brief  Returns 1 only after repeated, recent successful I2C samples.
 * @note   Safe to call from the 1 kHz control ISR; all inspected values are
 *         naturally atomic on Cortex-M3.
 */
uint8_t AS5600_IsHealthy(uint32_t min_success_samples,
                         uint32_t max_sample_age_ms);

/**
 * @brief  读取原始角度值（0 ~ 4095，12 位）
 */
uint16_t AS5600_ReadRaw(I2C_HandleTypeDef *hi2c);

/**
 * @brief  读取已缓存的机械角度（0 ~ 360°），作为 read_angle 函数指针的实现
 */
float AS5600_ReadAngle(void *ctx);

#endif
