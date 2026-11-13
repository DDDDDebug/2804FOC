/**
 * @file    angle_as5600.c
 * @brief   AS5600 磁编码器实现
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * AS5600：12 位 I2C 磁编码器
 *   - I2C 地址：0x6C
 *   - 角度寄存器：0x0C
 *   - 接口：实现 angle_sensor.h 的 read_angle 函数指针
 */

#include "angle_as5600.h"
#include "i2c.h"

#define AS5600_ADDR      0x6C
#define AS5600_ANGLE_REG 0x0C
#define AS5600_POLL_TIMEOUT_MS  5U

/*
 * FOC runs in a TIM1 ISR and must never block on I2C.  The main loop owns
 * the bus transaction and publishes the last valid result here.  A 32-bit
 * aligned float load/store is atomic on Cortex-M3.
 */
static volatile float s_last_angle = 0.0f;
volatile uint32_t g_as5600_poll_ok = 0U;
volatile uint32_t g_as5600_poll_error = 0U;
volatile uint32_t g_as5600_last_error = 0U;
volatile uint16_t g_as5600_last_raw = 0U;
volatile uint32_t g_as5600_last_success_ms = 0U;

/**
 * @brief  读取原始 ADC 值（0 ~ 4095）
 */
uint16_t AS5600_ReadRaw(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2] = {0};
    HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, AS5600_ANGLE_REG,
                     I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/**
 * @brief  读取机械角度（0 ~ 360°），作为 read_angle 函数指针的实现
 *
 *  I2C 通信失败时保持上次有效角度，防止角度跳变导致 PID 飞车。
 */
void AS5600_Poll(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[2] = {0};
    uint16_t raw;

    /*
     * This runs only in the main loop, where SysTick can advance HAL timeouts.
     * A failed transaction leaves the prior sample intact so a temporary bus
     * error cannot inject an angle jump into the position controller.
     */
    if (HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, AS5600_ANGLE_REG,
                         I2C_MEMADD_SIZE_8BIT, buf, 2,
                         AS5600_POLL_TIMEOUT_MS) != HAL_OK)
    {
        g_as5600_poll_error++;
        g_as5600_last_error = hi2c->ErrorCode;

        /* A disconnected device or a line held low can leave STM32F1 I2C1
           with START/BUSY latched.  This is main-loop code, so recovery is
           safe here and avoids requiring a board reset after a wire is fixed. */
        (void)HAL_I2C_DeInit(hi2c);
        (void)HAL_I2C_Init(hi2c);
        return;
    }

    raw = ((uint16_t)buf[0] << 8) | buf[1];
    g_as5600_poll_ok++;
    g_as5600_last_raw = raw;
    s_last_angle = (float)raw * (360.0f / 4096.0f);
    g_as5600_last_success_ms = HAL_GetTick();
}

uint8_t AS5600_IsHealthy(uint32_t min_success_samples,
                         uint32_t max_sample_age_ms)
{
    uint32_t last_success_ms = g_as5600_last_success_ms;

    if ((g_as5600_poll_ok < min_success_samples) || (last_success_ms == 0U))
    {
        return 0U;
    }

    return ((uint32_t)(HAL_GetTick() - last_success_ms) <= max_sample_age_ms)
        ? 1U
        : 0U;
}

float AS5600_ReadAngle(void *ctx)
{
    (void)ctx;
    return s_last_angle;
}

/**
 * @brief  创建 AS5600 传感器实例
 *
 * 用法：
 *   AngleSensor_t sensor = AS5600_Create(&hi2c1);
 *   float angle = sensor.read_angle(sensor.ctx);
 */
AngleSensor_t AS5600_Create(I2C_HandleTypeDef *hi2c)
{
    AngleSensor_t sensor;
    sensor.read_angle = AS5600_ReadAngle;
    sensor.ctx        = (void *)hi2c;
    return sensor;
}
