/**
 * @file    motor_config.h
 * @brief   电机硬件参数统一配置
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 集中管理极对数、母线电压、控制频率等硬件参数。
 * 所有模块从此处读取唯一数据源，消除重复配置。
 */

#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include <stdint.h>

/*
 * Power-stage permission. The current torque-test build enables this only
 * together with the reset-latched SmartKnob one-shot below.
 */
#define MOTOR_POWER_STAGE_ENABLED   1U

/*
 * One-shot hardware validation.
 *
 * This runs the required full 180-degree/3-second trajectory once: align,
 * move forward, return, then disable the driver.  Set MOTOR_BENCH_TEST_ONCE
 * to 0 only after this one-shot test passes.
 */
#define MOTOR_AUTORUN_DEMO          0U

/*
 * S1 safety gate: accept only STANDBY and one explicit CALIBRATE command.
 * SPEED, POSITION and TORQUE commands are rejected before they can enable
 * the bridge. Keep this enabled until S1 waveform and alignment data pass.
 */
#define MOTOR_S1_ALIGNMENT_ONLY     1U

/*
 * Reset-triggered S1-D capture aid. The first boot after flashing only arms
 * the backup-register latch; the following manual Reset may run one bounded
 * health-gated alignment attempt.
 */
#define MOTOR_S1_RESET_ONE_SHOT      0U
#define MOTOR_S1_RESET_DELAY_MS   1000U
#define MOTOR_S1_MAX_POWERED_MS   2800U
#define MOTOR_BENCH_TEST_ONCE        0U

/*
 * S2 direction/phase observation.
 *
 * The first boot after programming only arms this test. The next manual
 * reset waits for the I2C-only baseline, aligns at low voltage, sweeps a
 * fixed open-loop electrical vector forward and back once, then disables
 * PB1 and PWM. It does not use position, speed, torque, or PID control.
 */
#define MOTOR_S2_DIRECTION_TEST       0U
#define MOTOR_S2_RESET_ONE_SHOT       0U
#define MOTOR_S2_RESET_DELAY_MS    1000U
#define MOTOR_S2_MAX_POWERED_MS    4600U
#define MOTOR_S2_ALIGN_RAMP_UP_MS   900U
#define MOTOR_S2_ALIGN_HOLD_MS      300U
#define MOTOR_S2_SWEEP_FORWARD_MS  1000U
#define MOTOR_S2_SWEEP_HOLD_MS      200U
#define MOTOR_S2_SWEEP_RETURN_MS   1000U
#define MOTOR_S2_RAMP_DOWN_MS       700U
#define MOTOR_S2_VOLTAGE_V         0.35f
#define MOTOR_S2_ELEC_SWEEP_DEG    90.0f

/*
 * S2 offset capture and commit. This repeats only the previously proven
 * low-voltage alignment, captures the encoder value while the rotor is held,
 * ramps down, then writes a versioned calibration record with PB1 disabled.
 */
#define MOTOR_S2_OFFSET_COMMIT_ONE_SHOT  0U
#define MOTOR_S2_OFFSET_RESET_DELAY_MS 1000U
#define MOTOR_S2_OFFSET_MAX_POWERED_MS 3500U

/*
 * S3A: one-shot static hold at the filtered mechanical angle captured just
 * before enable.  This path is deliberately separate from the S1/S2
 * calibration flows and never uses the general position PID.
 */
#define MOTOR_S3_HOLD_ONE_SHOT          0U
#define MOTOR_S3_HOLD_RESET_DELAY_MS  1000U
#define MOTOR_S3_HOLD_MAX_POWERED_MS  1800U
#define MOTOR_S3_HOLD_MAX_VOLTAGE_V    0.20f
#define MOTOR_S3_HOLD_KP_V_PER_DEG     0.010f
#define MOTOR_S3_HOLD_DEADBAND_DEG     0.20f
#define MOTOR_S3_HOLD_SLEW_V_PER_S     0.50f

/* S3B: one tiny position step after a short static baseline. */
#define MOTOR_S3B_STEP_ONE_SHOT         0U
#define MOTOR_S3B_STEP_DEG             2.0f
#define MOTOR_S3B_PRESTEP_HOLD_MS       300U
#define MOTOR_S3B_RESET_DELAY_MS       1000U
#define MOTOR_S3B_MAX_POWERED_MS       1800U

/* One-shot low-voltage SmartKnob hand-torque test. The reset latch below
 * prevents the programming reset from energizing the bridge. */
#define MOTOR_SMARTKNOB_ENABLED         1U
#define MOTOR_SMARTKNOB_RESET_DELAY_MS  1000U
#define MOTOR_SMARTKNOB_MAX_POWERED_MS  30000U
#define MOTOR_SMARTKNOB_SPACING_DEG     30.0f
#define MOTOR_SMARTKNOB_STRENGTH_V_PER_DEG 0.120f
#define MOTOR_SMARTKNOB_DAMPING_V_PER_RPM  0.0035f
#define MOTOR_SMARTKNOB_DETENT_DAMPING_V_PER_RPM 0.0f
#define MOTOR_SMARTKNOB_DETENT_DAMPING_START_RPM 15.0f
#define MOTOR_SMARTKNOB_MAX_DAMPING_V     0.24f
#define MOTOR_SMARTKNOB_SPRING_NONLINEAR_GAIN 0.0f
#define MOTOR_SMARTKNOB_START_RAMP_MS      250U
#define MOTOR_SMARTKNOB_OSC_WINDOW_MS     350U
#define MOTOR_SMARTKNOB_OSC_MIN_SWING_DEG 1.5f
#define MOTOR_SMARTKNOB_OSC_CAPTURE_MAX_UQ_V 0.45f
#define MOTOR_SMARTKNOB_OSC_CAPTURE_RAMP_MS 200U
#define MOTOR_SMARTKNOB_OSC_CAPTURE_RELEASE_DEG 9.0f
#define MOTOR_SMARTKNOB_MAX_SPEED_RPM   60.0f
#define MOTOR_SMARTKNOB_MAX_UQ_V        0.80f

/* Shared reset-latch interface used by the bounded S1/S2 test paths. */
#define MOTOR_OPEN_LOOP_TEST_ACTIVE \
    (MOTOR_S1_RESET_ONE_SHOT || MOTOR_S2_RESET_ONE_SHOT \
     || MOTOR_S2_OFFSET_COMMIT_ONE_SHOT || MOTOR_S3_HOLD_ONE_SHOT \
     || MOTOR_S3B_STEP_ONE_SHOT || MOTOR_SMARTKNOB_ENABLED)

#if MOTOR_SMARTKNOB_ENABLED
#define MOTOR_TEST_RESET_DELAY_MS  MOTOR_SMARTKNOB_RESET_DELAY_MS
#define MOTOR_TEST_MAX_POWERED_MS  MOTOR_SMARTKNOB_MAX_POWERED_MS
#define MOTOR_TEST_CALIB_FLAG      0U
#define MOTOR_TEST_COMMIT_OFFSET   0U
#elif MOTOR_S3B_STEP_ONE_SHOT
#define MOTOR_TEST_RESET_DELAY_MS  MOTOR_S3B_RESET_DELAY_MS
#define MOTOR_TEST_MAX_POWERED_MS  MOTOR_S3B_MAX_POWERED_MS
#define MOTOR_TEST_CALIB_FLAG      0U
#define MOTOR_TEST_COMMIT_OFFSET   0U
#elif MOTOR_S3_HOLD_ONE_SHOT
#define MOTOR_TEST_RESET_DELAY_MS  MOTOR_S3_HOLD_RESET_DELAY_MS
#define MOTOR_TEST_MAX_POWERED_MS  MOTOR_S3_HOLD_MAX_POWERED_MS
#define MOTOR_TEST_CALIB_FLAG      0U
#define MOTOR_TEST_COMMIT_OFFSET   0U
#elif MOTOR_S2_RESET_ONE_SHOT
#define MOTOR_TEST_RESET_DELAY_MS  MOTOR_S2_RESET_DELAY_MS
#define MOTOR_TEST_MAX_POWERED_MS  MOTOR_S2_MAX_POWERED_MS
#define MOTOR_TEST_CALIB_FLAG      2U
#define MOTOR_TEST_COMMIT_OFFSET    0U
#elif MOTOR_S2_OFFSET_COMMIT_ONE_SHOT
#define MOTOR_TEST_RESET_DELAY_MS  MOTOR_S2_OFFSET_RESET_DELAY_MS
#define MOTOR_TEST_MAX_POWERED_MS  MOTOR_S2_OFFSET_MAX_POWERED_MS
#define MOTOR_TEST_CALIB_FLAG      1U
#define MOTOR_TEST_COMMIT_OFFSET    1U
#else
#define MOTOR_TEST_RESET_DELAY_MS  MOTOR_S1_RESET_DELAY_MS
#define MOTOR_TEST_MAX_POWERED_MS  MOTOR_S1_MAX_POWERED_MS
#define MOTOR_TEST_CALIB_FLAG      1U
#define MOTOR_TEST_COMMIT_OFFSET    0U
#endif

/*
 * S1 uses the logic analyzer as its only observation path. Keep unconnected
 * CAN/UART peripherals silent so their interrupts cannot perturb alignment.
 */
#define MOTOR_COMMUNICATION_ENABLED   0U

/*
 * D1 is a logic-analyzer diagnostic, not a motor-control stage.
 *
 * It uses a two-reset latch: the first boot after programming only arms the
 * next reset. The following manual Reset emits a fixed sequence of PWM-only
 * and PB1-only windows, then returns to PB1-low/PWM-stopped. It never calls
 * FOC_Enable(), calibration, PID, position, speed, or torque control.
 */
#define MOTOR_DIAGNOSTIC_D1          0U
#define MOTOR_DIAG_BASELINE_MS    1000U
#define MOTOR_DIAG_PWM_WINDOW_MS   600U
#define MOTOR_DIAG_QUIET_MS        300U
#define MOTOR_DIAG_PB1_ONLY_MS      80U

/*
 * D2 isolates the PB1/PWM coexistence observed in S1-B. It never invokes
 * FOC, PID, alignment, speed, position or torque control. Its combined
 * window drives all three PWM channels synchronously at 50%, so it has no
 * commanded rotating voltage vector.
 */
#define MOTOR_DIAGNOSTIC_D2          0U
#define MOTOR_D2_BASELINE_MS      1000U
#define MOTOR_D2_PB1_ONLY_MS       100U
#define MOTOR_D2_QUIET_MS           300U
#define MOTOR_D2_COMBINED_MS        600U
#define DEMO_TRAVEL_DEG             180.0f
#define DEMO_TRAVEL_TIME_MS         3000U
#define DEMO_HOLD_TIME_MS           1000U
/*
 * Use a short smoothstep acceleration/deceleration envelope.  Its effective
 * constant-speed area is (travel_time - ramp_time), so 10.91 rpm produces
 * 180 degrees in 3 seconds with 250 ms at each end for a smooth transition.
 */
#define DEMO_TRAJECTORY_PEAK_RPM    10.9091f
#define DEMO_TRAJECTORY_RAMP_MS      250U
#define DEMO_FINAL_SETTLE_TIME_MS   1500U

/*
 * V2 calibration is deliberately conservative because this project has no
 * verified current feedback. The vector starts at 0 V and ramps to this
 * limit only after PWM preload and a separate PB1 enable delay have passed.
 */
#define MOTOR_PWM_PRELOAD_TIME_MS       10U
#define MOTOR_ALIGNMENT_RAMP_UP_MS    1200U
#define MOTOR_ALIGNMENT_HOLD_MS        500U
#define MOTOR_ALIGNMENT_RAMP_DOWN_MS   750U
#define MOTOR_ALIGNMENT_VOLTAGE_V    0.40f
#define DEMO_MAX_UQ_V               0.6f

/* A power-stage request requires repeated, recent successful I2C samples. */
#define AS5600_POLL_PERIOD_MS         5U
#define AS5600_HEALTH_MIN_SAMPLES    10U
#define AS5600_MAX_SAMPLE_AGE_MS     25U
/*
 * AS5600 mounting direction relative to the current U/V/W phase sequence.
 * The first powered test locked but could not follow a target, which is the
 * expected symptom of a reversed encoder direction.  This software reversal
 * is electrically equivalent to swapping any two motor phases.
 */
#define MOTOR_SENSOR_DIRECTION      (-1)
#define DEMO_POSITION_KP            0.6f
#define DEMO_POSITION_KD            0.02f
#define DEMO_POSITION_SPEED_RPM     30.0f
#define DEMO_SPEED_KP               0.03f
#define DEMO_SPEED_KI               0.12f

/**
 * @brief 速度限幅 (rpm)
 *
 * target_speed 被钳制在 ±此值内。
 * 龙伯格观测器的 x2_max 独立设为此值 x5（10000 rpm），
 * 确保观测器自身不限幅控制目标。
 */
#define SPEED_LIMIT_RPM     2000.0f

/**
 * @brief 速度模式软启动斜率 (rpm/s)
 *
 * 限制 target_speed 每秒最大变化量，防止指令跳变导致电流冲击。
 * 每 1ms 控制周期步进 = SPEED_RAMP_RPM_S / 1000。
 *
 * 例如：从 0→2000 rpm 需要 2000/500 = 4 秒。
 * 设为 0 可禁用软启动（target_speed 立即跟随指令）。
 */
#define SPEED_RAMP_RPM_S    500.0f

/**
 * @brief 电角度前馈补偿延迟时间 (s)
 *
 * 补偿从 AS5600 角度采样到 PWM 输出实际生效之间的总延迟。
 * 包括: I2C 读取 + 观测器计算 + PWM 影子寄存器更新。
 *
 * 在 FOC_Output 的反 Park 变换前，根据当前转速推算此期间转子转过的
 * 电角度，提前补偿，使电压矢量始终对准真实转子位置。
 *
 * 公式: angle_advance_deg = speed_rpm × pole_pairs × 6 × 此值
 * 1600 RPM, 7 极对: advance = 1600 × 7 × 6 × 0.00021 = 14.1°
 * 2000 RPM, 7 极对: advance = 2000 × 7 × 6 × 0.00021 = 17.6°
 *
 * 调节方法:
 *   太小 (<0.00015): 补偿不足, 高速时 Id 偏负, 速度上不去
 *   太大 (>0.00035): 过补偿, 高速时 Id 偏正, 可能振荡
 *   推荐从 0.00021 开始, 用 VOFA+ 观察 Id 随速度变化的趋势
 */
#define ANGLE_COMP_DELAY_S  0.00021f

/**
 * @brief 电压调制系数 —— 决定线性调制区最大相电压
 *
 * 实际限制值为 bus_voltage × 此系数。
 * 当 UqUd 矢量模长超过此值时做等比例缩小，避免进入过调制区。
 *
 * 选值：
 *   0.5f      = SPWM  线性调制上限 (Udc/2)
 *   0.57735f  = SVPWM 线性调制上限 (Udc/√3)
 *
 * 当前使用 SVPWM，故设为 0.57735f。
 * 若将来切回 SPWM，改成 0.5f 即可。
 */
#define VOLTAGE_MODULATION_FACTOR  0.577350269f

/**
 * @brief 电机硬件参数
 *
 * 在 main.c 中初始化后传递给 FOC_Init，PWM 驱动层不再关心母线电压。
 */
typedef struct
{
    uint8_t  pole_pairs;      /**< 极对数                      */
    float    bus_voltage;     /**< 母线电压 (V)                */
    float    rate_hz;         /**< 控制频率 (Hz)               */

} MotorConfig_t;

#endif
