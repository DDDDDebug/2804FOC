/**
 * @file    motor_ctrl.c
 * @brief   电机控制任务调度 —— 抽象指令分发 + FOC 流水线
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 由 TIM1 周期中断回调触发，运行频率 1 kHz。
 *
 * 与通信介质解耦：通过 motor_msg.h 统一接口收发指令/状态，
 * 不再直接依赖 can_proto 或 uart_proto。
 *
 * 每轮控制流程：
 *   1. 轮询消息总线获取最新指令 → 解析运行模式与目标值
 *   2. 处理模式切换（待机 ↔ 运行，使能 / 关断驱动芯片）
 *   3. 按当前模式执行对应控制策略
 *   4. 运行 FOC 流水线（传感器 → 控制环 → SVPWM → 输出）
 *   5. 更新状态 → 触发所有通道回传（1 kHz）
 */

#include "motor_ctrl.h"
#include "Foc.h"
#include "motor_msg.h"
#include "data_process.h"
#include "drv_tim.h"
#include "led_indicator.h"
#include "main.h"
#include "angle_as5600.h"
#include "smartknob_ctrl.h"

/* SPEED_LIMIT_RPM 定义于 motor_config.h —— 所有模块共享 */

static uint8_t Motor_SensorReady(void)
{
    return AS5600_IsHealthy(AS5600_HEALTH_MIN_SAMPLES,
                            AS5600_MAX_SAMPLE_AGE_MS);
}

static uint8_t g_reset_one_shot_armed = 0U;

void Motor_SetResetOneShotArmed(uint8_t armed)
{
    g_reset_one_shot_armed = (armed != 0U) ? 1U : 0U;
}

#if MOTOR_AUTORUN_DEMO
typedef enum {
    DEMO_BOOT = 0,
    DEMO_ALIGN,
    DEMO_FORWARD,
    DEMO_HOLD,
    DEMO_REVERSE,
    DEMO_FINAL_SETTLE,
    DEMO_COMPLETE
} DemoState_t;

/*
 * Built-in 2804 demonstration.
 *
 * The target advances in 1 ms increments instead of jumping by 180 degrees.
 * That makes the requested direction unambiguous at the 0/360-degree wrap and
 * produces a constant 60 degree/s command. In virtual mode the exact same
 * state machine runs, but the PWM/GPIO gates keep the bridge disabled.
 */
static void Motor_RunDemo(void)
{
    static DemoState_t state = DEMO_BOOT;
    static uint32_t state_ms = 0U;
    static float target_deg = 0.0f;
    static int8_t travel_direction = 1;
    const float step_deg = DEMO_TRAVEL_DEG / (float)DEMO_TRAVEL_TIME_MS;

    /* The travel profile is velocity controlled.  Do not carry integral
       torque from one segment into the next direction or endpoint hold. */
    #define RESET_TRAJECTORY_SPEED_LOOP() do { \
        Motor.SpdPID.integral = 0.0f;            \
        Motor.SpdPID.output = 0.0f;              \
        Motor.SpdPID.error = 0.0f;               \
        Motor.SpdPID.differential = 0.0f;        \
    } while (0)

    switch (state)
    {
    case DEMO_BOOT:
        target_deg = Motor.mech_angle;
        FOC_SetTarget(&Motor, target_deg);
        FOC_Enable(&Motor);
        RESET_TRAJECTORY_SPEED_LOOP();
        LED_SetPattern(LED_PATTERN_FAST_BLINK);
        state_ms = 0U;
        travel_direction = 1;
#if MOTOR_POWER_STAGE_ENABLED
        Motor.calib_flag = 1U;
        Motor.calib_count = 0U;
        state = DEMO_ALIGN;
#else
        state = DEMO_FORWARD;
#endif
        break;

    case DEMO_ALIGN:
        /* V2 never promotes an automatic alignment into motion or a Flash
           write. The candidate remains in RAM for explicit later validation. */
        if (Motor.calib_flag == 0U)
        {
            FOC_Disable(&Motor);
            LED_SetPattern(LED_PATTERN_HEARTBEAT);
            state = DEMO_COMPLETE;
        }
        break;

    case DEMO_FORWARD:
    {
        float normalized;
        float trajectory_rpm;

        if (state_ms < DEMO_TRAJECTORY_RAMP_MS)
        {
            normalized = (float)state_ms / (float)DEMO_TRAJECTORY_RAMP_MS;
        }
        else if (state_ms >= (DEMO_TRAVEL_TIME_MS - DEMO_TRAJECTORY_RAMP_MS))
        {
            normalized = (float)(DEMO_TRAVEL_TIME_MS - state_ms)
                       / (float)DEMO_TRAJECTORY_RAMP_MS;
        }
        else
        {
            normalized = 1.0f;
        }
        /* Smoothstep: zero acceleration discontinuity at both endpoints. */
        trajectory_rpm = DEMO_TRAJECTORY_PEAK_RPM
                       * normalized * normalized * (3.0f - 2.0f * normalized);

        target_deg += step_deg;
        FOC_SetTarget(&Motor, target_deg);
        /* Keep the travel phase velocity-driven.  Position correction is
           intentionally deferred to the hold state so it cannot slow the
           motor before the commanded 3-second trajectory finishes. */
        Motor.target_speed = (float)travel_direction * trajectory_rpm;
        FOC_SpdCtrl(&Motor);
        if (++state_ms >= DEMO_TRAVEL_TIME_MS)
        {
            RESET_TRAJECTORY_SPEED_LOOP();
            state_ms = 0U;
            travel_direction = -1;
            state = DEMO_HOLD;
        }
        break;
    }

    case DEMO_HOLD:
        FOC_PosCtrl(&Motor);
        FOC_SpdCtrl(&Motor);
        if (++state_ms >= DEMO_HOLD_TIME_MS)
        {
            RESET_TRAJECTORY_SPEED_LOOP();
            state_ms = 0U;
            state = (travel_direction > 0) ? DEMO_FORWARD : DEMO_REVERSE;
        }
        break;

    case DEMO_REVERSE:
    {
        float normalized;
        float trajectory_rpm;

        if (state_ms < DEMO_TRAJECTORY_RAMP_MS)
        {
            normalized = (float)state_ms / (float)DEMO_TRAJECTORY_RAMP_MS;
        }
        else if (state_ms >= (DEMO_TRAVEL_TIME_MS - DEMO_TRAJECTORY_RAMP_MS))
        {
            normalized = (float)(DEMO_TRAVEL_TIME_MS - state_ms)
                       / (float)DEMO_TRAJECTORY_RAMP_MS;
        }
        else
        {
            normalized = 1.0f;
        }
        trajectory_rpm = DEMO_TRAJECTORY_PEAK_RPM
                       * normalized * normalized * (3.0f - 2.0f * normalized);

        target_deg -= step_deg;
        FOC_SetTarget(&Motor, target_deg);
        Motor.target_speed = (float)travel_direction * trajectory_rpm;
        FOC_SpdCtrl(&Motor);
        if (++state_ms >= DEMO_TRAVEL_TIME_MS)
        {
            RESET_TRAJECTORY_SPEED_LOOP();
            state_ms = 0U;
#if MOTOR_BENCH_TEST_ONCE
            state = DEMO_FINAL_SETTLE;
#else
            travel_direction = 1;
            state = DEMO_HOLD;
#endif
        }
        break;
    }

    case DEMO_FINAL_SETTLE:
        /* The 3-second reverse trajectory is complete.  Briefly hold the
           original target with no velocity feedforward to remove residual
           lag before disabling the bridge. */
        FOC_SetTarget(&Motor, target_deg);
        FOC_PosCtrl(&Motor);
        FOC_SpdCtrl(&Motor);
        {
            float settle_error = ANGLE_limit(target_deg - Motor.mech_angle,
                                             360.0f, -180.0f, 180.0f);
            if (settle_error < 0.0f)
            {
                settle_error = -settle_error;
            }

            if ((state_ms >= 250U && settle_error <= 1.5f)
                || ++state_ms >= DEMO_FINAL_SETTLE_TIME_MS)
            {
                FOC_Disable(&Motor);
                LED_SetPattern(LED_PATTERN_HEARTBEAT);
                state = DEMO_COMPLETE;
            }
        }
        break;

    case DEMO_COMPLETE:
        /* One-shot bench test is over: keep PB1 low and all PWM stopped. */
        FOC_Disable(&Motor);
        break;

    default:
        FOC_Disable(&Motor);
        state = DEMO_BOOT;
        break;
    }

    #undef RESET_TRAJECTORY_SPEED_LOOP
}
#endif

void Motor_Loop(void)
{
    static MotorMode_t   g_mode     = MOTOR_MODE_STANDBY;    /**< 当前运行模式 */
    static uint16_t      g_comms_timeout = 0;                /**< 通信超时计数器 */
    static float         g_speed_desired = 0.0f;             /**< 速度指令目标（rpm），target_speed 斜坡逼近 */
#if MOTOR_OPEN_LOOP_TEST_ACTIVE
    static uint32_t      test_loop_ready_ms = 0U;
    static uint32_t      test_powered_start_ms = 0U;
    static uint8_t       test_start_attempted = 0U;
    static uint8_t       test_powered_window_active = 0U;
#if MOTOR_S3_HOLD_ONE_SHOT
    static uint8_t       s3_hold_active = 0U;
    static float         s3_hold_target_angle = 0.0f;
#endif
#if MOTOR_S3B_STEP_ONE_SHOT
    static uint8_t       s3b_active = 0U;
    static float         s3b_base_angle = 0.0f;
#endif
#if MOTOR_SMARTKNOB_ENABLED
    static uint8_t       smartknob_active = 0U;
    static SmartKnob_t   smartknob;
#endif
#endif

    MotorCommand_t       cmd;
    uint8_t              new_cmd    = MotorMsg_PollCommand(&cmd);

    /* 喂独立看门狗 —— 如果此处卡死，IWDG ~2s 后自动复位 MCU */
    IWDG_REFRESH();      /* 喂独立看门狗 —— IWDG 寄存器直接操作 */

#if MOTOR_OPEN_LOOP_TEST_ACTIVE
    /*
     * Dedicated bounded test path: give the logic analyzer one second of
     * PB1-low/I2C-only baseline, then make at most one health-gated attempt.
     * A failed health check is a terminal safe standby for this reset cycle;
     * it must never become an unexpected delayed motor start.
     */
    if (test_loop_ready_ms == 0U)
    {
        test_loop_ready_ms = HAL_GetTick();
    }
    else if (g_reset_one_shot_armed != 0U
             && test_start_attempted == 0U
             && (uint32_t)(HAL_GetTick() - test_loop_ready_ms)
                    >= MOTOR_TEST_RESET_DELAY_MS)
    {
        test_start_attempted = 1U;
        if (Motor_SensorReady()
#if MOTOR_S3_HOLD_ONE_SHOT || MOTOR_S3B_STEP_ONE_SHOT || MOTOR_SMARTKNOB_ENABLED
            && FOC_IsCalibrationValid(&Motor)
#endif
           )
        {
#if MOTOR_SMARTKNOB_ENABLED
            SmartKnobConfig_t smartknob_config = {
                MOTOR_SMARTKNOB_SPACING_DEG,
                MOTOR_SMARTKNOB_STRENGTH_V_PER_DEG,
                MOTOR_SMARTKNOB_DAMPING_V_PER_RPM,
                MOTOR_SMARTKNOB_DETENT_DAMPING_V_PER_RPM,
                MOTOR_SMARTKNOB_DETENT_DAMPING_START_RPM,
                MOTOR_SMARTKNOB_MAX_DAMPING_V,
                MOTOR_SMARTKNOB_SPRING_NONLINEAR_GAIN,
                (float)MOTOR_SMARTKNOB_START_RAMP_MS / 1000.0f,
                (float)MOTOR_SMARTKNOB_OSC_WINDOW_MS / 1000.0f,
                MOTOR_SMARTKNOB_OSC_MIN_SWING_DEG,
                MOTOR_SMARTKNOB_OSC_CAPTURE_MAX_UQ_V,
                (float)MOTOR_SMARTKNOB_OSC_CAPTURE_RAMP_MS / 1000.0f,
                MOTOR_SMARTKNOB_OSC_CAPTURE_RELEASE_DEG,
                MOTOR_SMARTKNOB_MAX_UQ_V,
                MOTOR_SMARTKNOB_MAX_SPEED_RPM
            };
            SmartKnob_Init(&smartknob);
            SmartKnob_SetConfig(&smartknob, &smartknob_config);
            SmartKnob_SetEnabled(&smartknob, 1U);
#elif MOTOR_S3B_STEP_ONE_SHOT
            s3b_base_angle = Motor.mech_angle;
            FOC_S3HoldReset(&Motor);
#elif MOTOR_S3_HOLD_ONE_SHOT
            /* Lock the filtered present angle before enabling the bridge. */
            s3_hold_target_angle = Motor.mech_angle;
            FOC_S3HoldReset(&Motor);
#endif
            FOC_Enable(&Motor);
#if MOTOR_SMARTKNOB_ENABLED
            smartknob_active = 1U;
#elif MOTOR_S3B_STEP_ONE_SHOT
            s3b_active = 1U;
#elif MOTOR_S3_HOLD_ONE_SHOT
            s3_hold_active = 1U;
#else
            Motor.calib_flag = MOTOR_TEST_CALIB_FLAG;
            Motor.calib_count = 0U;
            Motor.calib_start_ms = 0U;
            Motor.calib_candidate_captured = 0U;
            Motor.calib_result_ready = 0U;
#endif
            test_powered_start_ms = HAL_GetTick();
            test_powered_window_active = 1U;
#if MOTOR_S3_HOLD_ONE_SHOT || MOTOR_S3B_STEP_ONE_SHOT || MOTOR_SMARTKNOB_ENABLED
            /* S3A uses its dedicated low-voltage controller below. */
#else
            g_mode = MOTOR_MODE_CALIBRATE;
#endif
            LED_SetPattern(LED_PATTERN_FAST_BLINK);
        }
    }
#endif

#if MOTOR_SMARTKNOB_ENABLED
    if (smartknob_active != 0U)
    {
        float dt_s = (Motor.rate > 0.0f) ? (1.0f / Motor.rate) : 0.001f;

        FOC_UpdateSensor(&Motor);
        FOC_ReadCurrents(&Motor);

        if (!Motor_SensorReady()
            || (uint32_t)(HAL_GetTick() - test_powered_start_ms)
                   >= MOTOR_SMARTKNOB_MAX_POWERED_MS)
        {
            SmartKnob_SetEnabled(&smartknob, 0U);
            FOC_Disable(&Motor);
            smartknob_active = 0U;
            test_powered_window_active = 0U;
            LED_SetPattern(LED_PATTERN_HEARTBEAT);
        }
        else
        {
            Motor.Ud = 0.0f;
            Motor.Uq = SmartKnob_Update(&smartknob,
                                        Motor.mech_angle,
                                        Motor.speed,
                                        dt_s);
            FOC_Output(&Motor);
        }

        MotorMsg_UpdateStatus(Motor.mech_angle * DEG_TO_RAD,
                              Motor.speed * RPM_TO_RAD_S,
                              Motor.Uq);
#if MOTOR_COMMUNICATION_ENABLED
        MotorMsg_TxStatus();
#endif
        return;
    }
#elif MOTOR_S3B_STEP_ONE_SHOT
    if (s3b_active != 0U)
    {
        uint32_t elapsed_ms;
        float target_angle;

        FOC_UpdateSensor(&Motor);
        FOC_ReadCurrents(&Motor);
        elapsed_ms = (uint32_t)(HAL_GetTick() - test_powered_start_ms);

        if (!Motor_SensorReady() || elapsed_ms >= MOTOR_S3B_MAX_POWERED_MS)
        {
            FOC_Disable(&Motor);
            s3b_active = 0U;
            test_powered_window_active = 0U;
            LED_SetPattern(LED_PATTERN_HEARTBEAT);
        }
        else
        {
            target_angle = (elapsed_ms < MOTOR_S3B_PRESTEP_HOLD_MS)
                ? s3b_base_angle
                : ANGLE_limit(s3b_base_angle + MOTOR_S3B_STEP_DEG,
                              360.0f, 0.0f, 360.0f);
            FOC_S3HoldStep(&Motor, target_angle);
            FOC_Output(&Motor);
        }

        MotorMsg_UpdateStatus(Motor.mech_angle * DEG_TO_RAD,
                              Motor.speed * RPM_TO_RAD_S,
                              Motor.Uq);
#if MOTOR_COMMUNICATION_ENABLED
        MotorMsg_TxStatus();
#endif
        return;
    }
#elif MOTOR_S3_HOLD_ONE_SHOT
    if (s3_hold_active != 0U)
    {
        FOC_UpdateSensor(&Motor);
        FOC_ReadCurrents(&Motor);

        if (!Motor_SensorReady()
            || (uint32_t)(HAL_GetTick() - test_powered_start_ms)
                   >= MOTOR_S3_HOLD_MAX_POWERED_MS)
        {
            /* Any health failure or timeout is an immediate hard stop. */
            FOC_Disable(&Motor);
            s3_hold_active = 0U;
            test_powered_window_active = 0U;
            LED_SetPattern(LED_PATTERN_HEARTBEAT);
        }
        else
        {
            FOC_S3HoldStep(&Motor, s3_hold_target_angle);
            FOC_Output(&Motor);
        }

        MotorMsg_UpdateStatus(Motor.mech_angle * DEG_TO_RAD,
                              Motor.speed * RPM_TO_RAD_S,
                              Motor.Uq);
#if MOTOR_COMMUNICATION_ENABLED
        MotorMsg_TxStatus();
#endif
        return;
    }
#endif

#if MOTOR_AUTORUN_DEMO
    /* Autonomous virtual/bench trajectory. Communication commands are
       intentionally ignored while this demo build is selected. */
    FOC_UpdateSensor(&Motor);
    FOC_ReadCurrents(&Motor);
    Motor_RunDemo();
    FOC_Output(&Motor);
    MotorMsg_UpdateStatus(Motor.mech_angle * DEG_TO_RAD,
                          Motor.speed * RPM_TO_RAD_S,
                          Motor.Uq);
#if MOTOR_COMMUNICATION_ENABLED
    MotorMsg_TxStatus();
#endif
    return;
#endif

    /* ================================================================
     *  第 1 步：处理新指令 → 模式切换 + 参数更新
     *
     *  校准模式中忽略其他指令（避免异常切换破坏校准流程）。
     *  收到 STANDBY 可随时中止校准。
     * ================================================================ */
    if (new_cmd)
    {
        g_comms_timeout = 0;    /* 收到新指令 → 复位通信超时 */

#if MOTOR_S1_ALIGNMENT_ONLY
        /* S1 accepts only a deliberate one-shot calibration request or an
           immediate standby abort. No position, speed or torque command may
           energize the bridge during the alignment-only stage. */
        if (cmd.mode != MOTOR_MODE_STANDBY
            && cmd.mode != MOTOR_MODE_CALIBRATE)
        {
            cmd.mode = MOTOR_MODE_STANDBY;
        }
#endif

#if MOTOR_OPEN_LOOP_TEST_ACTIVE
        /* During the reset-triggered capture build, only STANDBY remains an
           accepted external action.  CALIBRATE must not bypass the 1-second
           baseline or create a second powered attempt. */
        if (cmd.mode != MOTOR_MODE_STANDBY)
        {
            cmd.mode = MOTOR_MODE_STANDBY;
        }
#endif

        /* ---- 校准中收到待机：中止校准 ---- */
        if (g_mode == MOTOR_MODE_CALIBRATE && cmd.mode == MOTOR_MODE_STANDBY)
        {
            FOC_Disable(&Motor);
            g_mode = MOTOR_MODE_STANDBY;
            LED_SetPattern(LED_PATTERN_HEARTBEAT);
        }
        /* ---- 校准中忽略其他指令 ---- */
        else if (g_mode == MOTOR_MODE_CALIBRATE)
        {
            /* 丢弃，保持校准状态 */
        }
        /* ---- 正常模式切换 ---- */
        else
        {
            /* 进入 / 退出待机 */
            if (cmd.mode == MOTOR_MODE_STANDBY && g_mode != MOTOR_MODE_STANDBY)
            {
                FOC_Disable(&Motor);
                g_speed_desired    = 0.0f;
                Motor.target_speed = 0.0f;
                LED_SetPattern(LED_PATTERN_HEARTBEAT);
            }
            else if (cmd.mode != MOTOR_MODE_STANDBY && g_mode == MOTOR_MODE_STANDBY)
            {
                if (!Motor_SensorReady())
                {
                    /* Recent encoder data is a prerequisite for PB1 enable. */
                    FOC_Disable(&Motor);
                    LED_SetPattern(LED_PATTERN_HEARTBEAT);
                    cmd.mode = MOTOR_MODE_STANDBY;
                }
                else
                {
                    /* 从待机恢复时，从当前实际转速起步（避免斜坡从旧值起跑） */
                    Motor.target_speed = Motor.speed;
                    FOC_Enable(&Motor);
                }
            }

            g_mode = cmd.mode;

            switch (g_mode)
            {
            case MOTOR_MODE_STANDBY:
                break;

            case MOTOR_MODE_SPEED:
                g_speed_desired  = cmd.target * RAD_S_TO_RPM;
                g_speed_desired  = DATA_limit(g_speed_desired,
                                              -SPEED_LIMIT_RPM, SPEED_LIMIT_RPM);
                FOC_SetSpdExtPID(&Motor, cmd.spd_kp, cmd.spd_ki, 0.0f, Motor.Umax);
                break;

            case MOTOR_MODE_POSITION:
                FOC_SetTarget(&Motor, cmd.target * RAD_TO_DEG);
                FOC_SetPosPID(&Motor, cmd.pos_kp, 0.0f, cmd.pos_kd, 2000.0f);
                break;

            case MOTOR_MODE_CALIBRATE:
#if MOTOR_POWER_STAGE_ENABLED
                /* V2 only creates an in-memory candidate after a low-voltage
                   ramp. It never writes Flash automatically. */
                Motor.calib_flag = 1U;
                Motor.calib_count = 0U;
                Motor.calib_start_ms = 0U;
                Motor.calib_candidate_captured = 0U;
                Motor.calib_result_ready = 0U;
                LED_SetPattern(LED_PATTERN_FAST_BLINK);
#else
                /* The safe default build does not simulate a successful
                   calibration when the bridge is compiled out. */
                FOC_Disable(&Motor);
                g_mode = MOTOR_MODE_STANDBY;
                LED_SetPattern(LED_PATTERN_HEARTBEAT);
#endif
                break;

            default:
                break;
            }
        }
    }

    /* ================================================================
     *  通信超时保护：超过 100ms 未收到指令 → 自动待机
     *
     *  （仅 SPEED / POSITION / TORQUE 模式生效，
     *   CALIBRATE 模式持续 ~3s 无指令，跳过超时检查）
     * ================================================================ */
    if (g_mode == MOTOR_MODE_SPEED || g_mode == MOTOR_MODE_POSITION
        || g_mode == MOTOR_MODE_TORQUE)
    {
        if (++g_comms_timeout >= 100)
        {
            FOC_Disable(&Motor);
            g_mode = MOTOR_MODE_STANDBY;
            g_speed_desired    = 0.0f;
            Motor.target_speed = 0.0f;
            LED_SetPattern(LED_PATTERN_HEARTBEAT);
        }
    }
    else
    {
        g_comms_timeout = 0;
    }

    /* ================================================================
     *  第 2 步：FOC 流水线（按模式执行不同控制环）
     * ================================================================ */
    FOC_UpdateSensor(&Motor);
    FOC_ReadCurrents(&Motor);       /* 读三相电流 → Clarke → Park → Id/Iq */

    if (g_mode != MOTOR_MODE_STANDBY && !Motor_SensorReady())
    {
        /* Loss of recent encoder samples while active is an immediate stop. */
        FOC_Disable(&Motor);
        g_mode = MOTOR_MODE_STANDBY;
        g_speed_desired = 0.0f;
        Motor.target_speed = 0.0f;
        Motor.calib_flag = 0U;
#if MOTOR_OPEN_LOOP_TEST_ACTIVE
        test_powered_window_active = 0U;
#endif
        LED_SetPattern(LED_PATTERN_HEARTBEAT);
    }

#if MOTOR_OPEN_LOOP_TEST_ACTIVE
    /* A second, independent exit guard for the one-shot S1-C build. */
    if (test_powered_window_active != 0U
        && (uint32_t)(HAL_GetTick() - test_powered_start_ms)
               >= MOTOR_TEST_MAX_POWERED_MS)
    {
        FOC_Disable(&Motor);
        Motor.calib_flag = 0U;
        g_mode = MOTOR_MODE_STANDBY;
        test_powered_window_active = 0U;
        LED_SetPattern(LED_PATTERN_HEARTBEAT);
    }
#endif

    switch (g_mode)
    {
    case MOTOR_MODE_STANDBY:
        break;

    case MOTOR_MODE_SPEED:
        /* 软启动：每周期把 target_speed 往 g_speed_desired 拉一步
         * SPEED_RAMP_RPM_S=0 时 step=0，退化为直通（编译器优化掉） */
        {
            float step = SPEED_RAMP_RPM_S / Motor.rate;  /* rpm/周期 */
            float err  = g_speed_desired - Motor.target_speed;
            if (err > step)
                Motor.target_speed += step;
            else if (err < -step)
                Motor.target_speed -= step;
            else
                Motor.target_speed = g_speed_desired;
        }
        FOC_SpdCtrl_Ext(&Motor);
        break;

    case MOTOR_MODE_POSITION:
        FOC_PosCtrl(&Motor);
        FOC_SpdCtrl(&Motor);
        break;

    case MOTOR_MODE_CALIBRATE:
        /* FOC_Output injects only the configured low-voltage ramp. */
        break;

    default:
        break;
    }

    FOC_Output(&Motor);

    /* ================================================================
     *  LED 指示器 —— 亮度来自扭矩，模式由事件驱动
     *
     *  亮度（Brightness）：
     *    待机/校准：强制 100%，不受扭矩影响（要闪烁可见）
     *    运行态：20%~100% 线性映射到 0~12V Uq
     *  模式（Pattern）：
     *    待机/校准时由模式切换事件设置，运行态默认常亮。
     * ================================================================ */
    {
        uint8_t brightness;
        if (g_mode == MOTOR_MODE_CALIBRATE) {
            brightness = 100;
            /* 校准已在进入时设为 FAST_BLINK */
        } else if (g_mode == MOTOR_MODE_STANDBY) {
            brightness = 100;
            /* 待机已在切换事件中设为 HEARTBEAT */
        } else {
            /* 运行态：5%~100% 对应 0~12V */
            float ratio = Motor.Uq / Motor.Umax;
            if (ratio < 0.0f)  ratio = -ratio;
            if (ratio > 1.0f)  ratio = 1.0f;
            brightness = 5 + (uint8_t)(ratio * 95.0f);
            LED_SetPattern(LED_PATTERN_ON);
        }
        LED_SetBrightness(brightness);
    }

    /* ================================================================
     *  第 3 步：校准完成检测
     *
     *  V2 对齐完成时只保留内存中的候选偏移值。方向、相序和极对数
     *  还未验证，因此严禁在此阶段自动写入 Flash。
     * ================================================================ */
    if (g_mode == MOTOR_MODE_CALIBRATE && Motor.calib_result_ready != 0U)
    {
        LED_SetPattern(LED_PATTERN_HEARTBEAT);

        /* 关断驱动 → 回到待机；候选值供下一阶段显式确认。 */
        FOC_Disable(&Motor);
        g_mode = MOTOR_MODE_STANDBY;
#if MOTOR_OPEN_LOOP_TEST_ACTIVE
        test_powered_window_active = 0U;
#endif
#if MOTOR_TEST_COMMIT_OFFSET
        /* The bridge is already disabled. Commit only this fully verified
           one-shot candidate, then re-read it inside FOC_CommitCalibration. */
        (void)FOC_CommitCalibration(&Motor);
#endif
    }

    /* ================================================================
     *  第 4 步：更新状态 → 触发所有通道回传
     *
     *  MotorMsg_TxStatus() 内部会编码为 8 字节统一 payload，
     *  然后分别调 CANProto_SendPayload() 和 UARTProto_SendPayload()。
     *  两个通道每 1ms 都发（1 kHz），各自独立。
     * ================================================================ */
    MotorMsg_UpdateStatus(
        Motor.mech_angle * DEG_TO_RAD,
        Motor.speed * RPM_TO_RAD_S,
        Motor.Uq
    );
#if MOTOR_COMMUNICATION_ENABLED
    MotorMsg_TxStatus();
#endif
}
