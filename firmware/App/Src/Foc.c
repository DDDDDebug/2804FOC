/**
 * @file    Foc.c
 * @brief   单电机 FOC 控制器实现
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 The FOC Firmware Contributors
 *
 * 本文件对应重构前的 Foc.c，核心变化：
 *   1. 不直接调用 HAL_I2C / HAL_TIM，改用 sensor->read_angle() 和 motor->set_duty()
 *   2. 坐标变换委托给 foc_math.c 的纯函数
 *   3. PID 使用 PID_Calc() 替代旧的 pid_output()
 *   4. 只保留一个电机实例 Motor
 */

#include "Foc.h"
#include "main.h"
#include <string.h>

/* ---- 全局单电机实例 ---- */
FOC_t Motor = {0};

/* Keep the encoder-coordinate transform in one place so calibration and
 * closed-loop commutation always use the same direction convention. */
static float FOC_ApplySensorDirection(const FOC_t *foc, float raw_angle)
{
    if (foc->dir < 0)
    {
        return ANGLE_limit(-raw_angle, 360.0f, 0.0f, 360.0f);
    }

    return raw_angle;
}

static void FOC_ResetControlState(FOC_t *foc)
{
    foc->PosPID.integral = 0.0f;
    foc->PosPID.output = 0.0f;
    foc->SpdPID.integral = 0.0f;
    foc->SpdPID.output = 0.0f;
    foc->SpdPID_Ext.integral = 0.0f;
    foc->SpdPID_Ext.output = 0.0f;
    foc->Uq = 0.0f;
    foc->Ud = 0.0f;
    foc->s3_hold_uq = 0.0f;
}

static uint32_t FOC_FloatBits(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static uint32_t FOC_CalibrationChecksum(const FocCalibrationRecord_t *record)
{
    uint32_t metadata = (uint32_t)record->pole_pairs
                      | ((uint32_t)(uint8_t)record->sensor_direction << 8)
                      | ((uint32_t)record->reserved << 16);

    return record->version ^ FOC_FloatBits(record->elec_offset)
           ^ metadata ^ 0xA5C39E71UL;
}

static uint8_t FOC_IsCalibrationRecordValid(const FOC_t *foc,
                                             const FocCalibrationRecord_t *record)
{
    float max_offset = (float)foc->pole_pairs * 360.0f;

    return (record->version == FOC_CALIBRATION_RECORD_VERSION
            && record->pole_pairs == foc->pole_pairs
            && record->sensor_direction == foc->dir
            && record->checksum == FOC_CalibrationChecksum(record)
            && record->elec_offset >= 0.0f
            && record->elec_offset < max_offset) ? 1U : 0U;
}

/* ================================================================
 *  FOC_Init —— 初始化 FOC 控制器
 *
 *  存入硬件接口指针 + 观测器 + 电机配置 + 存储接口
 *  → 初始化观测器 → 读取初始角度 → 计算电角度 → 保持功率级关断
 *
 *  cfg 参数集中管理极对数、母线电压、控制频率，消除重复配置。
 *  storage 提供非易失存储抽象，换存储介质不影响 FOC 逻辑。
 *  observer 提供角度滤波和速度估计（差分/龙伯格/卡尔曼可互换）。
 * ================================================================ */
void FOC_Init(FOC_t *foc,
              AngleSensor_t *sensor, PwmOutput_t *motor,
              const MotorConfig_t *cfg,
              const NvStorage_t *storage,
              Observer_t *observer)
{
    foc->sensor     = sensor;
    foc->motor      = motor;
    foc->storage    = storage;
    foc->observer   = observer;
    foc->pole_pairs = cfg->pole_pairs;
    foc->Umax       = cfg->bus_voltage;
    foc->rate       = cfg->rate_hz;
    foc->dir        = MOTOR_SENSOR_DIRECTION;

    /* 读取初始角度并喂给观测器 */
    float initial = FOC_ApplySensorDirection(
        foc, sensor->read_angle(sensor->ctx));
    foc->observer->update(foc->observer->ctx, initial);
    foc->mech_angle      = foc->observer->get_position(foc->observer->ctx);
    foc->last_mech_angle = foc->mech_angle;
    foc->speed           = foc->observer->get_velocity(foc->observer->ctx);
    foc->last_speed      = foc->speed;
    foc->elec_angle      = (float)cfg->pole_pairs * foc->mech_angle;

    /* Only accept a versioned record matching this pole count and direction.
       The legacy single-float value has no such safeguards and is ignored. */
    FocCalibrationRecord_t record;
    if (!foc->storage->read(foc->storage->ctx, &record, sizeof(record))
        || !FOC_IsCalibrationRecordValid(foc, &record))
    {
        foc->elec_offset = FOC_ELEC_OFFSET;
        foc->calibration_record_valid = 0U;
    }
    else
    {
        foc->elec_offset = record.elec_offset;
        foc->calibration_record_valid = 1U;
    }
    foc->calib_flag             = 0U;
    foc->calib_count            = 0U;
    foc->calib_start_ms         = 0U;
    foc->calib_offset_candidate = foc->elec_offset;
    foc->calib_candidate_captured = 0U;
    foc->calib_result_ready     = 0U;
    foc->power_stage_state      = FOC_POWER_STAGE_DISABLED;
    foc->power_stage_count      = 0U;

    /*
     * Startup is always safe: output remains disabled until FOC_Enable().
     * This prevents merely flashing/resetting the MCU from energizing the
     * SimpleFOCmini bridge.
     */
    motor->disable(motor->ctx);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    /* 初始化独立速度环 PI（默认参数，速度模式可通过 CAN 覆盖） */
    PID_Init(&foc->SpdPID_Ext, 0.05f, 0.01f, 0.0f, cfg->rate_hz,
             1000.0f, cfg->bus_voltage);
}

/* ================================================================
 *  FOC_SetPosPID / FOC_SetSpdPID / FOC_SetSpdExtPID —— 设置 PID 参数
 *
 * 注意：PID 已做时间归一化（integral += error × dt），频率改变时 ki 行为不变。
 * ================================================================ */
void FOC_SetPosPID(FOC_t *foc, float kp, float ki, float kd, float out_max)
{
    PID_Init(&foc->PosPID, kp, ki, kd, foc->rate, out_max, out_max);
}

void FOC_SetSpdPID(FOC_t *foc, float kp, float ki, float kd, float out_max)
{
    PID_Init(&foc->SpdPID, kp, ki, kd, foc->rate, 1000.0f, out_max);
}

void FOC_SetSpdExtPID(FOC_t *foc, float kp, float ki, float kd, float out_max)
{
    PID_Init(&foc->SpdPID_Ext, kp, ki, kd, foc->rate, 1000.0f, out_max);
}

/* ================================================================
 *  FOC_SetTarget —— 设置目标角度（自动限幅 0~360°）
 * ================================================================ */
void FOC_SetTarget(FOC_t *foc, float target_angle)
{
    foc->target_angle = ANGLE_limit(target_angle, 360.0f, 0.0f, 360.0f);
}

/* ================================================================
 *  FOC_UpdateSensor —— 刷新传感器数据
 *
 *  通过 Observer 接口获取滤波后的角度和估计速度。
 *  无论底层是差分法、龙伯格还是卡尔曼，此处接口不变。
 *
 *  三件事：
 *    1. 读取原始角度 → 喂给观测器
 *    2. 从观测器拿滤波后角度 → 计算电角度
 *    3. 从观测器拿估计速度（无噪声放大）
 *
 *  注意：last_mech_angle / last_speed 保留给 PID 微分项使用，
 *  不受观测器影响。
 * ================================================================ */
void FOC_UpdateSensor(FOC_t *foc)
{
    /* 保存上一周期值（给 PID 微分项用） */
    foc->last_mech_angle = foc->mech_angle;
    foc->last_speed      = foc->speed;

    /* 读取原始角度 → 喂给观测器 */
    float raw = FOC_ApplySensorDirection(
        foc, foc->sensor->read_angle(foc->sensor->ctx));
    foc->observer->update(foc->observer->ctx, raw);

    /* 位置和速度都来自观测器
     *
     * 之前在 原始值（噪声→微分抖）和 观测值（滞后→过冲）间反复横跳。
     * 现在：提高 l1 让观测器快速收敛，控制滞后在 1~2ms 内，
     * 同时观测器内置的一阶滞后本身就在滤除传感器噪声，
     * 比原始值直通更适合进 PD 控制环的微分项。
     */
    foc->mech_angle = foc->observer->get_position(foc->observer->ctx);
    foc->speed      = foc->observer->get_velocity(foc->observer->ctx);

    /* 电角度 = 极对数 × 机械角度 - 零点偏移 */
    foc->elec_angle = ANGLE_limit(
        (float)foc->pole_pairs * foc->mech_angle - foc->elec_offset,
        (float)foc->pole_pairs * 360.0f,
        0.0f,
        (float)foc->pole_pairs * 360.0f);
}

/* ================================================================
 *  FOC_PosCtrl —— 位置环 PD
 *
 *  误差 = 目标角度 - 当前角度（最短路径）
 *  微分项采用 Derivative-on-Measurement：传入 -Δθ 起阻尼作用（抑制超调）
 * ================================================================ */
void FOC_PosCtrl(FOC_t *foc)
{
    float error = ANGLE_limit(
        foc->target_angle - foc->mech_angle,
        360.0f, -180.0f, 180.0f);

    float angle_delta = ANGLE_limit(
        foc->mech_angle - foc->last_mech_angle,
        360.0f, -180.0f, 180.0f);

    foc->target_speed = PID_Calc(&foc->PosPID, error, -angle_delta);
}

/* ================================================================
 *  FOC_SpdCtrl —— 串级速度环 PI
 *
 *  误差 = 目标速度 - 当前速度
 *  输出 = Uq（交轴电压）
 *  使用 SpdPID（与位置环联调），用于位置模式的串级内环。
 *
 *  速度限幅：target_speed 被钳制在 ±SPEED_LIMIT_RPM 内，
 *  定义于 motor_config.h，所有控制路径（SPEED/POSITION 指令、位置环输出）统一。
 * ================================================================ */
void FOC_SpdCtrl(FOC_t *foc)
{
    foc->target_speed = DATA_limit(foc->target_speed,
                                    -SPEED_LIMIT_RPM, SPEED_LIMIT_RPM);
    float error = foc->target_speed - foc->speed;
    foc->Uq = PID_Calc(&foc->SpdPID, error, foc->speed - foc->last_speed);
}

/* ================================================================
 *  FOC_SpdCtrl_Ext —— 独立速度环 PI
 *
 *  使用 SpdPID_Ext，与串级 SpdPID 互不影响。
 *  默认 PI 控制（kd=0），参数通过 CAN 或 FOC_SetSpdExtPID 配置。
 * ================================================================ */
void FOC_SpdCtrl_Ext(FOC_t *foc)
{
    foc->target_speed = DATA_limit(foc->target_speed,
                                    -SPEED_LIMIT_RPM, SPEED_LIMIT_RPM);
    float error = foc->target_speed - foc->speed;
    foc->Uq = PID_Calc(&foc->SpdPID_Ext, error, foc->speed - foc->last_speed);
}

/* ================================================================
 *  FOC_Enable —— 使能电机输出
 *
 *  拉高 PB1（驱动芯片使能） + 启动 PWM 输出。
 *  从待机态切回运行态时调用。
 * ================================================================ */
void FOC_Enable(FOC_t *foc)
{
    /* Keep PB1 low while PWM compare registers and output channels settle. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

#if MOTOR_POWER_STAGE_ENABLED
    foc->motor->set_duty(foc->motor->ctx, 0.5f, 0.5f, 0.5f);
    foc->motor->enable(foc->motor->ctx);
    foc->power_stage_state = FOC_POWER_STAGE_PRELOAD;
    foc->power_stage_count = 0U;
#else
    /* Virtual mode is hard-disabled at both the GPIO and PWM layers. */
    foc->motor->disable(foc->motor->ctx);
    foc->power_stage_state = FOC_POWER_STAGE_DISABLED;
    foc->power_stage_count = 0U;
    foc->calib_start_ms = 0U;
#endif
}

/* ================================================================
 *  FOC_Disable —— 关闭电机输出
 *
 *  拉低 PB1 → 驱动芯片失能 → 内部自动关断 MOSFET。
 *  同时清零所有 PID 积分项，避免切回运行态时积分 windup。
 * ================================================================ */
void FOC_Disable(FOC_t *foc)
{
    /* PB1 拉低 → 驱动芯片自动关断全部输出 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    /* 停止 PWM 输出 */
    foc->motor->disable(foc->motor->ctx);

    foc->power_stage_state = FOC_POWER_STAGE_DISABLED;
    foc->power_stage_count = 0U;

    /* 清零全部 PID 积分（防 windup） */
    FOC_ResetControlState(foc);
}

uint8_t FOC_IsPowerStageReady(const FOC_t *foc)
{
    return (foc->power_stage_state == FOC_POWER_STAGE_ENABLED) ? 1U : 0U;
}

uint8_t FOC_IsCalibrationValid(const FOC_t *foc)
{
    return (foc->calibration_record_valid != 0U) ? 1U : 0U;
}

/* ================================================================
 *  FOC_Output —— FOC 输出
 *
 *  流水线：
 *    1. 校准检测（calib_flag=1 时注入低 Ud 拉转子对齐）
 *    2. 反 Park 变换：d-q → α-β
 *    3. 反 Clarke 变换：α-β → 三相电压（量纲为 V）
 *    4. 限幅 → 归一化到 0.0~1.0 → 输出到 PWM
 *
 *  set_duty 接收归一化值，PWM 驱动内部自行换算为定时器比较值。
 * ================================================================ */
void FOC_Output(FOC_t *foc)
{
    if (foc->power_stage_state == FOC_POWER_STAGE_PRELOAD)
    {
        /* PB1 is still low here, so this cannot generate motor torque. */
        foc->motor->set_duty(foc->motor->ctx, 0.5f, 0.5f, 0.5f);
        if (++foc->power_stage_count >= (uint32_t)MOTOR_PWM_PRELOAD_TIME_MS)
        {
            FOC_ResetControlState(foc);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
            foc->power_stage_state = FOC_POWER_STAGE_ENABLED;
        }
        return;
    }

    if (foc->power_stage_state != FOC_POWER_STAGE_ENABLED)
    {
        return;
    }

    uint8_t open_loop_test_active = 0U;

    /* ---- 电角度校准流程 ---- */
    if (foc->calib_flag == 1)
    {
        open_loop_test_active = 1U;
        uint32_t elapsed_ms;

        if (foc->calib_start_ms == 0U)
        {
            foc->calib_start_ms = HAL_GetTick();
        }
        elapsed_ms = (uint32_t)(HAL_GetTick() - foc->calib_start_ms);
        foc->calib_count = elapsed_ms;
        FOC_CalibStep(foc, elapsed_ms);
        if (elapsed_ms >= (MOTOR_ALIGNMENT_RAMP_UP_MS
                           + MOTOR_ALIGNMENT_HOLD_MS)
            && foc->calib_candidate_captured == 0U)
        {
            /* Capture while the d-axis vector is still at full strength. */
            foc->calib_offset_candidate = (float)foc->pole_pairs
                                       * FOC_ApplySensorDirection(
                                             foc, foc->sensor->read_angle(foc->sensor->ctx));
            foc->calib_candidate_captured = 1U;
        }
        if (elapsed_ms >= (MOTOR_ALIGNMENT_RAMP_UP_MS
                           + MOTOR_ALIGNMENT_HOLD_MS
                           + MOTOR_ALIGNMENT_RAMP_DOWN_MS))
        {
            /* Save only a candidate. Direction and phase order are not yet
               verified, so no calibration result may be committed here. */
            if (foc->calib_candidate_captured == 0U)
            {
                foc->calib_offset_candidate = (float)foc->pole_pairs
                                           * FOC_ApplySensorDirection(
                                                 foc, foc->sensor->read_angle(foc->sensor->ctx));
                foc->calib_candidate_captured = 1U;
            }
            foc->calib_result_ready = 1U;
            foc->calib_flag = 0U;
            foc->Ud = 0.0f;
            foc->calib_start_ms = 0U;
        }
    }
    else if (foc->calib_flag == 2U)
    {
        uint32_t elapsed_ms;

        open_loop_test_active = 1U;
        if (foc->calib_start_ms == 0U)
        {
            foc->calib_start_ms = HAL_GetTick();
        }
        elapsed_ms = (uint32_t)(HAL_GetTick() - foc->calib_start_ms);
        foc->calib_count = elapsed_ms;
        FOC_S2DirectionStep(foc, elapsed_ms);
    }

    /* ---- 电角度前馈补偿 ----
     *
     * 问题: 从 AS5600 角度采样到 PWM 生效之间存在 ~210μs 延迟。
     *       高速时转子在此期间转过了可观的电角度, 导致反 Park 输出的
     *       电压矢量滞后于实际转子位置, 部分电压跑到 d 轴。
     *
     * 补偿: 根据当前转速推算延迟期间的电角度增量, 前馈到输出角度。
     *   angle_advance_deg = speed_rpm × pole_pairs × 6 × ANGLE_COMP_DELAY_S
     *
     * 此补偿只影响 FOC_Output 的反 Park 变换,
     * foc->elec_angle 本身不变（保持为观测器估计值）。
     *
     * 原理: speed 是 rpm, 乘以 6 转为 deg/s (360/60=6)。
     *       再乘极对数转为电角度 deg/s。
     *       再乘延迟时间 s 得到应补偿的电角度。
     *
     * 补偿系数定义于 motor_config.h。
     * ================================================================ */
    float angle_advance = (open_loop_test_active != 0U)
        ? 0.0f
        : foc->speed * (float)foc->pole_pairs * 6.0f * ANGLE_COMP_DELAY_S;
    float comp_elec_angle = foc->elec_angle + angle_advance;

    /* ---- (Ud, Uq) 矢量限幅 —— 防止进入过调制区 ----
     *
     * 当 Ud/Uq 合成矢量模长超过线性调制区上限时，
     * 等比例缩小（不改变方向），避免电压削波导致电机发热。
     *
     * 上限 = Umax × VOLTAGE_MODULATION_FACTOR
     *   0.5      × Umax = SPWM   上限（Udc/2）
     *   0.57735  × Umax = SVPWM  上限（Udc/√3）
     *
     * 限幅系数定义于 motor_config.h，换调制方式时更新它即可。
     * ================================================================ */
    {
        float vmag = sqrtf(foc->Ud * foc->Ud + foc->Uq * foc->Uq);
        float vlim = foc->Umax * VOLTAGE_MODULATION_FACTOR;
        if (vmag > vlim)
        {
            float scale = vlim / vmag;
            foc->Ud *= scale;
            foc->Uq *= scale;
        }
    }

    /* ---- 反 Park 变换：d-q → α-β（使用补偿后的电角度） ---- */
    float u_alpha, u_beta;
    FOC_Math_InvPark(foc->Ud, foc->Uq,
                     comp_elec_angle * DEG_TO_RAD,
                     &u_alpha, &u_beta);

    /* ---- 反 Clarke 变换：α-β → 三相电压（量纲为 V）
     *
     * 使用 SVPWM（零序分量注入法），直流利用率 ~100%。
     * 如果需要切回 SPWM，把 FOC_Math_InvClarke_SVPWM 换回 FOC_Math_InvClarke。
     * ================================================================ */
    float duty_a, duty_b, duty_c;
    FOC_Math_InvClarke_SVPWM(u_alpha, u_beta, foc->Umax,
                              &duty_a, &duty_b, &duty_c);

    /* ---- 限幅到 [0, Umax] 并归一化到 [0.0, 1.0] ---- */
    float inv_umax = 1.0f / foc->Umax;
    foc->Ua = DATA_limit(duty_a, 0.0f, foc->Umax) * inv_umax;
    foc->Ub = DATA_limit(duty_b, 0.0f, foc->Umax) * inv_umax;
    foc->Uc = DATA_limit(duty_c, 0.0f, foc->Umax) * inv_umax;

    /* ---- 输出到 PWM（归一化值 0.0 ~ 1.0） ---- */
    foc->motor->set_duty(foc->motor->ctx, foc->Ua, foc->Ub, foc->Uc);
}

/* ================================================================
 *  FOC_CalibStep —— 执行单步电角度校准
 *
 *  注入低 Ud、Uq=0 → 转子被拉到 d 轴对齐 → 此时编码器读数 = 电角度零点偏移。
 *  校准时长和电压由 motor_config.h 统一配置。
 * ================================================================ */
void FOC_CalibStep(FOC_t *foc, uint32_t elapsed_ms)
{
    uint32_t ramp_up_ms = MOTOR_ALIGNMENT_RAMP_UP_MS;
    uint32_t hold_ms = MOTOR_ALIGNMENT_HOLD_MS;
    uint32_t ramp_down_ms = MOTOR_ALIGNMENT_RAMP_DOWN_MS;
    float voltage_scale;

    foc->elec_angle = 0.0f;
    foc->Uq = 0.0f;

    if (elapsed_ms < ramp_up_ms)
    {
        voltage_scale = (float)elapsed_ms / (float)ramp_up_ms;
    }
    else if (elapsed_ms < (ramp_up_ms + hold_ms))
    {
        voltage_scale = 1.0f;
    }
    else
    {
        uint32_t ramp_down_count = elapsed_ms - ramp_up_ms - hold_ms;
        voltage_scale = (ramp_down_count >= ramp_down_ms)
            ? 0.0f
            : 1.0f - ((float)ramp_down_count / (float)ramp_down_ms);
    }

    foc->Ud = MOTOR_ALIGNMENT_VOLTAGE_V * voltage_scale;
}

/* ================================================================
 *  FOC_S2DirectionStep —— S2 低压开环方向扫动
 *
 *  先把转子温和拉到已知电角度 0，再把定子电压矢量正向扫过一小段
 *  电角度并原路返回。该过程不读取 PID 输出，也不保存任何校准值；
 *  它只为用 AS5600 的实际角度变化确认相序与传感器方向关系。
 * ================================================================ */
void FOC_S2DirectionStep(FOC_t *foc, uint32_t elapsed_ms)
{
    const uint32_t align_end = MOTOR_S2_ALIGN_RAMP_UP_MS;
    const uint32_t align_hold_end = align_end + MOTOR_S2_ALIGN_HOLD_MS;
    const uint32_t sweep_forward_end = align_hold_end + MOTOR_S2_SWEEP_FORWARD_MS;
    const uint32_t sweep_hold_end = sweep_forward_end + MOTOR_S2_SWEEP_HOLD_MS;
    const uint32_t sweep_return_end = sweep_hold_end + MOTOR_S2_SWEEP_RETURN_MS;
    const uint32_t ramp_down_end = sweep_return_end + MOTOR_S2_RAMP_DOWN_MS;
    float vector_angle = 0.0f;
    float voltage_scale = 0.0f;

    if (elapsed_ms < align_end)
    {
        voltage_scale = (float)elapsed_ms / (float)MOTOR_S2_ALIGN_RAMP_UP_MS;
    }
    else if (elapsed_ms < align_hold_end)
    {
        voltage_scale = 1.0f;
    }
    else if (elapsed_ms < sweep_forward_end)
    {
        voltage_scale = 1.0f;
        vector_angle = MOTOR_S2_ELEC_SWEEP_DEG
            * (float)(elapsed_ms - align_hold_end)
            / (float)MOTOR_S2_SWEEP_FORWARD_MS;
    }
    else if (elapsed_ms < sweep_hold_end)
    {
        voltage_scale = 1.0f;
        vector_angle = MOTOR_S2_ELEC_SWEEP_DEG;
    }
    else if (elapsed_ms < sweep_return_end)
    {
        voltage_scale = 1.0f;
        vector_angle = MOTOR_S2_ELEC_SWEEP_DEG
            * (1.0f - (float)(elapsed_ms - sweep_hold_end)
                         / (float)MOTOR_S2_SWEEP_RETURN_MS);
    }
    else if (elapsed_ms < ramp_down_end)
    {
        voltage_scale = 1.0f - (float)(elapsed_ms - sweep_return_end)
                                  / (float)MOTOR_S2_RAMP_DOWN_MS;
    }
    else
    {
        foc->Ud = 0.0f;
        foc->Uq = 0.0f;
        foc->calib_flag = 0U;
        foc->calib_start_ms = 0U;
        foc->calib_result_ready = 1U;
        return;
    }

    foc->elec_angle = vector_angle;
    foc->Ud = MOTOR_S2_VOLTAGE_V * voltage_scale;
    foc->Uq = 0.0f;
}

void FOC_S3HoldReset(FOC_t *foc)
{
    foc->s3_hold_uq = 0.0f;
    foc->Ud = 0.0f;
    foc->Uq = 0.0f;
}

void FOC_S3HoldStep(FOC_t *foc, float target_angle)
{
    float error = ANGLE_limit(target_angle - foc->mech_angle,
                              360.0f, -180.0f, 180.0f);
    float desired = 0.0f;
    float max_delta;

    /* A small deadband prevents encoder quantisation from chattering the
       bridge when the rotor is already at the captured angle. */
    if (error > MOTOR_S3_HOLD_DEADBAND_DEG
        || error < -MOTOR_S3_HOLD_DEADBAND_DEG)
    {
        desired = MOTOR_S3_HOLD_KP_V_PER_DEG * error;
        desired = DATA_limit(desired,
                             -MOTOR_S3_HOLD_MAX_VOLTAGE_V,
                             MOTOR_S3_HOLD_MAX_VOLTAGE_V);
    }

    max_delta = MOTOR_S3_HOLD_SLEW_V_PER_S / foc->rate;
    if (foc->s3_hold_uq < desired - max_delta)
    {
        foc->s3_hold_uq += max_delta;
    }
    else if (foc->s3_hold_uq > desired + max_delta)
    {
        foc->s3_hold_uq -= max_delta;
    }
    else
    {
        foc->s3_hold_uq = desired;
    }

    foc->Ud = 0.0f;
    foc->Uq = DATA_limit(foc->s3_hold_uq,
                         -MOTOR_S3_HOLD_MAX_VOLTAGE_V,
                         MOTOR_S3_HOLD_MAX_VOLTAGE_V);
}

uint8_t FOC_CommitCalibration(FOC_t *foc)
{
    FocCalibrationRecord_t record;
    FocCalibrationRecord_t verify;

    if (foc->calib_result_ready == 0U
        || foc->calib_candidate_captured == 0U)
    {
        return 0U;
    }

    record.version = FOC_CALIBRATION_RECORD_VERSION;
    record.elec_offset = ANGLE_limit(foc->calib_offset_candidate,
                                     (float)foc->pole_pairs * 360.0f,
                                     0.0f,
                                     (float)foc->pole_pairs * 360.0f);
    record.pole_pairs = foc->pole_pairs;
    record.sensor_direction = foc->dir;
    record.reserved = 0U;
    record.checksum = FOC_CalibrationChecksum(&record);

    foc->storage->write(foc->storage->ctx, &record, sizeof(record));
    if (!foc->storage->read(foc->storage->ctx, &verify, sizeof(verify))
        || !FOC_IsCalibrationRecordValid(foc, &verify))
    {
        return 0U;
    }

    foc->elec_offset = verify.elec_offset;
    return 1U;
}

/* ================================================================
 *  FOC_ReadCurrents —— 读三相电流 → Clarke → Park → Id/Iq
 *
 *  当前无电流反馈，Id/Iq 恒为 0。保留此函数供后续硬件升级后恢复使用。
 *
 *  调用位置: Motor_Loop 中 FOC_UpdateSensor 之后
 * ================================================================ */
void FOC_ReadCurrents(FOC_t *foc)
{
    foc->Ia = 0.0f;
    foc->Ib = 0.0f;
    foc->Ic = 0.0f;
    foc->I_alpha = 0.0f;
    foc->I_beta = 0.0f;
    foc->Id = 0.0f;
    foc->Iq = 0.0f;
}
