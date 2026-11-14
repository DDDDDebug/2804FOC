/**
 * @file    smartknob_ctrl.h
 * @brief   SmartKnob virtual-detent controller (software-only stage).
 */

#ifndef __SMARTKNOB_CTRL_H__
#define __SMARTKNOB_CTRL_H__

#include <stdint.h>

typedef struct
{
    float detent_spacing_deg;       /**< 机械角卡点间距 */
    float detent_strength_v_per_deg; /**< 连续卡点曲线增益 */
    float spring_center_scale;      /**< 卡点中心斜率相对外沿峰值的比例 */
    float damping_v_per_rpm;        /**< 速度阻尼增益 */
    float detent_damping_v_per_rpm; /**< 卡点附近附加阻尼 */
    float detent_damping_start_rpm; /**< 附加阻尼开始增强的速度 */
    float max_damping_v;            /**< 阻尼电压软上限 */
    float spring_nonlinear_gain;    /**< 偏离卡点后的非线性恢复增强 */
    float startup_ramp_s;           /**< 使能后的输出软启动时间 */
    float oscillation_window_s;     /**< 反转计数时间窗 */
    float oscillation_min_swing_deg; /**< 计为反转的最小摆幅 */
    float oscillation_capture_max_uq_v; /**< 捕获阶段输出上限 */
    float oscillation_capture_ramp_s; /**< 捕获输出平滑时间 */
    float oscillation_capture_release_deg; /**< 解除锁定角度 */
    float max_uq_v;                 /**< 输出电压上限 */
    float max_speed_rpm;            /**< 参与阻尼计算的速度上限 */
} SmartKnobConfig_t;

typedef struct
{
    SmartKnobConfig_t config;
    float target_angle_deg;
    float output_uq_v;
    int32_t detent_index;
    uint8_t enabled;
    uint8_t initialized;
    float startup_elapsed_s;
    float last_angle_deg;
    float swing_since_reversal_deg;
    float oscillation_elapsed_s;
    float oscillation_capture_elapsed_s;
    float oscillation_capture_target_deg;
    int8_t motion_direction;
    uint8_t oscillation_reversals;
    uint8_t oscillation_capture_active;
    uint8_t motion_initialized;
} SmartKnob_t;

void SmartKnob_Init(SmartKnob_t *knob);
void SmartKnob_SetConfig(SmartKnob_t *knob, const SmartKnobConfig_t *config);
void SmartKnob_SetEnabled(SmartKnob_t *knob, uint8_t enabled);
float SmartKnob_Update(SmartKnob_t *knob,
                       float angle_deg,
                       float speed_rpm,
                       float dt_s);
float SmartKnob_GetTargetAngle(const SmartKnob_t *knob);
int32_t SmartKnob_GetDetentIndex(const SmartKnob_t *knob);

#endif /* __SMARTKNOB_CTRL_H__ */
