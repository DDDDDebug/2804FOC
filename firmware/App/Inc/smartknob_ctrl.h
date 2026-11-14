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
    float damping_v_per_rpm;        /**< 速度阻尼增益 */
    float detent_damping_v_per_rpm; /**< 卡点附近附加阻尼 */
    float detent_damping_start_rpm; /**< 附加阻尼开始增强的速度 */
    float max_damping_v;            /**< 阻尼电压软上限 */
    float spring_nonlinear_gain;    /**< 偏离卡点后的非线性恢复增强 */
    float startup_ramp_s;           /**< 使能后的输出软启动时间 */
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
