/**
 * @file    smartknob_ctrl.c
 * @brief   Deterministic virtual-detent and damping calculation.
 */

#include "smartknob_ctrl.h"
#include <math.h>

#define SMARTKNOB_DEFAULT_SPACING_DEG       30.0f
#define SMARTKNOB_DEFAULT_STRENGTH_V_PER_DEG 0.120f
#define SMARTKNOB_DEFAULT_DAMPING_V_PER_RPM  0.002f
#define SMARTKNOB_DEFAULT_DETENT_DAMPING_V_PER_RPM 0.012f
#define SMARTKNOB_DEFAULT_DETENT_DAMPING_START_RPM 15.0f
#define SMARTKNOB_DEFAULT_MAX_DAMPING_V    0.24f
#define SMARTKNOB_DEFAULT_SPRING_NONLINEAR_GAIN 0.0f
#define SMARTKNOB_DEFAULT_STARTUP_RAMP_S  0.25f
#define SMARTKNOB_DEFAULT_MAX_UQ_V           0.80f
#define SMARTKNOB_DEFAULT_MAX_SPEED_RPM      60.0f
#define SMARTKNOB_HARD_MAX_UQ_V              0.80f
#define SMARTKNOB_DETENT_HYSTERESIS          0.25f

static float clampf(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static float wrap_angle_deg(float angle_deg)
{
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float shortest_error_deg(float target_deg, float angle_deg)
{
    float error = target_deg - angle_deg;
    while (error > 180.0f) {
        error -= 360.0f;
    }
    while (error < -180.0f) {
        error += 360.0f;
    }
    return error;
}

static int32_t nearest_detent(float angle_deg, float spacing_deg)
{
    return (int32_t)floorf((angle_deg / spacing_deg) + 0.5f);
}

static void sanitize_config(SmartKnobConfig_t *config)
{
    if (config->detent_spacing_deg < 1.0f) {
        config->detent_spacing_deg = SMARTKNOB_DEFAULT_SPACING_DEG;
    }
    if (config->detent_strength_v_per_deg < 0.0f) {
        config->detent_strength_v_per_deg = 0.0f;
    }
    if (config->damping_v_per_rpm < 0.0f) {
        config->damping_v_per_rpm = 0.0f;
    }
    if (config->detent_damping_v_per_rpm < 0.0f) {
        config->detent_damping_v_per_rpm = 0.0f;
    }
    if (config->detent_damping_start_rpm < 0.0f) {
        config->detent_damping_start_rpm = 0.0f;
    }
    if (config->max_damping_v < 0.0f) {
        config->max_damping_v = 0.0f;
    }
    if (config->spring_nonlinear_gain < 0.0f) {
        config->spring_nonlinear_gain = 0.0f;
    }
    if (config->startup_ramp_s < 0.0f) {
        config->startup_ramp_s = 0.0f;
    }
    if (config->max_uq_v < 0.0f) {
        config->max_uq_v = 0.0f;
    }
    if (config->max_uq_v > SMARTKNOB_HARD_MAX_UQ_V) {
        config->max_uq_v = SMARTKNOB_HARD_MAX_UQ_V;
    }
    if (config->max_speed_rpm < 0.0f) {
        config->max_speed_rpm = 0.0f;
    }
}

void SmartKnob_Init(SmartKnob_t *knob)
{
    if (knob == 0) {
        return;
    }

    knob->config.detent_spacing_deg = SMARTKNOB_DEFAULT_SPACING_DEG;
    knob->config.detent_strength_v_per_deg = SMARTKNOB_DEFAULT_STRENGTH_V_PER_DEG;
    knob->config.damping_v_per_rpm = SMARTKNOB_DEFAULT_DAMPING_V_PER_RPM;
    knob->config.detent_damping_v_per_rpm = SMARTKNOB_DEFAULT_DETENT_DAMPING_V_PER_RPM;
    knob->config.detent_damping_start_rpm = SMARTKNOB_DEFAULT_DETENT_DAMPING_START_RPM;
    knob->config.max_damping_v = SMARTKNOB_DEFAULT_MAX_DAMPING_V;
    knob->config.spring_nonlinear_gain = SMARTKNOB_DEFAULT_SPRING_NONLINEAR_GAIN;
    knob->config.startup_ramp_s = SMARTKNOB_DEFAULT_STARTUP_RAMP_S;
    knob->config.max_uq_v = SMARTKNOB_DEFAULT_MAX_UQ_V;
    knob->config.max_speed_rpm = SMARTKNOB_DEFAULT_MAX_SPEED_RPM;
    knob->target_angle_deg = 0.0f;
    knob->output_uq_v = 0.0f;
    knob->detent_index = 0;
    knob->enabled = 0U;
    knob->initialized = 1U;
    knob->startup_elapsed_s = 0.0f;
}

void SmartKnob_SetConfig(SmartKnob_t *knob, const SmartKnobConfig_t *config)
{
    if ((knob == 0) || (config == 0)) {
        return;
    }

    if (knob->initialized == 0U) {
        SmartKnob_Init(knob);
    }
    knob->config = *config;
    sanitize_config(&knob->config);
}

void SmartKnob_SetEnabled(SmartKnob_t *knob, uint8_t enabled)
{
    if (knob == 0) {
        return;
    }

    if (knob->initialized == 0U) {
        SmartKnob_Init(knob);
    }
    if ((enabled != 0U) && (knob->enabled == 0U)) {
        knob->startup_elapsed_s = 0.0f;
    }
    knob->enabled = (enabled != 0U) ? 1U : 0U;
    if (knob->enabled == 0U) {
        knob->output_uq_v = 0.0f;
        knob->startup_elapsed_s = 0.0f;
    }
}

float SmartKnob_Update(SmartKnob_t *knob,
                       float angle_deg,
                       float speed_rpm,
                       float dt_s)
{
    float angle;
    float speed;
    float spacing;
    int32_t candidate;
    float candidate_target;
    float command;
    float phase_rad;
    float detent_peak_v;
    float dynamic_damping;
    float speed_gate;
    float damping_command;
    float spring;
    float startup_scale;

    if (knob == 0) {
        return 0.0f;
    }
    if (knob->initialized == 0U) {
        SmartKnob_Init(knob);
    }
    if (knob->enabled == 0U) {
        knob->output_uq_v = 0.0f;
        return 0.0f;
    }

    angle = wrap_angle_deg(angle_deg);
    spacing = knob->config.detent_spacing_deg;
    candidate = nearest_detent(angle, spacing);
    candidate_target = wrap_angle_deg((float)candidate * spacing);

    /* Hysteresis prevents detent chatter when the knob rests at a boundary. */
    if (candidate != knob->detent_index) {
        float current_error = shortest_error_deg(knob->target_angle_deg, angle);
        if (fabsf(current_error) > (spacing * SMARTKNOB_DETENT_HYSTERESIS)) {
            knob->detent_index = candidate;
            knob->target_angle_deg = candidate_target;
        }
    }

    speed = clampf(speed_rpm,
                   -knob->config.max_speed_rpm,
                   knob->config.max_speed_rpm);
    /* A periodic sine curve keeps the detent force continuous through the
       whole rotation: no hard switch between capture and travel phases. */
    phase_rad = angle * (2.0f * 3.14159265359f) / spacing;
    detent_peak_v = knob->config.detent_strength_v_per_deg * spacing
                  / (2.0f * 3.14159265359f);
    spring = sinf(phase_rad);
    spring += knob->config.spring_nonlinear_gain
            * spring * spring * spring;
    command = -detent_peak_v * spring;
    if (knob->config.detent_damping_start_rpm > 0.0f) {
        speed_gate = clampf(fabsf(speed)
                            / knob->config.detent_damping_start_rpm,
                            0.0f, 1.0f);
        /* Smoothstep avoids a new force discontinuity at the threshold. */
        speed_gate = speed_gate * speed_gate * (3.0f - 2.0f * speed_gate);
    } else {
        speed_gate = 1.0f;
    }
    {
        float proximity = fabsf(cosf(phase_rad));
        dynamic_damping = knob->config.damping_v_per_rpm;
        if (knob->config.detent_damping_v_per_rpm > 0.0f) {
            dynamic_damping += knob->config.detent_damping_v_per_rpm
                             * proximity * proximity * speed_gate;
        }
    }
    damping_command = dynamic_damping * speed;
    if ((knob->config.detent_damping_v_per_rpm > 0.0f)
        && (knob->config.max_damping_v > 0.0f)) {
        /* Smoothly limit damping voltage to avoid a bang-bang limit cycle. */
        damping_command = knob->config.max_damping_v
                        * tanhf(damping_command / knob->config.max_damping_v);
    }
    command -= damping_command;
    if (knob->config.startup_ramp_s > 0.0f) {
        knob->startup_elapsed_s += dt_s;
        startup_scale = clampf(knob->startup_elapsed_s
                               / knob->config.startup_ramp_s,
                               0.0f, 1.0f);
        command *= startup_scale;
    }
    knob->output_uq_v = clampf(command,
                               -knob->config.max_uq_v,
                               knob->config.max_uq_v);
    return knob->output_uq_v;
}

float SmartKnob_GetTargetAngle(const SmartKnob_t *knob)
{
    return (knob == 0) ? 0.0f : knob->target_angle_deg;
}

int32_t SmartKnob_GetDetentIndex(const SmartKnob_t *knob)
{
    return (knob == 0) ? 0 : knob->detent_index;
}
