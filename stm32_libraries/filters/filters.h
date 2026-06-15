#pragma once

#include <stdint.h>

// ab -------------------------------------------------------------------
typedef struct
{
    int64_t x_Qfmt;
    int64_t x_dot_Qfmt;
    int64_t x_dot_dot_Qfmt;
    uint16_t alpha_fixed;
    uint16_t beta_fixed;
    uint8_t q_format;
} alpha_beta_filter_t;

void alpha_beta_filter_init(alpha_beta_filter_t *ab_filt, float alpha, float beta, uint8_t q_format);
void alpha_beta_filter_update(alpha_beta_filter_t *ab_filt, int64_t measurement);
void alpha_beta_filter_set(alpha_beta_filter_t *ab_filt, int64_t x, int64_t x_dot, int64_t x_dot_dot);
void alpha_beta_filter_set_alpha(alpha_beta_filter_t *ab_filt, float alpha);

static inline int64_t alpha_beta_filter_get_x(alpha_beta_filter_t *ab_filt)
{
    return ab_filt->x_Qfmt >> (ab_filt->q_format);
}

static inline int64_t alpha_beta_filter_get_x_dot(alpha_beta_filter_t *ab_filt)
{
    return ab_filt->x_dot_Qfmt >> (ab_filt->q_format);
}

static inline int64_t alpha_beta_filter_get_x_dot_dot(alpha_beta_filter_t *ab_filt)
{
    return ab_filt->x_dot_dot_Qfmt;
}
// ab ----------------------------------------------------------------------------------------------------------------
// pi ----------------------------------------------------------------------------------------------------------------
typedef struct
{
    float L;
    float R;
    float bw;
    float kp;
    float ki;
    float integrator;
    float integral_limit; // anti-windup clamp
    float error_prev;
} pi_controller_t;
// dt is 20us
void pi_controller_init(pi_controller_t *controller);
float pi_controller_set_gains(pi_controller_t *controller, float L, float R, float bw);
float pi_controller_update(pi_controller_t *controller, float error);
// pi ----------------------------------------------------------------------------------------------------------------
