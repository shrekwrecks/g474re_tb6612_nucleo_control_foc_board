#include "filters.h"
#include "profiling.h"
void alpha_beta_filter_init(alpha_beta_filter_t *ab_filt, float alpha, float beta, uint8_t q_format)
{
    ab_filt->x_Qfmt = 0;
    ab_filt->x_dot_Qfmt = 0;
    ab_filt->x_dot_dot_Qfmt = 0;
    ab_filt->q_format = q_format;
    ab_filt->alpha_fixed = (uint16_t)(alpha * (1 << q_format));
    ab_filt->beta_fixed = (uint16_t)(beta * (1 << q_format));
}

void alpha_beta_filter_update(alpha_beta_filter_t *ab_filt, int64_t measurement)
{
    int64_t meas_Qfmt = measurement << (ab_filt->q_format);

    int64_t pos_pred = ab_filt->x_Qfmt + ab_filt->x_dot_Qfmt;
    int64_t vel_pred = ab_filt->x_dot_Qfmt + (ab_filt->x_dot_dot_Qfmt);

    int64_t residual = meas_Qfmt - pos_pred;

    ab_filt->x_dot_dot_Qfmt = (ab_filt->beta_fixed * residual) >> (ab_filt->q_format);
    ab_filt->x_dot_Qfmt = vel_pred + (ab_filt->x_dot_dot_Qfmt);
    ab_filt->x_Qfmt = pos_pred + ((ab_filt->alpha_fixed * residual) >> (ab_filt->q_format));
}

void alpha_beta_filter_set(alpha_beta_filter_t *ab_filt, int64_t x, int64_t x_dot, int64_t x_dot_dot)
{
    ab_filt->x_Qfmt = x << (ab_filt->q_format);
    ab_filt->x_dot_Qfmt = x_dot << (ab_filt->q_format);
    ab_filt->x_dot_dot_Qfmt = x_dot_dot << (ab_filt->q_format);
}

void alpha_beta_filter_set_alpha(alpha_beta_filter_t *ab_filt, float alpha)
{
    float beta = (alpha * alpha) / (2.0f - alpha);
    ab_filt->alpha_fixed = (uint16_t)(alpha * (1 << (ab_filt->q_format)));
    ab_filt->beta_fixed = (uint16_t)(beta * (1 << (ab_filt->q_format)));
}

// pi control ----------------------------------------------------
void pi_controller_init(pi_controller_t *controller)
{
    controller->L = 0.001f;
    controller->R = 1.0f;
    controller->bw = 1000.0f;
    float dt = 20e-6f;

    controller->integrator = 0;
    controller->error_prev = 0;

    controller->kp = controller->L * controller->bw;
    controller->ki = controller->R * controller->bw * dt;
    controller->integral_limit = 4.0f / controller->ki;

    // controller->ki = 0;
    // controller->kp = 0;
}

float pi_controller_set_gains(pi_controller_t *controller, float L, float R, float bw)
{
    float dt = 20e-6f;

    if (L != 0.0f)
        controller->L = L;
    if (R != 0.0f)
        controller->R = R;
    if (bw != 0.0f)
        controller->bw = bw;

    controller->kp = controller->L * controller->bw;
    controller->ki = controller->R * controller->bw * dt;
    controller->integral_limit = 4.0f / controller->ki;
    // integrator intentionally left unchanged
}
float pi_controller_update(pi_controller_t *controller, float error)
{
    // Tustin integration: trapezoidal sum
    float new_integrator = controller->integrator + 0.5f * (error + controller->error_prev);
    // old way
    // controller->integrator += error;

    // Clamp
    if (new_integrator > controller->integral_limit)
        new_integrator = controller->integral_limit;
    if (new_integrator < -controller->integral_limit)
        new_integrator = -controller->integral_limit;

    controller->integrator = new_integrator;
    controller->error_prev = error;

    return controller->kp * error + controller->ki * controller->integrator;
}