// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper (STM32G0/G4 HAL port)
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited) — original RP2040 implementation
//          STM32 HAL port: translated for TIM8 + TB6612FNG, INx-PWM mode
// --------------------------------------------------------------------------------------

#include "tb6612.h"
#include <math.h>
#include "stm32g4xx_ll_cordic.h"
/* ------------------------------------------------------------------ */
/*  Internal state                                                      */
/* ------------------------------------------------------------------ */
static float _amplitude = 0.0f;
static float _amplitude_raw = 0.0f; /* 0.0 – ARR counts */
static float _field_angle = 0.0f;   /* radians, unbounded */

/* ------------------------------------------------------------------ */
/*  Helpers                                                             */
/* ------------------------------------------------------------------ */

float _clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

int32_t _round_i32(float v)
{
    return (v >= 0.0f) ? (int32_t)(v + 0.5f) : (int32_t)(v - 0.5f);
}

uint32_t _arr(void)
{
    return STEP_TIM->ARR;
}

/**
 * Drive one coil using INx-PWM mode.
 *
 * The TIM8 complementary output pair (CHx / CHxN) drives IN1 and IN2.
 * TB6612 truth table with PWM pin tied HIGH:
 *   IN1=PWM, IN2=0   →  CW  (positive current)
 *   IN1=0,   IN2=PWM →  CCW (negative current)
 *   IN1=H,   IN2=H   →  short brake  ← avoid
 *   IN1=L,   IN2=L   →  Hi-Z / stop  ← avoid (deadband clamp prevents this)
 *
 * Strategy: CCR sets the magnitude on the active IN pin.
 *   value > 0: write CCR = value  → IN1 active (CH output), IN2 low (CHN=0
 *              since complementary with no dead-time and CHN polarity inverted)
 *   value < 0: write CCR = ARR - |value| → IN2 active (CHN output), IN1 low
 *
 * With TIM8 in PWM mode 1, no dead-time, CHN polarity = active-high inverted:
 *   CH  duty = CCR/ARR
 *   CHN duty = 1 - CCR/ARR   (hardware complement)
 *
 * So:
 *   CCR = magnitude       → CH high for `magnitude` counts (IN1), CHN low
 *   CCR = ARR - magnitude → CH low, CHN high for `magnitude` counts (IN2)
 */
static void _set_coil(uint32_t tim_channel, int32_t value)
{
    uint32_t arr = _arr();
    uint32_t half = arr >> 1; /* ARR/2 — the zero-current midpoint */

    /* Clamp value to ±half so CCR stays within [0, ARR] */
    if (value > (int32_t)half)
        value = (int32_t)half;
    if (value < -(int32_t)half)
        value = -(int32_t)half;

    /* No deadband floor needed — zero maps to CCR=half, both sides
     * switching at 50%, so neither IN pin is ever fully tri-stated.  */

    uint32_t ccr = (uint32_t)((int32_t)half + value);
    __HAL_TIM_SET_COMPARE(&STEP_TIM_HANDLE, tim_channel, ccr);
}
void _write_coil_a(int32_t value)
{
    _set_coil(STEP_CH_A, value);
}

void _write_coil_b(int32_t value)
{
    _set_coil(STEP_CH_B, value);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void TB6612_Init(void)
{

    /* Configure CORDIC peripheral */
    __HAL_RCC_CORDIC_CLK_ENABLE();
    LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE, /* cosine function */
                     LL_CORDIC_PRECISION_6CYCLES,       /* max precision for q1.31 cosine */
                     LL_CORDIC_SCALE_0,                 /* no scale */
                     LL_CORDIC_NBWRITE_1,               /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
                     LL_CORDIC_NBREAD_2,                /* Two output data: cosine, then sine */
                     LL_CORDIC_INSIZE_32BITS,           /* q1.31 format for input data */
                     LL_CORDIC_OUTSIZE_32BITS);         /* q1.31 format for output data */

    // testing
    HAL_TIMEx_PWMN_Start(&STEP_TIM_HANDLE, STEP_CH_A);
    HAL_TIMEx_PWMN_Start(&STEP_TIM_HANDLE, STEP_CH_B);
    HAL_TIM_PWM_Start(&STEP_TIM_HANDLE, STEP_CH_A);
    HAL_TIM_PWM_Start(&STEP_TIM_HANDLE, STEP_CH_B);

    _amplitude = (float)STEP_DEFAULT_DUTY / 100.0f;

    _amplitude_raw = _amplitude * (float)(_arr() >> 1);
    _field_angle = 0.0f;

    TB6612_Enable();
    TB6612_SetFieldAngle(0.0f);
}

void TB6612_Enable(void)
{
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_SET);
}

void TB6612_Disable(void)
{
    _write_coil_a(0);
    _write_coil_b(0);
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_RESET);
}
void TB6612_SetAmplitude(float amplitude, int immediate_update)
{
    _amplitude = _clampf(amplitude, 0.0f, 1.0f);
    _amplitude_raw = _amplitude * (float)(_arr() >> 1);
    if (immediate_update)
        TB6612_SetFieldAngle(_field_angle);
}

void TB6612_SetAmplitudeSmooth(float amplitude, uint32_t ramp_time_ms)
{
    amplitude = _clampf(amplitude, 0.0f, 1.0f);
    float start = _amplitude;
    uint32_t step_ms = 10u;
    uint32_t steps = ramp_time_ms / step_ms;
    if (steps == 0u)
        steps = 1u;

    for (uint32_t i = 1u; i <= steps; i++)
    {
        float t = (float)i / (float)steps;
        TB6612_SetAmplitude(start + t * (amplitude - start), 1);
        HAL_Delay(step_ms);
    }
}

float TB6612_GetAmplitude(void)
{
    return _amplitude;
}

void TB6612_SetFieldAngle(float angle_rad)
{
    _field_angle = angle_rad;
    // float sin_a = sinf(angle_rad);
    // float cos_a = cosf(angle_rad);
    float wrapped = fmodf(angle_rad, 2.0f * (float)M_PI);
    if (wrapped > (float)M_PI)
        wrapped -= 2.0f * (float)M_PI;
    if (wrapped < -(float)M_PI)
        wrapped += 2.0f * (float)M_PI;

    int32_t cordic_in = (int32_t)(wrapped * (float)(0x7FFFFFFF) / 3.141592f);
    // int32_t cordic_in = (int32_t)(angle_rad * (float)(0x7FFFFFFF) / M_PI);

    LL_CORDIC_WriteData(CORDIC, cordic_in);
    int32_t cos_raw = LL_CORDIC_ReadData(CORDIC);
    int32_t sin_raw = LL_CORDIC_ReadData(CORDIC);

    float sin_a = (float)sin_raw / (float)(0x7FFFFFFF);
    float cos_a = (float)cos_raw / (float)(0x7FFFFFFF);

    _write_coil_a(_round_i32(sin_a * _amplitude_raw));
    _write_coil_b(_round_i32(cos_a * _amplitude_raw));
}
// ensure angle is between -pi and pi
#define INV_PI_Q31 (2147483648.0f / 3.141592f)
void TB6612_GetCurrentRefs(float angle_rad, float *iref_a, float *iref_b)
{

    int32_t cordic_in = (int32_t)(angle_rad * INV_PI_Q31);

    LL_CORDIC_WriteData(CORDIC, cordic_in);
    CORDIC->WDATA = cordic_in;
    int32_t cos_raw = LL_CORDIC_ReadData(CORDIC);
    int32_t sin_raw = LL_CORDIC_ReadData(CORDIC);

    *iref_a = (float)cos_raw / (float)(0x7FFFFFFF);
    *iref_b = (float)sin_raw / (float)(0x7FFFFFFF);
}

float TB6612_GetFieldAngle(void)
{
    return _field_angle;
}
