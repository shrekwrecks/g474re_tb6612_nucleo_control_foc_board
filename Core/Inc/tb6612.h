#pragma once

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Hardware definitions                                               */
/* ------------------------------------------------------------------ */
#define STEP_TIM TIM8
#define STEP_TIM_HANDLE htim8

#define STEP_CH_A TIM_CHANNEL_1 /* TIM8_CH1/CH1N → AIN1/AIN2 */
#define STEP_CH_B TIM_CHANNEL_2 /* TIM8_CH2/CH2N → BIN1/BIN2 */

#define ENA_Pin GPIO_PIN_2
#define ENA_GPIO_Port GPIOC
#define ENB_Pin GPIO_PIN_3
#define ENB_GPIO_Port GPIOC
#define STBY_Pin GPIO_PIN_5
#define STBY_GPIO_Port GPIOB

#define STEP_DEFAULT_DUTY 30u

/* ------------------------------------------------------------------ */
/*  External HAL handle                                                */
/* ------------------------------------------------------------------ */
extern TIM_HandleTypeDef STEP_TIM_HANDLE;

/* ------------------------------------------------------------------ */
/*  Optional per-iteration callback (e.g. encoder poll)               */
/* ------------------------------------------------------------------ */
typedef void (*StepCallback)(void);

/* ------------------------------------------------------------------ */
/*  API                                                                 */
/* ------------------------------------------------------------------ */
void TB6612_Init(void);
void TB6612_Enable(void);
void TB6612_Disable(void);

void TB6612_SetFieldAngle(float angle_rad);
float TB6612_GetFieldAngle(void);

void TB6612_SetAmplitude(float amplitude, int immediate_update);
void TB6612_SetAmplitudeSmooth(float amplitude, uint32_t ramp_time_ms);
float TB6612_GetAmplitude(void);
void TB6612_GetCurrentRefs(float angle_rad, float *iref_a, float *iref_b);

void _write_coil_a(int32_t value);
void _write_coil_b(int32_t value);
int32_t _round_i32(float v);
uint32_t _arr(void);
float _clampf(float v, float lo, float hi);
