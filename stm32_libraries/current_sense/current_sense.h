#pragma once

#include <stdint.h>
#include "stm32g4xx_hal.h"

#define ADC_RESOLUTION_BITS 12
#define ADC_COUNTS 4095
#define VDD_V 3.3f
#define GAIN 7
#define DAC_CODE 254

uint32_t current_sense_sample_adcs_for_offset();
void calibrateOpAmp(OPAMP_HandleTypeDef *hopamp);
void getADCOffset_Independent(uint16_t *adc3_offset, uint16_t *adc4_offset);
/*
 * OP AMP CONFIG
 *                     x2 of:
 *       VDrive [tb6612 currently]
 *          |
 *     +---------+
 *     |         |
 *    [Q1]      [Q2]
 *     |         |
 *     +----+----+
 *     |         |
 *    [Q3]      [Q4]
 *     |         |
 *     +---------+
 *          |
 *       [shunt]
 *          |
 *         GND
 *
 * Inductor holds current. Sampling at mid point means reads negative current
 *  when complementary channel dominates. Because inductor will pull current through shunt into vbus
 * the inductors current however will not have changed. just the path that the current takes to pass through it
 * true direction of current must be obtained from software.
 *
 * Gain = (Vout_Max - Vout_Min) / (Vin_High - Vin_Low)
 * Vbias = (Vout_Min + G * Vin_High) / (G + 1)
 * when done with external components op amp should be able to withstand -0.8 volts vin. but would hit the rail at -0.2.
 *
 * DAC reference connted to positive input of op amp
 * must be calculated such that desired currents map correctly.
 * must use small enough shunt not to blow shtuff up (negative voltages must not exceed 0.3 volts at gpio)
 * dac reference will fight negative voltage to some extent though and keep it at VRef until op amp plateaus
 *
 */

/* DAC-derived bias */
#define VBIAS_V (DAC_CODE * VDD_V / 4096.0f) /* 0.205 */
#define VMID_V (VBIAS_V * (GAIN + 1))        /*  */

/* Designed boundary points */
#define VOUT_AT_VIN_MAX 0.1f /* Vout when Vin = +0.25V */
#define VOUT_AT_VIN_MIN 3.2f /* Vout when Vin = -0.18V */
#define VIN_MAX_V 0.22f
#define VIN_MIN_V -0.22f

/* Spans for linear remap */
#define VOUT_SPAN (VOUT_AT_VIN_MIN - VOUT_AT_VIN_MAX) /* 3.1V  */
#define VIN_SPAN (VIN_MAX_V - VIN_MIN_V)              /* 0.44V */

/*
 * adc_to_vin() - Convert raw ADC count to input voltage (amps-proxy)
 *
 * @param  adc_raw   Raw 12-bit ADC count (0–4095)
 * @return           Reconstructed Vin in volts (negative = reverse current)
 *
 * Pipeline:
 *   1. ADC count → Vout  (linear, referenced to VDD)
 *   2. Vout → Vin        (two-point inverting remap, no gain/vbias math)
 */
// assuming centered around 0
static inline float adc_to_vin(int16_t adc_raw)
{
    /* Step 1: ADC counts → output voltage at ADC pin */
    float vout = (adc_raw / (float)ADC_COUNTS) * VDD_V;

    /* Step 2: Vout → Vin via two-point inverting linear map
     *   Vin = (Vout - VOUT_AT_VIN_MAX) * -(VIN_SPAN / VOUT_SPAN) + VIN_MAX_V
     *   negative slope because amplifier is inverting
     */
    float vin = vout * -(VIN_SPAN / VOUT_SPAN);

    return vin;
}

/*
 * vin_to_amps() - Scale Vin to current if shunt resistance is known
 *
 * @param  vin_v       Output of adc_to_vin()
 * @param  shunt_ohms  Shunt resistor value in ohms
 * @return             Current in amps (negative = reverse)
 */
static inline float vin_to_amps(float vin_v, float shunt_ohms)
{
    return vin_v / shunt_ohms;
}
