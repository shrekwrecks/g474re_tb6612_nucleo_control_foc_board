#include "current_sense.h"

void getADCOffset_Dual(ADC_HandleTypeDef *hadc, uint16_t *adc3_offset, uint16_t *adc4_offset)
{
    uint64_t accum3 = 0;
    uint64_t accum4 = 0;
    uint32_t n = 0;
    uint32_t elapsed = 0;
    uint16_t prev = TIM6->CNT;

    while (elapsed < 480000)
    {
        while (!(hadc->Instance->ISR & ADC_ISR_EOC))
            ;
        uint32_t packed = ADC345_COMMON->CDR;
        accum3 += (uint16_t)(packed & 0xFFFF);
        accum4 += (uint16_t)((packed >> 16) & 0xFFFF);
        n++;
        uint16_t now = TIM6->CNT;
        elapsed += (uint16_t)(now - prev);
        prev = now;
    }

    *adc3_offset = (uint16_t)(accum3 / n);
    *adc4_offset = (uint16_t)(accum4 / n);
}
// should return both adcs packed in adc1 lower 16 bits, adc2 top 16 bits.
__attribute__((weak)) uint32_t current_sense_sample_adcs_for_offset() { return 0xDEADBEEF; }

void getADCOffset_Independent(uint16_t *adc_offset1, uint16_t *adc_offset2)
{
    uint64_t accum1 = 0;
    uint64_t accum2 = 0;
    uint32_t n = 0;
    uint32_t elapsed = 0;
    uint16_t prev = TIM6->CNT;

    while (elapsed < 48000)
    {
        uint32_t packed = current_sense_sample_adcs_for_offset();
        accum1 += packed & 0xFFFF;
        accum2 += (packed >> 16) & 0xFFFF;
        n++;
        uint16_t now = TIM6->CNT;
        elapsed += (uint16_t)(now - prev);
        prev = now;
    }

    *adc_offset1 = (uint16_t)(accum1 / n);
    *adc_offset2 = (uint16_t)(accum2 / n);
}

// void calibrateOpAmp(OPAMP_HandleTypeDef *hopamp)
// {
//     // snapshot entire init struct
//     OPAMP_InitTypeDef saved_init = hopamp->Init;

//     HAL_OPAMP_Stop(hopamp);

//     hopamp->Init.Mode = OPAMP_STANDALONE_MODE;
//     if (HAL_OPAMP_Init(hopamp) != HAL_OK)
//         return;

//     HAL_OPAMP_SelfCalibrate(hopamp);

//     // restore original config but keep the new trim values
//     uint32_t trimN = hopamp->Init.TrimmingValueN;
//     uint32_t trimP = hopamp->Init.TrimmingValueP;

//     hopamp->Init = saved_init;
//     hopamp->Init.TrimmingValueN = trimN;
//     hopamp->Init.TrimmingValueP = trimP;
//     hopamp->Init.UserTrimming = OPAMP_TRIMMING_USER;

//     if (HAL_OPAMP_Init(hopamp) != HAL_OK)
//         return;

//     HAL_OPAMP_Start(hopamp);
// }
