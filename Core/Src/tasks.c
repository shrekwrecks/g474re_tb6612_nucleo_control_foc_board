#include "tasks.h"
#include "mt6835.h"
#include "stm32g4xx_hal_adc.h"
#include "profiling.h"
#include "filters.h"
#include "math.h"
#include "stm32g4xx_ll_cordic.h"

extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc5;

extern CORDIC_HandleTypeDef hcordic;

extern DAC_HandleTypeDef hdac3;
extern DAC_HandleTypeDef hdac4;

extern HRTIM_HandleTypeDef hhrtim1;

extern I2C_HandleTypeDef hi2c1;

extern OPAMP_HandleTypeDef hopamp3;
extern OPAMP_HandleTypeDef hopamp4;

extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart2;

extern PCD_HandleTypeDef hpcd_USB_FS;

#include "tasks.h"
void calibrateOpAmp(OPAMP_HandleTypeDef *hopamp)
{
    // snapshot entire init struct
    OPAMP_InitTypeDef saved_init = hopamp->Init;

    HAL_OPAMP_Stop(hopamp);

    hopamp->Init.Mode = OPAMP_STANDALONE_MODE;
    if (HAL_OPAMP_Init(hopamp) != HAL_OK)
        return;

    HAL_OPAMP_SelfCalibrate(hopamp);

    // restore original config but keep the new trim values
    uint32_t trimN = hopamp->Init.TrimmingValueN;
    uint32_t trimP = hopamp->Init.TrimmingValueP;

    hopamp->Init = saved_init;
    hopamp->Init.TrimmingValueN = trimN;
    hopamp->Init.TrimmingValueP = trimP;
    hopamp->Init.UserTrimming = OPAMP_TRIMMING_USER;

    if (HAL_OPAMP_Init(hopamp) != HAL_OK)
        return;

    HAL_OPAMP_Start(hopamp);
}

uint32_t current_sense_sample_adcs_for_offset()
{
    uint32_t adc3_val = 0;
    uint32_t adc5_val = 0;

    __HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_EOC);
    __HAL_ADC_CLEAR_FLAG(&hadc5, ADC_FLAG_EOC);

    HAL_ADC_Start(&hadc3);
    HAL_ADC_Start(&hadc5);

    // Clear any stale flags first

    while (!__HAL_ADC_GET_FLAG(&hadc3, ADC_FLAG_EOC))
    {
    }

    while (!__HAL_ADC_GET_FLAG(&hadc5, ADC_FLAG_EOC))
    {
    }

    adc3_val = HAL_ADC_GetValue(&hadc3); // also clears EOC on read
    adc5_val = HAL_ADC_GetValue(&hadc5);

    HAL_ADC_Stop(&hadc3);
    HAL_ADC_Stop(&hadc5);

    return adc3_val + (adc5_val << 16);
}
void SWO_Print(const char *str)
{
    while (*str)
        ITM_SendChar(*str++);
}

volatile MT6835_AngleResult encoder_reading;
MT6835_Handle encoder;
uint16_t adc3_offset = 0;
uint16_t adc5_offset = 0;

static void drive_quadrature(int step, int amplitude)
{
    static const int8_t quad_a[4] = {1, 0, -1, 0};
    static const int8_t quad_b[4] = {0, 1, 0, -1};
    _write_coil_a(quad_a[step & 3] * amplitude);
    _write_coil_b(quad_b[step & 3] * amplitude);
}

#define POLE_TABLE_SIZE 201
#define SETTLE_MS 100
uint32_t pole_table[POLE_TABLE_SIZE];

uint32_t offset = 0;
MT6835_Status record_pole_angle(MT6835_Handle *dev, uint32_t table[POLE_TABLE_SIZE])
{
    uint32_t half = _arr() >> 2;
    int amplitude = (int)(half);

    // Drive to step 0 and zero
    drive_quadrature(0, amplitude);
    HAL_Delay(SETTLE_MS * 5);

    // MT6835_Status s = MT6835_SendZeroCommand(dev);
    // if (s != MT6835_OK)
    // return s;

    HAL_Delay(50); // was 5ms — zero needs much longer to propagate

    // Flush stale reads — discard several samples until register settles
    MT6835_AngleResult dummy;
    for (int i = 0; i < 10; i++)
    {
        MT6835_ReadAngle(dev, &dummy);
        HAL_Delay(2);
    }

    // Now record
    for (int step = 0; step < 200; step++)
    {
        drive_quadrature(step, amplitude);
        HAL_Delay(SETTLE_MS);
        table[step] = MT6835_read_angle_avg(dev);
    }

    table[200] = table[0] + (1UL << 21);

    offset = table[0];
    for (int i = 0; i < POLE_TABLE_SIZE; i++)
        table[i] = (table[i] - offset) << 11;

    for (int i = 0; i < POLE_TABLE_SIZE - 1; i++)
    {
        uint32_t dy = table[i + 1] - table[i];
        if (dy == 0 || dy >= 0x80000000UL)
        {
            _write_coil_a(0);
            _write_coil_b(0);
            return MT6835_CAL_FAILED;
        }
    }

    _write_coil_a(0);
    _write_coil_b(0);
    return MT6835_OK;
}
static uint32_t last_idx = 0;
#define NUM_SECTORS 10
uint32_t get_electrical_angle(uint32_t *pole_table, uint32_t raw_angle, uint32_t *elec_angle)
{
    uint32_t angle32 = raw_angle << 11;

    // Map into sector 0-9 to narrow binary search window
    // sector = angle32 * 10 / 2^32, no lookup table needed
    uint32_t sector = (uint32_t)(((uint64_t)angle32 * NUM_SECTORS) >> 32);
    if (sector >= NUM_SECTORS)
        sector = NUM_SECTORS - 1;

    uint32_t entries_per_sector = (POLE_TABLE_SIZE - 1) / NUM_SECTORS;
    uint32_t lo = (sector * entries_per_sector > 2) ? sector * entries_per_sector - 2 : 0;
    uint32_t hi = ((sector + 1) * entries_per_sector + 2 < POLE_TABLE_SIZE - 2)
                      ? (sector + 1) * entries_per_sector + 2
                      : POLE_TABLE_SIZE - 2;

    while (lo < hi)
    {
        uint32_t mid = (lo + hi + 1) >> 1;
        if (angle32 >= pole_table[mid])
            lo = mid;
        else
            hi = mid - 1;
    }

    uint32_t idx = lo;
    last_idx = idx;

    uint32_t y0 = pole_table[idx];
    uint32_t y1 = pole_table[idx + 1];
    uint32_t dy = y1 - y0;
    uint32_t dx = angle32 - y0;

    uint32_t frac_q32;
    if (dy == 0)
        frac_q32 = 0;
    else if (dx >= dy)
        frac_q32 = 0xFFFFFFFFu;
    else
        frac_q32 = (uint32_t)(((uint64_t)dx << 32) / dy);

    uint32_t elec_idx = idx % 4;
    *elec_angle = (elec_idx << 30) + (frac_q32 >> 2);

    return idx;
}
void doSamplingTask()
{
    int step = 0;
    const int amplitude = 20;

    for (;;)
    {
        drive_quadrature(step, amplitude);

        step = (step + 1) & 3; // 0,1,2,3,0,...

        osDelay(3);
    }
}

pi_controller_t pi_controller_q;
pi_controller_t pi_controller_d;
alpha_beta_filter_t pos_filt;
volatile uint32_t adc_trace_word;
volatile uint32_t enc_trace_word;
void doHardwareInit()
{
    HAL_TIM_Base_Start(&htim6);
    MT6835_Init(&encoder, &hspi3, SPI3_CS_GPIO_Port, SPI3_CS_Pin, &htim6);
    MT6835_LowLevelInit(&encoder, &encoder_reading,
                        DMA2, LL_DMA_CHANNEL_1, LL_DMA_CHANNEL_2);

    MT6835_ReadAngle(&encoder, &encoder_reading);

    alpha_beta_filter_set(&pos_filt, encoder_reading.raw, 0, 0);
    alpha_beta_filter_init(&pos_filt, 0.0f, 0.0f, 16);
    alpha_beta_filter_set_alpha(&pos_filt, 0.5f);

    // get initial angle for filter
    TB6612_Init();

    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);

    HAL_DAC_SetValue(&hdac4, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_CODE);
    HAL_DAC_Start(&hdac4, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_CODE);
    HAL_DAC_Start(&hdac3, DAC_CHANNEL_2);

    // HAL_OPAMP_Start(&hopamp3);
    // HAL_OPAMP_Start(&hopamp4);

    calibrateOpAmp(&hopamp3);
    calibrateOpAmp(&hopamp4);

    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Init(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);

    getADCOffset_Independent(&adc3_offset, &adc5_offset);

    HAL_TIM_Base_Stop(&htim8);

    pi_controller_init(&pi_controller_q);
    pi_controller_init(&pi_controller_d);

    pi_controller_set_gains(&pi_controller_q, 0.0033f, 3.0f, 1000.0f);
    pi_controller_set_gains(&pi_controller_d, 0.0033f, 3.0f, 1000.0f);

    HAL_ADC_Stop(&hadc3);
    HAL_ADC_Stop(&hadc5);

    record_pole_angle(&encoder, pole_table);

    HAL_ADC_Start(&hadc3);
    HAL_ADC_Start_IT(&hadc5);
    HAL_TIM_Base_Start(&htim8);

    // MT6835_NONBLOCKING_CALIBRATE(&encoder);
}

uint8_t try_SWO_Send(uint8_t portnum, uint32_t data)
{
    if (ITM->PORT[portnum].u32 == 0UL)
    {
        return 0;
    }
    ITM->PORT[portnum].u32 = data;
    return 1;
}

uint8_t SWO_Blocking_Send(uint8_t portnum, uint32_t data)
{
    while (ITM->PORT[portnum].u32 == 0UL)
    {
    }
    ITM->PORT[portnum].u32 = data;
    return 1;
}
uint32_t elec_angle;

#define ENCODER_BITS 21
#define ENCODER_SHIFT_TO_32_BIT (32 - ENCODER_BITS) // this lets you take advantage of uint wrapping to do angle unwrapping :P
#define ENC_COUNTS (1 << ENCODER_BITS)

#define SHUNT_CONDUCTANCE 10.0f
#define M_PI_F (3.14159265f)
#define Q31_TO_RADIANS (2.0f * M_PI_F / 4294967296.0f) // 6.28 over 1<<32
static int64_t angle_accum = 0;
static uint32_t prev_raw = 0;

float iref_scale = 1.0;
float inertial_force = 0.0f;
static float do_pos_loop(int64_t int_pos_est, int64_t int_vel_est, int64_t int_acc_est)
{
    static float stiffness = 1.0f / (float)(1u << 21);
    static float mass = -0.0f;
    static float damping = 1.0f;

    uint32_t desired_pos = 0;
    uint32_t desired_pos_shifted = desired_pos;

    int32_t pos_error = (int32_t)(int_pos_est - desired_pos_shifted);
    float p_term = -stiffness * (float)pos_error;
    inertial_force = -(mass * int_acc_est) / (65000.0f * 10000.0f);
    // inertial_force = 0.0;
    float damping_force = damping * int_vel_est;

    float iref = _clampf(p_term + inertial_force, -iref_scale, iref_scale);

    return iref;
}

int64_t global_est = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

    uint32_t centered_reading = encoder_reading.raw - offset;
    uint32_t rotor_raw = (centered_reading << ENCODER_SHIFT_TO_32_BIT);
    int32_t delta = ((int32_t)(rotor_raw - prev_raw)) >> ENCODER_SHIFT_TO_32_BIT;
    prev_raw = rotor_raw;
    angle_accum += delta;

    alpha_beta_filter_update(&pos_filt, angle_accum); // 1us

    int64_t int_pos_est = alpha_beta_filter_get_x(&pos_filt);
    int64_t int_vel_est = alpha_beta_filter_get_x_dot(&pos_filt);
    int64_t int_acc_est = alpha_beta_filter_get_x_dot_dot(&pos_filt);

    uint8_t ts = TIM6->CNT & 0xFF;
    PROFILE1_SET();

    while (!__HAL_ADC_GET_FLAG(&hadc3, ADC_FLAG_EOC))
    {
    }

    uint16_t adc3_raw = (uint16_t)hadc3.Instance->DR;
    uint16_t adc5_raw = (uint16_t)hadc5.Instance->DR;

    int16_t adc3_val = (int16_t)adc3_raw - adc3_offset;
    int16_t adc5_val = (int16_t)adc5_raw - adc5_offset;

    float current_shunt2 = (adc_to_vin(adc3_val) * 10.0f);
    float current_shunt1 = (adc_to_vin(adc5_val) * 10.0f);
    while (!MT6835_DMA_Ready())
    {
    }
    // uint32_t reading = ((encoder_reading.raw) - (offset)) & 0x1FFFFF;
    MT6835_StartDMA();

    // float f_elec_angle = elec_angle * Q31_TO_RADIANS;

    uint32_t idx = get_electrical_angle(pole_table, int_pos_est, &elec_angle); // 1.2us

    LL_CORDIC_WriteData(CORDIC, elec_angle);

    // could technically pipeline it. but doesnt actually save any time.

    int32_t cos_raw = LL_CORDIC_ReadData(CORDIC);
    int32_t sin_raw = LL_CORDIC_ReadData(CORDIC);
    // float cos_t = cosf(f_elec_angle);
    // float sin_t = sinf(f_elec_angle);
    float cos_t = (float)cos_raw / (float)(0x7FFFFFFF);
    float sin_t = (float)sin_raw / (float)(0x7FFFFFFF);
    //----------------

    static int64_t prev_accum_vel = 0;
    static int64_t prev_accum_pos = 0;

    volatile int64_t acc_raw = int_acc_est; // raw ticks/sample²

    prev_accum_pos = angle_accum;

#define LPF_ALPHA 0.0002f
    static float acc_filt_f = 0.0f;
    acc_filt_f = LPF_ALPHA * (float)acc_raw + (1.0f - LPF_ALPHA) * acc_filt_f;
    int64_t acc_filt = (int64_t)(acc_filt_f * 10000);
    // float i_q_ref = do_pos_loop(int_pos_est, acc_filt);

    //--------------
    float i_d_ref = -0.0f;
    float i_q_ref = do_pos_loop(int_pos_est, 0, acc_filt);

    // fwd parke
    float i_d = current_shunt1 * cos_t + current_shunt2 * sin_t;
    float i_q = current_shunt1 * -sin_t + current_shunt2 * cos_t;

    // error correction in parke
    float err_d = i_d_ref - i_d;
    float err_q = i_q_ref - i_q;

    float v_q = pi_controller_update(&pi_controller_q, err_q) / (4.0f); // vbus, must normalize to volts from -1 to 1
    float v_d = pi_controller_update(&pi_controller_d, err_d) / (4.0f);

    // voltage limiting in dq frame - circle limit is better than box
    float v_mag = sqrtf(v_d * v_d + v_q * v_q);

    float v_limit = 0.9f;
    if (v_mag > v_limit)
    {
        float scale = v_limit / v_mag;
        v_d *= scale;
        v_q *= scale;
    }

    // reverse parke

    float out_a = (v_d * cos_t - v_q * sin_t);
    float out_b = (v_d * sin_t + v_q * cos_t);

    // set a and b currents

    static uint32_t print_count = 0;
    if (++print_count >= 500) // print every 500 samples = 100Hz
    {
        print_count = 0;
        char buf[32];
        snprintf(buf, sizeof(buf), "%ld\r\n", (int32_t)(i_d * 1000));
        SWO_Print(buf);
    }

    uint32_t half = _arr() >> 1;

    int32_t outa_raw = _round_i32((out_a)*half);
    int32_t outb_raw = _round_i32((out_b)*half);
    _write_coil_a(outa_raw);
    _write_coil_b(outb_raw);

#define ACC_BIAS (1 << 20)      // center of 21-bit field
#define ACC_MAX ((1 << 21) - 1) // 21-bit max

    int32_t acc_biased = (int32_t)acc_filt * 1000 + ACC_BIAS;
    if (acc_biased < 0)
        acc_biased = 0;
    if (acc_biased > ACC_MAX)
        acc_biased = ACC_MAX;

    uint32_t enc_reading_packed = (uint32_t)ts | ((uint32_t)acc_biased << 8);
    try_SWO_Send(2, enc_reading_packed);

    uint32_t swo_adc_packed = (uint32_t)ts | (((uint32_t)(uint16_t)adc3_val & 0xFFF) << 8) | (((uint32_t)(uint16_t)adc5_val & 0xFFF) << 20);
    adc_trace_word = swo_adc_packed;

    // try_SWO_Send(1, adc_trace_word);

    PROFILE1_RESET();
}