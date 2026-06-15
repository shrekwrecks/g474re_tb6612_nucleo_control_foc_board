#pragma once
#include "main.h"

#define PROFILE1_SET() (PROFILE1_GPIO_Port->BSRR = PROFILE1_Pin)
#define PROFILE1_RESET() (PROFILE1_GPIO_Port->BSRR = ((uint32_t)PROFILE1_Pin << 16))
#define PROFILE2_SET() (PROFILE2_GPIO_Port->BSRR = PROFILE2_Pin)
#define PROFILE2_RESET() (PROFILE2_GPIO_Port->BSRR = ((uint32_t)PROFILE2_Pin << 16))