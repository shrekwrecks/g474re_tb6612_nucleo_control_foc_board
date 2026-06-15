#pragma once

#include "main.h"
#include "stm32g4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "lib_usbpd.h"
#include "mt6835.h"
#include "tb6612.h"
#include "current_sense.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

void doHardwareInit();
void doSamplingTask();