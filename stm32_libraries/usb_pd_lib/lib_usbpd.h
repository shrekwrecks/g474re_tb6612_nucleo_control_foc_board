/**
 * lib_usbpd.h — USB PD Library
 *
 * ┌─────────────────────────────────────────────────────────┐
 *  SETUP — two steps per new project:
 *
 *  1. In usbpd_dpm_user.c, inside USER CODE BEGIN Includes:
 *       #include "usbpd_sink.h"
 *
 *  2. Copy the forwarding calls below into each DPM function
 *     (see "DROP INTO DPM" comments in this file).
 *     Paste them inside the USER CODE BEGIN / END blocks so
 *     CubeMX code generation does not overwrite them.
 *
 *  3. VBUS voltage: BSP_USBPD_PWR_VBUSGetVoltage is declared
 *     __weak by ST. Override it in any .c file in your project
 *     (e.g. main.c or a bsp file) — see usbpd_sink.c for a
 *     stub that always returns 5000 mV. If you have a real ADC
 *     measurement, replace the stub body with your read.
 *
 *  4. Call SINK_Init() before osKernelStart().
 * └─────────────────────────────────────────────────────────┘
 */

#pragma once
#include <stdint.h>
#include "usbpd_def.h"
#include "usbpd_dpm_user.h"
#include "usbpd_dpm_core.h"
#include "usbpd_core.h"

// ════════════════════════════════════════════════════════════
//  GENERAL API — no middleware dependencies
//  Include this in any .c file that needs sink status.
// ════════════════════════════════════════════════════════════

typedef enum
{
    SINK_MODE_NONE,
    SINK_MODE_FIXED,
    SINK_MODE_PPS,
} SINK_Mode_t;

typedef struct
{
    SINK_Mode_t mode;
    uint8_t pdo_index;
    uint32_t voltage_mV;
    uint32_t current_mA;
    uint8_t capability_mismatch; // only meaningful for fixed
} SINK_Selection_t;

typedef struct
{
    uint8_t index;
    uint8_t type;        // PDO_TYPE_FIXED or PDO_TYPE_APDO
    uint8_t apdo_type;   // APDO_TYPE_PPS, only valid if type == PDO_TYPE_APDO
    uint32_t voltage_mV; // fixed: voltage, PPS: 0
    uint32_t min_mV;     // PPS only
    uint32_t max_mV;     // PPS only
    uint32_t max_mA;
} SINK_PDOInfo_t;

typedef struct
{
    SINK_PDOInfo_t pdos[8];
    uint32_t count;
} SINK_PDOList_t;

void SINK_GetPDOList(SINK_PDOList_t *list);
void SINK_FormatPDOList(const SINK_PDOList_t *list, char *buf, uint32_t buf_size);

// taken from st.

typedef struct
{
    uint32_t DPM_RDOPosition;         /*!< RDO Position of requested DO in Source list of capabilities          */
    uint32_t DPM_RDOPositionPrevious; /*!< RDO Position of requested DO in Source list of capabilities          */
    uint32_t DPM_RequestedVoltage;    /*!< Value of requested voltage                                           */
    uint32_t DPM_RequestDOMsg;
    uint32_t DPM_NumberOfRcvSRCPDO;
    uint32_t DPM_ListOfRcvSRCPDO[8];
} USBPD_HandleTypeDef;

extern USBPD_ParamsTypeDef DPM_Params[];

extern USBPD_HandleTypeDef DPM_Ports[];

//

void SINK_Init(SINK_Selection_t request);

/** Returns 1 after an explicit PD contract has been established. */
uint8_t SINK_IsContractActive(void);

/** Returns the voltage actually negotiated, in mV. 0  before contract. */
uint32_t SINK_GetNegotiatedVoltage_mV(void);

/** Returns 1 if the connected source advertises at least one PPS APDO. */
uint8_t SINK_SourceHasPPS(void);

/** Returns 1 if the connected source advertises at least one EPR AVS APDO. */
uint8_t SINK_SourceHasEPR(void);

// ════════════════════════════════════════════════════════════
//  INTERNAL — called by the ST glue below, not by your app
// ════════════════════════════════════════════════════════════

void SINK_OnRcvSrcPDO(uint8_t *ptr, uint32_t size);
SINK_Selection_t SINK_EvaluateRequest(const uint32_t *pdos, uint32_t pdo_count);

void SINK_SetContractActive(void);
USBPD_StatusTypeDef SINK_PPSRequest(uint32_t voltage_mV);

// ════════════════════════════════════════════════════════════
//  ST GLUE FUNCTIONS
//  Rename avoids duplicate-symbol conflict with the CubeMX-
//  generated non-weak versions of the same DPM functions.
//
//  DROP INTO usbpd_dpm_user.c — one call per function body,
//  inside the USER CODE BEGIN / USER CODE END guards:
//
//  void USBPD_DPM_SetDataInfo(...) {
//    /* USER CODE BEGIN USBPD_DPM_SetDataInfo */
//    SINK_ST_SetDataInfo(PortNum, (uint32_t)DataId, Ptr, Size);
//    /* USER CODE END USBPD_DPM_SetDataInfo */
//  }
//
//  void USBPD_DPM_SNK_EvaluateCapabilities(...) {
//    /* USER CODE BEGIN USBPD_DPM_SNK_EvaluateCapabilities */
//    SINK_ST_EvaluateCapabilities(PortNum, PtrRequestData,
//                                 (uint32_t *)PtrPowerObjectType);
//    /* USER CODE END USBPD_DPM_SNK_EvaluateCapabilities */
//  }
//
//  void USBPD_DPM_Notification(...) {
//    /* USER CODE BEGIN USBPD_DPM_Notification */
//    SINK_ST_Notification(PortNum, (uint32_t)EventVal);
//    /* USER CODE END USBPD_DPM_Notification */
//  }
//
//  VBUS override — add to any .c file (e.g. main.c):
//  The __weak stub in usbpd_sink.c returns 5000 mV always.
//  If you have a real ADC measurement, override it instead:
//
//  int32_t BSP_USBPD_PWR_VBUSGetVoltage(uint32_t Instance,
//                                        uint32_t *pVoltage) {
//    *pVoltage = MY_ADC_ReadVBUS_mV();
//    return 0;
//  }
// ════════════════════════════════════════════════════════════

void SINK_ST_SetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId,
                         uint8_t *Ptr, uint32_t Size);

void SINK_ST_EvaluateCapabilities(uint8_t PortNum,
                                  uint32_t *PtrRequestData,
                                  USBPD_CORE_PDO_Type_TypeDef *PtrPowerObjectType);

void SINK_ST_Notification(uint8_t PortNum, uint32_t EventVal);