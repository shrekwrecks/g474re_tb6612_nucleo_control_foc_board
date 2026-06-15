/**
 * lib_usbpd.c — Lightweight USB PD Sink Library
 *
 * ST middleware headers are included here only — not in the
 * public .h — so your app code stays middleware-free.
 */

#include "lib_usbpd.h"
#include <string.h>
#include <stdio.h>

// ════════════════════════════════════════════════════════════
//  ST middleware includes — isolated to this file
// ════════════════════════════════════════════════════════════

// ════════════════════════════════════════════════════════════
//  Private state
// ════════════════════════════════════════════════════════════

static SINK_Selection_t _request;
static uint32_t _srcPDOs[8];
static uint32_t _srcPDOCount;
static uint8_t _contractActive;
static uint32_t _negotiatedVoltage;
static uint8_t _ppsPDOIndex;

USBPD_HandleTypeDef DPM_Ports[USBPD_PORT_COUNT];

// PDO bit field helpers
#define PDO_TYPE(pdo) (((pdo) >> 30) & 0x3U)
#define PDO_APDO_TYPE(pdo) (((pdo) >> 28) & 0x3U)
#define PDO_TYPE_FIXED 0x0U
#define PDO_TYPE_APDO 0x3U
#define APDO_TYPE_PPS 0x0U
#define APDO_TYPE_EPR 0x1U

// ════════════════════════════════════════════════════════════
//  General API
// ════════════════════════════════════════════════════════════

void SINK_Init(SINK_Selection_t request)
{
    _request = request;
    _contractActive = 0;
    _negotiatedVoltage = 0;
    memset(_srcPDOs, 0, sizeof(_srcPDOs));
    _srcPDOCount = 0;
}

uint8_t SINK_SelectPDO(const uint32_t *pdos,
                       uint32_t pdo_count,
                       uint32_t requested_voltage_mv)
{
    for (uint32_t i = 0; i < pdo_count; i++)
    {
        uint32_t pdo = pdos[i];

        if (PDO_TYPE(pdo) != PDO_TYPE_FIXED)
            continue;

        uint32_t voltage_mv =
            ((pdo >> 10) & 0x3FFU) * 50U;

        if (voltage_mv == requested_voltage_mv)
            return i + 1; // PDO positions are 1-based
    }

    return 0;
}

void SINK_OnRcvSrcPDO(uint8_t *ptr, uint32_t size)
{
    _srcPDOCount = size / 4;
    for (uint32_t i = 0; i < _srcPDOCount; i++)
        memcpy(&_srcPDOs[i], ptr + (i * 4), 4);
}

SINK_Selection_t SINK_EvaluateRequest(const uint32_t *pdos, uint32_t pdo_count)
{
    SINK_Selection_t sel = {0};
    sel.mode = SINK_MODE_NONE;

    if (_request.mode == SINK_MODE_PPS)
    {
        for (uint32_t i = 0; i < pdo_count; i++)
        {
            uint32_t pdo = pdos[i];
            if (PDO_TYPE(pdo) != PDO_TYPE_APDO)
                continue;
            if (PDO_APDO_TYPE(pdo) != APDO_TYPE_PPS)
                continue;

            uint32_t maxV = ((pdo >> 17) & 0xFFU) * 100;
            uint32_t minV = ((pdo >> 8) & 0xFFU) * 100;

            if (_request.voltage_mV >= minV && _request.voltage_mV <= maxV)
            {
                sel.mode = SINK_MODE_PPS;
                sel.pdo_index = i + 1;
                sel.voltage_mV = _request.voltage_mV;
                sel.current_mA = _request.current_mA;
                _negotiatedVoltage = sel.voltage_mV;
                // save
                _ppsPDOIndex = sel.pdo_index;
                return sel;
            }
        }
        // PPS not found, fall through to fixed
    }

    for (uint32_t i = 0; i < pdo_count; i++)
    {
        uint32_t pdo = pdos[i];
        if (PDO_TYPE(pdo) != PDO_TYPE_FIXED)
            continue;

        uint32_t voltage = ((pdo >> 10) & 0x3FFU) * 50;
        if (voltage != _request.voltage_mV)
            continue;

        uint32_t src_max_mA = ((pdo >> 0) & 0x3FFU) * 10;
        uint32_t actual_mA = (_request.current_mA > src_max_mA)
                                 ? src_max_mA
                                 : _request.current_mA;

        sel.mode = SINK_MODE_FIXED;
        sel.pdo_index = i + 1;
        sel.voltage_mV = voltage;
        sel.current_mA = actual_mA;
        sel.capability_mismatch = (_request.current_mA > src_max_mA) ? 1 : 0;
        _negotiatedVoltage = voltage;
        return sel;
    }

    // fallback — 5V PDO 1
    sel.mode = SINK_MODE_FIXED;
    sel.pdo_index = 1;
    sel.voltage_mV = 5000;
    sel.current_mA = 500;
    _negotiatedVoltage = 5000;
    return sel;
}

void SINK_SetContractActive(void) { _contractActive = 1; }
uint8_t SINK_IsContractActive(void) { return _contractActive; }
uint32_t SINK_GetNegotiatedVoltage_mV(void) { return _negotiatedVoltage; }

uint8_t SINK_SourceHasPPS(void)
{
    for (uint32_t i = 0; i < _srcPDOCount; i++)
        if (PDO_TYPE(_srcPDOs[i]) == PDO_TYPE_APDO &&
            PDO_APDO_TYPE(_srcPDOs[i]) == APDO_TYPE_PPS)
            return 1;
    return 0;
}

uint8_t SINK_SourceHasEPR(void)
{
    for (uint32_t i = 0; i < _srcPDOCount; i++)
        if (PDO_TYPE(_srcPDOs[i]) == PDO_TYPE_APDO &&
            PDO_APDO_TYPE(_srcPDOs[i]) == APDO_TYPE_EPR)
            return 1;
    return 0;
}

USBPD_StatusTypeDef SINK_PPSRequest(uint32_t voltage_mV)
{
    _request.voltage_mV = voltage_mV;

    USBPD_SNKRDO_TypeDef rdo = {0};
    rdo.ProgRDO.ObjectPosition = _ppsPDOIndex;
    rdo.ProgRDO.OutputVoltageIn20mV = voltage_mV / 20;
    rdo.ProgRDO.OperatingCurrentIn50mAunits = _request.current_mA / 50;
    rdo.ProgRDO.NoUSBSuspend = 1;

    return USBPD_PE_Send_Request(0, rdo.d32, USBPD_CORE_PDO_TYPE_APDO);
}

void SINK_GetPDOList(SINK_PDOList_t *list)
{
    list->count = _srcPDOCount;
    for (uint32_t i = 0; i < _srcPDOCount; i++)
    {
        uint32_t pdo = _srcPDOs[i];
        SINK_PDOInfo_t *info = &list->pdos[i];
        info->index = i + 1;
        info->type = PDO_TYPE(pdo);
        info->apdo_type = 0;
        info->voltage_mV = 0;
        info->min_mV = 0;
        info->max_mV = 0;
        info->max_mA = 0;

        if (info->type == PDO_TYPE_FIXED)
        {
            info->voltage_mV = ((pdo >> 10) & 0x3FFU) * 50;
            info->max_mA = ((pdo >> 0) & 0x3FFU) * 10;
        }
        else if (info->type == PDO_TYPE_APDO)
        {
            info->apdo_type = PDO_APDO_TYPE(pdo);
            if (info->apdo_type == APDO_TYPE_PPS)
            {
                info->max_mV = ((pdo >> 17) & 0xFFU) * 100;
                info->min_mV = ((pdo >> 8) & 0xFFU) * 100;
                info->max_mA = ((pdo >> 0) & 0x7FU) * 50;
            }
        }
    }
}

void SINK_FormatPDOList(const SINK_PDOList_t *list, char *buf, uint32_t buf_size)
{
    uint32_t pos = 0;

    pos += snprintf(buf + pos, buf_size - pos, "PDO count: %lu\r\n", list->count);

    for (uint32_t i = 0; i < list->count; i++)
    {
        const SINK_PDOInfo_t *p = &list->pdos[i];

        if (p->type == PDO_TYPE_FIXED)
        {
            pos += snprintf(buf + pos, buf_size - pos,
                            "[%u] FIXED: %lu mV, max %lu mA\r\n",
                            p->index, p->voltage_mV, p->max_mA);
        }
        else if (p->type == PDO_TYPE_APDO && p->apdo_type == APDO_TYPE_PPS)
        {
            pos += snprintf(buf + pos, buf_size - pos,
                            "[%u] PPS: %lu mV to %lu mV, max %lu mA\r\n",
                            p->index, p->min_mV, p->max_mV, p->max_mA);
        }
        else
        {
            pos += snprintf(buf + pos, buf_size - pos,
                            "[%u] unknown type %u\r\n",
                            p->index, p->type);
        }
    }
}

// ════════════════════════════════════════════════════════════
//  ST glue — thin wrappers called from usbpd_dpm_user.c
// ════════════════════════════════════════════════════════════

void SINK_ST_SetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId,
                         uint8_t *Ptr, uint32_t Size)
{
    (void)PortNum;
    switch (DataId)
    {
    case USBPD_CORE_DATATYPE_RCV_SRC_PDO:
        if (Size <= (USBPD_MAX_NB_PDO * 4))
        {
            // middleware bookkeeping
            DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO = (Size / 4);
            for (uint32_t i = 0; i < (Size / 4); i++)
            {
                uint8_t *rdo = (uint8_t *)&DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO[i];
                (void)memcpy(rdo, (Ptr + (i * 4u)), (4u * sizeof(uint8_t)));
            }

            // library logic
            SINK_OnRcvSrcPDO(Ptr, Size);
        }
        break;
    //  case USBPD_CORE_DATATYPE_RCV_SNK_PDO:       /*!< Storage of Received Sink PDO values          */
    // break;
    //  case USBPD_CORE_EXTENDED_CAPA:              /*!< Source Extended capability message content   */
    // break;
    //  case USBPD_CORE_PPS_STATUS:                 /*!< PPS Status message content                   */
    // break;
    //  case USBPD_CORE_INFO_STATUS:                /*!< Information status message content           */
    // break;
    //  case USBPD_CORE_ALERT:                      /*!< Storing of received Alert message content    */
    // break;
    //  case USBPD_CORE_GET_MANUFACTURER_INFO:      /*!< Storing of received Get Manufacturer info message content */
    // break;
    //  case USBPD_CORE_GET_BATTERY_STATUS:         /*!< Storing of received Get Battery status message content    */
    // break;
    //  case USBPD_CORE_GET_BATTERY_CAPABILITY:     /*!< Storing of received Get Battery capability message content*/
    // break;
    //  case USBPD_CORE_SNK_EXTENDED_CAPA:          /*!< Storing of Sink Extended capability message content       */
    // break;
    default:
        break;
    }
}
void SINK_ST_EvaluateCapabilities(uint8_t PortNum,
                                  uint32_t *PtrRequestData,
                                  USBPD_CORE_PDO_Type_TypeDef *PtrPowerObjectType)
{
    SINK_Selection_t sel = SINK_EvaluateRequest(
        DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO,
        DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO);

    USBPD_SNKRDO_TypeDef rdo = {0};

    if (sel.mode == SINK_MODE_PPS)
    {
        rdo.ProgRDO.ObjectPosition = sel.pdo_index;
        rdo.ProgRDO.OutputVoltageIn20mV = sel.voltage_mV / 20;
        rdo.ProgRDO.OperatingCurrentIn50mAunits = sel.current_mA / 50;
        rdo.ProgRDO.NoUSBSuspend = 1;
        *PtrPowerObjectType = USBPD_CORE_PDO_TYPE_APDO;
    }
    else
    {
        rdo.FixedVariableRDO.ObjectPosition = sel.pdo_index;
        rdo.FixedVariableRDO.OperatingCurrentIn10mAunits = sel.current_mA / 10;
        rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits = sel.current_mA / 10;
        rdo.FixedVariableRDO.CapabilityMismatch = sel.capability_mismatch;
        rdo.FixedVariableRDO.USBCommunicationsCapable = 0;
        rdo.FixedVariableRDO.NoUSBSuspend = 1;
        *PtrPowerObjectType = USBPD_CORE_PDO_TYPE_FIXED;
    }

    *PtrRequestData = rdo.d32;
    DPM_Ports[PortNum].DPM_RequestDOMsg = rdo.d32;
    DPM_Ports[PortNum].DPM_RequestedVoltage = sel.voltage_mV;
}
void SINK_ST_Notification(uint8_t PortNum, uint32_t EventVal)
{
    (void)PortNum;
    if (EventVal == (uint32_t)USBPD_NOTIFY_POWER_EXPLICIT_CONTRACT)
        SINK_SetContractActive();
}

// ════════════════════════════════════════════════════════════
//  VBUS voltage stub
//
//  ST declares BSP_USBPD_PWR_VBUSGetVoltage as __weak so this
//  definition will be used.
// ════════════════════════════════════════════════════════════

int32_t BSP_USBPD_PWR_VBUSGetVoltage(uint32_t Instance,
                                     uint32_t *pVoltage)
{
    (void)Instance;
    *pVoltage = 5000;
    return 0;
}