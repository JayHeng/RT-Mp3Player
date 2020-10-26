/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_pd_config.h"
#include "usb_pd.h"
#include "string.h"
#include "pd_board_config.h"
#include "usb_pd_alt_mode.h"
#include "usb_pd_alt_mode_dp.h"
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
#include "fsl_debug_console.h"
#endif

#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
#if (defined PD_CONFIG_ALT_MODE_DP_SUPPORT) && (PD_CONFIG_ALT_MODE_DP_SUPPORT)
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define HPD_DETECT_QUEUE_LEN 4

typedef struct _pd_alt_mode_displayport
{
    pd_handle pdHandle;
    void *altModeHandle;
    void *dpBoardChipHandle;
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
    pd_alt_mode_dp_host_config_t *dpHostConfig;
#endif
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
    pd_alt_mode_dp_slave_config_t *dpSlaveConfig;
#endif

    uint32_t taskEvent;
    uint32_t hpdTime;
    uint32_t pdMsgBuffer[7];
    uint32_t pdMsgReceivedBuffer[7];
    volatile uint32_t delayTime;
    volatile uint32_t delayEvents;
    pd_dp_status_obj_t dpSelfStatus;
    pd_dp_status_obj_t dpPartnerStatus;
    pd_dp_configure_obj_t dpConfigure;
    pd_structured_vdm_header_t pdVDMMsgReceivedHeader;
    volatile uint32_t retryCount;
    volatile uint8_t retryCommand;
    uint8_t pdMsgReceivedVDOCount;
    uint8_t selectModeIndex;
    uint8_t occupied;
    uint8_t dpState; /* pd_dp_state_t */
    uint8_t triggerCommand;
    volatile uint8_t dpCommand;
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
    volatile pd_hpd_detect_type_t attnStatusHPD;
    volatile uint8_t hpdDetectQueueLength;
    volatile uint8_t hpdDetectQueueGetPos;
    volatile uint8_t hpdDetectQueuePutPos;
    volatile pd_hpd_detect_type_t hpdDetectQueue[HPD_DETECT_QUEUE_LEN];
#endif

    uint8_t waitSendResult : 1;
    uint8_t CommandDoing : 1;
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
    volatile uint8_t hpdDetectEnable : 1;
#endif
} pd_alt_mode_dp_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void PD_DpmAltModeCallback(pd_handle pdHandle, uint32_t event, void *param);

/*******************************************************************************
 * Variables
 ******************************************************************************/

pd_alt_mode_dp_t s_AltModeDisplayPortInstance[PD_CONFIG_MAX_PORT];

/*******************************************************************************
 * Code
 ******************************************************************************/

static void PD_DpDelayRetryCommand(pd_alt_mode_dp_t *dpInstance, uint8_t command, uint32_t delay)
{
    dpInstance->delayTime = delay;
}

/* DP DFP and UFP */
static pd_status_t PD_DpSendCommand(pd_alt_mode_dp_t *dpInstance, uint8_t command)
{
    pd_svdm_command_param_t structuredVDMCommandParam;
    uint32_t vdmCommand = 0;

    if (dpInstance->CommandDoing)
    {
        return kStatus_PD_Error;
    }
    structuredVDMCommandParam.vdmSop                          = kPD_MsgSOP;
    structuredVDMCommandParam.vdmHeader.bitFields.SVID        = DP_SVID;
    structuredVDMCommandParam.vdmHeader.bitFields.vdmType     = 1;
    structuredVDMCommandParam.vdmHeader.bitFields.objPos      = 0;
    structuredVDMCommandParam.vdmHeader.bitFields.commandType = kVDM_Initiator;

    switch (command)
    {
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
        case kVDM_DiscoverModes:
            if (dpInstance->dpState != kDPMode_Exited)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 0;
            structuredVDMCommandParam.vdoData  = NULL;
            vdmCommand                         = PD_DPM_CONTROL_DISCOVERY_MODES;
            break;

        case kVDM_EnterMode:
            if (dpInstance->dpState != kDPMode_Exited)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount                    = 0;
            structuredVDMCommandParam.vdoData                     = NULL;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos  = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_EnterMode;
            vdmCommand                                            = PD_DPM_CONTROL_ENTER_MODE;
            break;

        case kDPVDM_StatusUpdate:
            if (dpInstance->dpState < kDPMode_EnterDPDone)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount                    = 1;
            structuredVDMCommandParam.vdoData                     = (uint32_t *)&dpInstance->dpSelfStatus;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos  = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kDPVDM_StatusUpdate;
            structuredVDMCommandParam.vendorVDMNeedResponse       = 1;
            vdmCommand                                            = PD_DPM_SEND_VENDOR_STRUCTURED_VDM;
            break;

        case kDPVDM_Configure:
            if (dpInstance->dpState < kDPMode_EnterDPDone)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount                    = 1;
            structuredVDMCommandParam.vdoData                     = (uint32_t *)&dpInstance->dpConfigure;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos  = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kDPVDM_Configure;
            structuredVDMCommandParam.vendorVDMNeedResponse       = 1;
            vdmCommand                                            = PD_DPM_SEND_VENDOR_STRUCTURED_VDM;
            break;

        case kVDM_ExitMode:
            if (dpInstance->dpState < kDPMode_EnterDPDone)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount                    = 0;
            structuredVDMCommandParam.vdoData                     = NULL;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos  = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_ExitMode;
            vdmCommand                                            = PD_DPM_SEND_VENDOR_STRUCTURED_VDM;
            break;
#endif

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
        case kVDM_Attention:
            structuredVDMCommandParam.vdoCount                    = 1;
            structuredVDMCommandParam.vdoData                     = (uint32_t *)&dpInstance->dpSelfStatus;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos  = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_Attention;
            vdmCommand                                            = PD_DPM_CONTROL_SEND_ATTENTION;
            break;
#endif

        default:
            break;
    }

    if (vdmCommand != 0)
    {
        dpInstance->CommandDoing = 1;
        if (PD_Command(dpInstance->pdHandle, vdmCommand, &structuredVDMCommandParam) != kStatus_PD_Success)
        {
            dpInstance->CommandDoing = 0;
            /* wait and retry again */
            PD_DpDelayRetryCommand(dpInstance, command, PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
        }
    }

    return kStatus_PD_Success;
}

/* DP DFP or UFP */
static void PD_DpTrigerCommand(pd_alt_mode_dp_t *dpInstance, uint8_t command, uint8_t retry)
{
    USB_OSA_SR_ALLOC();

    if (!retry)
    {
        dpInstance->retryCount = PD_ALT_MODE_COMMAND_RETRY_COUNT;
    }
    dpInstance->dpCommand = command;
    USB_OSA_ENTER_CRITICAL();
    dpInstance->triggerCommand = 1;
    USB_OSA_EXIT_CRITICAL();
    PD_AltModeModuleTaskWakeUp(dpInstance->altModeHandle, dpInstance);
}

#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
/* DP DFP function */
static pd_status_t PD_DpDFPGetModesCheckHaveSupportedMode(pd_alt_mode_dp_t *dpInstance)
{
    if (dpInstance->pdMsgReceivedVDOCount < 1)
    {
        return kStatus_PD_Success;
    }
    dpInstance->selectModeIndex = 0;

#if !(defined PD_CONFIG_ALT_MODE_DP_AUTO_SELECT_MODE) || (!PD_CONFIG_ALT_MODE_DP_AUTO_SELECT_MODE)
    {
        pd_altmode_dp_modes_sel_t dpModes;
        dpModes.modesCount  = dpInstance->pdMsgReceivedVDOCount;
        dpModes.modes       = (pd_dp_mode_obj_t *)&(dpInstance->pdMsgReceivedBuffer[0]);
        dpModes.selectIndex = 0;

        PD_DpmAltModeCallback(dpInstance->pdHandle, PD_DPM_ALTMODE_DP_DFP_SELECT_MODE_AND_PINASSIGN, &dpModes);
        if ((dpModes.selectIndex > 0) && (dpModes.selectIndex <= dpModes.modesCount))
        {
            dpInstance->selectModeIndex                        = dpModes.selectIndex;
            dpInstance->dpConfigure.bitFields.configureUFPUPin = dpModes.selectPinAssign;
        }
    }
#else
    {
        pd_dp_mode_obj_t modeObj;
        uint8_t configurePin = 0;
        uint8_t index;
        for (index = 0; index < dpInstance->pdMsgReceivedVDOCount; ++index)
        {
            modeObj.modeVal = dpInstance->pdMsgReceivedBuffer[index];
            if ((modeObj.modeVal & 0xFF000000u) || ((modeObj.modeVal & 0x00FFFFFFu) == 0) ||
                ((modeObj.bitFields.portCap & kDPPortCap_UFPD) == 0))
            {
                /* invalid mode */
                continue;
            }

            if (modeObj.bitFields.receptacleIndication)
            {
                /* receptacle */
                configurePin = modeObj.bitFields.UFPDPinSupport;
            }
            else
            {
                configurePin = modeObj.bitFields.DFPDPinSupport;
            }

            if (configurePin & (dpInstance->dpHostConfig->supportPinAssigns))
            {
                dpInstance->selectModeIndex = index + 1;
                break;
            }
        }
    }
#endif

    if ((dpInstance->selectModeIndex > 0) && (dpInstance->selectModeIndex <= dpInstance->pdMsgReceivedVDOCount))
    {
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
        uint8_t configurePin = 0;
        pd_dp_mode_obj_t modeObj;

        modeObj.modeVal = dpInstance->pdMsgReceivedBuffer[dpInstance->selectModeIndex - 1];
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }

        PRINTF("device supported pin assignments: ");
        if (configurePin & kPinAssign_A)
        {
            PRINTF("A");
        }
        if (configurePin & kPinAssign_B)
        {
            PRINTF("B");
        }
        if (configurePin & kPinAssign_C)
        {
            PRINTF("C");
        }
        if (configurePin & kPinAssign_D)
        {
            PRINTF("D");
        }
        if (configurePin & kPinAssign_E)
        {
            PRINTF("E");
        }
        PRINTF("\r\n");
#endif
        return kStatus_PD_Success;
    }

    return kStatus_PD_Error;
}

/* DP DFP function */
static pd_status_t PD_DpDFPConstructConfigure(pd_alt_mode_dp_t *dpInstance)
{
    pd_dp_mode_obj_t modeObj;
    uint8_t setSignal    = 0;
    uint8_t configurePin = 0;

    if (dpInstance->selectModeIndex == 0)
    {
        return kStatus_PD_Error;
    }
    modeObj.modeVal = dpInstance->pdMsgReceivedBuffer[dpInstance->selectModeIndex - 1];

#if !(defined PD_CONFIG_ALT_MODE_DP_AUTO_SELECT_MODE) || (!PD_CONFIG_ALT_MODE_DP_AUTO_SELECT_MODE)
    {
        configurePin = dpInstance->dpConfigure.bitFields.configureUFPUPin;
    }
#else
    /* if prefer multi function, kPinAssign_B and kPinAssign_D has high priority */
    if ((dpInstance->dpPartnerStatus.bitFields.multiFunctionPreferred) ||
        (dpInstance->dpHostConfig->multiFunctionPrefered))
    {
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }
        configurePin &= (kPinAssign_B | kPinAssign_D);
        configurePin &= dpInstance->dpHostConfig->supportPinAssigns;
        if (configurePin != 0)
        {
            if (configurePin & kPinAssign_D)
            {
                setSignal    = kDPSignal_DP;
                configurePin = kPinAssign_D;
            }
            else if (configurePin & kPinAssign_B)
            {
                setSignal    = kDPSignal_USBGEN2;
                configurePin = kPinAssign_B;
            }
            else
            {
            }
        }
    }

    /* multi function is not prefered or don't get kPinAssign_B and kPinAssign_D
     * prefer the 4 lane pin assignment*/
    if (configurePin == 0)
    {
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }

        configurePin &= (~(kPinAssign_B | kPinAssign_D));
        configurePin &= dpInstance->dpHostConfig->supportPinAssigns;
        if (configurePin != 0)
        {
            if (configurePin & kPinAssign_C)
            {
                setSignal    = kDPSignal_DP;
                configurePin = kPinAssign_C;
            }
            else if (configurePin & kPinAssign_E)
            {
                setSignal    = kDPSignal_DP;
                configurePin = kPinAssign_E;
            }
            else if (configurePin & kPinAssign_A)
            {
                setSignal    = kDPSignal_USBGEN2;
                configurePin = kPinAssign_A;
            }
            else
            {
            }
        }
    }

    /* get the first one */
    if (configurePin == 0)
    {
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }

        configurePin &= dpInstance->dpHostConfig->supportPinAssigns;
        if (configurePin != 0)
        {
            if (configurePin & kPinAssign_A)
            {
                setSignal    = kDPSignal_USBGEN2;
                configurePin = kPinAssign_A;
            }
            else if (configurePin & kPinAssign_B)
            {
                setSignal    = kDPSignal_USBGEN2;
                configurePin = kPinAssign_B;
            }
            else if (configurePin & kPinAssign_C)
            {
                setSignal    = kDPSignal_DP;
                configurePin = kPinAssign_C;
            }
            else if (configurePin & kPinAssign_D)
            {
                setSignal    = kDPSignal_DP;
                configurePin = kPinAssign_D;
            }
            else if (configurePin & kPinAssign_E)
            {
                setSignal    = kDPSignal_DP;
                configurePin = kPinAssign_E;
            }
            else
            {
            }
        }
    }
#endif

    if (configurePin != 0)
    {
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
        PRINTF("select pin assignments: ");
        if (configurePin & kPinAssign_A)
        {
            PRINTF("A");
        }
        else if (configurePin & kPinAssign_B)
        {
            PRINTF("B");
        }
        else if (configurePin & kPinAssign_C)
        {
            PRINTF("C");
        }
        else if (configurePin & kPinAssign_D)
        {
            PRINTF("D");
        }
        else if (configurePin & kPinAssign_E)
        {
            PRINTF("E");
        }
        else
        {
        }
        PRINTF("\r\n");
#endif
        dpInstance->dpConfigure.bitFields.setConfig        = kDPConfig_UFPD;
        dpInstance->dpConfigure.bitFields.setSignal        = setSignal;
        dpInstance->dpConfigure.bitFields.configureUFPUPin = configurePin;
        return kStatus_PD_Success;
    }

    return kStatus_PD_Error;
}

static void PD_DpDFPSetConfigureAsUSB(pd_alt_mode_dp_t *dpInstance)
{
    dpInstance->dpConfigure.bitFields.setConfig        = kDPConfig_USB;
    dpInstance->dpConfigure.bitFields.setSignal        = kDPSignal_Unspecified;
    dpInstance->dpConfigure.bitFields.configureUFPUPin = kPinAssign_DeSelect;
}

static void PD_DpDFPProcessUFPstatus(pd_alt_mode_dp_t *dpInstance)
{
    uint8_t driverVal = kDPHPDDriver_Low;
    if (dpInstance->dpPartnerStatus.bitFields.HPDInterrupt)
    {
        driverVal = kDPHPDDriver_IRQ;
    }
    else if (dpInstance->dpPartnerStatus.bitFields.HPDState)
    {
        driverVal = kDPHPDDriver_High;
    }
    else
    {
        driverVal = kDPHPDDriver_Low;
    }
    dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(dpInstance->dpBoardChipHandle,
                                                                       kDPPeripheal_ControlHPDValue, &driverVal);

    /* Figure 5-4 */
    if ((dpInstance->dpPartnerStatus.bitFields.exitDPModeReq) || (dpInstance->dpPartnerStatus.bitFields.USBConfigReq) ||
        (!(dpInstance->dpPartnerStatus.bitFields.DFPDUFPDConnected & kUFP_D_Connected)))
    {
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
        PRINTF("start exit mode\r\n");
#endif
        driverVal = kDPHPDDriver_Low;
        dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(dpInstance->dpBoardChipHandle,
                                                                           kDPPeripheal_ControlHPDValue, &driverVal);
        /* The DFP_U shall issue an Exit Mode command only when the port is configured to be in USB configuration.
         */
        /* Receipt of an Exit Mode command while not configured in USB Configuration indicates an error in the
         * DFP_U. */
        dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(dpInstance->dpBoardChipHandle,
                                                                           kDPPeripheal_ControlHPDSetLow, NULL);
        if (dpInstance->dpState == kDPMode_ConfigureDone)
        {
            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxSaftMode, NULL);
            PD_DpDFPSetConfigureAsUSB(dpInstance);
            PD_DpSendCommand(dpInstance, kDPVDM_Configure);
        }
        else
        {
            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxUSB3Only, NULL);
            PD_DpSendCommand(dpInstance, kVDM_ExitMode);
        }
    }
}
#endif

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)

static pd_hpd_detect_type_t PD_DpUFPHPDDetectPeek(pd_alt_mode_dp_t *dpInstance)
{
    pd_hpd_detect_type_t result;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if (dpInstance->hpdDetectQueueLength == 0)
    {
        result = kDPHPDDetect_Empty;
    }
    else
    {
        result = dpInstance->hpdDetectQueue[dpInstance->hpdDetectQueueGetPos];
    }
    USB_OSA_EXIT_CRITICAL();
    return result;
}

static void PD_DpUFPHPDDetectPop(pd_alt_mode_dp_t *dpInstance, pd_hpd_detect_type_t popValue)
{
    USB_OSA_SR_ALLOC();

    if (popValue == kDPHPDDetect_Empty)
    {
        return;
    }

    USB_OSA_ENTER_CRITICAL();
    if (dpInstance->hpdDetectQueueLength != 0)
    {
        if (dpInstance->hpdDetectQueue[dpInstance->hpdDetectQueueGetPos] == popValue)
        {
            dpInstance->hpdDetectQueueGetPos = (dpInstance->hpdDetectQueueGetPos + 1) % HPD_DETECT_QUEUE_LEN;
            --(dpInstance->hpdDetectQueueLength);
        }
    }
    USB_OSA_EXIT_CRITICAL();
}

static void PD_DpUFPHPDDetectReset(pd_alt_mode_dp_t *dpInstance)
{
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    dpInstance->hpdDetectEnable      = 0;
    dpInstance->hpdDetectQueueGetPos = 0;
    dpInstance->hpdDetectQueuePutPos = 0;
    dpInstance->hpdDetectQueueLength = 0;
    USB_OSA_EXIT_CRITICAL();
}

static void PD_DpUFPHPDDetectEnable(pd_alt_mode_dp_t *dpInstance)
{
    dpInstance->hpdDetectEnable = 1;
}

static void PD_DpUFPHPDDetectAdd(pd_alt_mode_dp_t *dpInstance, pd_hpd_detect_type_t detectValue)
{
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if (dpInstance->hpdDetectQueueLength < HPD_DETECT_QUEUE_LEN)
    {
        if (detectValue == kDPHPDDetect_IRQ)
        {
            uint8_t lastIndex = (dpInstance->hpdDetectQueuePutPos - 1);
            if (dpInstance->hpdDetectQueuePutPos == 0)
            {
                lastIndex = (HPD_DETECT_QUEUE_LEN - 1);
            }
            if (dpInstance->hpdDetectQueue[lastIndex] == kDPHPDDetect_High)
            {
                dpInstance->hpdDetectQueue[lastIndex] = kDPHPDDetect_IRQ;
                USB_OSA_EXIT_CRITICAL();
                return;
            }
        }
        dpInstance->hpdDetectQueue[dpInstance->hpdDetectQueuePutPos] = detectValue;
        dpInstance->hpdDetectQueuePutPos = (dpInstance->hpdDetectQueuePutPos + 1) % HPD_DETECT_QUEUE_LEN;
        dpInstance->hpdDetectQueueLength++;
    }
    USB_OSA_EXIT_CRITICAL();
}

static void PD_DpUFPHPDDetectPut(pd_alt_mode_dp_t *dpInstance, pd_hpd_detect_type_t detectValue)
{
    if (!dpInstance->hpdDetectEnable)
    {
        return;
    }

    if (detectValue == kDPHPDDetect_Low)
    {
        /* Low detected clears the queue */
        PD_DpUFPHPDDetectReset(dpInstance);
        PD_DpUFPHPDDetectEnable(dpInstance);
        PD_DpUFPHPDDetectAdd(dpInstance, kDPHPDDetect_Low);
    }
    else if (detectValue == kDPHPDDetect_High)
    {
        /* Just add the high in (shouldn't go beyond the end of the queue, but better to be safe) */
        PD_DpUFPHPDDetectAdd(dpInstance, kDPHPDDetect_High);
    }
    else
    {
        uint8_t index    = 0;
        uint8_t irqFound = 0;
        if (dpInstance->hpdDetectQueueLength > 0)
        {
            index = dpInstance->hpdDetectQueueGetPos;
            do
            {
                if (dpInstance->hpdDetectQueue[index] == kDPHPDDetect_IRQ)
                {
                    irqFound++;
                    if (irqFound >= 2)
                    {
                        break;
                    }
                }
                index = (index + 1) % HPD_DETECT_QUEUE_LEN;
            } while (index != dpInstance->hpdDetectQueuePutPos);
        }

        if (irqFound < 2)
        {
            PD_DpUFPHPDDetectAdd(dpInstance, kDPHPDDetect_IRQ);
        }
    }
}

/* DP UFP */
static void PD_DpUFPTriggerTask(pd_alt_mode_dp_t *dpInstance)
{
    PD_AltModeModuleTaskWakeUp(dpInstance->altModeHandle, dpInstance);
}

/* DP UFP function */
static void PD_DpUFPGetStatus(pd_alt_mode_dp_t *dpInstance)
{
    pd_hpd_detect_type_t getState;
    uint8_t hpdState = 0;
    uint8_t irqState = 0;

    getState = PD_DpUFPHPDDetectPeek(dpInstance);
    /* We will only clear this from the detect queue when the message TX is confirmed.
     * Store it here as the queue may change when we check again */
    dpInstance->attnStatusHPD = getState;

    dpInstance->dpSelfStatus.statusVal                        = 0;
    dpInstance->dpSelfStatus.bitFields.DFPDUFPDConnected      = dpInstance->dpSlaveConfig->displayPortConnection;
    dpInstance->dpSelfStatus.bitFields.powerLow               = dpInstance->dpSlaveConfig->adapterPowerLow;
    dpInstance->dpSelfStatus.bitFields.enabled                = dpInstance->dpSlaveConfig->adapterEnabled;
    dpInstance->dpSelfStatus.bitFields.multiFunctionPreferred = dpInstance->dpSlaveConfig->multiFunctionPreferred;

    if (getState != kDPHPDDetect_Empty)
    {
        if (getState == kDPHPDDetect_High)
        {
            hpdState = 0x01u;
        }
        else if (getState == kDPHPDDetect_IRQ)
        {
            hpdState = 0x01u;
            irqState = 0x01u;
        }
    }
    else
    {
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
        /* else just get the state of the HPD input */
        dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
            dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDGetCurrentState, &getState);
#endif

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
        /* else just get the state of the HPD input */
        dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
            dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDGetCurrentState, &getState);
#endif

        if (getState == kDPHPDDetect_High)
        {
            hpdState = 0x01u;
        }
    }

    dpInstance->dpSelfStatus.bitFields.HPDState     = hpdState;
    dpInstance->dpSelfStatus.bitFields.HPDInterrupt = irqState;

#if 0
#ifndef USBPD_DP_INTEGRATED_UFP_D
    /* Connected status is based on HPD status if we do not have an integrated UFP_D. */
    if (ret & HPD_HIGH)
    {
        dpInstance->dpSelfStatus.bitFields.DFPDUFPDConnected |= kUFP_D_Connected;
    }
#endif
#endif
}

/* DP UFP function */
static void PD_DpUFPProcessStatusUpdate(pd_alt_mode_dp_t *dpInstance)
{
    PD_DpUFPGetStatus(dpInstance);
    PD_DpUFPHPDDetectPop(dpInstance, dpInstance->attnStatusHPD);
}

/* DP UFP function */
static uint8_t PD_DpUFPProcessConfigure(pd_alt_mode_dp_t *dpInstance)
{
    /* Configure has arrived - change MUXes and respond
     * By default NACK the config */
    uint8_t configAck = 0;

    if (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_USB)
    {
        /* Only expect to receive this in: Figure 5-10: UFP_U DisplayPort Operation */
        if (dpInstance->dpState >= kDPMode_ConfigureDone)
        {
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxUSB3Only, NULL);
#endif
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxUSB3Only, NULL);
#endif

            /* we are a dock here so don't need HpdDriverSetLow() */
            configAck = 1;

            /* Figure 5-10: UFP_U DisplayPort Operation DisplayPort <ConfigureCommand: USB Configuration?> */
            dpInstance->dpState = kDPMode_EnterDPDone;
        }
        else
        {
            /* Already in USB mode */
            configAck = 1;
        }
    }
    else if (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_UFPD)
    {
        uint32_t pinAssignment = dpInstance->dpConfigure.bitFields.configureUFPUPin;
        /* We can receive this in :
         *       Figure 5-8: UFP_U USB Configuration Wait State
         *       Figure 5-10: UFP_U DisplayPort Operation
         * Default NACK - Stay in: Figure 5-8: UFP_U USB Configuration Wait State */
        configAck = 0;
        if (dpInstance->dpState < kDPMode_EnterDPDone)
        {
            return configAck;
        }

        /* configAck set: Figure 5-9: UFP_U DisplayPort Configuration */

#ifdef USBPD_ENABLE_FAST_CENTER_ALT_MODE_CONTROL
        if ((pinAssignment & kPinAssign_VR))
        {
            uint32_t pdo = (0x80 << 8);
            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxDP4LANEUSB3, &pdo);
            /* can handle the configure so ack it */
            configAck = 1;
        }
        else
#endif
            if ((pinAssignment & (kPinAssign_D | kPinAssign_F))
                /* Pin assignment in the old DP Alt Mode 1.0 location */
                || (dpInstance->dpConfigure.bitFields.reserved1 & (kPinAssign_D)))
        {
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxDP2LANEUSB3, &dpInstance->dpConfigure);
#endif

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxDP2LANEUSB3, &dpInstance->dpConfigure);
#endif

            /* can handle the configure so ack it */
            configAck = 1;
        }
        else
        {
            /* Default to mandatory pin assignment C in all other cases. */
            uint32_t pdo = (kPinAssign_C << 8u);

#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(dpInstance->dpBoardChipHandle,
                                                                               kDPPeripheal_ControlSetMuxDP4LANE, &pdo);
#endif

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxDP4LANE, &pdo);
#endif

            /* can handle the configure so ack it */
            configAck = 1;
        }
        /* There is intentionally no error case here.  We default to pin assignment C.
         * Figure 5-10: UFP_U DisplayPort Operation DisplayPort <ConfigureCommand: USB Configuration?> */
        dpInstance->dpState = kDPMode_ConfigureDone;
    }
    else
    {
        configAck = 0;
    }

    return configAck;
}
#endif

static void PD_DpInstanceReset(pd_alt_mode_dp_t *dpInstance)
{
    dpInstance->dpState      = kDPMode_Exited;
    dpInstance->taskEvent    = 0u;
    dpInstance->CommandDoing = 0;
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
    uint8_t driverVal = kDPHPDDriver_Low;
    dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(dpInstance->dpBoardChipHandle,
                                                                       kDPPeripheal_ControlHPDValue, &driverVal);
    dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(dpInstance->dpBoardChipHandle,
                                                                       kDPPeripheal_ControlSetMuxUSB3Only, NULL);
#endif

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
    dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(dpInstance->dpBoardChipHandle,
                                                                        kDPPeripheal_ControlSetMuxUSB3Only, NULL);
#endif
}

void PD_DPModule1msISR(void *moduleInstance)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;

    if (dpInstance->delayTime > 0)
    {
        dpInstance->delayTime--;
        if (dpInstance->delayTime == 0)
        {
            /* PD_DpModuleSetEvent(dpInstance, dpInstance->delayEvents); */
            PD_DpTrigerCommand(dpInstance, dpInstance->dpCommand, 1);
        }
    }
}

/*
 * pdHandle - PD tack handle.
 * altModeHandle - alt mode driver handle.
 * moduleConfig - displayport module configuration parameter.
 * moduleInstance - return the displayport module instance handle
 *
 */
pd_status_t PD_DPInit(pd_handle pdHandle, void *altModeHandle, const void *moduleConfig, void **moduleInstance)
{
    uint32_t index               = 0;
    pd_alt_mode_dp_t *dpInstance = NULL;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    for (index = 0; index < sizeof(s_AltModeDisplayPortInstance) / sizeof(pd_alt_mode_dp_t); ++index)
    {
        if (s_AltModeDisplayPortInstance[index].occupied == 0)
        {
            s_AltModeDisplayPortInstance[index].occupied = 1;
            dpInstance                                   = &s_AltModeDisplayPortInstance[index];
            break;
        }
    }

    if (dpInstance == NULL)
    {
        USB_OSA_EXIT_CRITICAL();
        return kStatus_PD_Error;
    }
    USB_OSA_EXIT_CRITICAL();
    dpInstance->pdHandle      = pdHandle;
    dpInstance->altModeHandle = altModeHandle;

#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
    dpInstance->dpHostConfig = (pd_alt_mode_dp_host_config_t *)moduleConfig;
    if (dpInstance->dpHostConfig->peripheralInterface->dpPeripheralInit(
            &(dpInstance->dpBoardChipHandle), pdHandle, (void *)dpInstance->dpHostConfig->peripheralConfig) !=
        kStatus_PD_Success)
    {
        dpInstance->occupied = 0;
        return kStatus_PD_Error;
    }
#endif

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
    dpInstance->dpSlaveConfig = (pd_alt_mode_dp_slave_config_t *)moduleConfig;
    if (dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralInit(
            &(dpInstance->dpBoardChipHandle), pdHandle, (void *)dpInstance->dpSlaveConfig->peripheralConfig) !=
        kStatus_PD_Success)
    {
        dpInstance->occupied = 0;
        return kStatus_PD_Error;
    }
#endif

    PD_DpInstanceReset(dpInstance);

    *moduleInstance = dpInstance;
    return kStatus_PD_Success;
}

pd_status_t PD_DPDeinit(void *moduleInstance)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;

#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
    dpInstance->dpHostConfig->peripheralInterface->dpPeripheralDeinit(dpInstance->dpBoardChipHandle);
#endif

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
    dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralDeinit(dpInstance->dpBoardChipHandle);
#endif

    dpInstance->occupied = 0;
    return kStatus_PD_Success;
}

pd_status_t PD_DPControl(void *moduleInstance, uint32_t controlCode, void *controlParam)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;
    pd_status_t status           = kStatus_PD_Success;

    /* dfp and ufp */
    switch (controlCode)
    {
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
        /* DFP start to enter mode sequence */
        case kAltMode_TriggerEnterMode:
            PD_DpTrigerCommand(dpInstance, kVDM_DiscoverModes, 0);
            break;
#endif

        case kAltMode_TriggerExitMode:
        {
            uint8_t dataRole;
            PD_Control(dpInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &dataRole);
            if (dpInstance->dpState >= kDPMode_EnterDPDone)
            {
                *((uint8_t *)controlParam) = 0;
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
                if (dataRole == kPD_DataRoleDFP)
                {
                    PD_DpTrigerCommand(dpInstance, kVDM_ExitMode, 0);
                }
#endif
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
                if (dataRole == kPD_DataRoleUFP)
                {
                    dpInstance->dpSelfStatus.bitFields.exitDPModeReq = 1;
                    PD_DpTrigerCommand(dpInstance, kVDM_Attention, 0);
                }
#endif
            }
            else
            {
                *((uint8_t *)controlParam) = 1;
            }
            break;
        }

        case kAltMode_GetModeState:
        {
            if (controlParam == NULL)
            {
                status = kStatus_PD_Error;
            }
            else
            {
                pd_alt_mode_state_t *modeState = (pd_alt_mode_state_t *)controlParam;
                modeState->SVID                = 0u;
                modeState->mode                = 0;
                if (dpInstance->dpState >= kDPMode_EnterDPDone)
                {
                    modeState->SVID = DP_SVID;
                    modeState->mode = dpInstance->selectModeIndex;
                }
            }
            break;
        }

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
        case kDPControl_HPDDetectEvent:
        {
            if ((dpInstance->dpState >= kDPMode_ConfigureDone) ||
                (dpInstance->dpState == kDPMode_StatusUpdateDone &&
                 dpInstance->dpSelfStatus.bitFields.DFPDUFPDConnected == kDFP_D_NonConnected))
            {
                /* add hpd detect to queue */
                PD_DpUFPHPDDetectPut(dpInstance, (pd_hpd_detect_type_t)(*(uint8_t *)controlParam));
                PD_DpUFPTriggerTask(dpInstance);
            }
            break;
        }
#endif

        default:
            break;
    }

    return status;
}

/* msgSVID: 0 - this msg related event doesn't know SVID. */
pd_status_t PD_DPCallbackEvent(void *moduleInstance, uint32_t processCode, uint16_t msgSVID, void *param)
{
    pd_status_t status           = kStatus_PD_Error;
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;
    uint32_t index               = 0;

    if ((msgSVID != 0) && (msgSVID != 0xFF01u))
    {
        return status;
    }

    /* process the msg related events, if not self msg or self shouldn't process this event return error. */
    switch (processCode)
    {
        case kAltMode_Attach:
        case kAltMode_HardReset:
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxUSB3Only, NULL);
            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(dpInstance->dpBoardChipHandle,
                                                                               kDPPeripheal_ControlHPDSetLow, NULL);
#endif
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxUSB3Only, NULL);
            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDQueueDisable, NULL);
            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDDetectStart, NULL);
            PD_DpUFPHPDDetectReset(dpInstance);
#endif
            PD_DpInstanceReset(dpInstance);
            break;

        case kAltMode_Detach:
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxShutDown, NULL);
            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(dpInstance->dpBoardChipHandle,
                                                                               kDPPeripheal_ControlHPDSetLow, NULL);
#endif
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxShutDown, NULL);
            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDQueueDisable, NULL);
            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDDetectStop, NULL);
            PD_DpUFPHPDDetectReset(dpInstance);
#endif
            PD_DpInstanceReset(dpInstance);
            break;

        case kAltMode_StructedVDMMsgReceivedProcess:
        {
            pd_svdm_command_request_t *svdmRequest = (pd_svdm_command_request_t *)param;

            if (msgSVID == 0xFF01)
            {
                status = kStatus_PD_Success;
                switch (svdmRequest->vdmHeader.bitFields.command)
                {
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
                    case kVDM_DiscoverModes: /* DP UFP */
                        for (index = 0; index < dpInstance->dpSlaveConfig->modesCount; ++index)
                        {
                            dpInstance->pdMsgBuffer[index] = dpInstance->dpSlaveConfig->modesList[index];
                        }
                        svdmRequest->vdoData             = (uint32_t *)&dpInstance->pdMsgBuffer[0];
                        svdmRequest->vdoCount            = dpInstance->dpSlaveConfig->modesCount;
                        svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                        break;

                    case kVDM_EnterMode: /* DP UFP */
                        dpInstance->dpConfigure.configureVal = svdmRequest->vdoData[0];
                        svdmRequest->vdoData                 = NULL;
                        svdmRequest->vdoCount                = 0;
                        if (svdmRequest->vdmHeader.bitFields.objPos <= dpInstance->dpSlaveConfig->modesCount)
                        {
                            PD_DpUFPHPDDetectReset(dpInstance);
                            PD_DpUFPHPDDetectEnable(dpInstance);
                            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDQueueEnable, NULL);
                            dpInstance->dpState              = kDPMode_EnterDPDone;
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                        }
                        break;

                    case kVDM_ExitMode: /* DP UFP */
                        svdmRequest->vdoData  = NULL;
                        svdmRequest->vdoCount = 0;
                        if ((dpInstance->dpState == kDPMode_EnterDPDone) ||
                            (dpInstance->dpState == kDPMode_StatusUpdateDone) ||
                            (dpInstance->dpState == kDPMode_ConfigureDone))
                        {
                            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDQueueDisable, NULL);
                            PD_DpUFPHPDDetectReset(dpInstance);
                            dpInstance->dpState              = kDPMode_Exited;
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                        }
                        else
                        {
                            svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                        }
                        break;

                    case kDPVDM_StatusUpdate: /* DP UFP */
                        /* can receive at any time */
                        if ((dpInstance->dpState == kDPMode_EnterDPDone) ||
                            (dpInstance->dpState == kDPMode_StatusUpdateDone) ||
                            (dpInstance->dpState == kDPMode_ConfigureDone))
                        {
                            dpInstance->dpPartnerStatus.statusVal = svdmRequest->vdoData[0];
                            PD_DpUFPProcessStatusUpdate(dpInstance);
                            svdmRequest->vdoData             = (uint32_t *)&dpInstance->dpSelfStatus;
                            svdmRequest->vdoCount            = 1;
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                            if (dpInstance->dpState == kDPMode_EnterDPDone)
                            {
                                dpInstance->dpState = kDPMode_StatusUpdateDone;
                            }
                        }
                        else
                        {
                            svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                        }
                        break;

                    case kDPVDM_Configure: /* DP UFP */
                        /* can receive at any time */
                        if ((dpInstance->dpState == kDPMode_EnterDPDone) ||
                            (dpInstance->dpState == kDPMode_StatusUpdateDone) ||
                            (dpInstance->dpState == kDPMode_ConfigureDone))
                        {
                            dpInstance->dpConfigure.configureVal = svdmRequest->vdoData[0];
                            if ((dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_DFPD) ||
                                (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_UFPD))
                            {
                                if (dpInstance->dpState != kDPMode_ConfigureDone)
                                {
                                    dpInstance->dpState = kDPMode_ConfigureDone;
                                }
                            }
                            else
                            {
                                /* back the state */
                                dpInstance->dpState = kDPMode_StatusUpdateDone;
                            }

                            svdmRequest->vdoData  = NULL;
                            svdmRequest->vdoCount = 0;
                            /* after set the displayport signal then ACK */
                            dpInstance->dpSlaveConfig->peripheralInterface->dpPeripheralControl(
                                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxSaftMode, NULL);

                            if (PD_DpUFPProcessConfigure(dpInstance))
                            {
                                svdmRequest->requestResultStatus = kCommandResult_VDMACK;

#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
                                PRINTF("dp configure success\r\n");
#endif
                                PD_DpmAltModeCallback(dpInstance->pdHandle, PD_DPM_ALTMODE_DP_UFP_MODE_CONFIGURED,
                                                      NULL);
                            }
                            else
                            {
                                svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                            }
                        }
                        else
                        {
                            svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                        }
                        break;
#endif

#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
                    case kVDM_Attention: /* DP DFP */
                        if (svdmRequest->vdoCount == 1)
                        {
                            dpInstance->dpPartnerStatus.statusVal = svdmRequest->vdoData[0];
                            /* process DP status */
                            if (dpInstance->dpState == kDPMode_StatusUpdateDone)
                            {
                                if (dpInstance->dpPartnerStatus.bitFields.DFPDUFPDConnected & kUFP_D_Connected)
                                {
                                    dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                                        dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxSaftMode, NULL);
                                    if (PD_DpDFPConstructConfigure(dpInstance) == kStatus_PD_Success)
                                    {
                                        PD_DpTrigerCommand(dpInstance, kDPVDM_Configure, 0);
                                    }
                                }
                            }
                            else if (dpInstance->dpState == kDPMode_ConfigureDone)
                            {
/* PD_DpModuleSetEvent(dpInstance, DP_TASK_EVENT_DFP_DP_CHECK_UFP_STATUS); */
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
                                PRINTF("receive attention\r\n");
#endif
                                PD_DpDFPProcessUFPstatus(dpInstance);
                            }
                            else
                            {
                                /* don't process */
                            }
                        }
                        break;
#endif
                    default:
                        break;
                }
            }
            break;
        }

        case kAltMode_StructedVDMMsgSuccess:
        {
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
            /* ACK msg, DP DFP */
            /* DFP is doing ASM command */
            if (dpInstance->CommandDoing)
            {
                pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;
                dpInstance->CommandDoing             = 0;
                status                               = kStatus_PD_Success;
                switch (svdmResult->vdmCommand)
                {
                    case kVDM_DiscoverModes:
                        dpInstance->dpSelfStatus.bitFields.DFPDUFPDConnected = kDFP_D_Connected;
                        dpInstance->pdMsgReceivedVDOCount                    = svdmResult->vdoCount;
                        for (index = 0; index < svdmResult->vdoCount; ++index)
                        {
                            dpInstance->pdMsgReceivedBuffer[index] = svdmResult->vdoData[index];
                        }
                        dpInstance->pdVDMMsgReceivedHeader.structuredVdmHeaderVal =
                            svdmResult->vdmHeader.structuredVdmHeaderVal;
                        if (PD_DpDFPGetModesCheckHaveSupportedMode(dpInstance) == kStatus_PD_Success)
                        {
                            PD_DpTrigerCommand(dpInstance, kVDM_EnterMode, 0);
                        }
                        break;

                    case kVDM_EnterMode:
                        dpInstance->dpState = kDPMode_EnterDPDone;
                        PD_DpTrigerCommand(dpInstance, kDPVDM_StatusUpdate, 0);
                        break;

                    case kDPVDM_StatusUpdate:
                        dpInstance->dpState                   = kDPMode_StatusUpdateDone;
                        dpInstance->dpPartnerStatus.statusVal = svdmResult->vdoData[0];
                        if (dpInstance->dpPartnerStatus.bitFields.DFPDUFPDConnected & kUFP_D_Connected)
                        {
                            /* if false, wait attention message */
                            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxSaftMode, NULL);
                            if (PD_DpDFPConstructConfigure(dpInstance) == kStatus_PD_Success)
                            {
                                PD_DpTrigerCommand(dpInstance, kDPVDM_Configure, 0);
                            }
                        }
                        break;

                    case kDPVDM_Configure:
                        /* 1. configure as DP; 2. configure as USB (exit DP) */
                        if ((dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_DFPD) ||
                            (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_UFPD))
                        {
                            dpInstance->dpState = kDPMode_ConfigureDone;
                            if ((dpInstance->dpConfigure.bitFields.configureUFPUPin == kPinAssign_C) ||
                                (dpInstance->dpConfigure.bitFields.configureUFPUPin == kPinAssign_E))
                            {
                                dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                                    dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxDP4LANE,
                                    &dpInstance->dpConfigure.configureVal);
                            }
                            else
                            {
                                dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                                    dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxDP2LANEUSB3,
                                    &dpInstance->dpConfigure.configureVal);
                            }
                            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDReleaseLow, NULL);
                            /* PD_DpModuleSetEvent(dpInstance, DP_TASK_EVENT_DFP_DP_CHECK_UFP_STATUS); */
                            PD_DpDFPProcessUFPstatus(dpInstance);
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
                            PRINTF("dp configure success\r\n");
#endif
                            PD_DpmAltModeCallback(dpInstance->pdHandle, PD_DPM_ALTMODE_DP_DFP_MODE_CONFIGURED, NULL);
                        }
                        else if (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_USB)
                        {
                            /* back the state */
                            dpInstance->dpState = kDPMode_StatusUpdateDone;
                            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDSetLow, NULL);
                            dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                                dpInstance->dpBoardChipHandle, kDPPeripheal_ControlSetMuxUSB3Only, NULL);
                            PD_DpTrigerCommand(dpInstance, kVDM_ExitMode, 0);
                            PD_DpmAltModeCallback(dpInstance->pdHandle, PD_DPM_ALTMODE_DP_DFP_MODE_UNCONFIGURED, NULL);
                        }
                        break;

                    case kVDM_ExitMode:
                        dpInstance->dpHostConfig->peripheralInterface->dpPeripheralControl(
                            dpInstance->dpBoardChipHandle, kDPPeripheal_ControlHPDSetLow, NULL);
                        dpInstance->dpState = kDPMode_Exited;
                        break;

                    default:
                        break;
                }
            }
            else
#endif
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
                if (dpInstance->CommandDoing)
            {
                pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;

                dpInstance->CommandDoing = 0;
                /* UFP send attention successfully */
                if (svdmResult->vdmCommand == kVDM_Attention)
                {
                    PD_DpUFPHPDDetectPop(dpInstance, dpInstance->attnStatusHPD);
                    /* check next HPD queue item */
                    PD_DpUFPTriggerTask(dpInstance);
                    status = kStatus_PD_Success;
                }
            }
            else
#endif
            {
            }

            break;
        }

        case kAltMode_StructedVDMMsgFail:
        {
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
            /* NAK/Not_supported/BUSY/time_out, DP DFP */
            /* DFP is doing ASM command */
            if (dpInstance->CommandDoing)
            {
                pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;
                uint32_t command                     = 0;
                status                               = kStatus_PD_Success;
                dpInstance->CommandDoing             = 0;
                if (svdmResult->vdmCommandResult == kCommandResult_VDMNAK)
                {
                    /* don't support this command */
                    return status;
                }
                if ((svdmResult->vdmCommand == kVDM_DiscoverModes) || (svdmResult->vdmCommand == kVDM_EnterMode) ||
                    (svdmResult->vdmCommand == kVDM_ExitMode) || (svdmResult->vdmCommand == kDPVDM_StatusUpdate) ||
                    (svdmResult->vdmCommand == kDPVDM_Configure))
                {
                    command = svdmResult->vdmCommand;
                }

                if (command != 0)
                {
                    PD_DpDelayRetryCommand(dpInstance, command, PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
                }
            }
            else
#endif
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
                if (dpInstance->CommandDoing)
            {
                dpInstance->CommandDoing = 0;
                /* UFP send attention fail, trigger to get HPD again */
                PD_DpUFPTriggerTask(dpInstance);
            }
            else
#endif
            {
            }
            break;
        }

        case kAltMode_UnstructedVDMMsgReceived:
        case kAltMode_UnstructedVDMMsgSentResult:
            /* DP doesn't have this type message */
            break;

        default:
            break;
    }
    return status;
}

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
static uint8_t PD_DpCtrlReadyToSendHpdDetection(pd_alt_mode_dp_t *dpInstance)
{
    return (uint8_t)((dpInstance->dpState >= kDPMode_EnterDPDone) && (!dpInstance->CommandDoing));
}
#endif

/* 1. send msg from self.
 * wait for send result callback (timer); wait for ACK reply msg callback.
 *
 * 2. ACK received VDM msg.
 * ACK is sent in the callback -> task wait the send result (timer);
 *
 * 3. HPD
 * Dock board detect HPD, Host board driver HPD.
 */
void PD_DPTask(void *taskParam)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)taskParam;

    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();
    if (dpInstance->triggerCommand)
    {
        dpInstance->triggerCommand = 0;
        USB_OSA_EXIT_CRITICAL();

        if (dpInstance->retryCount > 0)
        {
            dpInstance->retryCount--;
            PD_DpSendCommand(dpInstance, dpInstance->dpCommand);
        }
        else
        {
            /* do hard reset */
            if (PD_Command(dpInstance->pdHandle, PD_DPM_CONTROL_HARD_RESET, NULL) != kStatus_PD_Success)
            {
                return;
            }
        }
    }
    else
    {
        USB_OSA_EXIT_CRITICAL();

#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
        /* HPD detection */
        uint8_t getState;
        PD_Control(dpInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &getState);
        if (getState == kPD_DataRoleUFP)
        {
            getState = (uint8_t)PD_DpUFPHPDDetectPeek(dpInstance);
            /* detect queue has items and can send in current state */
            if (((pd_hpd_detect_type_t)getState != kDPHPDDetect_Empty) &&
                (PD_DpCtrlReadyToSendHpdDetection(dpInstance)))
            {
                PD_DpUFPGetStatus(dpInstance);
                PD_DpTrigerCommand(dpInstance, kVDM_Attention, 0);
            }
        }
#endif
    }
}

#endif
#endif
