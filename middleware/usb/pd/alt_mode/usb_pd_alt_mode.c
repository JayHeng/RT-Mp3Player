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
#include "usb_pd_alt_mode.h"
#include "pd_board_config.h"
#if (defined PD_CONFIG_ALT_MODE_DP_SUPPORT) && (PD_CONFIG_ALT_MODE_DP_SUPPORT)
#include "usb_pd_alt_mode_dp.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)

#define PD_ALT_MODE_MSG_Q_ITEM_COUNT (8)

typedef struct pd_alt_mode_module_fun
{
    uint16_t SVID;
    pd_status_t (*pd_alt_mode_init)(pd_handle pdHandle,
                                    void *altModeHandle,
                                    const void *moduleConfig,
                                    void **moduleInstance);
    pd_status_t (*pd_alt_mode_deinit)(void *moduleInstance);
    pd_status_t (*pd_alt_mode_callback_event)(void *moduleInstance,
                                              uint32_t processCode,
                                              uint16_t msgSVID,
                                              void *param);
    pd_status_t (*pd_alt_mode_control)(void *moduleInstance, uint32_t controlCode, void *controlParam);
    void (*pd_alt_mode_task)(void *taskParam);
    void (*pd_alt_mode_1ms_isr)(void *moduleInstance);
} pd_alt_mode_module_interface_t;

typedef struct _pd_dpm_alt_mode
{
    pd_handle pdHandle;
    const pd_alt_mode_config_t *altModeConfig;
    void *altModeModuleInstance[PD_ALT_MODE_MAX_MODULES];
    pd_alt_mode_module_interface_t *modulesInterfaces[PD_ALT_MODE_MAX_MODULES];
    volatile uint32_t delayTime; /* retry delay */
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
    uint32_t dataBuff[7];
#endif
    volatile uint8_t retryCount;   /* how many time one command has been retried */
    volatile uint8_t retryCommand; /* will do this command after delay */
    uint8_t dfpCommand;            /* the processing command */

    volatile uint8_t occupied;
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
    volatile uint8_t altModeStart;
#endif
} pd_alt_mode_t;

typedef struct _pd_alt_mode_q_item
{
    void *altModeInstanceHandle;
    uint32_t event;
} pd_alt_mode_q_item_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static pd_alt_mode_t s_AltModeInstances[PD_CONFIG_MAX_PORT];
static usb_osa_msgq_handle s_MsgQHandle;
static pd_alt_mode_module_interface_t s_ModuleDPInterfaces[] = {
#if (defined PD_CONFIG_ALT_MODE_DP_SUPPORT) && (PD_CONFIG_ALT_MODE_DP_SUPPORT)
    {DP_SVID, PD_DPInit, PD_DPDeinit, PD_DPCallbackEvent, PD_DPControl, PD_DPTask, PD_DPModule1msISR}
#endif
};

/*******************************************************************************
 * Code
 ******************************************************************************/

static void PD_AltModeGetModuleInterface(pd_alt_mode_t *altModeInstance)
{
    for (uint8_t moduleIndex = 0; moduleIndex < sizeof(s_ModuleDPInterfaces) / sizeof(pd_alt_mode_module_interface_t);
         ++moduleIndex)
    {
        altModeInstance->modulesInterfaces[moduleIndex] = NULL;
        for (uint8_t interfaceIndex = 0;
             interfaceIndex < sizeof(s_ModuleDPInterfaces) / sizeof(pd_alt_mode_module_interface_t); ++interfaceIndex)
        {
            if (altModeInstance->altModeConfig->altModeHostConfig.modules[moduleIndex].SVID ==
                s_ModuleDPInterfaces[interfaceIndex].SVID)
            {
                altModeInstance->modulesInterfaces[moduleIndex] = &s_ModuleDPInterfaces[interfaceIndex];
                break;
            }
        }
    }
}

static void PD_AltModeSetMsg(pd_alt_mode_t *altModeInstance, uint32_t events)
{
    pd_alt_mode_q_item_t qItem;

    if (!(s_MsgQHandle))
    {
        return;
    }

    qItem.altModeInstanceHandle = altModeInstance;
    qItem.event                 = events;
    USB_OsaMsgqSend(s_MsgQHandle, &qItem);
}

static uint32_t PD_AltModeGetMsg(pd_alt_mode_q_item_t *qItem)
{
    if (!(s_MsgQHandle))
    {
        return kStatus_USB_OSA_Error;
    }

    return USB_OsaMsgqRecv(s_MsgQHandle, qItem, 0);
}

static void PD_AltModeDelayRetryCommand(pd_alt_mode_t *altModeInstance, uint8_t command, uint32_t delay)
{
    altModeInstance->delayTime    = delay;
    altModeInstance->retryCommand = command;
}

static void PD_AltModeTrigerCommand(pd_alt_mode_t *altModeInstance, uint8_t command)
{
    altModeInstance->dfpCommand = command;
    PD_AltModeSetMsg(altModeInstance, PD_ALT_MODE_EVENT_COMMAND);
}

static pd_status_t PD_AltModeSendCommand(pd_alt_mode_t *altModeInstance, uint8_t command)
{
    pd_svdm_command_param_t structuredVDMCommandParam;
    uint32_t vdmCommand = 0;
    void *param         = NULL;

    structuredVDMCommandParam.vdmSop                          = kPD_MsgSOP;
    structuredVDMCommandParam.vdmHeader.bitFields.SVID        = 0xFF00u;
    structuredVDMCommandParam.vdmHeader.bitFields.vdmType     = 1;
    structuredVDMCommandParam.vdmHeader.bitFields.objPos      = 0;
    structuredVDMCommandParam.vdmHeader.bitFields.commandType = kVDM_Initiator;

    switch (command)
    {
        case PD_DPM_CONTROL_DISCOVERY_IDENTITY:
            structuredVDMCommandParam.vdoCount                    = 0;
            structuredVDMCommandParam.vdoData                     = NULL;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_DiscoverIdentity;
            vdmCommand                                            = PD_DPM_CONTROL_DISCOVERY_IDENTITY;
            param                                                 = &structuredVDMCommandParam;
            break;

        case PD_DPM_CONTROL_DISCOVERY_SVIDS:
            structuredVDMCommandParam.vdoCount                    = 0;
            structuredVDMCommandParam.vdoData                     = NULL;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_DiscoverSVIDs;
            vdmCommand                                            = PD_DPM_CONTROL_DISCOVERY_SVIDS;
            param                                                 = &structuredVDMCommandParam;
            break;

        case PD_DPM_CONTROL_DR_SWAP:
        {
            uint8_t dataRole;
            PD_Control(altModeInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &dataRole);
            if (((altModeInstance->altModeConfig->altModeRole == kDataConfig_DFP) ||
                 (altModeInstance->altModeConfig->altModeRole == kDataConfig_DRD)) &&
                (dataRole == kPD_DataRoleUFP))
            {
                vdmCommand = PD_DPM_CONTROL_DR_SWAP;
            }
            else
            {
                PD_AltModeTrigerCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_IDENTITY);
            }
            param = NULL;
            break;
        }

        default:
            break;
    }

    if (vdmCommand != 0)
    {
        if (PD_Command(altModeInstance->pdHandle, vdmCommand, param) != kStatus_PD_Success)
        {
            /* wait and retry again */
            PD_AltModeDelayRetryCommand(altModeInstance, vdmCommand, PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
        }
    }

    return kStatus_PD_Success;
}

static void PD_AltModeInstanceReset(pd_alt_mode_t *altModeInstance)
{
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
    altModeInstance->dfpCommand   = 0;
    altModeInstance->altModeStart = 0;
#endif
    altModeInstance->retryCommand = 0;
}

#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
static pd_status_t PD_AltModeEnter(pd_alt_mode_t *altModeInstance)
{
    uint8_t dataRole;

    if ((altModeInstance->altModeConfig->altModeRole == kDataConfig_DFP) ||
        (altModeInstance->altModeConfig->altModeRole == kDataConfig_DRD))
    {
        altModeInstance->retryCommand = 0;
        PD_Control(altModeInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &dataRole);
        if (dataRole == kPD_DataRoleDFP)
        {
            /* start discover */
            PD_AltModeTrigerCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_IDENTITY);
        }
    }
    else
    {
        /* do nothing */
    }

    return kStatus_PD_Success;
}
#endif

void PD_AltModeModuleTaskWakeUp(pd_alt_mode_handle altModeHandle, void *moduleHandle)
{
    uint8_t index;
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;

    if ((altModeHandle == NULL) || (moduleHandle == NULL))
    {
        return;
    }

    for (index = 0; index < altModeInstance->altModeConfig->altModeHostConfig.moduleCount; ++index)
    {
        if (altModeInstance->altModeModuleInstance[index] == moduleHandle)
        {
            PD_AltModeSetMsg(altModeInstance, (0x00000001u << index));
        }
    }
}

void PD_AltModeTimer1msISR(void)
{
    pd_alt_mode_t *altModeInstance = NULL;
    uint8_t index;

    for (index = 0; index < sizeof(s_AltModeInstances) / sizeof(pd_alt_mode_t); ++index)
    {
        if (s_AltModeInstances[index].occupied)
        {
            altModeInstance = (pd_alt_mode_t *)&s_AltModeInstances[index];
            if (altModeInstance->delayTime > 0)
            {
                altModeInstance->delayTime--;
                if (altModeInstance->delayTime == 0)
                {
                    PD_AltModeTrigerCommand(altModeInstance, altModeInstance->retryCommand);
                }
            }

            /* modules' 1ms ISR */
            for (index = 0; index < altModeInstance->altModeConfig->altModeHostConfig.moduleCount; ++index)
            {
                if ((altModeInstance->altModeModuleInstance[index] != NULL) &&
                    (altModeInstance->modulesInterfaces[index]->pd_alt_mode_1ms_isr != NULL))
                {
                    altModeInstance->modulesInterfaces[index]->pd_alt_mode_1ms_isr(
                        altModeInstance->altModeModuleInstance[index]);
                }
            }
        }
    }
}

pd_status_t PD_AltModeInit(void)
{
    if (USB_OsaMsgqCreate(&s_MsgQHandle, PD_ALT_MODE_MSG_Q_ITEM_COUNT,
                          sizeof(pd_alt_mode_q_item_t) / sizeof(uint32_t)) == kStatus_USB_OSA_Success)
    {
        return kStatus_PD_Success;
    }

    return kStatus_PD_Error;
}

pd_status_t PD_AltModeDeinit(void)
{
    uint8_t index;

    for (index = 0; index < sizeof(s_AltModeInstances) / sizeof(pd_alt_mode_t); ++index)
    {
        if (s_AltModeInstances[index].occupied == 1)
        {
            return kStatus_PD_Error;
        }
    }
    if (s_MsgQHandle)
    {
        USB_OsaMsgqDestroy(s_MsgQHandle);
        s_MsgQHandle = NULL;
    }

    return kStatus_PD_Success;
}

pd_status_t PD_AltModeInstanceInit(pd_handle pdHandle, const pd_alt_mode_config_t *altModeConfig, void **altModeHandle)
{
    pd_alt_mode_t *altModeInstance = NULL;
    pd_status_t status             = kStatus_PD_Success;
    uint8_t index                  = 0;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    for (index = 0; index < sizeof(s_AltModeInstances) / sizeof(pd_alt_mode_t); ++index)
    {
        if (s_AltModeInstances[index].occupied == 0)
        {
            s_AltModeInstances[index].occupied = 1;
            altModeInstance                    = &s_AltModeInstances[index];
            break;
        }
    }
    if (altModeInstance == NULL)
    {
        USB_OSA_EXIT_CRITICAL();
        return kStatus_PD_Error;
    }
    USB_OSA_EXIT_CRITICAL();

    altModeInstance->pdHandle      = pdHandle;
    altModeInstance->altModeConfig = altModeConfig;
    PD_AltModeInstanceReset(altModeInstance);
    PD_AltModeGetModuleInterface(altModeInstance);
    /* according SVID list, initialize every alt mode module. */
    for (index = 0; index < altModeConfig->altModeHostConfig.moduleCount; ++index)
    {
        if (altModeInstance->modulesInterfaces[index] == NULL)
        {
            status = kStatus_PD_Error;
            break;
        }

        status = altModeInstance->modulesInterfaces[index]->pd_alt_mode_init(
            pdHandle, altModeInstance, altModeConfig->altModeHostConfig.modules[index].config,
            &(altModeInstance->altModeModuleInstance[index]));
        if ((altModeInstance->altModeModuleInstance[index] == NULL) || (status != kStatus_PD_Success))
        {
            status = kStatus_PD_Error;
            break;
        }
    }

    if (status != kStatus_PD_Success)
    {
        for (index = 0; index < altModeConfig->altModeHostConfig.moduleCount; ++index)
        {
            if (altModeInstance->altModeModuleInstance[index] != NULL)
            {
                altModeInstance->modulesInterfaces[index]->pd_alt_mode_deinit(
                    altModeInstance->altModeModuleInstance[index]);
                altModeInstance->altModeModuleInstance[index] = NULL;
            }
        }
        s_AltModeInstances[index].occupied = 0;
        return status;
    }

    *altModeHandle = altModeInstance;
    return kStatus_PD_Success;
}

pd_status_t PD_AltModeInstanceDeinit(pd_alt_mode_handle altModeHandle)
{
    uint8_t index                  = 0;
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;

    if (altModeHandle == NULL)
    {
        return kStatus_PD_Error;
    }

    /* according SVID list, initialize every alt mode module. */
    for (index = 0; index < altModeInstance->altModeConfig->altModeHostConfig.moduleCount; ++index)
    {
        if (altModeInstance->altModeModuleInstance[index] != NULL)
        {
            altModeInstance->modulesInterfaces[index]->pd_alt_mode_deinit(
                altModeInstance->altModeModuleInstance[index]);
            altModeInstance->altModeModuleInstance[index] = NULL;
        }
    }

    altModeInstance->occupied = 0;
    return kStatus_PD_Success;
}

pd_status_t PD_AltModeState(pd_handle pdHandle, uint8_t *state)
{
    uint8_t index                  = 0;
    pd_alt_mode_t *altModeInstance = NULL;
    pd_alt_mode_state_t enteredMode;

    if ((pdHandle == NULL) || (state == NULL))
    {
        return kStatus_PD_Error;
    }
    for (index = 0; index < sizeof(s_AltModeInstances) / sizeof(pd_alt_mode_t); ++index)
    {
        if ((s_AltModeInstances[index].occupied == 1) && (s_AltModeInstances[index].pdHandle == pdHandle))
        {
            altModeInstance = &s_AltModeInstances[index];
            break;
        }
    }
    if (altModeInstance == NULL)
    {
        return kStatus_PD_Error;
    }

    for (index = 0; index < altModeInstance->altModeConfig->altModeHostConfig.moduleCount; ++index)
    {
        if (altModeInstance->altModeModuleInstance[index] != NULL)
        {
            enteredMode.SVID = 0;
            altModeInstance->modulesInterfaces[index]->pd_alt_mode_control(
                altModeInstance->altModeModuleInstance[index], kAltMode_GetModeState, &enteredMode);
            if (enteredMode.SVID != 0)
            {
                *state = 1;
                return kStatus_PD_Success;
            }
        }
    }

    return kStatus_PD_Success;
}

void PD_AltModeTask(void)
{
    pd_alt_mode_t *altModeInstance;
    pd_alt_mode_q_item_t qItem;
    uint8_t index;

    /* Alt Mode events process */
    if (PD_AltModeGetMsg(&qItem) == kStatus_USB_OSA_Success)
    {
        altModeInstance = (pd_alt_mode_t *)qItem.altModeInstanceHandle;
        if ((qItem.altModeInstanceHandle == NULL) || (qItem.event == 0))
        {
            return;
        }
        if (qItem.event & PD_ALT_MODE_EVENT_COMMAND)
        {
            if (altModeInstance->retryCommand != altModeInstance->dfpCommand)
            {
                altModeInstance->retryCount   = PD_ALT_MODE_COMMAND_RETRY_COUNT;
                altModeInstance->retryCommand = altModeInstance->dfpCommand;
                PD_AltModeSendCommand(altModeInstance, altModeInstance->dfpCommand);
            }
            else
            {
                if (altModeInstance->retryCount > 0)
                {
                    altModeInstance->retryCount--;
                    PD_AltModeSendCommand(altModeInstance, altModeInstance->dfpCommand);
                }
                else
                {
                    /* do hard reset */
                    /* PD_Command(altModeInstance->pdHandle, PD_DPM_CONTROL_HARD_RESET, NULL); */
                }
            }
        }
        else
        {
        }

        if (qItem.event & PD_ALT_MODE_EVENT_MODULES)
        {
            for (index = 0; index < altModeInstance->altModeConfig->altModeHostConfig.moduleCount; ++index)
            {
                if (qItem.event & (0x00000001u << index))
                {
                    altModeInstance->modulesInterfaces[index]->pd_alt_mode_task(
                        altModeInstance->altModeModuleInstance[index]);
                }
            }
        }
    }
}

static pd_status_t PD_AltModeStandardVDMCallbackProcess(pd_alt_mode_t *altModeInstance, uint32_t event, void *param)
{
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
    pd_svdm_command_request_t *svdmRequest = (pd_svdm_command_request_t *)param;
#endif
    pd_status_t status = kStatus_PD_Error;

    switch (event)
    {
        case PD_DPM_STRUCTURED_VDM_REQUEST:
#if (defined PD_CONFIG_ALT_MODE_SLAVE_SUPPORT) && (PD_CONFIG_ALT_MODE_SLAVE_SUPPORT)
            /* ack or nak, no busy */
            /* partner return nak if it is not in the alternate mode */
            switch (svdmRequest->vdmHeader.bitFields.command)
            {
                case kVDM_DiscoverIdentity: /* UFP */
                    svdmRequest->vdoData  = altModeInstance->altModeConfig->altModeSlaveConfig.identityData;
                    svdmRequest->vdoCount = altModeInstance->altModeConfig->altModeSlaveConfig.identityObjectCount;
                    svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                    break;
                case kVDM_DiscoverSVIDs: /* UFP */
                {
                    uint8_t index;
                    for (index = 0; index < altModeInstance->altModeConfig->altModeSlaveConfig.moduleCount; ++index)
                    {
                        uint32_t svidsObj = 0;
                        if (index & 0x01u)
                        {
                            svidsObj = altModeInstance->dataBuff[index / 2];
                        }
                        if (!(index & 0x01u))
                        {
                            svidsObj &= 0x0000FFFFu;
                            svidsObj |=
                                ((uint32_t)altModeInstance->altModeConfig->altModeSlaveConfig.modules[index].SVID
                                 << 16);
                        }
                        else
                        {
                            svidsObj &= 0xFFFF0000u;
                            svidsObj |= altModeInstance->altModeConfig->altModeSlaveConfig.modules[index].SVID;
                        }
                        altModeInstance->dataBuff[index / 2] = svidsObj;
                    }
                    svdmRequest->vdoData             = altModeInstance->dataBuff;
                    svdmRequest->vdoCount            = ((index + 1) >> 1);
                    svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                    break;
                }
                case kVDM_DiscoverModes: /* UFP (module self process) */
                case kVDM_EnterMode:     /* UFP (module self process) */
                case kVDM_ExitMode:      /* UFP (module self process) */
                case kVDM_Attention:     /* DFP (module self process) */
                    /* PD VID NAK */
                    svdmRequest->vdoData             = NULL;
                    svdmRequest->vdoCount            = 0;
                    svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                    break;

                default:
                    break;
            }
            status = kStatus_PD_Success;
#endif
            break;

        case PD_DPM_STRUCTURED_VDM_SUCCESS:
        {
            pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;
            switch (svdmResult->vdmCommand)
            {
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
                case kVDM_DiscoverIdentity: /* ACK msg, DFP */
                    /* 1. TODO, receive identity */
                    /* 2. get SVIDs */
                    PD_AltModeTrigerCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_SVIDS);
                    break;

                case kVDM_DiscoverSVIDs: /* ACK msg, DFP */
                {
                    uint8_t index;
                    uint16_t svid = 0;
                    /* what SVID to enter?  */
                    for (index = 0; index < altModeInstance->altModeConfig->altModeHostConfig.moduleCount; ++index)
                    {
                        uint8_t index1 = 0;
                        for (index1 = 0; index1 < svdmResult->vdoCount * 2; ++index1)
                        {
                            svid = (uint16_t)(svdmResult->vdoData[index1 >> 1] >> ((index1 & 0x01u) ? 0 : 16));
                            if (svid == altModeInstance->altModeConfig->altModeHostConfig.modules[index].SVID)
                            {
                                break;
                            }
                        }
                        if (index1 < svdmResult->vdoCount * 2)
                        {
                            break;
                        }
                    }
                    if (index < altModeInstance->altModeConfig->altModeHostConfig.moduleCount)
                    {
                        altModeInstance->modulesInterfaces[index]->pd_alt_mode_control(
                            altModeInstance->altModeModuleInstance[index], kAltMode_TriggerEnterMode, NULL);
                    }
                    else
                    {
                        /* if SVIDs is not end, get next SVIDs */
                        /* last SVID */
                        svid = (uint16_t)(svdmResult->vdoData[svdmResult->vdoCount - 1] & 0x0000FFFFu);
                        if (svid != 0x0000u)
                        {
                            /* get next SVIDs LIST */
                            PD_AltModeTrigerCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_SVIDS);
                        }
                    }
                    break;
                }

                case kVDM_DiscoverModes: /* ACK msg, DFP */
                case kVDM_EnterMode:     /* ACK msg, DFP */
                case kVDM_ExitMode:      /* ACK msg, DFP */
                                         /* shoudn't go here becase don't do PD VID discover/enter/exit,
                                            they all dedicated module related */
                    ;
                    break;
#endif

                default:
                    break;
            }
            status = kStatus_PD_Success;
            break;
        }

        case PD_DPM_STRUCTURED_VDM_FAIL:
        {
            pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;
            if (svdmResult->vdmCommandResult == kCommandResult_VDMNAK)
            {
                /* don't support this command */
                return kStatus_PD_Success;
            }
            if (svdmResult->vdmCommand == kVDM_DiscoverIdentity)
            {
                /* wait and retry again */
                PD_AltModeDelayRetryCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_IDENTITY,
                                            PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
            }
            else if (svdmResult->vdmCommand == kVDM_DiscoverSVIDs)
            {
                /* wait and retry again */
                PD_AltModeDelayRetryCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_SVIDS,
                                            PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
            }
            else
            {
            }
            status = kStatus_PD_Success;
            break;
        }
    }

    return status;
}

static pd_status_t PD_AltModeNotifyModulesEvent(pd_alt_mode_t *altModeInstance,
                                                uint32_t eventCode,
                                                void *eventParam,
                                                uint16_t svid)
{
    pd_status_t status = kStatus_PD_Error;
    pd_status_t statusTmp;

    if (eventCode != kAltMode_Invalid)
    {
        for (uint8_t index = 0; index < altModeInstance->altModeConfig->altModeHostConfig.moduleCount; ++index)
        {
            if (altModeInstance->altModeModuleInstance[index] != NULL)
            {
                if (((svid != 0) && (svid == altModeInstance->altModeConfig->altModeHostConfig.modules[index].SVID)) ||
                    (svid == 0))
                {
                    statusTmp = altModeInstance->modulesInterfaces[index]->pd_alt_mode_callback_event(
                        altModeInstance->altModeModuleInstance[index], eventCode, svid, eventParam);
                    if (status != kStatus_PD_Success)
                    {
                        status = statusTmp;
                    }
                }
            }
        }
    }
    return status;
}

pd_status_t PD_AltModeCallback(pd_handle pdHandle, uint32_t event, void *param)
{
    /* if this event is processed, return kStatus_PD_Success */
    pd_status_t status             = kStatus_PD_Error;
    uint32_t index                 = 0;
    pd_alt_mode_t *altModeInstance = NULL;
    uint32_t controlCode           = kAltMode_Invalid;
    void *controlParam             = NULL;
    uint16_t msgSVID               = 0;

    if (pdHandle == NULL)
    {
        return kStatus_PD_Error;
    }
    for (index = 0; index < sizeof(s_AltModeInstances) / sizeof(pd_alt_mode_t); ++index)
    {
        if ((s_AltModeInstances[index].occupied == 1) && (s_AltModeInstances[index].pdHandle == pdHandle))
        {
            altModeInstance = &s_AltModeInstances[index];
            break;
        }
    }
    if (altModeInstance == NULL)
    {
        return kStatus_PD_Error;
    }

    /* process attach/detach events */
    switch (event)
    {
        case PD_CONNECTED:
        case PD_CONNECT_ROLE_CHANGE:
            PD_AltModeInstanceReset(altModeInstance);
            controlCode  = kAltMode_Attach;
            controlParam = NULL;
            break;

        case PD_DISCONNECTED:
            PD_AltModeInstanceReset(altModeInstance);
            controlCode  = kAltMode_Detach;
            controlParam = NULL;
            break;

        case PD_DPM_SNK_HARD_RESET_REQUEST:
        case PD_DPM_SRC_HARD_RESET_REQUEST:
            PD_AltModeInstanceReset(altModeInstance);
            controlCode  = kAltMode_HardReset;
            controlParam = NULL;
            break;

        case PD_DPM_SNK_RDO_SUCCESS:
        case PD_DPM_SRC_RDO_SUCCESS:
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
            if (!altModeInstance->altModeStart)
            {
                altModeInstance->altModeStart = 1;
                /* trigger to start alt mode function */
                PD_AltModeEnter(altModeInstance);
            }
#endif
            break;

        default:
            break;
    }

    if (controlCode != kAltMode_Invalid)
    {
        return PD_AltModeNotifyModulesEvent(altModeInstance, controlCode, controlParam, 0x00u);
    }

    /* process AMS events */
    switch (event)
    {
        /* structured vdm */
        case PD_DPM_STRUCTURED_VDM_REQUEST:
        case PD_DPM_STRUCTURED_VDM_SUCCESS:
        case PD_DPM_STRUCTURED_VDM_FAIL:
            if (event == PD_DPM_STRUCTURED_VDM_SUCCESS)
            {
                controlCode = kAltMode_StructedVDMMsgSuccess;
            }
            else if (event == PD_DPM_STRUCTURED_VDM_REQUEST)
            {
                controlCode = kAltMode_StructedVDMMsgReceivedProcess;
            }
            else
            {
                controlCode = kAltMode_StructedVDMMsgFail;
            }
            controlParam = param;
            msgSVID      = ((pd_svdm_command_result_t *)param)->vdmHeader.bitFields.SVID;
            break;

        /* unstructured vdm */
        case PD_DPM_UNSTRUCTURED_VDM_RECEIVED:
            controlCode  = kAltMode_UnstructedVDMMsgReceived;
            controlParam = param;
            msgSVID      = (((pd_unstructured_vdm_command_param_t *)param)->vdmHeaderAndVDOsData[0] >> 16);
            break;

        case PD_DPM_SEND_UNSTRUCTURED_VDM_SUCCESS:
        case PD_DPM_SEND_UNSTRUCTURED_VDM_FAIL:
            controlCode  = kAltMode_UnstructedVDMMsgSentResult;
            controlParam = param;
            msgSVID      = 0;
            break;

        default:
            break;
    }

    if (controlCode != kAltMode_Invalid)
    {
        if (msgSVID == 0xFF00u)
        {
            /* standard VDM process */
            status = PD_AltModeStandardVDMCallbackProcess(altModeInstance, event, param);
        }
        else
        {
            status = PD_AltModeNotifyModulesEvent(altModeInstance, controlCode, controlParam, msgSVID);
        }
    }

    /* process AMS events */
    switch (event)
    {
        /* dr swap */
        case PD_DPM_DR_SWAP_SUCCESS:
#if (defined PD_CONFIG_ALT_MODE_HOST_SUPPORT) && (PD_CONFIG_ALT_MODE_HOST_SUPPORT)
            altModeInstance->altModeStart = 1;
            PD_AltModeEnter(altModeInstance);
#endif
            break;

        case PD_DPM_DR_SWAP_FAIL:
#if 0
            if (altModeInstance->dfpCommand == PD_DPM_CONTROL_DR_SWAP)
            {
                /* wait and retry again */
                PD_AltModeDelayRetryCommand(altModeInstance, PD_DPM_CONTROL_DR_SWAP, PD_ALT_MODE_DR_SWAP_DELAY_TIME);
            }
#endif
            break;

        default:
            break;
    }

    return status;
}

void PD_AltModeControl(void *altModeHandle, void *controlParam)
{
    pd_alt_mode_t *altModeInstance        = (pd_alt_mode_t *)altModeHandle;
    pd_alt_mode_control_t *altModeControl = (pd_alt_mode_control_t *)controlParam;

    if ((altModeControl->altModeModuleIndex <= 0) ||
        (altModeControl->altModeModuleIndex > altModeInstance->altModeConfig->altModeHostConfig.moduleCount))
    {
        return;
    }
    if (altModeInstance->altModeModuleInstance[altModeControl->altModeModuleIndex - 1] == NULL)
    {
        return;
    }

    altModeInstance->modulesInterfaces[altModeControl->altModeModuleIndex - 1]->pd_alt_mode_control(
        altModeInstance->altModeModuleInstance[altModeControl->altModeModuleIndex - 1], altModeControl->controlCode,
        altModeControl->controlParam);
    return;
}

uint8_t PD_AltModeExitModeForDrSwap(pd_alt_mode_handle altModeHandle)
{
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;
    uint8_t result                 = 0;

    for (uint8_t index = 0; index < altModeInstance->altModeConfig->altModeHostConfig.moduleCount; ++index)
    {
        if (altModeInstance->altModeModuleInstance[index] != NULL)
        {
            result = altModeInstance->modulesInterfaces[index]->pd_alt_mode_control(
                altModeInstance->altModeModuleInstance[index], kAltMode_TriggerExitMode, &result);
        }
    }

    return result;
}

#endif
