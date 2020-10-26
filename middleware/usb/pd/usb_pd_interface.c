/*
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_timer.h"
#include "usb_pd_interface.h"
#if ((defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT))
#include "usb_pd_alt_mode.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void PD_MsgInit(pd_instance_t *pdInstance);
void PD_MsgReceived(pd_instance_t *pdInstance, uint32_t msgLength, pd_status_t result);
void PD_MsgSendDone(pd_instance_t *pdInstance, pd_status_t result);
void PD_StackStateMachine(pd_instance_t *pdInstance);
void PD_ConnectSetPowerProgress(pd_instance_t *pdInstance, uint8_t state);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static pd_instance_t s_PDInstance[PD_CONFIG_MAX_PORT];
#if (defined PD_CONFIG_COMMON_TASK) && (PD_CONFIG_COMMON_TASK)
usb_osa_event_handle g_PDStackEventHandle;
#endif
#if ((defined PD_CONFIG_COMMON_TASK) && (PD_CONFIG_COMMON_TASK)) || \
    ((defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT))
static uint8_t g_PDInitializeLabel = 0xFFu;
#endif

#if ((defined PD_CONFIG_PTN5100_PORT) && (PD_CONFIG_PTN5100_PORT))
static const pd_phy_api_interface_t s_PTN5100Interface = {
    PDphy_PTN5100Init, PDphy_PTN5100Deinit, PDphy_PTN5100Send, PDphy_PTN5100Receive, PDphy_PTN5100Control,
};
#endif /* PD_CONFIG_PTN5100_PORT */

#if ((defined PD_CONFIG_PTN5110_PORT) && (PD_CONFIG_PTN5110_PORT))
static const pd_phy_api_interface_t s_PTN5110Interface = {
    PDPTN5110_Init, PDPTN5110_Deinit, PDPTN5110_Send, PDPTN5110_Receive, PDPTN5110_Control,
};
#endif /* PD_CONFIG_PTN5110_PORT */

/*******************************************************************************
 * Code
 ******************************************************************************/
static pd_instance_t *PD_GetInstance(void)
{
    uint8_t i = 0;
    uint32_t j;
    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();
    for (; i < PD_CONFIG_MAX_PORT; i++)
    {
        if (s_PDInstance[i].occupied != 1)
        {
            uint8_t *buffer = (uint8_t *)&s_PDInstance[i];
            for (j = 0U; j < sizeof(pd_instance_t); j++)
            {
                buffer[j] = 0x00U;
            }
            s_PDInstance[i].occupied = 1;
            USB_OSA_EXIT_CRITICAL();
            return &s_PDInstance[i];
        }
    }
    USB_OSA_EXIT_CRITICAL();
    return NULL;
}

static void PD_ReleaseInstance(pd_instance_t *pdInstance)
{
    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();
    pdInstance->occupied = 0;
    USB_OSA_EXIT_CRITICAL();
}

static void PD_GetPhyInterface(uint8_t phyType, const pd_phy_api_interface_t **controllerTable)
{
#if ((defined PD_CONFIG_PTN5100_PORT) && (PD_CONFIG_PTN5100_PORT))
    if (phyType == kPD_PhyPTN5100)
    {
        *controllerTable = &s_PTN5100Interface;
    }
#endif
#if ((defined PD_CONFIG_PTN5110_PORT) && (PD_CONFIG_PTN5110_PORT))
    if (phyType == kPD_PhyPTN5110)
    {
        *controllerTable = &s_PTN5110Interface;
    }
#endif
}

uint8_t PD_StackHasPendingEvent(pd_instance_t *pdInstance)
{
    uint32_t event = 0;
#if (defined PD_CONFIG_COMMON_TASK) && (PD_CONFIG_COMMON_TASK)
    USB_OsaEventCheck(g_PDStackEventHandle, 0xffffu, &event);
#else
    USB_OsaEventCheck(pdInstance->taskEventHandle, 0xffffu, &event);
#endif
    if (event)
    {
        return 1;
    }
    return 0;
}

void PD_StackSetEvent(pd_instance_t *pdInstance, uint32_t event)
{
    USB_OsaEventSet(pdInstance->taskEventHandle, event);
#if (defined PD_CONFIG_COMMON_TASK) && (PD_CONFIG_COMMON_TASK)
    USB_OsaEventSet(g_PDStackEventHandle, (0x00000001u << (pdInstance - &s_PDInstance[0])));
#endif
}

pd_status_t PD_PhyControl(pd_instance_t *pdInstance, uint32_t control, void *param)
{
    if ((control == PD_PHY_UPDATE_STATE) ||
        (USB_OsaEventCheck(pdInstance->taskEventHandle, PD_TASK_EVENT_PHY_STATE_CHAGNE, NULL) ==
         kStatus_USB_OSA_Success))
    {
        USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_PHY_STATE_CHAGNE);
        pdInstance->phyInterface->pdPhyControl(pdInstance->pdPhyHandle, PD_PHY_UPDATE_STATE, NULL);
        NVIC_EnableIRQ((IRQn_Type)pdInstance->pdConfig->phyInterruptNum);
    }

    if (control != PD_PHY_UPDATE_STATE)
    {
        return pdInstance->phyInterface->pdPhyControl(pdInstance->pdPhyHandle, control, param);
    }
    else
    {
        return kStatus_PD_Success;
    }
}

pd_status_t PD_InstanceInit(pd_handle *pdHandle,
                            pd_stack_callback_t callbackFn,
                            pd_power_handle_callback_t *callbackFunctions,
                            void *callbackParam,
                            pd_instance_config_t *config)
{
    pd_instance_t *pdInstance;
    pd_status_t status = kStatus_PD_Success;
    pd_phy_config_t phyConfig;

#if ((defined PD_CONFIG_COMMON_TASK) && (PD_CONFIG_COMMON_TASK)) || \
    ((defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT))
    if (g_PDInitializeLabel == 0xFFu)
    {
        g_PDInitializeLabel = 0x00u;
#if (defined PD_CONFIG_COMMON_TASK) && (PD_CONFIG_COMMON_TASK)
        if (kStatus_USB_OSA_Success != USB_OsaEventCreate(&(g_PDStackEventHandle), kUSB_OsaEventAutoClear))
        {
            return kStatus_PD_Error;
        }
        for (uint8_t index = 0; index < PD_CONFIG_MAX_PORT; ++index)
        {
            s_PDInstance[index].occupied = 0;
        }
#endif
#if ((defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT))
        PD_AltModeInit();
#endif
    }
#endif

    pdInstance = PD_GetInstance();
    if (pdInstance == NULL)
    {
        return kStatus_PD_Error;
    }

    /* get phy API table */
    pdInstance->phyInterface = NULL;
    PD_GetPhyInterface(config->phyType, &pdInstance->phyInterface);
    if ((pdInstance->phyInterface == NULL) || (pdInstance->phyInterface->pdPhyInit == NULL) ||
        (pdInstance->phyInterface->pdPhyDeinit == NULL) || (pdInstance->phyInterface->pdPhySend == NULL) ||
        (pdInstance->phyInterface->pdPhyReceive == NULL) || (pdInstance->phyInterface->pdPhyControl == NULL))
    {
        PD_ReleaseInstance(pdInstance);
        return kStatus_PD_Error;
    }
    pdInstance->pdCallback = callbackFn;
    pdInstance->callbackFns = callbackFunctions;
    pdInstance->callbackParam = callbackParam;
    pdInstance->revision = PD_CONFIG_REVISION;
    pdInstance->sendingMsgHeader.bitFields.specRevision = pdInstance->revision;
    pdInstance->initializeLabel = 0;
    pdInstance->taskWaitTime = PD_WAIT_EVENT_TIME;
    pdInstance->curConnectState = TYPEC_DISABLED;
    pdInstance->partnerSourcePDOsCount = 0;

    if (kStatus_USB_OSA_Success != USB_OsaEventCreate(&(pdInstance->taskEventHandle), 0))
    {
        PD_ReleaseInstance(pdInstance);
        return kStatus_PD_Error;
    }

    /* initialize PHY */
    pdInstance->pdPhyHandle = NULL;
    pdInstance->pdConfig = config;
    if (pdInstance->pdConfig->deviceType == kDeviceType_NormalPowerPort)
    {
        pdInstance->pdPowerPortConfig = (pd_power_port_config_t *)pdInstance->pdConfig->deviceConfig;
        pdInstance->pendingSOP = kPD_MsgSOPMask;
    }
    else if (pdInstance->pdConfig->deviceType == kDeviceType_Cable)
    {
        pdInstance->pendingSOP = kPD_MsgSOPpMask | kPD_MsgSOPppMask;
    }
    else
    {
        pdInstance->pendingSOP = kPD_MsgSOPMask;
    }
    phyConfig.interface = config->phyInterface;
    phyConfig.interfaceParam = config->interfaceParam;
    status = pdInstance->phyInterface->pdPhyInit(pdInstance, &(pdInstance->pdPhyHandle), &phyConfig);
    if ((status != kStatus_PD_Success) || (pdInstance->pdPhyHandle == NULL))
    {
        USB_OsaEventDestroy(pdInstance->taskEventHandle);
        PD_ReleaseInstance(pdInstance);
        return kStatus_PD_Error;
    }
    /* initialize pd stack */
    PD_MsgInit(pdInstance);
    PD_TimerInit(pdInstance);

    PD_PhyControl(pdInstance, PD_PHY_GET_PHY_VENDOR_INFO, &pdInstance->phyInfo);

    PD_StackSetEvent(pdInstance, PD_TASK_EVENT_RESET_CONFIGURE);

#if ((defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT))
    /* initialize alt mode */
    if (pdInstance->pdPowerPortConfig->altModeConfig != NULL)
    {
        if (PD_AltModeInstanceInit(pdInstance, (pd_alt_mode_config_t *)(pdInstance->pdPowerPortConfig->altModeConfig),
                                   &(pdInstance->altModeHandle)) != kStatus_PD_Success)
        {
            pdInstance->phyInterface->pdPhyDeinit(pdInstance->pdPhyHandle);
            USB_OsaEventDestroy(pdInstance->taskEventHandle);
            PD_ReleaseInstance(pdInstance);
        }
    }
#endif
    *pdHandle = pdInstance;

    return kStatus_PD_Success;
}

pd_status_t PD_InstanceDeinit(pd_handle pdHandle)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    pd_status_t status = kStatus_PD_Success;

#if ((defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT))
    PD_AltModeInstanceDeinit(pdInstance->altModeHandle);
#endif
    USB_OsaEventDestroy(pdInstance->taskEventHandle);
    PD_ReleaseInstance(pdInstance);
    status = pdInstance->phyInterface->pdPhyDeinit(pdInstance->pdPhyHandle);
    PD_ReleaseInstance(pdInstance);

    return status;
}

#if !((defined PD_CONFIG_COMMON_TASK) && (PD_CONFIG_COMMON_TASK))
void PD_InstanceTask(pd_handle pdHandle)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    if (pdHandle == NULL)
    {
        return;
    }
    PD_StackStateMachine(pdInstance);
}
#endif

#if (defined PD_CONFIG_COMMON_TASK) && (PD_CONFIG_COMMON_TASK)
void PD_Task(void)
{
    uint32_t taskEventSet = 0;
    uint8_t index;

    if ((g_PDInitializeLabel == 0xFFu) || (g_PDStackEventHandle == NULL))
    {
        return;
    }

    USB_OsaEventWait(g_PDStackEventHandle, 0xffffu, 0, PD_WAIT_EVENT_TIME, &taskEventSet);
    /* if wait forever this will not enter or BM enter */
    if (taskEventSet == 0)
    {
        if (PD_WAIT_EVENT_TIME != 0)
        {
            for (index = 0; index < PD_CONFIG_MAX_PORT; ++index)
            {
                if (s_PDInstance[index].occupied)
                {
                    PD_StackStateMachine(&s_PDInstance[index]);
                }
            }
        }
        return;
    }

    /* process events */
    for (index = 0; index < PD_CONFIG_MAX_PORT; ++index)
    {
        if (taskEventSet & (0x00000001u << index))
        {
            /* The index instance has events */
            PD_StackStateMachine(&s_PDInstance[index]);
        }
    }
}
#endif

void PD_Notify(pd_handle pdHandle, uint32_t event, void *param)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    switch (event)
    {
        case PD_PHY_EVENT_STATE_CHANGE:
            NVIC_DisableIRQ((IRQn_Type)pdInstance->pdConfig->phyInterruptNum);
            PD_StackSetEvent(pdInstance, PD_TASK_EVENT_PHY_STATE_CHAGNE);
            PD_StackSetEvent(pdInstance, PD_TASK_EVENT_TYPEC_STATE_PROCESS);
            break;

        case PD_PHY_EVENT_SEND_COMPLETE:
        {
            PD_MsgSendDone(pdInstance, (pd_status_t) * ((uint8_t *)param));
            break;
        }

        case PD_PHY_EVENT_FR_SWAP_SINGAL_RECEIVED:
        {
            if (pdInstance->inProgress != kVbusPower_InFRSwap)
            {
                PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InFRSwap);
#if 0
                if (!PD_DpmCheckLessOrEqualVsafe5v(pdInstance))
                {
                    pdInstance->callbackFns->PD_SrcTurnOffVbus(pdInstance->callbackParam, kVbusPower_Stable);
                }
#endif
                PD_StackSetEvent(pdInstance, PD_TASK_EVENT_FR_SWAP_SINGAL);
            }
            break;
        }

        case PD_PHY_EVENT_RECEIVE_COMPLETE:
        {
            pd_phy_rx_result_t *rxResult = (pd_phy_rx_result_t *)param;
            pdInstance->receivedSop = rxResult->rxSop;
            PD_MsgReceived(pdInstance, rxResult->rxLength, (pd_status_t)rxResult->rxResultStatus);
            break;
        }

        case PD_PHY_EVENT_HARD_RESET_RECEIVED:
        {
            PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InHardReset);
            pdInstance->hardResetReceived = 1;
            PD_StackSetEvent(pdInstance, PD_TASK_EVENT_RECEIVED_HARD_RESET);
            break;
        }

        case PD_PHY_EVENT_REQUEST_STACK_RESET:
            /* BootSoftRebootToMain(); */
            break;

        case PD_PHY_EVENT_VCONN_PROTECTION_FAULT:

            break;

        case PD_PHY_EVENT_TYPEC_0VP_OCP_FAULT:
        {
            pd_phy_config_t phyConfig;

            pdInstance->pdCallback(pdInstance->callbackParam, PD_DPM_OVP_OCP_FAULT, NULL);
            pdInstance->phyInterface->pdPhyDeinit(pdInstance->pdPhyHandle);
            phyConfig.interface = pdInstance->pdConfig->phyInterface;
            phyConfig.interfaceParam = pdInstance->pdConfig->interfaceParam;
            pdInstance->phyInterface->pdPhyInit(pdInstance, &(pdInstance->pdPhyHandle), &phyConfig);
            break;
        }

        case PD_PHY_EVENT_VBUS_STATE_CHANGE:
            PD_StackSetEvent(pdInstance, PD_TASK_EVENT_OTHER);
            break;

        default:
            break;
    }
}

void USB_WEAK_FUN PD_WaitUsec(uint32_t us)
{
    uint32_t usDelay;

    while (us--)
    {
        usDelay = 15;
        while (--usDelay)
        {
            __ASM("nop");
        }
    }
}
