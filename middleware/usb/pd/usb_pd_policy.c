/*
 * Copyright 2015 - 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_timer.h"
#include "usb_pd_interface.h"
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
#include "usb_pd_alt_mode.h"
#endif
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
#include "usb_pd_auto_policy.h"
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
#include "fsl_debug_console.h"
#endif
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if defined(PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG)
#undef PD_CONFIG_CABLE_COMMUNICATION_ENABLE
#define PD_CONFIG_CABLE_COMMUNICATION_ENABLE (1)
#endif

#if (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE))
#undef PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE
#define PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE (1)
#endif

#define N_CAPS_COUNT (50)
#define N_HARD_RESET_COUNT (2)
#define N_DISCOVER_IDENTITY_COUNTER (20)
#define PD_EXTENDED_SRC_CAP_DATA_LENGTH (23)
#define PD_EXTENDED_STATUS_MSG_DATA_LENGTH (3)
#define PD_EXTENDED_BATTERY_CAP_MSG_DATA_LENGTH (9)

#define MSG_DATA_BUFFER ((uint32_t *)(&(pdInstance->receivedData[1])))
#define MSG_DATA_HEADER (pdInstance->receivedData[0] >> 16)

#define PD_NOT_SUPPORT_REPLY_MSG ((pdInstance->revision >= PD_SPEC_REVISION_30) ? kPD_MsgNotSupported : kPD_MsgReject)

#define VDM_ID_HEADER_VDO_PASSIVE_CABLE_VAL (0x03u)
#define VDM_ID_HEADER_VDO_ACTIVE_CABLE_VAL (0x04u)

typedef enum _trigger_event
{
    PSM_TRIGGER_NONE,
    PSM_TRIGGER_NON_START,
    PSM_TRIGGER_DPM_MSG,
    PSM_TRIGGER_PD_MSG,
    PSM_TRIGGER_RECEIVE_HARD_RESET,
} trigger_event_t;

typedef struct _psm_trigger_info
{
    uint8_t triggerEvent;
    uint8_t pdMsgSop;
    uint8_t pdMsgType;
    uint8_t vdmMsgType;
    uint8_t *pdMsgDataBuffer;
    uint32_t pdMsgDataLength;
    uint32_t pdExtMsgLength;
    pd_structured_vdm_header_t vdmHeader;
    pd_msg_header_t msgHeader;

    uint8_t dpmMsg;
} psm_trigger_info_t;

typedef enum _pd_state_machine_state
{
    kSM_None = 0,
    kSM_Continue,
    kSM_WaitEvent,
    kSM_ErrorRecovery,
    kSM_Detach,
} pd_state_machine_state_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static uint8_t PD_PsmStartCommand(pd_instance_t *pdInstance, uint8_t command, uint8_t isInitiator);

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
static uint8_t Pd_PsmSecondaryStateHandler(pd_instance_t *pdInstance,
                                           uint8_t statIndex,
                                           psm_trigger_info_t *triggerInfo);
static void PD_PsmSecondaryStateHandlerTerminate(pd_instance_t *pdInstance, uint8_t sop);
#endif

void PD_MsgInit(pd_instance_t *pdInstance);
void PD_MsgReset(pd_instance_t *pdInstance);
void PD_MsgDisable(pd_instance_t *pdInstance);
void PD_MsgSetPortRole(pd_instance_t *pdInstance, uint8_t powerRole, uint8_t dataRole);

uint8_t PD_MsgWaitSendResult(pd_instance_t *pdInstance);

pd_status_t PD_MsgSendStructuredVDMAndWait(pd_instance_t *pdInstance,
                                           start_of_packet_t sop,
                                           pd_structured_vdm_header_t reponseVdmHeader,
                                           uint8_t count,
                                           uint32_t *vdos);

pd_status_t PD_MsgSendHardReset(pd_instance_t *pdInstance);

pd_status_t PD_MsgSend(
    pd_instance_t *pdInstance, start_of_packet_t sop, message_type_t msgType, uint32_t dataLength, uint8_t *dataBuffer);

void PD_MsgReceive(pd_instance_t *pdInstance);

void PD_MsgStopReceive(pd_instance_t *pdInstance);

void PD_MsgStartReceive(pd_instance_t *pdInstance);

uint8_t PD_MsgGetReceiveResult(pd_instance_t *pdInstance);

uint8_t PD_MsgRecvPending(pd_instance_t *pdInstance);

/* internal function */
static uint8_t PD_DpmGetMsg(pd_instance_t *pdInstance);

static void PD_DpmClearMsg(pd_instance_t *pdInstance, pd_command_t id);

static void PD_DpmSendMsg(pd_handle pdHandle, uint8_t id);

void PD_ConnectSetPowerProgress(pd_instance_t *pdInstance, uint8_t state);

pd_status_t PD_MsgSendRequestChunkMsg(pd_instance_t *pdInstance,
                                      start_of_packet_t sop,
                                      message_type_t extMsgType,
                                      pd_extended_msg_header_t extHeader);

pd_status_t PD_MsgSendExtendedMsg(pd_instance_t *pdInstance,
                                  start_of_packet_t sop,
                                  message_type_t extMsgType,
                                  uint32_t dataLength,
                                  uint8_t *dataBuffer);

pd_status_t PD_MsgSendChunkedExtendedMsg(pd_instance_t *pdInstance,
                                         start_of_packet_t sop,
                                         message_type_t extMsgType,
                                         pd_extended_msg_header_t extHeader,
                                         uint32_t dataLength,
                                         uint8_t *dataBuffer);

pd_status_t PD_MsgSendUnchunkedExtendedMsg(pd_instance_t *pdInstance,
                                           start_of_packet_t sop,
                                           message_type_t extMsgType,
                                           uint32_t dataLength,
                                           uint8_t *dataBuffer);

pd_status_t PD_MsgSendUnstructuredVDM(pd_instance_t *pdInstance,
                                      start_of_packet_t sop,
                                      uint8_t *dataBuffer,
                                      uint32_t dataLength);

void PD_DpmDischargeVbus(pd_instance_t *pdInstance, uint8_t enable);
void PD_ConnectInitRole(pd_instance_t *pdInstance, uint8_t errorRecovery);
uint8_t PD_ConnectCheck(pd_instance_t *pdInstance);
TypeCState_t PD_ConnectGetStateMachine(pd_instance_t *pdInstance);
void PD_ConnectAltModeEnterFail(pd_instance_t *pdInstance, uint8_t pdConnected);
void PD_DpmSetVconn(pd_instance_t *pdInstance, uint8_t enable);
void PD_ConnectSetPRSwapRole(pd_instance_t *pdInstance, uint8_t powerRole);
void PD_MsgSrcStartCommand(pd_instance_t *pdInstance);
uint8_t PD_MsgSnkCheckStartCommand(pd_instance_t *pdInstance);
void PD_MsgSrcEndCommand(pd_instance_t *pdInstance);

static pd_status_t PD_DpmAppCallback(pd_instance_t *pdInstance, uint32_t event, void *param, uint8_t done);
pd_status_t PD_PhyControl(pd_instance_t *pdInstance, uint32_t control, void *param);

void PD_DpmDischargeVconn(pd_instance_t *pdInstance, uint8_t enable);

TypeCState_t PD_ConnectGetInitRoleState(pd_instance_t *pdInstance);

void PD_StackSetEvent(pd_instance_t *pdInstance, uint32_t event);
uint8_t PD_StackHasPendingEvent(pd_instance_t *pdInstance);
#if defined(PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT)
static void PD_MsgChunkingLayerResetFlags(pd_instance_t *pdInstance);
#endif
#if defined(PD_CONFIG_ALT_MODE_DP_SUPPORT) && (PD_CONFIG_ALT_MODE_DP_SUPPORT)
void PD_AltModeControl(void *altModeHandle, void *controlParam);
#endif
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
uint8_t PD_CheckWhetherInitiateCableDiscoveryIdentityOrNot(pd_instance_t *pdInstance);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*! ***************************************************************************
   Send message transition functions
******************************************************************************/
static void PD_PsmTransitionOnMsgSendError(pd_instance_t *pdInstance,
                                           uint8_t interruptedState,
                                           pd_psm_state_t errorState)
{
#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)) || \
    ((defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT))
    if (0
#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE))
        || ((PD_MsgRecvPending(pdInstance)) &&
            ((pdInstance->receivedSop == kPD_MsgSOP) &&
             (((MSG_DATA_HEADER & PD_MSG_HEADER_MESSAGE_TYPE_MASK) >> PD_MSG_HEADER_MESSAGE_TYPE_POS) ==
              (kPD_MsgVendorDefined & PD_MSG_TYPE_VALUE_MASK))))
#endif
#if ((defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT))
        || ((PD_MsgRecvPending(pdInstance)) && (pdInstance->receivedSop == kPD_MsgSOP) &&
            (MSG_DATA_HEADER & PD_MSG_HEADER_EXTENDED_MASK))
#endif
    )
    {
        /* re-execute the psmCurState */
        if (pdInstance->psmCurState != PSM_INTERRUPTED_REQUEST)
        {
            pdInstance->psmInterruptedState = pdInstance->psmCurState;
            pdInstance->psmCurState         = PSM_INTERRUPTED_REQUEST;
        }
        pdInstance->psmNewState = PSM_INTERRUPTED_REQUEST;
    }
#endif
    if (PD_MsgRecvPending(pdInstance))
    {
        pdInstance->psmNewState = (pd_psm_state_t)interruptedState;
    }
    else
    {
        pdInstance->psmNewState = errorState;
    }
    return;
}

static uint8_t PD_MsgSendMsgCommonTransition(pd_instance_t *pdInstance,
                                             start_of_packet_t sop,
                                             uint8_t msgType,
                                             uint32_t dataLength,
                                             uint8_t *dataBuffer,
                                             pd_psm_state_t successNextState,
                                             uint8_t interruptedState,
                                             pd_psm_state_t errorState)
{
    pd_status_t result = kStatus_PD_Error;

#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    if (((msgType & PD_MSG_TYPE_BITS_MASK) == PD_MSG_CONTROL_TYPE_MASK) ||
        ((msgType & PD_MSG_TYPE_BITS_MASK) == PD_MSG_DATA_TYPE_MASK))
#endif
    {
        result = PD_MsgSend(pdInstance, sop, (message_type_t)msgType, 2 + (dataLength * 4), (uint8_t *)dataBuffer);
    }
#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    else
    {
        result = PD_MsgSendExtendedMsg(pdInstance, sop, (message_type_t)msgType, dataLength, dataBuffer);
    }
#endif

    if (result == kStatus_PD_Success)
    {
        if (PD_MsgWaitSendResult(pdInstance))
        {
            if (successNextState != PE_PSM_STATE_NO_CHANGE)
            {
                pdInstance->psmNewState = successNextState;
            }
            return 1; /* success */
        }
    }

    PD_PsmTransitionOnMsgSendError(pdInstance, interruptedState, errorState);

    return 0; /* fail */
}

static inline uint8_t PD_MsgSendDataTransition(pd_instance_t *pdInstance,
                                               uint8_t msgType,
                                               uint32_t doCount,
                                               uint32_t *dos,
                                               pd_psm_state_t successNextState,
                                               uint8_t interruptedState,
                                               pd_psm_state_t errorState)
{
    return PD_MsgSendMsgCommonTransition(pdInstance, kPD_MsgSOP, msgType, doCount, (uint8_t *)dos, successNextState,
                                         interruptedState, errorState);
}

static inline uint8_t PD_MsgSendControlTransition(pd_instance_t *pdInstance,
                                                  uint8_t msgType,
                                                  pd_psm_state_t successNextState,
                                                  uint8_t interruptedState,
                                                  pd_psm_state_t errorState)
{
    return PD_MsgSendMsgCommonTransition(pdInstance, kPD_MsgSOP, msgType, 0, NULL, successNextState, interruptedState,
                                         errorState);
}

#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
static inline uint8_t PD_MsgSendExtTransition(pd_instance_t *pdInstance,
                                              start_of_packet_t sop,
                                              uint8_t msgType,
                                              uint32_t dataLength,
                                              uint8_t *dataBuffer,
                                              pd_psm_state_t successNextState,
                                              uint8_t interruptedState,
                                              pd_psm_state_t errorState)
{
    return PD_MsgSendMsgCommonTransition(pdInstance, sop, msgType, dataLength, dataBuffer, successNextState,
                                         interruptedState, errorState);
}
#endif

static inline uint8_t PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pd_instance_t *pdInstance,
                                                                         uint8_t msgType,
                                                                         pd_psm_state_t successNextState)
{
    return PD_MsgSendControlTransition(pdInstance, msgType, successNextState, PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
}

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
static inline uint8_t PD_PsmSendControlTransitionWithErrorRecovery(pd_instance_t *pdInstance,
                                                                   uint8_t msgType,
                                                                   pd_psm_state_t successNextState)
{
    return PD_MsgSendControlTransition(pdInstance, msgType, successNextState, PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY,
                                       PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY);
}
#endif

static inline uint8_t PD_PsmSendControlTransitionWithHardReset(pd_instance_t *pdInstance,
                                                               uint8_t msgType,
                                                               pd_psm_state_t successNextState)
{
    return PD_MsgSendControlTransition(pdInstance, msgType, successNextState, PSM_HARD_RESET, PSM_HARD_RESET);
}

/* 0 - fail; 1 -success */
uint8_t PD_PsmSendControlTransitionWithTry(pd_instance_t *pdInstance,
                                           message_type_t msgType,
                                           pd_psm_state_t successNextState,
                                           uint8_t interruptedState,
                                           pd_psm_state_t errorState)
{
    uint8_t retryCount = 5;
    do
    {
        pd_status_t result = PD_MsgSend(pdInstance, kPD_MsgSOP, msgType, 2, NULL);
        if (result == kStatus_PD_Success)
        {
            if (PD_MsgWaitSendResult(pdInstance))
            {
                break; /* success */
            }
        }
    } while (--retryCount);

    if (retryCount == 0)
    {
        PD_PsmTransitionOnMsgSendError(pdInstance, interruptedState, errorState);
        return 0;
    }
    else
    {
        pdInstance->psmNewState = successNextState;
    }
    return 1;
}

static uint8_t PD_PsmSendControlTransitionWithSendResponserTimeOut(pd_instance_t *pdInstance,
                                                                   uint8_t msgType,
                                                                   pd_psm_state_t successNextState,
                                                                   uint8_t interruptedState,
                                                                   pd_psm_state_t errorState)
{
    if (PD_MsgSendControlTransition(pdInstance, msgType, successNextState, interruptedState, errorState))
    {
        PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
        return 1;
    }
    return 0;
}

/*! ***************************************************************************
   Other small functions
******************************************************************************/

static void PD_FRSControl(pd_instance_t *pdInstance, uint8_t enable)
{
    uint8_t control;
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
    if (enable)
    {
        if (pdInstance->revision >= PD_SPEC_REVISION_30)
        {
            control = 1;
            PD_PhyControl(pdInstance, PD_PHY_CONTROL_FR_SWAP, &control);
        }
    }
    else
#endif
    {
        control = 0;
        PD_PhyControl(pdInstance, PD_PHY_CONTROL_FR_SWAP, &control);
    }
}

static void PD_PsmReset(pd_instance_t *pdInstance)
{
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
    uint8_t uint8Tmp;
#endif

#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    pdInstance->vconnSwapResult                   = kCommandResult_None;
    pdInstance->drSwapResult                      = kCommandResult_None;
    pdInstance->portPartnerDrSwapToUFPRejected    = 0;
    pdInstance->portPartnerDrSwapToDFPRejected    = 0;
    pdInstance->portPartnerPrSwapToSinkRejected   = 0;
    pdInstance->portPartnerPrSwapToSourceRejected = 0;
    pdInstance->portPartnerVconnSwapToOnRejected  = 0;
    pdInstance->portPartnerVconnSwapToOffRejected = 0;
#endif
    pdInstance->psmGotoMinTx             = 0;
    pdInstance->psmGotoMinRx             = 0;
    pdInstance->psmHardResetNeedsVSafe0V = 0;
    pdInstance->psmPresentlyPdConnected  = 0;
    pdInstance->psmCablePlugResetNeeded  = 0;
    pdInstance->commandEvaluateResult    = kStatus_PD_Error;

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
    pdInstance->psmVdmActiveModeValidMask = 0u;
    for (uint8Tmp = 0; uint8Tmp < PSM_SECONDARY_STATE_COUNT; uint8Tmp++)
    {
        pdInstance->psmSecondaryState[uint8Tmp]    = PSM_IDLE;
        pdInstance->psmNewSecondaryState[uint8Tmp] = PSM_UNKNOWN;
    }
#endif

    PD_FRSControl(pdInstance, 0);
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
    pdInstance->frsEnabled           = 0;
    pdInstance->frSignaledWaitFrSwap = 0u;
#endif
    PD_TimerCancelAllTimers(pdInstance, tSenderResponseTimer, _tMaxPSMTimer);
}

static void PD_PsmSetNormalPower(pd_instance_t *pdInstance)
{
    switch (pdInstance->psmCurState)
    {
        /* the follow state is stable state (may stay long time) */
        case PSM_PE_SRC_READY:
        case PSM_PE_SNK_READY:
        case PSM_PE_SNK_WAIT_FOR_CAPABILITIES:
        case PSM_PE_SRC_SEND_CAPABILITIES:
        case PSM_PE_SRC_DISABLED:
        case PSM_EXIT_TO_ERROR_RECOVERY:
        case PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY:
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            if (pdInstance->inProgress != kVbusPower_InFRSwap)
#endif
            {
                PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
            }
            break;

        default:
            break;
    }
}

static void PD_PsmCheckRevision(pd_instance_t *pdInstance, pd_msg_header_t msgHeader)
{
    if (pdInstance->revision > msgHeader.bitFields.specRevision)
    {
        pdInstance->revision = msgHeader.bitFields.specRevision;
        PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
    }
}

#if (defined PD_CONFIG_PD3_PPS_ENABLE) && (PD_CONFIG_PD3_PPS_ENABLE)
/* for sink */
static uint8_t PD_PsmSinkIsPPSRDO(pd_instance_t *pdInstance)
{
    uint8_t rdoIndex = pdInstance->rdoRequest.bitFields.objectPosition - 1;

    if (rdoIndex < pdInstance->partnerSourcePDOsCount)
    {
        if ((pdInstance->partnerSourcePDOs[rdoIndex].commonPDO.pdoType == kPDO_APDO) &&
            (pdInstance->partnerSourcePDOs[rdoIndex].apdoPDO.APDOType == kAPDO_PPS))
        {
            return 1;
        }
    }
    return 0;
}

/* for source */
static uint8_t PD_PsmSourceIsPPSRDO(pd_instance_t *pdInstance)
{
    uint8_t rdoIndex = pdInstance->partnerRdoRequest.bitFields.objectPosition - 1;

    if (rdoIndex < pdInstance->pdPowerPortConfig->sourceCapCount)
    {
        pd_source_pdo_t sourcePDO;
        sourcePDO.PDOValue = pdInstance->pdPowerPortConfig->sourceCaps[rdoIndex];
        if ((sourcePDO.commonPDO.pdoType == kPDO_APDO) && (sourcePDO.apdoPDO.APDOType == kAPDO_PPS))
        {
            return 1;
        }
    }
    return 0;
}

#endif

#if ((defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)) || \
    ((defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE))
static uint8_t PD_PsmIsDualRole(pd_instance_t *pdInstance)
{
    if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault) ||
        (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceDefault) ||
        (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) ||
        (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSourcingDevice) ||
        (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSinkingHost))
    {
        return 1;
    }
    return 0;
}
#endif

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
static uint32_t *PD_PsmGetSourcePDOs(pd_instance_t *pdInstance)
{
    uint8_t index;
    pd_source_pdo_t *pdo;
    pd_source_pdo_t *destPdo;

    for (index = 0; index < pdInstance->pdPowerPortConfig->sourceCapCount; ++index)
    {
        pdo               = (pd_source_pdo_t *)&(pdInstance->pdPowerPortConfig->sourceCaps[index]);
        destPdo           = (pd_source_pdo_t *)&(pdInstance->sendingData[1 + index]);
        destPdo->PDOValue = pdo->PDOValue;

        switch (pdo->commonPDO.pdoType)
        {
            case kPDO_Fixed:
            {
                if (pdo->fixedPDO.maxCurrent > pdInstance->dpmCableMaxCurrent)
                {
                    destPdo->fixedPDO.maxCurrent = pdInstance->dpmCableMaxCurrent;
                }
                break;
            }

            case kPDO_Variable:
            {
                if (pdo->variablePDO.maxCurrent > pdInstance->dpmCableMaxCurrent)
                {
                    destPdo->variablePDO.maxCurrent = pdInstance->dpmCableMaxCurrent;
                }
                break;
            }

            case kPDO_Battery:
            {
                if ((pdo->batteryPDO.maxAllowPower * 500 / pdo->batteryPDO.minVoltage) > pdInstance->dpmCableMaxCurrent)
                {
                    destPdo->batteryPDO.maxAllowPower =
                        pdInstance->dpmCableMaxCurrent * 10 * (pdo->batteryPDO.minVoltage * 50 / 1000) / 250;
                }
                break;
            }

            default:
                break;
        }
    }

    destPdo = (pd_source_pdo_t *)&(pdInstance->sendingData[1]);
#if defined(PD_CONFIG_EXTERNAL_POWER_DETECTION_SUPPORT) && (PD_CONFIG_EXTERNAL_POWER_DETECTION_SUPPORT)
    uint8_t externalPowerState;
    if (pdInstance->pdCallback(pdInstance->callbackParam, PD_DPM_GET_EXTERNAL_POWER_STATE, &externalPowerState) ==
        kStatus_PD_Success)
    {
        destPdo->fixedPDO.externalPowered = externalPowerState;
    }
#endif
    if (pdInstance->revision < PD_SPEC_REVISION_30)
    {
        destPdo->fixedPDO.unchunkedSupported = 0;
    }

    return (uint32_t *)&(pdInstance->sendingData[1]);
}
#endif

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
static uint32_t *PD_PsmGetSinkPDOs(pd_instance_t *pdInstance)
{
    for (uint8_t index = 0; index < pdInstance->pdPowerPortConfig->sinkCapCount; ++index)
    {
        pdInstance->sendingData[1 + index] = pdInstance->pdPowerPortConfig->sinkCaps[index];
    }
    if (pdInstance->revision < PD_SPEC_REVISION_30)
    {
        pd_sink_fixed_pdo_t *pdo   = (pd_sink_fixed_pdo_t *)&(pdInstance->sendingData[1]);
        pdo->frSwapRequiredCurrent = 0;
    }
    return (uint32_t *)&(pdInstance->sendingData[1]);
}
#endif

static void PD_PsmClearStateFlags(pd_instance_t *pdInstance)
{
    /* Initialise static flags */
    PD_PsmReset(pdInstance);
    pdInstance->revision    = PD_CONFIG_REVISION;
    pdInstance->psmCurState = PSM_UNKNOWN;
    pdInstance->psmNewState = PSM_UNKNOWN;

    pdInstance->psmPresentlyPdConnected     = 0;
    pdInstance->psmPreviouslyPdConnected    = 0;
    pdInstance->psmExplicitContractExisted  = 0;
    pdInstance->unchunkedFeature            = 0;
    pdInstance->commandProcessing           = 0;
    pdInstance->dpmMsgBits                  = 0;
    pdInstance->taskWaitTime                = PD_WAIT_EVENT_TIME;
    pdInstance->asmHardResetSnkProcessing   = 0;
    pdInstance->hardResetReceived           = 0;
    pdInstance->psmCableIdentitiesDataCount = 0;
#if ((defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT))
    PD_MsgChunkingLayerResetFlags(pdInstance);
#endif
}

static void PD_PsmDisconnect(pd_instance_t *pdInstance)
{
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
    pdInstance->psmPresentlyVconnSource = kPD_VconnNone;
#endif
    PD_PsmClearStateFlags(pdInstance);
    /* disconnect message layer */
    PD_MsgDisable(pdInstance);
}

static void PD_PsmConnect(pd_instance_t *pdInstance)
{
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
#if 0
    uint8_t externalPowerState;
    pdInstance->pdCallback(pdInstance->callbackParam, PD_DPM_GET_EXTERNAL_POWER_STATE, &externalPowerState);
    pdInstance->externalPoweredState = externalPowerState;
#endif
    pdInstance->portPartnerSinkPDO1.PDOValue = 0;
#endif
    PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
    pdInstance->dpmCableMaxCurrent = 3000 / 10; /* 3A */
    pdInstance->dpmStateMachine    = 0;
    PD_PsmClearStateFlags(pdInstance);
}

static uint8_t PD_PsmDisconnectCheck(pd_instance_t *pdInstance, uint8_t state)
{
    if ((state != kSM_ErrorRecovery) && (state != kSM_Detach))
    {
        if (PD_ConnectCheck(pdInstance) == (uint8_t)kConnectState_Disconnected)
        {
            state = kSM_Detach;
        }
    }
    return state;
}

/* internal function */
#if ((defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)) || \
    ((defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE))
static uint8_t PD_PsmCheckVbus5V(pd_instance_t *pdInstance)
{
    uint8_t powerState = (PD_VBUS_POWER_STATE_VSAFE5V_MASK | PD_VBUS_POWER_STATE_VBUS_MASK);
    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &powerState);
    if ((powerState & PD_VBUS_POWER_STATE_VSAFE5V_MASK) || (powerState & PD_VBUS_POWER_STATE_VBUS_MASK))
    {
        return 1;
    }
    return 0;
}
#endif

static uint8_t PD_PsmCheckOnlyVbus(pd_instance_t *pdInstance)
{
    uint8_t powerState = (PD_VBUS_POWER_STATE_VBUS_MASK);
    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &powerState);
    if (powerState & PD_VBUS_POWER_STATE_VBUS_MASK)
    {
        return 1;
    }
    return 0;
}

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
static uint8_t PD_PsmCheckLessOrEqualVsafe5v(pd_instance_t *pdInstance)
{
    uint8_t powerState =
        (PD_VBUS_POWER_STATE_VSAFE5V_MASK | PD_VBUS_POWER_STATE_VBUS_MASK | PD_VBUS_POWER_STATE_VSAFE0V_MASK);
    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &powerState);
    if (powerState & PD_VBUS_POWER_STATE_VSAFE5V_MASK)
    {
        return 1;
    }
    if (powerState & PD_VBUS_POWER_STATE_VSAFE0V_MASK)
    {
        return 1;
    }
    if ((!(powerState & PD_VBUS_POWER_STATE_VSAFE5V_MASK)) && (!(powerState & PD_VBUS_POWER_STATE_VBUS_MASK)))
    {
        return 1;
    }

    return 0;
}
#endif

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
static void PD_PsmCheckFRS5V(pd_instance_t *pdInstance)
{
    if ((pdInstance->fr5VOpened == 0) && PD_PsmCheckLessOrEqualVsafe5v(pdInstance))
    {
        pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam, kVbusPower_InFRSwap);
        pdInstance->fr5VOpened = 1;
    }
}
#endif

static uint8_t PD_PsmCheckVsafe0V(pd_instance_t *pdInstance)
{
    uint8_t powerState = PD_VBUS_POWER_STATE_VSAFE0V_MASK;

    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &powerState);
    return (powerState & PD_VBUS_POWER_STATE_VSAFE0V_MASK);
}

static uint8_t PD_DpmGetMsg(pd_instance_t *pdInstance)
{
    uint8_t reVal = 0;

    if (pdInstance->dpmMsgBits)
    {
        for (uint8_t commandIndex = PD_DPM_CONTROL_POWER_NEGOTIATION; commandIndex < PD_DPM_CONTROL_COUNT;
             ++commandIndex)
        {
            if (pdInstance->dpmMsgBits & (0x01u << commandIndex))
            {
                reVal = commandIndex;
                break;
            }
        }
    }

    return reVal;
}

static void PD_DpmClearMsg(pd_instance_t *pdInstance, pd_command_t id)
{
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if (pdInstance->dpmMsgBits & (0x01u << id))
    {
        pdInstance->dpmMsgBits &= (~(0x01u << id));
    }
    USB_OSA_EXIT_CRITICAL();
}

static void PD_DpmSendMsg(pd_handle pdHandle, uint8_t id)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    pdInstance->dpmMsgBits |= (0x00000001uL << id);
    USB_OSA_EXIT_CRITICAL();
    PD_StackSetEvent(pdInstance, PD_TASK_EVENT_DPM_MSG);
}

/*! ***************************************************************************
   task process related functions
******************************************************************************/
void PD_PortTaskEventProcess(pd_instance_t *pdInstance, uint32_t eventSet)
{
    if (eventSet & PD_TASK_EVENT_PHY_STATE_CHAGNE)
    {
        PD_PhyControl(pdInstance, PD_PHY_UPDATE_STATE, NULL);
    }

    if (eventSet & PD_TASK_EVENT_OTHER)
    {
        USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_OTHER);
    }

    if (pdInstance->isConnected)
    {
        eventSet &= (~(uint32_t)(PD_TASK_EVENT_PHY_STATE_CHAGNE | PD_TASK_EVENT_OTHER | PD_TASK_EVENT_FR_SWAP_SINGAL |
                                 PD_TASK_EVENT_DPM_MSG));
    }
    else
    {
        eventSet &= (~(uint32_t)(PD_TASK_EVENT_PHY_STATE_CHAGNE | PD_TASK_EVENT_OTHER | PD_TASK_EVENT_FR_SWAP_SINGAL |
                                 PD_TASK_EVENT_EXTERNAL_POWER_CHANGE));
    }

    /* clear the events that aren't processed in this condition */
    if (eventSet)
    {
        if (!((pdInstance->dpmStateMachine == 1) && (pdInstance->isConnected)))
        {
            USB_OsaEventClear(pdInstance->taskEventHandle, eventSet);
        }
    }
}

/*! ***************************************************************************
   command flow and message flow and state machine related functions
******************************************************************************/

#if ((defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT))
static void PD_MsgChunkingLayerResetFlags(pd_instance_t *pdInstance)
{
    pdInstance->chunkingRXState = RCH_Wait_For_Message_From_Protocol_Layer;
    pdInstance->chunkingTXState = TCH_Wait_For_Message_Request_From_Policy_Engine;
}

static uint8_t PD_MsgSendChunkRequest(pd_instance_t *pdInstance,
                                      start_of_packet_t sop,
                                      message_type_t extMsgType,
                                      pd_extended_msg_header_t extHeader,
                                      uint8_t requestNumber)
{
    pd_status_t sendStatus;

    extHeader.bitFields.requestChunk = 1;
    extHeader.bitFields.chunkNumber  = requestNumber;
    extHeader.bitFields.dataSize     = 0;
    sendStatus                       = PD_MsgSendRequestChunkMsg(pdInstance, sop, extMsgType, extHeader);
    if ((sendStatus == kStatus_PD_Success) && PD_MsgWaitSendResult(pdInstance))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static uint8_t PD_PsmChunkingLayerRXTimerWaiting(pd_instance_t *pdInstance)
{
    return (RCH_Waiting_Chunk == pdInstance->chunkingRXState);
}

static uint8_t PD_PsmChunkingLayerRXStateMachine(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    pd_chunking_layer_state_t prevState = pdInstance->chunkingRXState;
    pd_extended_msg_header_t extHeader;
    extHeader.extendedMsgHeaderVal = (uint32_t)(USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS((triggerInfo->pdMsgDataBuffer)));

    /* state not change */
    switch (pdInstance->chunkingRXState)
    {
        case RCH_Wait_For_Message_From_Protocol_Layer: /* B */
        {
            if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) &&
                ((!triggerInfo->msgHeader.bitFields.extended) ||
                 ((!extHeader.bitFields.chunked) && (pdInstance->unchunkedFeature))))
            {
                pdInstance->chunkingRXState = RCH_Pass_Up_Message;
            }
            else if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->msgHeader.bitFields.extended) &&
                     (extHeader.bitFields.chunked))
            {
                pdInstance->chunkingRXState = RCH_Processing_Extended_Message;
            }
            else if (pdInstance->unchunkedFeature == extHeader.bitFields.chunked)
            {
                pdInstance->chunkingRXState = RCH_Report_Error;
            }
            else
            {
            }
            break;
        }

        case RCH_Waiting_Chunk: /* B */
            if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->msgHeader.bitFields.extended) &&
                (extHeader.bitFields.chunked) && (pdInstance->chunkNumberExpected == extHeader.bitFields.chunkNumber))
            {
                pdInstance->chunkingRXState = RCH_Processing_Extended_Message;
            }
            else if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) ||
                     (PD_TimerCheckInvalidOrTimeOut(pdInstance, timrChunkSenderResponseTimer)))
            {
                pdInstance->chunkingRXState = RCH_Report_Error;
            }
            else
            {
            }
            break;

        default:
            /* Any Message Received and not in state RCH_Waiting_Chunk or RCH_Wait_For_Message_From_Protocol_Layer */
            pdInstance->chunkingRXState = RCH_Report_Error;
            break;
    }

    while (prevState != pdInstance->chunkingRXState)
    {
        prevState = pdInstance->chunkingRXState;
        switch (pdInstance->chunkingRXState)
        {
            case RCH_Wait_For_Message_From_Protocol_Layer: /* C */
                /* pdInstance->abortFlag = 0; */
                /* TODO: clear extended Rx Buffer */
                break;

            case RCH_Pass_Up_Message: /* C */
                pdInstance->chunkingRXState = RCH_Wait_For_Message_From_Protocol_Layer;
                break;

            case RCH_Processing_Extended_Message: /* C */
                if (extHeader.bitFields.chunkNumber == 0)
                {
                    pdInstance->chunkNumberExpected   = 0;
                    pdInstance->chunkNumBytesReceived = 0;
                }

                if (extHeader.bitFields.chunkNumber == pdInstance->chunkNumberExpected)
                {
                    pdInstance->chunkNumBytesReceived += (triggerInfo->msgHeader.bitFields.NumOfDataObjs * 4 - 2);
                    pdInstance->chunkNumberExpected++;
                    /* TODO: copy data to buff */
                }
                else if (extHeader.bitFields.chunkNumber != pdInstance->chunkNumberExpected)
                {
                    pdInstance->chunkingRXState = RCH_Report_Error;
                }
                else
                {
                }

#if 0
                if (pdInstance->abortFlag)
                {
                    pdInstance->chunkingRXState = RCH_Wait_For_Message_From_Protocol_Layer;
                }
                else
#endif

                if ((extHeader.bitFields.chunkNumber * 26 + (triggerInfo->msgHeader.bitFields.NumOfDataObjs * 4 - 2)) >=
                    extHeader.bitFields.dataSize)
                {
                    pdInstance->chunkingRXState = RCH_Pass_Up_Message;
                }
                else
                {
                    /* message is not completeed */
                    pdInstance->chunkingRXState = RCH_Requesting_Chunk;
                }
                break;

            case RCH_Requesting_Chunk: /* C */
                if (PD_MsgSendChunkRequest(pdInstance, (start_of_packet_t)triggerInfo->pdMsgSop,
                                           (message_type_t)triggerInfo->pdMsgType, extHeader,
                                           pdInstance->chunkNumberExpected))
                {
                    pdInstance->chunkingRXState = RCH_Waiting_Chunk;
                }
                else
                {
                    pdInstance->chunkingRXState = RCH_Report_Error;
                }
                break;

            case RCH_Waiting_Chunk: /* C */
                PD_TimerStart(pdInstance, timrChunkSenderResponseTimer, T_CHUNK_SENDER_RESPONSE);
                break;

            case RCH_Report_Error: /* C */
                /* TODO: report error to policy engine */
                pdInstance->chunkingRXState = RCH_Wait_For_Message_From_Protocol_Layer;
                break;

            default:
                break;
        }
    }

    return (pdInstance->chunkingRXState == RCH_Wait_For_Message_From_Protocol_Layer);
}

#if 0 /* comments it temparily for the warning (declared but never referenced) */
static uint8_t PD_PsmChunkingLayerCheckSentDone(pd_instance_t *pdInstance)
{
    return pdInstance->chunkSentDone;
}

static pd_status_t PD_PsmChunkingLayerTXStateMachine(pd_instance_t *pdInstance, uint8_t sop, message_type_t msgType, uint8_t *dataBuffer, uint32_t dataLength, psm_trigger_info_t *triggerInfo)
{
    pd_chunking_layer_state_t prevState = pdInstance->chunkingTXState;
    pd_status_t status = kStatus_PD_Success;

    /* state not change */
    switch (pdInstance->chunkingTXState)
    {
        case TCH_Wait_For_Message_Request_From_Policy_Engine: /* B */
            if (pdInstance->chunkingRXState != RCH_Wait_For_Message_From_Protocol_Layer)
            {
                pdInstance->chunkingTXState = TCH_Report_Error;
            }
            else
            {
                /* only chunking msg is passed to this function */
                pdInstance->chunkingTXState = TCH_Prepare_To_Send_Chunked_Message;
            }
            break;

        case TCH_Wait_Chunk_Request: /* B */
        {
            pd_extended_msg_header_t extHeader;
            extHeader.extendedMsgHeaderVal = (uint32_t)(USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS((triggerInfo->pdMsgDataBuffer)));
            if ((triggerInfo != NULL) && (triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->msgHeader.bitFields.extended) &&
                (extHeader.bitFields.chunked) && (pdInstance->chunkNumberToSend == extHeader.bitFields.chunkNumber))
            {
                pdInstance->chunkingTXState = TCH_Sending_Chunked_Message;
            }
            else if ((triggerInfo != NULL) && (triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG))
            {
                status = kStatus_PD_Error;
                pdInstance->chunkingTXState = TCH_Wait_For_Message_Request_From_Policy_Engine;
            }
            else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, timrChunkSenderResponseTimer))
            {
                /* still think success */
                pdInstance->chunkingTXState = TCH_Message_Sent;
            }
            else
            {
            }
            break;
        }
    }

    while (prevState != pdInstance->chunkingTXState)
    {
        prevState = pdInstance->chunkingTXState;
        switch (pdInstance->chunkingTXState)
        {
            case TCH_Report_Error: /* C */
                status = kStatus_PD_Error;
                pdInstance->chunkingTXState = TCH_Wait_For_Message_Request_From_Policy_Engine;
                break;

            case TCH_Prepare_To_Send_Chunked_Message: /* C */
                pdInstance->chunkNumberToSend = 0;
                pdInstance->chunkSentDone = 0;
                pdInstance->chunkingTXState = TCH_Sending_Chunked_Message;
                break;

            case TCH_Sending_Chunked_Message: /* C */
            {
                pd_extended_msg_header_t extHeader;
                pd_status_t sendStatus;

                uint32_t sendLength = (dataLength - pdInstance->chunkNumberToSend * 26);
                if (sendLength > 26)
                {
                    sendLength = 26;
                }
                extHeader.bitFields.chunked = 1;
                extHeader.bitFields.chunkNumber = pdInstance->chunkNumberToSend;
                extHeader.bitFields.requestChunk = 0;
                extHeader.bitFields.dataSize = dataLength;
                sendStatus = PD_MsgSendChunkedExtendedMsg(pdInstance, (start_of_packet_t)sop, msgType,
                                                          extHeader, sendLength, dataBuffer);
                if ((sendStatus == kStatus_PD_Success) && (PD_MsgWaitSendResult(pdInstance)))
                {
                    if (pdInstance->chunkNumberToSend == ((dataLength + 25) / 26))
                    {
                        pdInstance->chunkingTXState = TCH_Message_Sent;
                    }
                    else
                    {
                        pdInstance->chunkingTXState = TCH_Wait_Chunk_Request;
                    }
                }
                else
                {
                    pdInstance->chunkingTXState = TCH_Report_Error;
                }
                break;
            }

            case TCH_Wait_Chunk_Request: /* C */
                pdInstance->chunkNumberToSend++;
                PD_TimerStart(pdInstance, timrChunkSenderRequestTimer, T_CHUNK_SENDER_REQUEST);
                break;

            case TCH_Message_Sent: /* C */
                pdInstance->chunkSentDone = 1;
                pdInstance->chunkingTXState = TCH_Wait_For_Message_Request_From_Policy_Engine;
                break;

            case TCH_Wait_For_Message_Request_From_Policy_Engine: /* C */
                pdInstance->chunkSentDone = 1;
                break;

            default:
                break;
        }
    }
    return status;
}
#endif
#endif

#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
static uint8_t PD_PsmCheckInAltMode(pd_instance_t *pdInstance, start_of_packet_t sop)
{
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
    uint8_t altModeState = 0;
    if ((PD_AltModeState(pdInstance, &altModeState) == kStatus_PD_Success) && (altModeState))
    {
        return 1;
    }
#endif
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
    if (pdInstance->psmVdmActiveModeValidMask)
    {
        return 1;
    }
#endif
    return 0;
}
#endif

#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
void PD_PsmPrintAutoPolicyReplySwapRequestLog(pd_instance_t *pdInstance, uint8_t message)
{
    PRINTF("auto reply ");
    if (pdInstance->commandEvaluateResult == kCommandResult_Accept)
    {
        PRINTF("accept");
    }
    else if (pdInstance->commandEvaluateResult == kCommandResult_Reject)
    {
        PRINTF("reject");
    }
    else if (pdInstance->commandEvaluateResult == kCommandResult_Wait)
    {
        PRINTF("wait");
    }
    else
    {
    }
    PRINTF(" for ");

    switch (message)
    {
        case kPD_MsgPrSwap:
            PRINTF("pr");
            break;
        case kPD_MsgDrSwap:
            PRINTF("dr");
            break;
        case kPD_MsgVconnSwap:
        default:
            PRINTF("vconn");
            break;
    }
    PRINTF(" swap\r\n");
}

void PD_PsmPrintAutoPolicyStartSwapLog(pd_instance_t *pdInstance, uint8_t message)
{
    PRINTF("start auto request ");
    switch (message)
    {
        case kPD_MsgPrSwap:
            PRINTF("pr");
            break;
        case kPD_MsgDrSwap:
            PRINTF("dr");
            break;
        case kPD_MsgVconnSwap:
        default:
            PRINTF("vconn");
            break;
    }

    PRINTF(" swap\r\n");
}
#endif

#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
static uint8_t PD_PsmAutoRequestVconnSwap(pd_instance_t *pdInstance)
{
    uint8_t autoSwap = PD_POLICY_GET_AUTO_REQUEST_VCONNSWAP(pdInstance);
    if ((!PD_POLICY_SUPPORT(pdInstance)) || (autoSwap == kPD_VconnNone))
    {
        return 0;
    }

    if ((pdInstance->pdPowerPortConfig->vconnSupported) && (pdInstance->vconnSwapResult == kCommandResult_None) &&
        (autoSwap != kPD_VconnNone) && (autoSwap != pdInstance->psmPresentlyVconnSource))
    {
        if ((!pdInstance->portPartnerVconnSwapToOffRejected) && (autoSwap == kPD_NotVconnSource))
        {
            return 1;
        }
        if ((!pdInstance->portPartnerVconnSwapToOnRejected) && (autoSwap == kPD_IsVconnSource))
        {
            return 1;
        }
    }
    return 0;
}
#endif

#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
static uint8_t PD_PsmAutoRequestDataSwap(pd_instance_t *pdInstance)
{
    uint8_t autoSwap = PD_POLICY_GET_AUTO_REQUEST_DRSWAP(pdInstance);
    if ((!PD_POLICY_SUPPORT(pdInstance)) || (autoSwap == kPD_DataRoleNone))
    {
        return 0;
    }

    /* Only auto-swap if there has been no previous activity
     * Do not auto-swap if we are already in an alternate mode
     */
    if ((pdInstance->pdPowerPortConfig->dataFunction == kDataConfig_DRD) &&
        (pdInstance->drSwapResult == kCommandResult_None) && (!PD_PsmCheckInAltMode(pdInstance, kPD_MsgSOP)) &&
        (autoSwap != kPD_DataRoleNone) && (autoSwap != pdInstance->curDataRole))
    {
        if ((!pdInstance->portPartnerDrSwapToDFPRejected) && (autoSwap == kPD_DataRoleDFP))
        {
            return 1;
        }
        if ((!pdInstance->portPartnerDrSwapToUFPRejected) && (autoSwap == kPD_DataRoleUFP))
        {
            return 1;
        }
    }
    return 0;
}
#endif

#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
static uint8_t PD_PsmAutoRequestPowerSwap(pd_instance_t *pdInstance)
{
    if ((!PD_POLICY_SUPPORT(pdInstance)) || ((!PD_POLICY_GET_AUTO_REQUEST_PRSWAP_AS_SOURCE(pdInstance)) &&
                                             (!PD_POLICY_GET_AUTO_REQUEST_PRSWAP_AS_SINK(pdInstance))))
    {
        return 0;
    }

    if (PD_PsmIsDualRole(pdInstance))
    {
        if ((pdInstance->curPowerRole == kPD_PowerRoleSource) && (!pdInstance->portPartnerPrSwapToSinkRejected) &&
            (PD_POLICY_GET_AUTO_REQUEST_PRSWAP_AS_SOURCE(pdInstance)))
        {
            return 1;
        }

        if ((pdInstance->curPowerRole == kPD_PowerRoleSink) && (!pdInstance->portPartnerPrSwapToSourceRejected) &&
            (PD_POLICY_GET_AUTO_REQUEST_PRSWAP_AS_SINK(pdInstance)))
        {
            return 1;
        }
    }
    return 0;
}
#endif

static void PD_PsmSetAutoPolicyState(pd_instance_t *pdInstance, uint8_t newState)
{
    if (pdInstance->autoPolicyState == newState)
    {
        return;
    }

    pdInstance->autoPolicyState = newState;
    if (newState & PSM_RDY_DELAY_FLAG)
    {
        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
        {
            PD_TimerStart(pdInstance, timrPsmRdyEvalDelayTimer, T_PSM_SRC_RDY_EVAL_DELAY);
        }
        else
        {
            PD_TimerStart(pdInstance, timrPsmRdyEvalDelayTimer, T_PSM_SNK_RDY_EVAL_DELAY);
        }
    }
}

static uint8_t PD_PsmGetExternalPowerState(pd_instance_t *pdInstance)
{
    uint8_t externalPowerState;
#if defined(PD_CONFIG_EXTERNAL_POWER_DETECTION_SUPPORT) && (PD_CONFIG_EXTERNAL_POWER_DETECTION_SUPPORT)
    if (pdInstance->pdCallback(pdInstance->callbackParam, PD_DPM_GET_EXTERNAL_POWER_STATE, &externalPowerState) !=
        kStatus_PD_Success)
#endif
    {
        if ((pdInstance->pdPowerPortConfig->sourceCapCount != 0) && (pdInstance->pdPowerPortConfig->sourceCaps != NULL))
        {
            pd_source_pdo_t sourcePDO;
            sourcePDO.PDOValue = pdInstance->pdPowerPortConfig->sourceCaps[0];
            externalPowerState = sourcePDO.fixedPDO.externalPowered;
        }
        else if ((pdInstance->pdPowerPortConfig->sinkCapCount != 0) &&
                 (pdInstance->pdPowerPortConfig->sinkCaps != NULL))
        {
            pd_sink_pdo_t sinkPDO;
            sinkPDO.PDOValue   = pdInstance->pdPowerPortConfig->sinkCaps[0];
            externalPowerState = sinkPDO.fixedPDO.externalPowered;
        }
        else
        {
            externalPowerState = 0;
        }
    }
    return externalPowerState;
}

/* this function is called in rdy state */
static uint8_t PD_PsmReadyAutoPolicyProcess(pd_instance_t *pdInstance)
{
    uint8_t newAutoPolicySate;
    uint8_t didNothing = 0;
    if (pdInstance->autoPolicyState & PSM_RDY_DELAY_FLAG)
    {
        if (PD_TimerCheckInvalidOrTimeOut(pdInstance, timrPsmRdyEvalDelayTimer))
        {
            pdInstance->autoPolicyState &= (uint8_t)(~(PSM_RDY_DELAY_FLAG));
        }
        else
        {
            return 1;
        }
    }

    USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_EXTERNAL_POWER_CHANGE);
    newAutoPolicySate = pdInstance->autoPolicyState;
    switch (pdInstance->autoPolicyState)
    {
        case PSM_RDY_EVAL_INIT:
        {
            /* Update whether we have seen external power, so we don't get into an idle loop */
            pdInstance->rdySeenExtPower = PD_PsmGetExternalPowerState(pdInstance);
            /* default enter into idle state */
            newAutoPolicySate = PSM_RDY_EVAL_IDLE;
            if (0
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
                || PD_PsmAutoRequestVconnSwap(pdInstance)
#endif
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                || PD_PsmAutoRequestDataSwap(pdInstance)
#endif
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                || PD_PsmAutoRequestPowerSwap(pdInstance)
// Need to also check get partner capabilities to evaluate swap requests
//|| (ConfigGetDualRolePower(CLPCfgTblPtr) && ConfigGetAutoAcceptPrSwap(CLPCfgTblPtr))
#endif
            )
            {
#if defined(PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
                if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                {
                    newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_GET_SNK_CAP;
                }
#endif
#if defined(PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
                if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                {
                    newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_CHECK_PARTNER_CAP;
                }
#endif
            }
            break;
        }

#if defined(PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
        case PSM_RDY_EVAL_GET_SNK_CAP:
            /* On return to PSM_SRC_READY check the result */
            newAutoPolicySate = PSM_RDY_EVAL_CHECK_PARTNER_CAP;
            /* Get the sink capabilities, on return to READY state */
            pdInstance->psmNewState = PSM_PE_SRC_GET_SINK_CAP;
            break;
#endif

#if (defined(PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)) || \
    (defined(PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE))
        case PSM_RDY_EVAL_CHECK_PARTNER_CAP:
            pdInstance->rdySeenExtPower = PD_PsmGetExternalPowerState(pdInstance);
#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
            if (PD_PsmAutoRequestVconnSwap(pdInstance))
            {
                /* Attempt to vconn swap */
                newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_VCONN_SWAP;
            }
            else
#endif
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                if (PD_PsmAutoRequestDataSwap(pdInstance) &&
                    (((pdInstance->curPowerRole == kPD_PowerRoleSource) &&
                      (pdInstance->portPartnerSinkPDO1.fixedPDO.dualRoleData)) ||
                     ((pdInstance->curPowerRole == kPD_PowerRoleSink) &&
                      (pdInstance->selfOrPartnerFirstSourcePDO.fixedPDO.dualRoleData))))
            {
                /* Attempt to data role swap */
                newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_DR_SWAP;
            }
            else
#endif
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                /* auto request swap as sink && self is not external powered && partner is external powered */
                if (PD_PsmAutoRequestPowerSwap(pdInstance) && (!pdInstance->rdySeenExtPower) &&
                    (pdInstance->curPowerRole == kPD_PowerRoleSource) &&
                    /* #if !defined(USBPD_ENABLE_IMMEDIATE_SWAP_TO_SINK_ON_POWER_LOSS) ||
                       (!USBPD_ENABLE_IMMEDIATE_SWAP_TO_SINK_ON_POWER_LOSS) */
                    (pdInstance->portPartnerSinkPDO1.fixedPDO.externalPowered) &&
                    /* #endif */
                    (pdInstance->portPartnerSinkPDO1.fixedPDO.dualRolePower))
            {
                /* Attempt to swap to sink */
                newAutoPolicySate = PSM_RDY_EVAL_SWAP_TO_SNK | PSM_RDY_DELAY_FLAG;
#if 0
#if !defined(USBPD_ENABLE_IMMEDIATE_SWAP_TO_SINK_ON_POWER_LOSS) || (!USBPD_ENABLE_IMMEDIATE_SWAP_TO_SINK_ON_POWER_LOSS)
                                    PSM_RDY_DELAY_FLAG;
#else
                                    0;
#endif
#endif
            }
            /* auto request swap as source && self is external powered && partner is not external powered. */
            else if (PD_PsmAutoRequestPowerSwap(pdInstance) && (pdInstance->rdySeenExtPower) &&
                     (pdInstance->curPowerRole == kPD_PowerRoleSink) &&
                     (!pdInstance->selfOrPartnerFirstSourcePDO.fixedPDO.externalPowered) &&
                     (pdInstance->selfOrPartnerFirstSourcePDO.fixedPDO.dualRolePower))
            {
                /* Attempt to swap to source */
                newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_SWAP_TO_SRC;
            }
            else
#endif
            {
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
                if (PD_PsmAutoRequestPowerSwap(pdInstance) && (pdInstance->curPowerRole == kPD_PowerRoleSource) &&
                    (pdInstance->portPartnerSinkPDO1.fixedPDO.externalPowered) &&
                    (pdInstance->portPartnerSinkPDO1.fixedPDO.dualRolePower))
                {
                    PRINTF("don't do auto request pr swap to sink when self is external powered");
                }
                else if (PD_PsmAutoRequestPowerSwap(pdInstance) && (pdInstance->curPowerRole == kPD_PowerRoleSink) &&
                         (!pdInstance->selfOrPartnerFirstSourcePDO.fixedPDO.externalPowered) &&
                         (pdInstance->selfOrPartnerFirstSourcePDO.fixedPDO.dualRolePower))
                {
                    PRINTF("don't do auto request pr swap to source when self is not external powered");
                }
                else
                {
                }
#endif
                newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_IDLE;
            }
            break;
#endif

#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
        case PSM_RDY_EVAL_SWAP_TO_SNK:
            if (PD_PsmAutoRequestPowerSwap(pdInstance) &&
                /* Check for a change in the externally powered status since last we saw it */
                pdInstance->rdySeenExtPower == PD_PsmGetExternalPowerState(pdInstance))
            {
                newAutoPolicySate       = PSM_RDY_EVAL_CHECK_SWAP_TO_SNK;
                pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
                PD_PsmPrintAutoPolicyStartSwapLog(pdInstance, kPD_MsgPrSwap);
#endif
            }
            else
            {
                newAutoPolicySate = PSM_RDY_EVAL_INIT;
            }
            break;
        case PSM_RDY_EVAL_SWAP_TO_SRC:
            if (PD_PsmAutoRequestPowerSwap(pdInstance) &&
                /* Check for a change in the externally powered status since last we saw it */
                pdInstance->rdySeenExtPower == PD_PsmGetExternalPowerState(pdInstance))
            {
                newAutoPolicySate       = PSM_RDY_EVAL_CHECK_SWAP_TO_SRC;
                pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
                PD_PsmPrintAutoPolicyStartSwapLog(pdInstance, kPD_MsgPrSwap);
#endif
            }
            else
            {
                newAutoPolicySate = PSM_RDY_EVAL_INIT;
            }
            break;
#endif

#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
        case PSM_RDY_EVAL_DR_SWAP:
            if (PD_PsmAutoRequestDataSwap(pdInstance))
            {
                newAutoPolicySate       = PSM_RDY_EVAL_CHECK_DR_SWAP;
                pdInstance->psmNewState = PSM_PE_DRS_SEND_DR_SWAP;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
                PD_PsmPrintAutoPolicyStartSwapLog(pdInstance, kPD_MsgDrSwap);
#endif
            }
            else
            {
                newAutoPolicySate = PSM_RDY_EVAL_INIT;
            }
            break;
#endif

#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
        case PSM_RDY_EVAL_VCONN_SWAP:
            if (PD_PsmAutoRequestVconnSwap(pdInstance))
            {
                newAutoPolicySate       = PSM_RDY_EVAL_CHECK_VCONN_SWAP;
                pdInstance->psmNewState = PSM_PE_VCS_SEND_SWAP;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
                PD_PsmPrintAutoPolicyStartSwapLog(pdInstance, kPD_MsgVconnSwap);
#endif
            }
            else
            {
                newAutoPolicySate = PSM_RDY_EVAL_INIT;
            }
            break;
#endif

        case PSM_RDY_EVAL_IDLE:
            /* Check for a change in the externally powered status since last we saw it */
            if (pdInstance->rdySeenExtPower != PD_PsmGetExternalPowerState(pdInstance))
            {
                newAutoPolicySate = PSM_RDY_EVAL_INIT;
            }
            else
            {
                /* No further action required until something changes */
                didNothing = 1;
            }
            break;

        default:
            didNothing = 1;
            break;
    }

    PD_PsmSetAutoPolicyState(pdInstance, newAutoPolicySate);
    return didNothing;
}

static void PD_PsmReadyAutoPolicyResult(pd_instance_t *pdInstance, uint8_t result)
{
    uint8_t newAutoPolicySate = pdInstance->autoPolicyState;
    switch (result)
    {
        // If far end accepts, then we will be reset to PSM_RDY_EVAL_INIT at swap
        // For reject we move to check the conditions again after a short delay
        // For wait we return to try the same thing again
        case kCommandResult_Reject:
            switch (newAutoPolicySate)
            {
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                case PSM_RDY_EVAL_CHECK_SWAP_TO_SNK:
                    pdInstance->portPartnerPrSwapToSinkRejected = true;
                    break;
                case PSM_RDY_EVAL_CHECK_SWAP_TO_SRC:
                    pdInstance->portPartnerPrSwapToSourceRejected = true;
                    break;
#endif
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                case PSM_RDY_EVAL_CHECK_DR_SWAP:
                    if (pdInstance->curDataRole == kPD_DataRoleDFP)
                    {
                        pdInstance->portPartnerDrSwapToUFPRejected = true;
                    }
                    else
                    {
                        pdInstance->portPartnerDrSwapToDFPRejected = true;
                    }
                    break;
#endif
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
                case PSM_RDY_EVAL_CHECK_VCONN_SWAP:
                    if (pdInstance->psmPresentlyVconnSource)
                    {
                        pdInstance->portPartnerVconnSwapToOffRejected = true;
                    }
                    else
                    {
                        pdInstance->portPartnerVconnSwapToOnRejected = true;
                    }
                    break;
#endif
            }
            newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_INIT;
            break;

        case kCommandResult_Error:
        case kCommandResult_Timeout:
            newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_INIT;
            break;

        case kCommandResult_Wait:
            switch (newAutoPolicySate)
            {
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                case PSM_RDY_EVAL_CHECK_SWAP_TO_SNK:
                    newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_SWAP_TO_SNK;
                    break;
                case PSM_RDY_EVAL_CHECK_SWAP_TO_SRC:
                    newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_SWAP_TO_SRC;
                    break;
#endif
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                case PSM_RDY_EVAL_CHECK_DR_SWAP:
                    newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_DR_SWAP;
                    break;
#endif
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
                case PSM_RDY_EVAL_CHECK_VCONN_SWAP:
                    newAutoPolicySate = PSM_RDY_DELAY_FLAG | PSM_RDY_EVAL_VCONN_SWAP;
                    break;
#endif
                default:
                    break;
            }
            break;

        default:
            return;
    }
    PD_PsmSetAutoPolicyState(pdInstance, newAutoPolicySate);
}

#if 1
static void PD_PsmExternalPowerChange(pd_instance_t *pdInstance)
{
#if defined(PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
    if (pdInstance->curPowerRole == kPD_PowerRoleSink)
    {
        PD_StackSetEvent(pdInstance, PD_TASK_EVENT_EXTERNAL_POWER_CHANGE);
    }
#if 0
    else
    {
#if defined(USBPD_ENABLE_IMMEDIATE_SWAP_TO_SINK_ON_POWER_LOSS) && (USBPD_ENABLE_IMMEDIATE_SWAP_TO_SINK_ON_POWER_LOSS)
        if (!pdInstance->externalPoweredState)
        {
            PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_CHECK_PARTNER_CAP);
        }
        else
#endif
        {
            /* A source must immediately send new capabilities */
            PD_Command(pdInstance, PD_DPM_CONTROL_POWER_NEGOTIATION, NULL);
        }
    }
#endif
#endif
}
#endif

#endif

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
static void PD_StateWaitReplyExtDataProcess(pd_instance_t *pdInstance,
                                            psm_trigger_info_t *triggerInfo,
                                            uint8_t requiredMsg,
                                            uint32_t successCallbackEvent,
                                            uint32_t failCallbackEvent,
                                            uint8_t *didNothingStepB)
{
    uint8_t commandResultCallback;

    if (triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG)
    {
        pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;

        if (triggerInfo->pdMsgType == requiredMsg)
        {
            pd_command_data_param_t extMsgParam;
            PD_TimerClear(pdInstance, tSenderResponseTimer);

            /* the fixed 23 bytes length */
            if (requiredMsg == kPD_MsgBatteryStatus)
            {
                extMsgParam.dataBuffer = (triggerInfo->pdMsgDataBuffer);
            }
            else
            {
                extMsgParam.dataBuffer = (triggerInfo->pdMsgDataBuffer + 2);
            }
            if ((requiredMsg == kPD_MsgSourceCapExtended) &&
                (triggerInfo->pdExtMsgLength) >= PD_EXTENDED_SRC_CAP_DATA_LENGTH)
            {
                extMsgParam.dataLength = PD_EXTENDED_SRC_CAP_DATA_LENGTH;
            }
            else if ((requiredMsg == kPD_MsgStatus) &&
                     ((triggerInfo->pdExtMsgLength) >= PD_EXTENDED_STATUS_MSG_DATA_LENGTH))
            {
                extMsgParam.dataLength = PD_EXTENDED_STATUS_MSG_DATA_LENGTH;
            }
            else if ((requiredMsg == kPD_MsgBatteryCapabilities) &&
                     ((triggerInfo->pdExtMsgLength) >= PD_EXTENDED_BATTERY_CAP_MSG_DATA_LENGTH))
            {
                extMsgParam.dataLength = PD_EXTENDED_BATTERY_CAP_MSG_DATA_LENGTH;
            }
            else
            {
                extMsgParam.dataLength = triggerInfo->pdExtMsgLength;
            }
            PD_DpmAppCallback(pdInstance, successCallbackEvent, &extMsgParam, 1);
        }
        else if ((triggerInfo->msgHeader.bitFields.NumOfDataObjs == 0) &&
                 (triggerInfo->msgHeader.bitFields.messageType == kPD_MsgNotSupported))
        {
            PD_TimerClear(pdInstance, tSenderResponseTimer);

            commandResultCallback = kCommandResult_NotSupported;
            PD_DpmAppCallback(pdInstance, failCallbackEvent, &commandResultCallback, 1);
        }
        else
        {
            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
        }
    }
    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
    {
        commandResultCallback = kCommandResult_Error;
        PD_DpmAppCallback(pdInstance, failCallbackEvent, &commandResultCallback, 1);
        pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;
    }
    else
    {
        *didNothingStepB = 1;
    }
}
#endif

static uint8_t PD_PsmStartCommand(pd_instance_t *pdInstance, uint8_t command, uint8_t isInitiator)
{
#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
    uint8_t start = 1;
    if (pdInstance->revision >= PD_SPEC_REVISION_30)
    {
        if (isInitiator)
        {
            if (pdInstance->curPowerRole == kPD_PowerRoleSource)
            {
                PD_MsgSrcStartCommand(pdInstance);
            }
            else
            {
                start = PD_MsgSnkCheckStartCommand(pdInstance);
            }
        }
        else
        {
            start = 1;
        }
    }

    if (start)
#endif
    {
        pdInstance->commandProcessing  = command;
        pdInstance->commandIsInitiator = isInitiator;
    }

#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
    return start;
#else
    return 1;
#endif
}

static void PD_PsmCommandFail(pd_instance_t *pdInstance, uint8_t command)
{
    uint32_t commandResultCallback = kCommandResult_Error;
    uint32_t event                 = 0xFFFFFFFFuL;

    switch (command)
    {
        case PD_DPM_CONTROL_POWER_NEGOTIATION:
        {
            if (pdInstance->commandIsInitiator)
            {
                event = PD_DPM_SRC_RDO_FAIL;
            }
            else
            {
                event = PD_DPM_SNK_RDO_FAIL;
            }
            break;
        }
        case PD_DPM_CONTROL_REQUEST:
        {
            if (pdInstance->curPowerRole == kPD_PowerRoleSink)
            {
                event = PD_DPM_SNK_RDO_FAIL;
            }
            else
            {
                event = PD_DPM_SRC_RDO_FAIL;
            }
            break;
        }
        case PD_DPM_CONTROL_GOTO_MIN:
        {
            if (pdInstance->commandIsInitiator)
            {
                event = PD_DPM_SRC_GOTOMIN_FAIL;
            }
            else
            {
                event = PD_DPM_SNK_GOTOMIN_FAIL;
            }
            break;
        }
        case PD_DPM_CONTROL_SOFT_RESET:
        {
            event = PD_DPM_SOFT_RESET_FAIL;
            break;
        }
        case PD_DPM_CONTROL_HARD_RESET:
        {
            /* hard reset will always success */
            break;
        }
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
        case PD_DPM_CONTROL_PR_SWAP:
        {
            event = PD_DPM_PR_SWAP_FAIL;
            break;
        }
#endif
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
        case PD_DPM_CONTROL_DR_SWAP:
        {
            event = PD_DPM_DR_SWAP_FAIL;
            break;
        }
#endif
#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
        case PD_DPM_CONTROL_VCONN_SWAP:
        {
            event = PD_DPM_VCONN_SWAP_FAIL;
            break;
        }
#endif
        case PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES:
        {
            event = PD_DPM_GET_PARTNER_SRC_CAP_FAIL;
            break;
        }
        case PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES:
        {
            event = PD_DPM_GET_PARTNER_SNK_CAP_FAIL;
            break;
        }
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
        case PD_DPM_CONTROL_DISCOVERY_IDENTITY:
        case PD_DPM_CONTROL_DISCOVERY_SVIDS:
        case PD_DPM_CONTROL_DISCOVERY_MODES:
        case PD_DPM_CONTROL_ENTER_MODE:
        case PD_DPM_CONTROL_EXIT_MODE:
        case PD_DPM_CONTROL_SEND_ATTENTION:
        case PD_DPM_SEND_VENDOR_STRUCTURED_VDM:
        {
            event = PD_DPM_STRUCTURED_VDM_FAIL;
            break;
        }
        case PD_DPM_SEND_UNSTRUCTURED_VDM:
        {
            event = PD_DPM_SEND_UNSTRUCTURED_VDM_FAIL;
            break;
        }
#endif
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
        case PD_DPM_CONTROL_CABLE_RESET:
        {
            break;
        }
#endif
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        case PD_DPM_GET_SRC_EXT_CAP:
        {
            event = PD_DPM_GET_SRC_EXT_CAP_FAIL;
            break;
        }
        case PD_DPM_GET_STATUS:
        {
            event = PD_DPM_GET_STATUS_FAIL;
            break;
        }
        case PD_DPM_GET_BATTERY_CAP:
        {
            event = PD_DPM_GET_BATTERY_CAP_FAIL;
            break;
        }
        case PD_DPM_GET_BATTERY_STATUS:
        {
            event = PD_DPM_GET_BATTERY_STATUS_FAIL;
            break;
        }
        case PD_DPM_GET_MANUFACTURER_INFO:
        {
            event = PD_DPM_GET_MANUFACTURER_INFO_FAIL;
            break;
        }
#if 0
        case PD_DPM_SECURITY_REQUEST:
        {
            event = PD_DPM_SECURITY_REQUEST_FAIL;
            break;
        }
        case PD_DPM_FIRMWARE_UPDATE_REQUEST:
        {
            break;
        }
#endif
        case PD_DPM_ALERT:
        {
            event = PD_DPM_SEND_ALERT_FAIL;
            break;
        }
#endif
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        case PD_DPM_FAST_ROLE_SWAP:
        {
            event = PD_DPM_FR_SWAP_FAIL;
            break;
        }
#endif

        default:
            break;
    }

    if (event != 0xFFFFFFFFu)
    {
        if (event != PD_DPM_STRUCTURED_VDM_FAIL)
        {
            PD_DpmAppCallback(pdInstance, event, &commandResultCallback, 1);
        }
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
        else
        {
            pd_svdm_command_result_t commandVdmResult;
            commandVdmResult.vdmCommandResult = kCommandResult_Error;
            commandVdmResult.vdmCommand       = pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command;
            commandVdmResult.vdoData          = NULL;
            commandVdmResult.vdoCount         = 0;

            PD_DpmAppCallback(pdInstance, event, &commandVdmResult, 1);
        }
#endif
    }
}

/* process the primary pd commands that don't must require snkRdy or srcRdy */
static uint8_t PD_PsmPrimaryStateProcessDpmMsg(pd_instance_t *pdInstance,
                                               uint8_t *returnNewState,
                                               psm_trigger_info_t *triggerInfo)
{
    uint8_t newStateTmp = PSM_UNKNOWN;
    uint8_t didNothing  = 1;

    /* assume the message is processed */
    triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
    switch (triggerInfo->dpmMsg)
    {
        case PD_DPM_CONTROL_HARD_RESET:
            newStateTmp = PSM_HARD_RESET;
            didNothing  = 0;
            break;

        case PD_DPM_CONTROL_SOFT_RESET:
            if (PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_SOFT_RESET, 1))
            {
                newStateTmp = PSM_SEND_SOFT_RESET;
                didNothing  = 0;
            }
            else
            {
                PD_PsmCommandFail(pdInstance, PD_DPM_CONTROL_SOFT_RESET);
            }
            break;
#if 0
        case PD_DPM_CONTROL_EXIT_MODE:
            pdInstance->vdmExitReceived[pdInstance->structuredVdmCommandParameter.vdmSop] = 1;
            break;
#endif

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        case PD_DPM_FAST_ROLE_SWAP:
            if (PD_PsmStartCommand(pdInstance, PD_DPM_FAST_ROLE_SWAP, 1))
            {
                /* need enter the PE_FRS_SRC_SNK_CC_Signal to wait the fr_swap msg */
                if (pdInstance->frSignaledWaitFrSwap)
                {
                    pdInstance->psmNewState = PE_FRS_SRC_SNK_CC_Signal;
                }
            }
            break;
#endif

        default:
            /* the event is not processed */
            triggerInfo->triggerEvent = PSM_TRIGGER_DPM_MSG;
            break;
    }

    if (newStateTmp != PSM_UNKNOWN)
    {
        *returnNewState = newStateTmp;
    }

    return didNothing;
}

static uint8_t PD_PsmSinkHardResetfunction(pd_instance_t *pdInstance)
{
    pdInstance->callbackFns->PD_SnkStopDrawVbus(pdInstance->callbackParam, kVbusPower_InHardReset);
    pdInstance->asmHardResetSnkProcessing = 1;
    return PSM_PE_SNK_TRANSITION_TO_DEFAULT;
}

/* process the primary pd message */
static uint8_t PD_PsmPrimaryStateProcessPdMsg(pd_instance_t *pdInstance,
                                              uint8_t *returnNewState,
                                              psm_trigger_info_t *triggerInfo)
{
    uint8_t newStateTmp = PSM_UNKNOWN;
    uint8_t didNothing  = 1;

    if (triggerInfo->triggerEvent == PSM_TRIGGER_RECEIVE_HARD_RESET)
    {
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;

        if (pdInstance->psmCurState == PSM_PE_BIST_TEST_DATA_MODE)
        {
            /* From the IAS: There is no exit from this test mode except some chip level reset.
               Reset the protocol layer to exit carrier mode */
            PD_PhyControl(pdInstance, PD_PHY_EXIT_BIST, NULL);
        }
        /* Exit any active alternate mode */
        /* All timers and secondary states are reset during this call */
        PD_PsmReset(pdInstance);

        PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InHardReset);
        PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_HARD_RESET, 0);
        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
        {
            /* source receive hard_reset, 1. start tPSHardReset */
            PD_TimerStart(pdInstance, tPSHardResetTimer, T_PS_HARD_RESET);
            newStateTmp = PSM_PE_SRC_HARD_RESET_RECEIVED;
        }
        else if (pdInstance->curPowerRole == kPD_PowerRoleSink)
        {
            newStateTmp = PD_PsmSinkHardResetfunction(pdInstance);
        }
        else
        {
        }
        didNothing = 0;
    }
    else if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgType == kPD_MsgBIST))
    {
#if (defined PD_CONFIG_COMPLIANCE_TEST_ENABLE) && (PD_CONFIG_COMPLIANCE_TEST_ENABLE)
        pd_bist_object_t bistObj;

        /* it the power > 5V, cannot do bist test */
        if (((pdInstance->curDataRole == kPD_DataRoleDFP) &&
             (pdInstance->partnerRdoRequest.bitFields.objectPosition > 1)) ||
            ((pdInstance->curDataRole == kPD_DataRoleUFP) && (pdInstance->rdoRequest.bitFields.objectPosition > 1)))
        {
            return didNothing;
        }
        bistObj.objVal = USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(triggerInfo->pdMsgDataBuffer);
        if (bistObj.bitFields.testMode == kBIST_TestData)
        {
            /* Do nothing, the HW does not have a mode for kBIST_TestData */
            newStateTmp = kBIST_TestData;
            PD_PhyControl(pdInstance, PD_PHY_ENTER_BIST, &newStateTmp);
            newStateTmp = PSM_PE_BIST_TEST_DATA_MODE;
        }
        else
        {
            PD_TimerClear(pdInstance, tNoResponseTimer);
            newStateTmp = PSM_PE_BIST_CARRIER_MODE_2;
        }
#endif
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
    }
    else if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgType == kPD_MsgSoftReset))
    {
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        /* Soft reset is ignored when in BIST carrier mode */
        /* Soft reset is ignored during fast role swap signalling */
        if ((pdInstance->psmCurState != PSM_PE_BIST_CARRIER_MODE_2) &&
            (pdInstance->psmCurState != PE_FRS_SRC_SNK_CC_Signal))
        {
            PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_SOFT_RESET, 0);
            newStateTmp = PSM_SOFT_RESET;
            didNothing  = 0;
        }
    }
    /* The received message's data role is not right */
    else if ((pdInstance->curDataRole == triggerInfo->msgHeader.bitFields.portDataRole) &&
             (triggerInfo->pdMsgType != kPD_MsgGoodCRC))
    {
        triggerInfo->triggerEvent = PSM_TRIGGER_NON_START;
        newStateTmp               = PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY;
        didNothing                = 0;
    }
    /* received source_capabilities */
    else if ((pdInstance->psmCurState != PSM_PE_SNK_DISCOVERY) && (triggerInfo->pdMsgType == kPD_MsgSourceCapabilities))
    {
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceOnly)
        {
            PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, PD_NOT_SUPPORT_REPLY_MSG,
                                                               pdInstance->psmCurState);
        }
        else
        {
            switch (pdInstance->psmCurState)
            {
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
                case PSM_PE_SRC_DISCOVERY:
                case PSM_PE_SRC_SEND_CAPABILITIES:
                case PSM_PE_SRC_TRANSITION_SUPPLY:
                case PSM_PE_SRC_READY:
                case PSM_PE_SRC_GET_SINK_CAP:
                {
                    newStateTmp = PSM_SEND_SOFT_RESET;
                }
                break;

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                case PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF:
                case PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON:
                case PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP:
                case PSM_PE_PRS_SNK_SRC_SOURCE_ON:
                {
                    /* A protocol error during power role swap triggers a Hard Reset */
                    newStateTmp = PSM_HARD_RESET;
                }
                break;
#endif

#endif

/* Source state, message expected */
#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                case PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP_GET_SOURCE_CAP:
#endif
                case PSM_PE_DR_SRC_GET_SOURCE_CAP:
                case PSM_PE_SNK_GET_SOURCE_CAP:
                {
                    pd_capabilities_t sourceCapa;
                    PD_TimerClear(pdInstance, tSenderResponseTimer);
                    /* Clear any pending message now that we have the latest */
                    (void)PD_DpmClearMsg(pdInstance, PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES);
                    sourceCapa.capabilities            = MSG_DATA_BUFFER;
                    sourceCapa.capabilitiesCount       = (triggerInfo->pdMsgDataLength);
                    pdInstance->partnerSourcePDOsCount = sourceCapa.capabilitiesCount;
                    for (uint32_t u32Tmp = 0; u32Tmp < sourceCapa.capabilitiesCount; ++u32Tmp)
                    {
                        pdInstance->partnerSourcePDOs[u32Tmp].PDOValue = sourceCapa.capabilities[u32Tmp];
                    }

#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                    if (pdInstance->psmCurState == PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP_GET_SOURCE_CAP)
                    {
                        pdInstance->swapToSnkSrcCapReceived = 1;
                    }
                    else
#endif
                    {
                        if (pdInstance->commandProcessing == PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES)
                        {
                            pdInstance->commandProcessing = 0;
                        }
                        PD_DpmAppCallback(pdInstance, PD_DPM_GET_PARTNER_SRC_CAP_SUCCESS, &sourceCapa, 1);
                    }
                    if (pdInstance->psmCurState == PSM_PE_SNK_GET_SOURCE_CAP)
                    {
                        newStateTmp = PSM_PE_SNK_READY;
                    }
                    else
                    {
                        newStateTmp = PSM_PE_SRC_READY;
                    }
                    break;
                }
#endif

/* Sink states, message accepted: */
#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
                case PSM_PE_SNK_WAIT_FOR_CAPABILITIES:
                case PSM_PE_SNK_SELECT_CAPABILITY:
                case PSM_PE_SNK_TRANSITION_SINK:
                case PSM_PE_SNK_READY:
                case PSM_PE_SNK_GIVE_SINK_CAP:
#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                case PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF:
                case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP:
                case PSM_PE_DR_SNK_GET_SINK_CAP:
#endif
                {
                    pd_capabilities_t sourceCapa;
                    PD_PsmCheckRevision(pdInstance, triggerInfo->msgHeader);
                    PD_TimerClear(pdInstance, tSinkWaitCapTimer);
                    pdInstance->psmPresentlyPdConnected  = 1;
                    pdInstance->psmPreviouslyPdConnected = 1;

                    /* Update the PDOs for the EC and send interrupt */
                    sourceCapa.capabilities                          = MSG_DATA_BUFFER;
                    sourceCapa.capabilitiesCount                     = (triggerInfo->pdMsgDataLength);
                    pdInstance->selfOrPartnerFirstSourcePDO.PDOValue = sourceCapa.capabilities[0];
                    pdInstance->partnerSourcePDOsCount               = sourceCapa.capabilitiesCount;
                    for (uint32_t u32Tmp = 0; u32Tmp < sourceCapa.capabilitiesCount; ++u32Tmp)
                    {
                        pdInstance->partnerSourcePDOs[u32Tmp].PDOValue = sourceCapa.capabilities[u32Tmp];
                    }
                    if (pdInstance->commandProcessing == PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES)
                    {
                        pdInstance->commandProcessing = 0;
                    }
                    PD_DpmAppCallback(pdInstance, PD_DPM_SNK_RECEIVE_PARTNER_SRC_CAP, &sourceCapa, 0);
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
                    /* Ensure secondary state machine does not transition primary state machine */
                    PD_PsmSecondaryStateHandlerTerminate(pdInstance, 0xffu);
#endif
                    newStateTmp = PSM_PE_SNK_EVALUATE_CAPABILITY;
                }
                break;
#endif

                default:
                    break;
            }
        }

        didNothing = 0;
    }
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
    else if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgType == kPD_MsgVendorDefined) &&
             (triggerInfo->vdmHeader.bitFields.command == kVDM_ExitMode) &&
             (triggerInfo->vdmHeader.bitFields.commandType == kVDM_Initiator))
    {
        pdInstance->vdmExitReceived[triggerInfo->pdMsgSop]    = triggerInfo->vdmHeader.structuredVdmHeaderVal;
        pdInstance->vdmExitReceivedSOP[triggerInfo->pdMsgSop] = triggerInfo->pdMsgSop;
    }
#endif
    else
    {
    }

    if (newStateTmp != PSM_UNKNOWN)
    {
        *returnNewState = newStateTmp;
    }

    return didNothing;
}

static void PD_PsmEndCommand(pd_instance_t *pdInstance)
{
    if (pdInstance->commandProcessing == 0)
    {
        return;
    }
    else
    {
        uint8_t commandFail = 0;

        switch (pdInstance->commandProcessing)
        {
            case PD_DPM_CONTROL_HARD_RESET:
            case PD_DPM_CONTROL_SOFT_RESET:
            case PD_DPM_CONTROL_CABLE_RESET:
            {
                /* process in the state machine */
                break;
            }

#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PD_DPM_CONTROL_PR_SWAP:
            {
                switch (pdInstance->psmCurState)
                {
                    case PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP:
                    case PSM_PE_PRS_SRC_SNK_ACCEPT_PR_SWAP:
                    case PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF:
                    case PSM_PE_PRS_SRC_SNK_ASSERT_RD:
                    case PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON:
                    case PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP:
                    case PSM_PE_PRS_SRC_SNK_REJECT_PR_SWAP:
                    case PSM_PE_PRS_SNK_SRC_EVALUATE_PR_SWAP:
                    case PSM_PE_PRS_SNK_SRC_ACCEPT_PR_SWAP:
                    case PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF:
                    case PSM_PE_PRS_SNK_SRC_ASSERT_RP:
                    case PSM_PE_PRS_SNK_SRC_SOURCE_ON:
                    case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP:
                    case PSM_PE_PRS_SNK_SRC_REJECT_PR_SWAP:
                    case PSM_PE_PRS_EVALUATE_PR_SWAP_WAIT_TIMER_TIME_OUT:
                        /* these states means the pr_swap ams is processing */
                        break;

                    default:
                        /* PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_FAIL, NULL, 1); */
                        commandFail = 1;
                        break;
                }
                break;
            }
#endif

            case PD_DPM_CONTROL_REQUEST:
                switch (pdInstance->psmCurState)
                {
                    case PSM_PE_SRC_NEGOTIATE_CAPABILITY:
                    case PSM_PE_SRC_TRANSITION_SUPPLY:
                    case PSM_PE_SNK_EVALUATE_CAPABILITY:
                    case PSM_PE_SNK_TRANSITION_SINK:
                    case PSM_PE_SNK_SELECT_CAPABILITY:
                    case PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT:
                        break;

                    case PSM_PE_SNK_READY:
                        if (pdInstance->psmSnkReceiveRdoWaitRetry)
                        {
                            /* even in snk_rdy bu it is wait reply */
                        }
                        else
                        {
                            /* PD_DpmAppCallback(pdInstance, PD_DPM_SNK_RDO_FAIL, NULL, 1); */
                            commandFail = 1;
                        }
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_CONTROL_POWER_NEGOTIATION:
                switch (pdInstance->psmCurState)
                {
                    case PSM_PE_SRC_DISCOVERY:
                    case PSM_PE_SRC_NEGOTIATE_CAPABILITY:
                    case PSM_PE_SRC_TRANSITION_SUPPLY:
                    case PSM_PE_SNK_TRANSITION_SINK:
                    case PSM_PE_SNK_SELECT_CAPABILITY:
                    case PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT:
                    case PSM_PE_SRC_SEND_CAPABILITIES:
                    case PSM_PE_SNK_WAIT_FOR_CAPABILITIES:
                    case PSM_PE_SNK_GIVE_SINK_CAP:
                    case PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF:
                    case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP:
                    case PSM_PE_DR_SNK_GET_SINK_CAP:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES:
                switch (pdInstance->psmCurState)
                {
                    case PSM_PE_DR_SRC_GET_SOURCE_CAP:
                    case PSM_PE_SNK_GET_SOURCE_CAP:
                    case PSM_PE_SNK_READY:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES:
                switch (pdInstance->psmCurState)
                {
                    case PSM_PE_DR_SNK_GET_SINK_CAP:
                    case PSM_PE_SRC_GET_SINK_CAP:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_CONTROL_GOTO_MIN:
                switch (pdInstance->psmCurState)
                {
                    case PSM_PE_SNK_TRANSITION_SINK:
                    case PSM_PE_SRC_TRANSITION_SUPPLY:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
            case PD_DPM_CONTROL_DR_SWAP:
                switch (pdInstance->psmCurState)
                {
                    case PSM_PE_DRS_SEND_DR_SWAP:
                    case PSM_PE_DRS_EVALUATE_DR_SWAP:
                    case PSM_PE_DRS_ACCEPT_DR_SWAP:
                    case PSM_PE_DRS_REJECT_DR_SWAP:
                    case PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;
#endif

#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
            case PD_DPM_CONTROL_VCONN_SWAP:
                switch (pdInstance->psmCurState)
                {
                    case PSM_PE_VCS_SEND_SWAP:
                    case PSM_PE_VCS_WAIT_FOR_VCONN:
                    case PSM_PE_VCS_TURN_OFF_VCONN:
                    case PSM_PE_VCS_TURN_ON_VCONN:
                    case PSM_PE_VCS_SEND_PS_RDY:
                    case PSM_PE_VCS_EVALUATE_SWAP:
                    case PSM_PE_VCS_ACCEPT_SWAP:
                    case PSM_PE_VCS_REJECT_SWAP:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;
#endif

            case PD_DPM_GET_SRC_EXT_CAP:
                switch (pdInstance->psmCurState)
                {
                    case PE_SNK_GET_SOURCE_CAP_EXT:
                    case PE_DR_SRC_GET_SOURCE_CAP_EXT:
                    case PE_SRC_GIVE_SOURCE_CAP_EXT:
                    case PE_DR_SNK_GIVE_SOURCE_CAP_EXT:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case PD_DPM_GET_STATUS:
                switch (pdInstance->psmCurState)
                {
                    case PE_SNK_Get_Source_Status:
                    case PE_SRC_Give_Source_Status:
                    case PE_SRC_Get_Sink_Status:
                    case PE_SNK_Give_Sink_Status:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_GET_BATTERY_CAP:
                switch (pdInstance->psmCurState)
                {
                    case PE_Give_Battery_Cap:
                    case PE_Get_Battery_Cap:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_GET_BATTERY_STATUS:
                switch (pdInstance->psmCurState)
                {
                    case PE_Get_Battery_Status:
                    case PE_Give_Battery_Status:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_GET_MANUFACTURER_INFO:
                switch (pdInstance->psmCurState)
                {
                    case PE_Get_Manufacturer_Info:
                    case PE_Give_Manufacturer_Info:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

#if 0
            case PD_DPM_SECURITY_REQUEST:
                switch (pdInstance->psmCurState)
                {
                    case PE_Send_Security_Request:
                    case PE_Send_Security_Response:
                    case PE_Security_Response_Received:
                    case PSM_PE_SNK_READY:
                    case PSM_PE_SRC_READY:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;
#endif

            case PD_DPM_ALERT:
                switch (pdInstance->psmCurState)
                {
                    case PE_SRC_Send_Source_Alert:
                    case PE_SNK_Source_Alert_Received:
                    case PE_SNK_Send_Sink_Alert:
                    case PE_SRC_Sink_Alert_Received:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;
#endif

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PD_DPM_FAST_ROLE_SWAP:
                switch (pdInstance->psmCurState)
                {
                    case PE_FRS_SRC_SNK_CC_Signal:
                    case PE_FRS_SRC_SNK_Evaluate_Swap:
                    case PE_FRS_SRC_SNK_Accept_Swap:
                    case PE_FRS_SRC_SNK_Transition_to_off:
                    case PE_FRS_SRC_SNK_Assert_Rd:
                    case PE_FRS_SRC_SNK_Wait_Source_on:
                    case PE_FRS_SNK_SRC_Send_Swap:
                    case PE_FRS_SNK_SRC_Transition_to_off:
                    case PE_FRS_SNK_SRC_Vbus_Applied:
                    case PE_FRS_SNK_SRC_Assert_Rp:
                    case PE_FRS_SNK_SRC_Source_on:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;
#endif

            default:
            {
                break;
            }
        }

        if (commandFail)
        {
            PD_PsmCommandFail(pdInstance, pdInstance->commandProcessing);
        }
    }
}

static uint8_t PD_PsmRdyStateCheckDpmMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    if (triggerInfo->dpmMsg >= PD_DPM_CONTROL_DISCOVERY_IDENTITY)
    {
        return 0;
    }
    triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
    if (!PD_PsmStartCommand(pdInstance, triggerInfo->dpmMsg, 1))
    {
        PD_PsmCommandFail(pdInstance, triggerInfo->dpmMsg);
        return 0;
    }
    return 1;
}

static void PD_PsmSinkAndSourceRdyProcessDpmMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    if (PD_PsmRdyStateCheckDpmMessage(pdInstance, triggerInfo))
    {
        switch (triggerInfo->dpmMsg)
        {
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PD_DPM_CONTROL_PR_SWAP:
            {
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPrSwapWaitTimer))
                {
                    if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                    {
                        pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP;
                    }
                    else
                    {
                        pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP;
                    }
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_PRS_EVALUATE_PR_SWAP_WAIT_TIMER_TIME_OUT;
                }
                break;
            }
#endif

#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
            case PD_DPM_CONTROL_DR_SWAP:
            {
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
                /* check alternate mode */
                if (PD_PsmCheckInAltMode(pdInstance, kPD_MsgSOP))
                {
                    /* exit alt mode and wait the exit done */
                    uint8_t drSwapState = PD_AltModeExitModeForDrSwap(pdInstance->altModeHandle);
                    if (drSwapState == 0)
                    {
                        /* wait exiting alt mode */
                        PD_TimerStart(pdInstance, tDrSwapWaitTimer, T_DRSWAP_WAIT_ALT_MODE);
                        pdInstance->psmNewState = PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_ALT_MODE_EXIT;
                    }
                    else if (drSwapState == 1)
#endif
                    {
                        /* accept dr swap */
                        if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tDrSwapWaitTimer))
                        {
                            pdInstance->psmNewState = PSM_PE_DRS_SEND_DR_SWAP;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_TIMER_TIME_OUT;
                        }
                    }
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
                    else
                    {
                        /* reject dr swap */
                        pdInstance->commandEvaluateResult = kCommandResult_Reject;
                        pdInstance->psmNewState           = PSM_PE_DRS_REJECT_DR_SWAP;
                        break;
                    }
                }
#endif
                break;
            }
#endif
#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
            case PD_DPM_CONTROL_VCONN_SWAP:
            {
                pdInstance->psmNewState = PSM_PE_VCS_SEND_SWAP;
                break;
            }
#endif
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case PD_DPM_GET_BATTERY_CAP:
            {
                pdInstance->psmNewState = PE_Get_Battery_Cap;
                break;
            }
            case PD_DPM_GET_BATTERY_STATUS:
            {
                pdInstance->psmNewState = PE_Get_Battery_Status;
                break;
            }
            case PD_DPM_GET_MANUFACTURER_INFO:
            {
                pdInstance->psmNewState = PE_Get_Manufacturer_Info;
                break;
            }
#if 0
            case PD_DPM_SECURITY_REQUEST:
            {
                pdInstance->psmNewState = PE_Send_Security_Request;
                break;
            }
#endif
#endif

            default:
                break;
        }
    }
}

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
static void PD_PsmSinkRdyProcessDpmMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    if (PD_PsmRdyStateCheckDpmMessage(pdInstance, triggerInfo))
    {
        switch (triggerInfo->dpmMsg)
        {
            case PD_DPM_CONTROL_REQUEST:
            {
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSinkRequestTimer))
                {
                    pdInstance->psmNewState = PSM_PE_SNK_SELECT_CAPABILITY;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT;
                }
                break;
            }
            case PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES:
            {
                pdInstance->psmNewState = PSM_PE_SNK_GET_SOURCE_CAP;
                break;
            }
            case PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES:
            {
                pdInstance->psmNewState = PSM_PE_DR_SNK_GET_SINK_CAP;
                break;
            }
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case PD_DPM_GET_SRC_EXT_CAP:
            {
                pdInstance->psmNewState = PE_SNK_GET_SOURCE_CAP_EXT;
                break;
            }
            case PD_DPM_GET_STATUS:
            {
                pdInstance->psmNewState = PE_SNK_Get_Source_Status;
                break;
            }
            case PD_DPM_ALERT:
            {
                pdInstance->psmNewState = PE_SNK_Send_Sink_Alert;
                break;
            }
#endif

            default:
                break;
        }

        PD_PsmSinkAndSourceRdyProcessDpmMessage(pdInstance, triggerInfo);
    }
}
#endif

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
static void PD_PsmSourceRdyProcessDpmMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    if (PD_PsmRdyStateCheckDpmMessage(pdInstance, triggerInfo))
    {
        switch (triggerInfo->dpmMsg)
        {
            case PD_DPM_CONTROL_POWER_NEGOTIATION:
            {
                pdInstance->psmNewState = PSM_PE_SRC_SEND_CAPABILITIES;
                break;
            }
            case PD_DPM_CONTROL_GOTO_MIN:
            {
                pdInstance->psmGotoMinTx = 1;
                pdInstance->psmNewState  = PSM_PE_SRC_TRANSITION_SUPPLY;
                break;
            }
            case PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES:
            {
                pdInstance->psmNewState = PSM_PE_DR_SRC_GET_SOURCE_CAP;
                break;
            }
            case PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES:
            {
                pdInstance->psmNewState = PSM_PE_SRC_GET_SINK_CAP;
                break;
            }
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case PD_DPM_GET_SRC_EXT_CAP:
            {
                pdInstance->psmNewState = PE_DR_SRC_GET_SOURCE_CAP_EXT;
                break;
            }
            case PD_DPM_GET_STATUS:
            {
                pdInstance->psmNewState = PE_SRC_Get_Sink_Status;
                break;
            }
            case PD_DPM_ALERT:
            {
                pdInstance->psmNewState = PE_SRC_Send_Source_Alert;
                break;
            }
#endif

            default:
                break;
        }

        PD_PsmSinkAndSourceRdyProcessDpmMessage(pdInstance, triggerInfo);
    }
}
#endif

static void PD_PsmRdyProcessPdMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    uint8_t replyNotSupport = 0;
    uint8_t sinkOrSoruceNotSupportState;
    uint8_t msgProcessed = 1;

    if ((triggerInfo->pdMsgSop != kPD_MsgSOP) || (triggerInfo->pdMsgType == kPD_MsgVendorDefined))
    {
        return;
    }

    triggerInfo->triggerEvent = PSM_TRIGGER_NONE;

/* note: vdm message will be processed before this */
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
    if (pdInstance->curPowerRole == kPD_PowerRoleSource)
    {
        sinkOrSoruceNotSupportState = PSM_PE_SRC_READY;
        switch (triggerInfo->pdMsgType)
        {
            case kPD_MsgGotoMin:
            {
                /* not_supported */
                replyNotSupport = sinkOrSoruceNotSupportState;
                break;
            }
            case kPD_MsgGetSourceCap:
            {
                pdInstance->psmNewState = PSM_PE_SRC_SEND_CAPABILITIES;
                break;
            }

            case kPD_MsgGetSinkCap:
            {
                if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceOnly)
                {
                    replyNotSupport = sinkOrSoruceNotSupportState;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_DR_SRC_GIVE_SINK_CAP;
                }
                break;
            }

            case kPD_MsgRequest:
            {
                pdInstance->partnerRdoRequest.rdoVal = *((uint32_t *)(&(triggerInfo->pdMsgDataBuffer[0])));
#if (defined PD_CONFIG_PD3_PPS_ENABLE) && (PD_CONFIG_PD3_PPS_ENABLE)
                /* SourcePPSCommTimer */
                if (PD_PsmSourceIsPPSRDO(pdInstance))
                {
                    PD_TimerStart(pdInstance, tSourcePPSCommTimer, T_PPS_TIMEOUT);
                }
#endif
                pdInstance->psmNewState = PSM_PE_SRC_NEGOTIATE_CAPABILITY;
                PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_REQUEST, 0);
                break;
            }

            case kPD_MsgGetStatus:
            {
                pdInstance->alertWaitReply = 0;
                pdInstance->psmNewState    = PE_SRC_Give_Source_Status;
                break;
            }

            case kPD_MsgFrSwap:
            {
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                if ((PD_PsmIsDualRole(pdInstance)) && (pdInstance->frSignaledWaitFrSwap))
                {
                    /* no need to enter PE_FRS_SRC_SNK_CC_Signal */
                    PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InFRSwap);
                    pdInstance->frSignaledWaitFrSwap = 0u;
                    pdInstance->psmNewState          = PE_FRS_SRC_SNK_Evaluate_Swap;
                }
                else
                {
                    pdInstance->psmNewState = PSM_HARD_RESET;
                }
#else
                replyNotSupport = sinkOrSoruceNotSupportState;
#endif
                break;
            }
            default:
                msgProcessed = 0;
                break;
        }
    }
    else
#endif
#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
    {
        sinkOrSoruceNotSupportState = PSM_PE_SNK_READY;
        switch (triggerInfo->pdMsgType)
        {
            case kPD_MsgRequest:
            {
                replyNotSupport = sinkOrSoruceNotSupportState;
                break;
            }

            case kPD_MsgGotoMin:
            {
                PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_GOTO_MIN, 0);
                pdInstance->psmGotoMinRx = 1;
                pdInstance->psmNewState  = PSM_PE_SNK_TRANSITION_SINK;
                break;
            }

            case kPD_MsgGetSinkCap:
            {
                pdInstance->psmNewState = PSM_PE_SNK_GIVE_SINK_CAP;
                break;
            }

            case kPD_MsgGetSourceCap:
            {
                if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkOnly)
                {
                    replyNotSupport = sinkOrSoruceNotSupportState;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_DR_SNK_GIVE_SOURCE_CAP;
                }
                break;
            }

            case kPD_MsgGetStatus:
            {
                pdInstance->alertWaitReply = 0;
                pdInstance->psmNewState    = PE_SNK_Give_Sink_Status;
                break;
            }

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case kPD_MsgFrSwap:
            {
                /* not_supported, only source can receive this msg */
                replyNotSupport = PSM_PE_SNK_READY;
                break;
            }
#endif

            default:
                msgProcessed = 0;
                break;
        }
    }
#else
    {
    }
#endif

    if (!msgProcessed)
    {
        msgProcessed = 1;
        uint32_t commandResultCallback;

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        pd_extended_msg_header_t extHeader;
        extHeader.extendedMsgHeaderVal =
            (uint32_t)(USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS((triggerInfo->pdMsgDataBuffer)));
#endif

        if ((triggerInfo->pdMsgSop != kPD_MsgSOP) || (triggerInfo->pdMsgType == kPD_MsgVendorDefined))
        {
            return;
        }

        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        /* sink and source all process these messages similarly */
        /* note: vdm message will be processed before this */
        switch (triggerInfo->pdMsgType)
        {
            case kPD_MsgPrSwap:
            {
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                if (PD_PsmIsDualRole(pdInstance))
                {
                    PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_PR_SWAP, 0);
                    if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                    {
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                        /*  auto accept is true and partner source cap is not got */
                        if ((!pdInstance->swapToSnkSrcCapReceived) && PD_POLICY_SUPPORT(pdInstance) &&
                            (PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SOURCE_SUPPORT(pdInstance)) &&
                            (PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SOURCE(pdInstance) == kAutoRequestProcess_Accept))
                        {
                            if (PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(
                                    pdInstance, kPD_MsgWait, PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP_GET_SOURCE_CAP))
                            {
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                                PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
#endif
                            }
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
                            PRINTF("auto reply wait for pr swap, and start to get the partner's source caps\r\n");
#endif
                            commandResultCallback = kCommandResult_Wait;
                            PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_FAIL, &commandResultCallback, 1);
                        }
                        else
#endif
                        {
                            pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP;
                        }
                    }
                    else
                    {
                        pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_EVALUATE_PR_SWAP;
                    }
                }
                else
#endif
                {
                    replyNotSupport = sinkOrSoruceNotSupportState;
                }
                break;
            }

            case kPD_MsgDrSwap:
            {
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                if (PD_PsmCheckInAltMode(pdInstance, kPD_MsgSOP))
                {
                    pdInstance->psmNewState = PSM_HARD_RESET;
                }
                else
                {
                    if (pdInstance->pdPowerPortConfig->dataFunction == kDataConfig_DRD)
                    {
                        PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_DR_SWAP, 0);
                        pdInstance->psmNewState = PSM_PE_DRS_EVALUATE_DR_SWAP;
                    }
                    else
                    {
                        replyNotSupport = sinkOrSoruceNotSupportState;
                    }
                }
#else
                replyNotSupport = sinkOrSoruceNotSupportState;
#endif
                break;
            }

            case kPD_MsgVconnSwap:
            {
#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
                if (pdInstance->pdPowerPortConfig->vconnSupported)
                {
                    PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_VCONN_SWAP, 0);
                    pdInstance->psmNewState = PSM_PE_VCS_EVALUATE_SWAP;
                }
                else
#endif
                {
                    replyNotSupport = sinkOrSoruceNotSupportState;
                }
                break;
            }

            case kPD_MsgGetSourceCapExtended:
            {
                if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkOnly)
                {
                    replyNotSupport = sinkOrSoruceNotSupportState;
                }
                else
                {
                    if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                    {
                        pdInstance->psmNewState = PE_SRC_GIVE_SOURCE_CAP_EXT;
                    }
                    else
                    {
                        pdInstance->psmNewState = PE_DR_SNK_GIVE_SOURCE_CAP_EXT;
                    }
                }
                break;
            }

            case kPD_MsgAlert:
            {
                pd_command_data_param_t commandExtParam;
                if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                {
                    pdInstance->psmNewState = PE_SNK_Source_Alert_Received;
                }
                else
                {
                    pdInstance->psmNewState = PE_SRC_Sink_Alert_Received;
                }
                commandExtParam.dataBuffer = &triggerInfo->pdMsgDataBuffer[0];
                commandExtParam.dataLength = 4;
                commandExtParam.sop        = kPD_MsgSOP;
                PD_DpmAppCallback(pdInstance, PD_DPM_ALERT_RECEIVED, &commandExtParam, 0);
                if (commandExtParam.resultStatus == kCommandResult_NotSupported)
                {
                    PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, kPD_MsgNotSupported,
                                                                       PE_PSM_STATE_ROLE_RDY_STATE);
                }
                break;
            }

            case kPD_MsgNotSupported:
                commandResultCallback = kCommandResult_NotSupported;
#if 0
                if (pdInstance->commandProcessing == PD_DPM_SECURITY_REQUEST)
                {
                    PD_DpmAppCallback(pdInstance, PD_DPM_SECURITY_REQUEST_FAIL, &commandResultCallback, 1);
                }
                else
#endif
                if (pdInstance->alertWaitReply)
                {
                    pdInstance->alertWaitReply = 0;
                    PD_DpmAppCallback(pdInstance, PD_DPM_SEND_ALERT_FAIL, &commandResultCallback, 0);
                }
                else
                {
                }
                break;

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case kPD_MsgGetBatteryCap:
            case kPD_MsgGetBatteryStatus:
            case kPD_MsgGetManufacturerInfo:
                pdInstance->commandExtParamCallback.dataBuffer = (uint8_t *)&(pdInstance->receivedData[1]);
                pdInstance->commandExtParamCallback.dataBuffer += 2; /* data pos */
                pdInstance->commandExtParamCallback.dataLength = extHeader.bitFields.dataSize;
                pdInstance->commandExtParamCallback.sop        = triggerInfo->pdMsgSop;
                if (triggerInfo->pdMsgType == kPD_MsgGetBatteryCap)
                {
                    pdInstance->psmNewState = PE_Give_Battery_Cap;
                }
                else if (triggerInfo->pdMsgType == kPD_MsgGetBatteryStatus)
                {
                    pdInstance->psmNewState = PE_Give_Battery_Status;
                }
                else
                {
                    pdInstance->psmNewState = PE_Give_Manufacturer_Info;
                }
                break;

            case kPD_MsgSecurityRequest:
            {
#if 0
                /* (initiator) chunked request received */
                pdInstance->psmNewState = PE_Send_Security_Response;
#endif
                replyNotSupport = sinkOrSoruceNotSupportState;
                break;
            }

#if 0
            case kPD_MsgSecurityResponse:
            {
                /* receive all the data */
                pdInstance->psmNewState = PE_Security_Response_Received;
                pdInstance->commandExtParamCallback.dataBuffer = (uint8_t *)&(pdInstance->receivedData[1]);
                pdInstance->commandExtParamCallback.dataBuffer += 2; /* data pos */
                pdInstance->commandExtParamCallback.dataLength = extHeader.bitFields.dataSize;
                pdInstance->commandExtParamCallback.sop = triggerInfo->pdMsgSop;
                break;
            }
#endif
#endif

            case kPD_MsgPing:
                /* nothing need do, in case the default soft_reset */
                break;

            default:
                if (((triggerInfo->pdMsgType > kPD_MsgInvalid) && (triggerInfo->pdMsgType <= kPD_MsgSoftReset)) ||
                    ((triggerInfo->pdMsgType >= kPD_MsgNotSupported) && (triggerInfo->pdMsgType <= kPD_MsgFrSwap)) ||
                    ((triggerInfo->pdMsgType >= kPD_MsgSourceCapabilities) &&
                     (triggerInfo->pdMsgType <= kPD_MsgVendorDefined)) ||
                    ((triggerInfo->pdMsgType >= kPD_MsgSourceCapExtended) &&
                     (triggerInfo->pdMsgType <= kPD_MsgFirmwareUpdaetResponse)))
                {
                    /* unexpected message */
                    pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                }
                else
                {
                    /* receive unrecognized message */
                    replyNotSupport = sinkOrSoruceNotSupportState;
                }
                break;
        }
    }

    if (replyNotSupport)
    {
        PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, PD_NOT_SUPPORT_REPLY_MSG,
                                                           (pd_psm_state_t)replyNotSupport);
    }
}

static uint8_t PD_PsmCanPendingReceive(pd_instance_t *pdInstance)
{
    if ((pdInstance->psmCurState == PSM_PE_SNK_STARTUP) || (pdInstance->psmCurState == PSM_PE_SNK_DISCOVERY) ||
        (pdInstance->psmCurState == PSM_PE_SNK_TRANSITION_TO_DEFAULT) || (pdInstance->psmCurState == PSM_HARD_RESET))
    {
        return 0;
    }

    if ((pdInstance->psmNewState == PSM_PE_SNK_STARTUP) || (pdInstance->psmNewState == PSM_PE_SNK_DISCOVERY) ||
        (pdInstance->psmNewState == PSM_PE_SNK_TRANSITION_TO_DEFAULT) || (pdInstance->psmNewState == PSM_HARD_RESET) ||
        (pdInstance->psmNewState == PSM_PE_SNK_WAIT_FOR_CAPABILITIES))
    {
        return 0;
    }

    return 1;
}

static void PD_PsmProcessImportEventBeforeNextStateMachine(pd_instance_t *pdInstance)
{
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
    uint32_t taskEventSet;
    if (USB_OsaEventCheck(pdInstance->taskEventHandle, PD_TASK_EVENT_FR_SWAP_SINGAL, &taskEventSet) ==
        kStatus_USB_OSA_Success)
    {
        if (taskEventSet & PD_TASK_EVENT_FR_SWAP_SINGAL)
        {
            USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_FR_SWAP_SINGAL);
            if (pdInstance->curPowerRole == kPD_PowerRoleSink)
            {
                pdInstance->psmNewState = PE_FRS_SNK_SRC_Send_Swap;
            }
        }
    }
#endif
}

static uint8_t PD_PsmNoResponseHardResetCountCheck(pd_instance_t *pdInstance)
{
    if ((PD_TimerCheckValidTimeOut(pdInstance, tNoResponseTimer)) &&
        (pdInstance->psmHardResetCount > N_HARD_RESET_COUNT))
    {
        if (pdInstance->psmPreviouslyPdConnected)
        {
            pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
            return 1;
        }
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
        else
        {
            if (pdInstance->curPowerRole == kPD_PowerRoleSource)
            {
                pdInstance->psmNewState = PSM_PE_SRC_DISABLED;
                return 1;
            }
        }
#endif
    }
    return 0;
}

#if (defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)) || \
    (defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)) ||   \
    (defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT))
static void PD_PsmStateWaitReply(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo, uint8_t *didNothingStepB)
{
    pd_psm_state_t acceptState;
    pd_psm_state_t otherState;
    uint8_t failCallbackEvent;
    uint8_t commandResultCallback;

    switch (pdInstance->psmCurState)
    {
#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
        case PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP:
            acceptState       = PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF;
            otherState        = PSM_PE_SRC_READY;
            failCallbackEvent = PD_DPM_PR_SWAP_FAIL;
            break;
        case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP:
            acceptState       = PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF;
            otherState        = PSM_PE_SNK_READY;
            failCallbackEvent = PD_DPM_PR_SWAP_FAIL;
            break;
#endif
#if defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
        case PSM_PE_DRS_SEND_DR_SWAP:
            acceptState       = PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP;
            otherState        = (pd_psm_state_t)pdInstance->psmDrSwapPrevState;
            failCallbackEvent = PD_DPM_DR_SWAP_FAIL;
            break;
#endif
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
        case PSM_PE_VCS_SEND_SWAP:
            acceptState       = (pdInstance->psmPresentlyVconnSource == kPD_IsVconnSource ? PSM_PE_VCS_WAIT_FOR_VCONN :
                                                                                      PSM_PE_VCS_TURN_ON_VCONN);
            otherState        = (pd_psm_state_t)pdInstance->psmVconnSwapPrevState;
            failCallbackEvent = PD_DPM_VCONN_SWAP_FAIL;
            break;
#endif
        default:
            acceptState       = PSM_UNKNOWN;
            otherState        = PSM_UNKNOWN;
            failCallbackEvent = 0;
            break;
    }

    if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (failCallbackEvent != 0))
    {
        switch (triggerInfo->pdMsgType)
        {
            case kPD_MsgAccept:
            {
#if defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                if (failCallbackEvent == PD_DPM_DR_SWAP_FAIL)
                {
                    pdInstance->curDataRole =
                        (pdInstance->curDataRole == kPD_DataRoleUFP) ? kPD_DataRoleDFP : kPD_DataRoleUFP;
                    PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
                }
#endif
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                commandResultCallback = kCommandResult_Accept;
#endif
                pdInstance->psmNewState = acceptState;
                break;
            }

            case kPD_MsgReject:
            {
                pdInstance->psmNewState = otherState;
                commandResultCallback   = kCommandResult_Reject;
                PD_DpmAppCallback(pdInstance, failCallbackEvent, &commandResultCallback, 1);
                break;
            }

            case kPD_MsgWait:
            {
                pdInstance->psmNewState = otherState;
#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                if (failCallbackEvent == PD_DPM_PR_SWAP_FAIL)
                {
                    PD_TimerStart(pdInstance, tPrSwapWaitTimer, T_PRSWAP_WAIT);
                }
#endif
#if defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                if (failCallbackEvent == PD_DPM_DR_SWAP_FAIL)
                {
                    PD_TimerStart(pdInstance, tDrSwapWaitTimer, T_DRSWAP_WAIT);
                }
#endif
                commandResultCallback = kCommandResult_Wait;
                PD_DpmAppCallback(pdInstance, failCallbackEvent, &commandResultCallback, 1);
                break;
            }

            default:
                if (triggerInfo->pdMsgType != kPD_MsgInvalid)
                {
                    /* SourceCapabilities and VendorDefined are handled in the global section. */
                    /* soft reset other packets */
                    pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                }
                break;
        }
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
        PD_PsmReadyAutoPolicyResult(pdInstance, commandResultCallback);
#endif
    }
    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
    {
        pdInstance->psmNewState = otherState;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
        PD_PsmReadyAutoPolicyResult(pdInstance, kCommandResult_Timeout);
#endif
    }
    else
    {
        *didNothingStepB = 1;
    }
}
#endif

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
static void PD_PsmPowerSwapSinkSourceTransitionOff(pd_instance_t *pdInstance,
                                                   psm_trigger_info_t *triggerInfo,
                                                   pd_psm_state_t newState,
                                                   uint8_t *didNothingStepB)
{
    if (triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG)
    {
        if ((triggerInfo->pdMsgSop == kPD_MsgSOP) && (triggerInfo->pdMsgType == kPD_MsgPsRdy))
        {
            PD_TimerClear(pdInstance, tPSSourceOffTimer);
            pdInstance->curPowerRole = kPD_PowerRoleSource;
            PD_MsgSetPortRole(pdInstance, kPD_PowerRoleSource, pdInstance->curDataRole);
            pdInstance->psmNewState = newState;
        }
        else if ((triggerInfo->pdMsgSop == kPD_MsgSOP) && (triggerInfo->pdMsgType == kPD_MsgPing))
        {
            /* Remain in the same state */
            pdInstance->psmNewState = pdInstance->psmCurState;
        }
        else
        {
            /* A protocol error during power role swap triggers a Hard Reset */
            pdInstance->psmNewState = PSM_HARD_RESET;
        }
    }
    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSSourceOffTimer))
    {
        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
    }
    else
    {
        *didNothingStepB = 1;
    }
}
#endif

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
static void PD_PsmPowerSwapSinkOpenVbus(pd_instance_t *pdInstance, uint8_t callbackEvent)
{
    if (PD_PsmSendControlTransitionWithErrorRecovery(pdInstance, kPD_MsgPsRdy, PSM_PE_SRC_STARTUP))
    {
        /* Swap from SINK to SOURCE */
        /* pr swap end */
        PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
        /* Cable plug will need a soft reset */
        pdInstance->psmCablePlugResetNeeded = 1;
#endif
        pdInstance->enterSrcFromSwap = 1;
        PD_DpmAppCallback(pdInstance, callbackEvent, NULL, 1);
    }
}
#endif

#if (defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)) || \
    (defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)) ||   \
    (defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT))
static void PD_PsmEvaluateSwap(pd_instance_t *pdInstance,
                               uint8_t requestEvent,
                               pd_psm_state_t acceptState,
                               pd_psm_state_t rejectState)
{
    if (requestEvent != PD_DPM_EVENT_INVALID)
    {
        PD_DpmAppCallback(pdInstance, requestEvent, &pdInstance->commandEvaluateResult, 0);
    }
    if (pdInstance->commandEvaluateResult == kCommandResult_Accept)
    {
        pdInstance->psmNewState = acceptState;
    }
    else
    {
        pdInstance->psmNewState = rejectState;
    }
}
#endif

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
static void PD_PsmPowerSwapAssertRdRp(pd_instance_t *pdInstance, uint8_t powerRole, pd_psm_state_t nextState)
{
    pdInstance->curPowerRole = powerRole;
    PD_ConnectSetPRSwapRole(pdInstance, pdInstance->curPowerRole);
    pdInstance->psmNewState = nextState;
}
#endif

static void PD_PsmResetStateWhenEnterReadyState(pd_instance_t *pdInstance)
{
#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE))
    pdInstance->psmSecondaryState[0] = PSM_IDLE;
    pdInstance->psmSecondaryState[1] = PSM_IDLE;
    pdInstance->psmSecondaryState[2] = PSM_IDLE;
#endif
    pdInstance->psmExplicitContractExisted = 1;
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
    if (PD_CheckWhetherInitiateCableDiscoveryIdentityOrNot(pdInstance))
    {
        pdInstance->pendingSOP = kPD_MsgSOPMask | kPD_MsgSOPpMask | kPD_MsgSOPppMask;
        PD_TimerStart(pdInstance, tDiscoverIdentityTimer, T_DISCOVER_IDENTITY);
    }
    else
#endif
    {
        pdInstance->pendingSOP = kPD_MsgSOPMask;
    }
}

#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
static void PD_StateSendReplyExtDataTransition(pd_instance_t *pdInstance, uint8_t msgType)
{
    if (pdInstance->commandExtParamCallback.resultStatus != kCommandResult_NotSupported)
    {
        if (msgType == kPD_MsgBatteryStatus)
        {
            PD_MsgSendDataTransition(pdInstance, msgType, pdInstance->commandExtParamCallback.dataLength >> 2,
                                     (uint32_t *)pdInstance->commandExtParamCallback.dataBuffer,
                                     PE_PSM_STATE_ROLE_RDY_STATE, PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
        }
        else
        {
            PD_MsgSendExtTransition(pdInstance, kPD_MsgSOP, msgType, pdInstance->commandExtParamCallback.dataLength,
                                    pdInstance->commandExtParamCallback.dataBuffer, PE_PSM_STATE_ROLE_RDY_STATE,
                                    PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
        }
    }
    else
    {
        PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, kPD_MsgNotSupported,
                                                           PE_PSM_STATE_ROLE_RDY_STATE);
    }
}
#endif

static void PD_PsmCheckChunkedFeature(pd_instance_t *pdInstance, pd_rdo_t rdo)
{
    if ((rdo.bitFields.unchunkedSupported) && (pdInstance->selfOrPartnerFirstSourcePDO.fixedPDO.unchunkedSupported))
    {
        pdInstance->unchunkedFeature = 1;
    }
    else
    {
        pdInstance->unchunkedFeature = 0;
    }
}

static void PD_PsmTimerWait(pd_instance_t *pdInstance, uint16_t time)
{
    PD_TimerStart(pdInstance, tDelayTimer, time);
    while (!(PD_TimerCheckInvalidOrTimeOut(pdInstance, tDelayTimer)))
    {
        ;
    }
}

#if ((defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)) || \
    ((PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE))
static void PD_PsmSetPhyMessageHeader(pd_instance_t *pdInstance, uint8_t dataRole)
{
    pd_phy_msg_header_info_t msgHeader;
    msgHeader.dataRole  = dataRole;
    msgHeader.powerRole = pdInstance->curPowerRole;
    msgHeader.cablePlug = 0;
    msgHeader.revision  = pdInstance->revision;
    PD_PhyControl(pdInstance, PD_PHY_SET_MSG_HEADER_INFO, &msgHeader);
}
#endif

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
static void PD_PsmTurnOffVconnAndVbus(pd_instance_t *pdInstance, uint8_t powerState)
{
#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
    if (pdInstance->callbackFns->PD_ControlVconn != NULL)
    {
        pdInstance->callbackFns->PD_ControlVconn(pdInstance->callbackParam, 0);
    }
#endif
    if (pdInstance->callbackFns->PD_SrcTurnOffVbus != NULL)
    {
        pdInstance->callbackFns->PD_SrcTurnOffVbus(pdInstance->callbackParam, powerState);
    }
}
#endif

/* state process small functions when enter the state */
static inline void PD_PsmEnterIdleState(pd_instance_t *pdInstance)
{
    pdInstance->psmHardResetCount = 0;
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
    pdInstance->psmSendCapsCounter = 0;
#endif
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
    pdInstance->cableDiscoverIdentityCounter = 0;
#endif
    /* kVbusPower_InHardReset is end */
    PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
    PD_MsgInit(pdInstance);
    PD_PsmReset(pdInstance);
    if (pdInstance->pdConfig->deviceType == kDeviceType_AlternateModeProduct)
    {
        /* If we are an alternate mode adapter, then set tAMETimeout. */
        PD_TimerStart(pdInstance, tAMETimeoutTimer, T_AME_TIMEOUT);
    }

    if (pdInstance->curPowerRole == kPD_PowerRoleSource)
    {
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
        pdInstance->curPowerRole     = kPD_PowerRoleSource;
        pdInstance->enterSrcFromSwap = 0;
        pdInstance->psmNewState      = PSM_PE_SRC_STARTUP;
#else
        pdInstance->psmNewState = PSM_BYPASS;
#endif
    }
    else if (pdInstance->curPowerRole == kPD_PowerRoleSink)
    {
#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
        pdInstance->curPowerRole = kPD_PowerRoleSink;
        pdInstance->psmNewState  = PSM_PE_SNK_STARTUP;
#else
        pdInstance->psmNewState = PSM_BYPASS;
#endif
    }
    else
    {
    }
}

static inline void PD_PsmEnterHardResetState(pd_instance_t *pdInstance)
{
#if ((defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT))
    PD_MsgChunkingLayerResetFlags(pdInstance);
#endif
#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
    /* Do this as early as possible, to prevent disconnects
     * Hard Reset is in progress, ignore VBus going away.
     * source need set this too, because judge vbus too (dead battery related) for disconnect.
     */
    PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InHardReset);
#endif
    PD_PsmReset(pdInstance);
    /* Delay for 1ms to make sure any previous GoodCRC tranmission has ended. */
    PD_PsmTimerWait(pdInstance, mSec(1));
    /* source send hard_reset, 1. send hard_reset msg. */
    PD_MsgSendHardReset(pdInstance);
    if (pdInstance->curPowerRole == kPD_PowerRoleSource)
    {
        PD_TimerStart(pdInstance, tPSHardResetTimer, T_PS_HARD_RESET);
    }
    pdInstance->psmHardResetCount++;
}

static inline void PD_PsmEnterSendSoftResetState(pd_instance_t *pdInstance)
{
    /* Interate at least 1 time to cover an RX between our initial discard, and tx completion */
    uint8_t retryCount = 1; /* as retry time variable */
#if ((defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT))
    PD_MsgChunkingLayerResetFlags(pdInstance);
#endif
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
    if (0 /*pdInstance->psmSoftResetSop != kPD_MsgSOP */)
    {
        PD_PsmSecondaryStateHandlerTerminate(pdInstance, pdInstance->psmSoftResetSop);
        /*Pd_PsmSecondaryStateHandler(pdInstance, pdInstance->psmSoftResetSop, pdInstance->psmSoftResetSop,
         */
        /*&triggerInfo); */
    }
#endif

    while (1)
    {
        if ((PD_MsgSend(pdInstance, kPD_MsgSOP, kPD_MsgSoftReset, 2, NULL) == kStatus_PD_Success) &&
            (PD_MsgWaitSendResult(pdInstance)))
        {
            PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
        }
        else if (((PD_MsgRecvPending(pdInstance)) && (((MSG_DATA_HEADER & PD_MSG_HEADER_MESSAGE_TYPE_MASK) >>
                                                       PD_MSG_HEADER_MESSAGE_TYPE_POS) == kPD_MsgSoftReset)) ||
                 (pdInstance->hardResetReceived))
        {
            PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_SUCCESS, NULL,
                              (pdInstance->commandProcessing == PD_DPM_CONTROL_SOFT_RESET) ? 1 : 0);
        }
        else if (PD_MsgRecvPending(pdInstance) && (retryCount > 0))
        {
            retryCount--;
            continue;
        }
        else
        {
            if (pdInstance->commandProcessing == PD_DPM_CONTROL_SOFT_RESET)
            {
                PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_FAIL, NULL, 1);
            }
            pdInstance->psmNewState = PSM_HARD_RESET;
        }
        break;
    }
}

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
static inline void PD_PsmEnterSrcStartUpState(pd_instance_t *pdInstance)
{
    pdInstance->psmPresentlyPdConnected = 0;
    pdInstance->psmSendCapsCounter      = 0;
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
    pdInstance->cableDiscoverIdentityCounter = 0;
#endif

    /* kVbusPower_InHardReset is end */
    PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
    /* Delay for 1ms to make sure any previous GoodCRC tranmission has ended. */
    PD_PsmTimerWait(pdInstance, mSec(1));
    PD_MsgReset(pdInstance);
    PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
    PD_PsmSetPhyMessageHeader(pdInstance, pdInstance->curDataRole);
    pdInstance->pendingSOP = kPD_MsgSOPMask;
    PD_MsgStartReceive(pdInstance);
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
#endif

    /* A large value allows the sink to debounce CC and VBUS after the connection. */
    if (pdInstance->enterSrcFromSwap)
    {
        PD_TimerStart(pdInstance, tSwapSourceStartTimer, T_SEND_SOURCE_CAP);
    }
    else
    {
        pdInstance->psmNewState = PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY;
    }
}

static inline void PD_PsmEnterImplicitCableDiscoveryState(pd_instance_t *pdInstance)
{
#if (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)) && \
    (defined(PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG))
    /* (The initial source && Ra present) || (swap to source from initial sink, cannot detect the Ra) */
    if (((pdInstance->raPresent) || (pdInstance->initialPowerRole == kPD_PowerRoleSink)) &&
        (pdInstance->psmPresentlyVconnSource) && (pdInstance->psmCableIdentitiesDataCount == 0))
    {
        pdInstance->pendingSOP = kPD_MsgSOPMask | kPD_MsgSOPpMask | kPD_MsgSOPppMask;
        if (pdInstance->psmCablePlugResetNeeded)
        {
            pdInstance->psmNewSecondaryState[1] = PSM_PE_SRC_IMPLICIT_CABLE_SOFT_RESET;
        }
        else
        {
            pdInstance->psmNewSecondaryState[1] = PSM_PE_SRC_VDM_IDENTITY_REQUEST;
        }
    }
    else
#endif
    {
#if (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)) && \
    (defined(PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG))
        pdInstance->pendingSOP = kPD_MsgSOPMask;
#endif
        pdInstance->psmNewState = PSM_PE_SRC_SEND_CAPABILITIES;
    }
}

static inline void PD_PsmEnterSrcSendCapsState(pd_instance_t *pdInstance)
{
    pdInstance->selfOrPartnerFirstSourcePDO.PDOValue = pdInstance->pdPowerPortConfig->sourceCaps[0];
    if (pdInstance->commandProcessing == 0)
    {
        /* case1: the start-up state machine; case2: dpm msg PD_DPM_CONTROL_POWER_NEGOTIATION */
        if (!PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_POWER_NEGOTIATION, 1))
        {
            /* invalid code, for coverity fix. */
            pdInstance->commandProcessing = 0;
        }
    }
    pdInstance->psmSendCapsCounter++;
    if (PD_MsgSendDataTransition(pdInstance, kPD_MsgSourceCapabilities, pdInstance->pdPowerPortConfig->sourceCapCount,
                                 PD_PsmGetSourcePDOs(pdInstance), PE_PSM_STATE_NO_CHANGE, PSM_CHECK_ASYNC_RX,
                                 pdInstance->psmPresentlyPdConnected ? PSM_SEND_SOFT_RESET : PSM_PE_SRC_DISCOVERY))
    {
        pdInstance->psmPresentlyPdConnected  = 1;
        pdInstance->psmPreviouslyPdConnected = 1;
        PD_TimerClear(pdInstance, tNoResponseTimer);
        pdInstance->psmHardResetCount  = 0;
        pdInstance->psmSendCapsCounter = 0;
        PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
    }
}

static inline void PD_PsmEnterSrcNegotiateCapState(pd_instance_t *pdInstance)
{
    pd_negotiate_power_request_t negotiateResult;

    PD_PsmCheckChunkedFeature(pdInstance, pdInstance->partnerRdoRequest);
    negotiateResult.rdo             = pdInstance->partnerRdoRequest;
    negotiateResult.negotiateResult = kCommandResult_Accept;
    PD_DpmAppCallback(pdInstance, PD_DPM_SRC_RDO_REQUEST, &negotiateResult, 0);
    if (negotiateResult.negotiateResult == kCommandResult_Accept)
    {
        pdInstance->psmGotoMinTx = 0;
        pdInstance->psmNewState  = PSM_PE_SRC_TRANSITION_SUPPLY;
    }
    else
    {
        pdInstance->commandEvaluateResult = negotiateResult.negotiateResult;
        pdInstance->psmNewState           = PSM_PE_SRC_CAPABILITY_RESPONSE;
    }
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
#endif
}

static inline void PD_PsmEnterSrcTransitionSupplyState(pd_instance_t *pdInstance)
{
    if (PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(
            pdInstance, (message_type_t)((pdInstance->psmGotoMinTx) ? kPD_MsgGotoMin : kPD_MsgAccept),
            PE_PSM_STATE_NO_CHANGE))
    {
        /* transition power */
        PD_PsmTimerWait(pdInstance, T_SRC_TRANSITION);
        if (pdInstance->psmGotoMinTx)
        {
            pdInstance->callbackFns->PD_SrcGotoMinReducePower(pdInstance->callbackParam);
        }
        else
        {
            pdInstance->callbackFns->PD_SrcTurnOnRequestVbus(pdInstance->callbackParam, pdInstance->partnerRdoRequest);
        }
    }
    else
    {
        if (pdInstance->psmGotoMinTx)
        {
            pdInstance->psmGotoMinTx = 0;
        }
    }
}

static inline void PD_PsmEnterSrcCapResponseState(pd_instance_t *pdInstance)
{
    uint8_t msgType;
    if (pdInstance->commandEvaluateResult == kCommandResult_Reject)
    {
        msgType = kPD_MsgReject;
    }
    else
    {
        msgType                           = kPD_MsgWait;
        pdInstance->commandEvaluateResult = kCommandResult_Error;
    }

    if (PD_PsmSendControlTransitionWithHardReset(pdInstance, (message_type_t)msgType, PE_PSM_STATE_NO_CHANGE))
    {
        PD_DpmAppCallback(pdInstance, PD_DPM_SRC_RDO_FAIL, &pdInstance->commandEvaluateResult, 1);
        if (!pdInstance->psmExplicitContractExisted)
        {
            pdInstance->psmNewState = PSM_PE_SRC_WAIT_NEW_CAPABILITIES;
        }
        else
        {
            uint8_t stillValid = 0;
            PD_DpmAppCallback(pdInstance, PD_DPM_SRC_CONTRACT_STILL_VALID, &stillValid, 0);
            if ((stillValid) || (msgType == kPD_MsgWait))
            {
                pdInstance->psmNewState = PSM_PE_SRC_READY;
            }
            else
            {
                pdInstance->psmNewState = PSM_HARD_RESET;
            }
        }
    }
}

static inline void PD_PsmEnterSrcTransitionToDefaultState(pd_instance_t *pdInstance)
{
    uint8_t delayTime;
    pdInstance->psmPresentlyPdConnected    = 0;
    pdInstance->psmExplicitContractExisted = 0u;
    pdInstance->psmHardResetNeedsVSafe0V   = 1;

    /* source send hard_reset, 2. change to supply vsafe5v. */
    /* source receive hard_reset, 2. change to supply vsafe5v. */
    PD_TimerClear(pdInstance, tSrcRecoverTimer);
    do
    {
        PD_PsmTurnOffVconnAndVbus(pdInstance, kVbusPower_InHardReset);
        PD_DpmDischargeVbus(pdInstance, 1);
        delayTime = 10;
        while ((!PD_PsmCheckVsafe0V(pdInstance)) && (delayTime != 0))
        {
            delayTime--;
        }
    } while (delayTime == 0);

    PD_DpmDischargeVbus(pdInstance, 0);
    /* source send hard_reset, 3. start tSrcRecover timer. */
    /* source receive hard_reset, 3. start tSrcRecover timer. */
    /* 1. vbus is vsafe0v -> start the tSrcRecover timer -> After tSrcRecover the Source applies power to
     * VBUS */
    PD_TimerStart(pdInstance, tSrcRecoverTimer, T_SRC_RECOVER);

    /* Message reception should not be re-enabled until PE_SNK_Startup */
    PD_MsgInit(pdInstance);
    /* Change our data role to DFP, and turn off vconn */
    pdInstance->curDataRole             = kPD_DataRoleDFP;
    pdInstance->psmPresentlyVconnSource = kPD_IsVconnSource;
    PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
#ifdef USBPD_ENABLE_VCONN_DISCHARGE
    PD_DpmDischargeVconn(pdInstance, 1);
#endif
#endif
}

static inline void PD_PsmEnterGiveSrcCapState(pd_instance_t *pdInstance)
{
    if (((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceOnly) ||
         (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault) ||
         (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceDefault) ||
         (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)) &&
        (pdInstance->pdPowerPortConfig->sourceCaps != NULL))
    {
        PD_MsgSendDataTransition(pdInstance, kPD_MsgSourceCapabilities, pdInstance->pdPowerPortConfig->sourceCapCount,
                                 PD_PsmGetSourcePDOs(pdInstance), PE_PSM_STATE_ROLE_RDY_STATE, PSM_CHECK_ASYNC_RX,
                                 PSM_SEND_SOFT_RESET);
    }
    else
    {
        PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, PD_NOT_SUPPORT_REPLY_MSG,
                                                           PE_PSM_STATE_ROLE_RDY_STATE);
    }
}
#endif

static inline void PD_PsmEnterGetSinkOrSourceCapState(pd_instance_t *pdInstance, uint8_t prevState)
{
    uint8_t msgType            = kPD_MsgGetSinkCap;
    pd_psm_state_t successSate = PE_PSM_STATE_NO_CHANGE;
    if ((pdInstance->psmCurState == PSM_PE_DR_SRC_GET_SOURCE_CAP) ||
        (pdInstance->psmCurState == PSM_PE_SNK_GET_SOURCE_CAP) ||
        (pdInstance->psmCurState == PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP_GET_SOURCE_CAP))
    {
        msgType = kPD_MsgGetSourceCap;
    }
    if (pdInstance->psmCurState == PSM_PE_SNK_GET_SOURCE_CAP)
    {
        successSate = PSM_PE_SNK_READY;
    }

    PD_PsmSendControlTransitionWithSendResponserTimeOut(pdInstance, msgType, successSate, prevState,
                                                        PSM_SEND_SOFT_RESET);
}

#if (defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)) || \
    ((defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE))
static inline void PD_PsmEnterGiveSinkCapState(pd_instance_t *pdInstance)
{
    if ((pdInstance->pdPowerPortConfig->typecRole != kPowerConfig_SourceOnly) &&
        (pdInstance->pdPowerPortConfig->sinkCaps != NULL))
    {
        PD_MsgSendDataTransition(pdInstance, kPD_MsgSinkCapabilities, pdInstance->pdPowerPortConfig->sinkCapCount,
                                 PD_PsmGetSinkPDOs(pdInstance), PE_PSM_STATE_ROLE_RDY_STATE, PSM_CHECK_ASYNC_RX,
                                 PSM_SEND_SOFT_RESET);
    }
    else
    {
        PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, PD_NOT_SUPPORT_REPLY_MSG,
                                                           PE_PSM_STATE_ROLE_RDY_STATE);
    }
}
#endif

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
static inline void PD_PsmEnterSnkStartUpState(pd_instance_t *pdInstance)
{
    /* Delay for 1ms to make sure any previous GoodCRC tranmission has ended. */
    PD_PsmTimerWait(pdInstance, mSec(1));
    PD_MsgReset(pdInstance);
    PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
    pdInstance->cableDiscoverIdentityCounter = 0;
#endif
    pdInstance->psmPresentlyPdConnected = 0;
/* Do not clear previouslyPdConnected here */
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
#endif
    pdInstance->psmNewState = PSM_PE_SNK_DISCOVERY;
}

#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
void PD_AutoSinkNegotiation(pd_instance_t *pdInstance, pd_rdo_t *rdoRequest)
{
    uint8_t snkCapIndex;
    uint8_t srcCapIndex;
    pd_sink_pdo_t sinkPdo;
    pd_source_pdo_t sourcePDO;
    uint32_t requestVoltageMin = 0; /* mV */
    uint32_t requestVoltageMax = 0; /* mV */
    uint32_t requestCurrent    = 0; /* mA */
    uint32_t currentPower      = 0; /* mW */

    rdoRequest->bitFields.objectPosition           = 1;
    rdoRequest->bitFields.giveBack                 = 0;
    rdoRequest->bitFields.capabilityMismatch       = 0;
    rdoRequest->bitFields.usbCommunicationsCapable = 0;
    rdoRequest->bitFields.noUsbSuspend             = 1;
#if ((defined PD_CONFIG_REVISION) && (PD_CONFIG_REVISION >= PD_SPEC_REVISION_30))
    rdoRequest->bitFields.unchunkedSupported = 1;
#endif

    for (snkCapIndex = (((pd_power_port_config_t *)pdInstance->pdConfig->deviceConfig)->sinkCapCount - 1);
         snkCapIndex > 0; --snkCapIndex)
    {
        sinkPdo.PDOValue = ((pd_power_port_config_t *)pdInstance->pdConfig->deviceConfig)->sinkCaps[snkCapIndex];

        switch (sinkPdo.commonPDO.pdoType)
        {
            case kPDO_Fixed:
            {
                requestVoltageMin = requestVoltageMax = sinkPdo.fixedPDO.voltage * PD_PDO_VOLTAGE_UNIT;
                requestCurrent                        = sinkPdo.fixedPDO.operateCurrent * PD_PDO_CURRENT_UNIT;
                break;
            }

            case kPDO_Variable:
            {
                requestVoltageMin = sinkPdo.variablePDO.minVoltage * PD_PDO_VOLTAGE_UNIT;
                requestVoltageMax = sinkPdo.variablePDO.maxVoltage * PD_PDO_VOLTAGE_UNIT;
                requestCurrent    = sinkPdo.variablePDO.operateCurrent * PD_PDO_CURRENT_UNIT;
                break;
            }

            case kPDO_Battery:
            {
                requestVoltageMin = sinkPdo.batteryPDO.minVoltage * PD_PDO_VOLTAGE_UNIT;
                requestVoltageMax = sinkPdo.batteryPDO.maxVoltage * PD_PDO_VOLTAGE_UNIT;
                requestCurrent    = (sinkPdo.batteryPDO.operatePower * PD_PDO_POWER_UNIT) * 1000 / requestVoltageMin;
                break;
            }

            case kPDO_APDO:
            {
                requestVoltageMin = sinkPdo.apdoPDO.minVoltage * PD_APDO_VOLTAGE_UNIT;
                requestVoltageMax = sinkPdo.apdoPDO.maxVoltage * PD_APDO_VOLTAGE_UNIT;
                requestCurrent    = sinkPdo.apdoPDO.maxCurrent * PD_APDO_CURRENT_UNIT;
                break;
            }

            default:
                break;
        }

        for (srcCapIndex = 0; srcCapIndex < pdInstance->partnerSourcePDOsCount; ++srcCapIndex)
        {
            uint32_t power;
            uint32_t current;
            uint8_t misMatch       = 0;
            uint32_t pdoMaxVoltage = 0;
            uint32_t pdoMinVoltage = 0;
            sourcePDO.PDOValue     = pdInstance->partnerSourcePDOs[srcCapIndex].PDOValue;

            switch (sourcePDO.commonPDO.pdoType)
            {
                case kPDO_Fixed:
                {
                    pdoMaxVoltage = pdoMinVoltage = sourcePDO.fixedPDO.voltage * PD_PDO_VOLTAGE_UNIT;
                    if ((pdoMaxVoltage > requestVoltageMax) || (pdoMinVoltage < requestVoltageMin))
                    {
                        continue;
                    }
                    current = requestCurrent;
                    if (current > sourcePDO.fixedPDO.maxCurrent * PD_PDO_CURRENT_UNIT)
                    {
                        current  = sourcePDO.fixedPDO.maxCurrent * PD_PDO_CURRENT_UNIT;
                        misMatch = 1;
                    }
                    power = pdoMinVoltage * current / 1000;
                    if (power > currentPower)
                    {
                        currentPower                               = power;
                        rdoRequest->bitFields.operateValue         = current / PD_PDO_CURRENT_UNIT;
                        rdoRequest->bitFields.capabilityMismatch   = misMatch;
                        rdoRequest->bitFields.objectPosition       = (srcCapIndex + 1);
                        rdoRequest->bitFields.maxOrMinOperateValue = rdoRequest->bitFields.operateValue;
                    }
                    break;
                }

                case kPDO_Variable:
                {
                    pdoMaxVoltage = sourcePDO.variablePDO.maxVoltage * PD_PDO_VOLTAGE_UNIT;
                    pdoMinVoltage = sourcePDO.variablePDO.minVoltage * PD_PDO_VOLTAGE_UNIT;
                    if ((pdoMaxVoltage > requestVoltageMax) || (pdoMinVoltage < requestVoltageMin))
                    {
                        continue;
                    }
                    current = requestCurrent;
                    if (current > sourcePDO.variablePDO.maxCurrent * PD_PDO_CURRENT_UNIT)
                    {
                        current  = sourcePDO.variablePDO.maxCurrent * PD_PDO_CURRENT_UNIT;
                        misMatch = 1;
                    }
                    power = pdoMinVoltage * current / 1000;
                    if (power > currentPower)
                    {
                        currentPower                               = power;
                        rdoRequest->bitFields.operateValue         = current / PD_PDO_CURRENT_UNIT;
                        rdoRequest->bitFields.capabilityMismatch   = misMatch;
                        rdoRequest->bitFields.objectPosition       = (srcCapIndex + 1);
                        rdoRequest->bitFields.maxOrMinOperateValue = rdoRequest->bitFields.operateValue;
                    }
                    break;
                }

                case kPDO_Battery:
                {
                    pdoMaxVoltage = sourcePDO.batteryPDO.maxVoltage * PD_PDO_VOLTAGE_UNIT;
                    pdoMinVoltage = sourcePDO.batteryPDO.minVoltage * PD_PDO_VOLTAGE_UNIT;
                    if ((pdoMaxVoltage > requestVoltageMax) || (pdoMinVoltage < requestVoltageMin))
                    {
                        continue;
                    }
                    power = requestVoltageMax * requestCurrent / 1000;
                    if (power > (sourcePDO.batteryPDO.maxAllowPower * PD_PDO_POWER_UNIT))
                    {
                        power    = sourcePDO.batteryPDO.maxAllowPower * PD_PDO_POWER_UNIT;
                        misMatch = 1;
                    }
                    if (power > currentPower)
                    {
                        currentPower                               = power;
                        rdoRequest->bitFields.operateValue         = power / PD_PDO_POWER_UNIT;
                        rdoRequest->bitFields.capabilityMismatch   = misMatch;
                        rdoRequest->bitFields.objectPosition       = (srcCapIndex + 1);
                        rdoRequest->bitFields.maxOrMinOperateValue = rdoRequest->bitFields.operateValue;
                    }
                    break;
                }

                case kPDO_APDO:
                {
                    uint32_t voltage;
                    pdoMaxVoltage = sourcePDO.apdoPDO.maxVoltage * PD_APDO_VOLTAGE_UNIT;
                    pdoMinVoltage = sourcePDO.apdoPDO.minVoltage * PD_APDO_VOLTAGE_UNIT;
                    if (((requestVoltageMin < pdoMinVoltage) && (requestVoltageMax < pdoMinVoltage)) ||
                        ((requestVoltageMin > pdoMaxVoltage) && (requestVoltageMax > pdoMaxVoltage)))
                    {
                        continue;
                    }

                    voltage = requestVoltageMax;
                    if (voltage > pdoMaxVoltage)
                    {
                        voltage = pdoMaxVoltage;
                    }
                    current = requestCurrent;
                    if (current > sourcePDO.apdoPDO.maxCurrent * PD_APDO_CURRENT_UNIT)
                    {
                        current  = sourcePDO.apdoPDO.maxCurrent * PD_APDO_CURRENT_UNIT;
                        misMatch = 1;
                    }
                    power = voltage * current / 1000;
                    if (power > currentPower)
                    {
                        currentPower                               = power;
                        rdoRequest->bitFields.operateValue         = current / PD_PRDO_CURRENT_UNIT;
                        rdoRequest->bitFields.capabilityMismatch   = misMatch;
                        rdoRequest->bitFields.objectPosition       = (srcCapIndex + 1);
                        rdoRequest->bitFields.maxOrMinOperateValue = rdoRequest->bitFields.operateValue;
                    }
                    break;
                }

                default:
                    break;
            }
        }
    }

    return;
}
#endif

static inline void PD_PsmEnterSnkEvaluateCapState(pd_instance_t *pdInstance)
{
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
#endif
    pdInstance->psmHardResetCount = 0;
    PD_TimerClear(pdInstance, tNoResponseTimer);

#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    if (PD_POLICY_GET_AUTO_SINK_NEGOTIATION_SUPPORT(pdInstance))
    {
        PD_AutoSinkNegotiation(pdInstance, &pdInstance->rdoRequest);
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
        PRINTF("sink auto do the request negotiation\r\n");
#endif
    }
    else
#endif
    {
        PD_DpmAppCallback(pdInstance, PD_DPM_SNK_GET_RDO, &pdInstance->rdoRequest, 0);
    }

    if (pdInstance->revision < PD_SPEC_REVISION_30)
    {
        pdInstance->rdoRequest.bitFields.unchunkedSupported = 0;
    }
    if (pdInstance->commandProcessing == 0)
    {
        PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_REQUEST, 0);
    }
    PD_PsmCheckChunkedFeature(pdInstance, pdInstance->rdoRequest);
    pdInstance->psmNewState = PSM_PE_SNK_SELECT_CAPABILITY;
}

static inline void PD_PsmEnterSnkReadyState(pd_instance_t *pdInstance)
{
    PD_PsmResetStateWhenEnterReadyState(pdInstance);
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
    if (!pdInstance->frsEnabled)
    {
        PD_FRSControl(pdInstance, 1);
        pdInstance->frsEnabled = 1;
    }
    /* Stop ignoring droops on VBus */
    /* kVbusPower_ChangeInProgress is done */
    if (pdInstance->inProgress != kVbusPower_InFRSwap)
#endif
    {
        PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
    }

#if (defined PD_CONFIG_PD3_PPS_ENABLE) && (PD_CONFIG_PD3_PPS_ENABLE)
    /* SinkPPSPeriodicTimer */
    if (PD_PsmSinkIsPPSRDO(pdInstance))
    {
        PD_TimerStart(pdInstance, tSinkPPSPeriodicTimer, T_PPS_REQUEST);
    }
#endif
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
#endif
}

static inline void PD_PsmEnterSnkTransitionToDefaultState(pd_instance_t *pdInstance)
{
    /* Message reception should not be re-enabled until PE_SNK_Startup */
    PD_MsgInit(pdInstance);
    pdInstance->psmExplicitContractExisted = 0u;
    pdInstance->psmPresentlyPdConnected    = 0;
    pdInstance->curDataRole                = kPD_DataRoleUFP;
    pdInstance->psmPresentlyVconnSource    = kPD_NotVconnSource;
    PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
    /* Request DPM to turn off vconn */
    PD_DpmSetVconn(pdInstance, 0);
#endif

    PD_TimerStart(pdInstance, tNoResponseTimer, T_NO_RESPONSE);
    if (!(PD_PsmCheckVsafe0V(pdInstance)))
    {
        pdInstance->psmHardResetNeedsVSafe0V = 1;
        PD_TimerStart(pdInstance, tPSHardResetTimer, T_PS_HARD_RESET + T_SAFE0V_MAX);
    }
    pdInstance->psmNewState = PSM_PE_SNK_STARTUP;
}

#endif

static uint8_t PD_PsmEnterState(pd_instance_t *pdInstance)
{
    pd_psm_state_t prevState;
    uint8_t didNothingC                  = 1;
    pd_state_machine_state_t returnState = kSM_None;

    while (pdInstance->psmNewState != pdInstance->psmCurState)
    {
        if (pdInstance->psmNewState == PE_PSM_STATE_ROLE_RDY_STATE)
        {
            if (pdInstance->curPowerRole == kPD_PowerRoleSource)
            {
                pdInstance->psmNewState = PSM_PE_SRC_READY;
            }
            else
            {
                pdInstance->psmNewState = PSM_PE_SNK_READY;
            }
        }

        didNothingC             = 0;
        prevState               = pdInstance->psmCurState;
        pdInstance->psmCurState = pdInstance->psmNewState;

        switch (pdInstance->psmCurState)
        {
            case PSM_EXIT_TO_ERROR_RECOVERY:            /* (C) */
            case PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY: /* (C) */
                if (pdInstance->psmCurState == PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY)
                {
                    /* Delay for 1ms to make sure any previous GoodCRC transmission has ended. */
                    PD_PsmTimerWait(pdInstance, mSec(1));
                }
                PD_PsmDisconnect(pdInstance);
                returnState = kSM_ErrorRecovery;
                break;

            case PSM_UNKNOWN: /* (C) */
            case PSM_IDLE:    /* (C) */
                pdInstance->psmNewState = pdInstance->psmCurState = PSM_IDLE;
                PD_PsmEnterIdleState(pdInstance);
                break;

            case PSM_INTERRUPTED_REQUEST: /* (C) */
                break;

            case PSM_HARD_RESET: /* (C) */
                PD_PsmEnterHardResetState(pdInstance);
                break;

            case PSM_SEND_SOFT_RESET: /* (C) */
                PD_PsmEnterSendSoftResetState(pdInstance);
                break;

            case PSM_SOFT_RESET: /* (C) */
#if ((defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT))
                PD_MsgChunkingLayerResetFlags(pdInstance);
#endif
                /* Alert the DPM so it can reset it's state */
                if (PD_PsmSendControlTransitionWithHardReset(
                        pdInstance, kPD_MsgAccept,
                        (pd_psm_state_t)((pdInstance->curPowerRole == kPD_PowerRoleSource) ?
                                             PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY :
                                             PSM_PE_SNK_WAIT_FOR_CAPABILITIES)))
                {
                    PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_REQUEST, NULL, 0);
                }
                break;

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
            case PSM_PE_SRC_STARTUP: /* (C) */
                PD_PsmEnterSrcStartUpState(pdInstance);
                break;

            case PSM_PE_SRC_DISCOVERY: /* (C) */
#if (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)) && \
    (defined(PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG))
                /* cable plug didn't response data object on PE_SRC_STARTUP state */
                if ((pdInstance->psmSecondaryState[1] == PSM_IDLE) &&
                    PD_CheckWhetherInitiateCableDiscoveryIdentityOrNot(pdInstance))
                {
                    pdInstance->pendingSOP              = kPD_MsgSOPMask | kPD_MsgSOPpMask | kPD_MsgSOPppMask;
                    pdInstance->psmNewSecondaryState[1] = PSM_PE_SRC_VDM_IDENTITY_REQUEST;
                }
#endif
                pdInstance->psmPresentlyPdConnected = 0;
                /* Do not clear previouslyPdConnected here */
                PD_TimerStart(pdInstance, tSourceCapabilityTimer, T_SEND_SOURCE_CAP);
                break;

            case PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY: /* (C) */
                PD_PsmEnterImplicitCableDiscoveryState(pdInstance);
                break;

            case PSM_PE_SRC_SEND_CAPABILITIES: /* (C) */
                PD_PsmEnterSrcSendCapsState(pdInstance);
                break;

            case PSM_PE_SRC_NEGOTIATE_CAPABILITY: /* (C) */
                PD_PsmEnterSrcNegotiateCapState(pdInstance);
                break;

            case PSM_PE_SRC_TRANSITION_SUPPLY: /* (C) */
                PD_PsmEnterSrcTransitionSupplyState(pdInstance);
                break;

            case PSM_PE_SRC_READY: /* (C) */
                PD_PsmResetStateWhenEnterReadyState(pdInstance);
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                if (!pdInstance->frsEnabled)
                {
                    PD_FRSControl(pdInstance, 1);
                    pdInstance->frsEnabled = 1;
                }
#endif
#if (defined PD_CONFIG_PD3_PPS_ENABLE) && (PD_CONFIG_PD3_PPS_ENABLE)
                /* SourcePPSCommTimer */
                if (PD_PsmSourceIsPPSRDO(pdInstance))
                {
                    PD_TimerStart(pdInstance, tSourcePPSCommTimer, T_PPS_TIMEOUT);
                }
#endif
                break;

            case PSM_PE_SRC_WAIT_NEW_CAPABILITIES: /* (C) */
                break;

            case PSM_PE_SRC_DISABLED: /* (C) */
                PD_PhyControl(pdInstance, PD_PHY_DISABLE_MSG_RX, NULL);
                PD_DpmAppCallback(pdInstance, PD_FUNCTION_DISABLED, NULL, 0);
                break;

            case PSM_PE_SRC_CAPABILITY_RESPONSE: /* (C) */
                PD_PsmEnterSrcCapResponseState(pdInstance);
                break;

            case PSM_PE_SRC_HARD_RESET_RECEIVED: /* (C) */
                break;

            case PSM_PE_SRC_TRANSITION_TO_DEFAULT: /* (C) */
                PD_PsmEnterSrcTransitionToDefaultState(pdInstance);
                break;

            case PSM_PE_SRC_GIVE_SOURCE_CAP: /* (C) */
#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PSM_PE_DR_SNK_GIVE_SOURCE_CAP: /* (C) */
#endif
                PD_PsmEnterGiveSrcCapState(pdInstance);
                break;
#endif

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
            case PSM_PE_SNK_STARTUP: /* (C) */
                PD_PsmEnterSnkStartUpState(pdInstance);
                break;

            case PSM_PE_SNK_DISCOVERY: /* (C) */
                break;

            case PSM_PE_SNK_WAIT_FOR_CAPABILITIES: /* (C) */
                PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
                pdInstance->pendingSOP = kPD_MsgSOPMask;
                PD_MsgStartReceive(pdInstance);
                PD_TimerStart(pdInstance, tSinkWaitCapTimer, T_SINK_WAIT_CAP);
                break;

            case PSM_PE_SNK_EVALUATE_CAPABILITY: /* (C) */
                PD_PsmEnterSnkEvaluateCapState(pdInstance);
                break;

            case PSM_PE_SNK_SELECT_CAPABILITY: /* (C) */
                if (PD_MsgSendDataTransition(
                        pdInstance, kPD_MsgRequest, 1, (uint32_t *)(&pdInstance->rdoRequest), PE_PSM_STATE_NO_CHANGE,
                        (prevState == PSM_PE_SNK_READY ? prevState : PSM_CHECK_ASYNC_RX), PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT: /* (C) */
                break;

            case PSM_PE_SNK_TRANSITION_SINK: /* (C) */
                PD_TimerStart(pdInstance, tPSTransitionTimer, T_PS_TRANSITION);
                if (pdInstance->psmGotoMinRx)
                {
                    pdInstance->callbackFns->PD_SnkGotoMinReducePower(pdInstance->callbackParam);
                }
                else
                {
                    /* Start ignoring droops on VBus */
                    PD_ConnectSetPowerProgress(pdInstance, kVbusPower_ChangeInProgress);
                }
                break;

            case PSM_PE_SNK_READY: /* (C) */
                PD_PsmEnterSnkReadyState(pdInstance);
                break;

            case PSM_PE_SNK_TRANSITION_TO_DEFAULT: /* (C) */
                PD_PsmEnterSnkTransitionToDefaultState(pdInstance);
                break;
#endif

#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
            case PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP_GET_SOURCE_CAP: /* (C) */
#endif
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
            case PSM_PE_SRC_GET_SINK_CAP: /* (C) */
#endif
#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PSM_PE_DR_SNK_GET_SINK_CAP:   /* (C) */
            case PSM_PE_DR_SRC_GET_SOURCE_CAP: /* (C) */
#endif
#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
            case PSM_PE_SNK_GET_SOURCE_CAP: /* (C) */
#endif
                PD_PsmEnterGetSinkOrSourceCapState(pdInstance, prevState);
                break;

#if (defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)) || \
    ((defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE))
            case PSM_PE_DR_SRC_GIVE_SINK_CAP: /* (C) */
            case PSM_PE_SNK_GIVE_SINK_CAP:    /* (C) */
                PD_PsmEnterGiveSinkCapState(pdInstance);
                break;
#endif

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP: /* (C) */
            {
                uint8_t requestEvent = PD_DPM_PR_SWAP_REQUEST;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
                if (PD_POLICY_SUPPORT(pdInstance) && (PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SOURCE_SUPPORT(pdInstance)))
                {
                    if (pdInstance->swapToSnkSrcCapReceived)
                    {
                        pd_source_pdo_t pdo;
                        pdInstance->swapToSnkSrcCapReceived = 0;
                        pdo.PDOValue                        = pdInstance->pdPowerPortConfig->sourceCaps[0];
                        if ((pdo.fixedPDO.externalPowered) &&
                            (!(pdInstance->partnerSourcePDOs[0].fixedPDO.externalPowered)))
                        {
                            requestEvent                      = PD_DPM_EVENT_INVALID;
                            pdInstance->commandEvaluateResult = kCommandResult_Reject;
                        }
                    }

                    if (requestEvent != PD_DPM_EVENT_INVALID)
                    {
                        pdInstance->commandEvaluateResult =
                            (uint8_t)PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SOURCE(pdInstance);
                        requestEvent = PD_DPM_EVENT_INVALID;
                    }
                }
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
                if (requestEvent == PD_DPM_EVENT_INVALID)
                {
                    PD_PsmPrintAutoPolicyReplySwapRequestLog(pdInstance, kPD_MsgPrSwap);
                }
#endif
#endif
                PD_PsmEvaluateSwap(pdInstance, requestEvent, PSM_PE_PRS_SRC_SNK_ACCEPT_PR_SWAP,
                                   PSM_PE_PRS_SRC_SNK_REJECT_PR_SWAP);
                break;
            }

            case PSM_PE_PRS_SNK_SRC_EVALUATE_PR_SWAP: /* (C) */
            {
                uint8_t requestEvent = PD_DPM_PR_SWAP_REQUEST;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
                if (PD_POLICY_SUPPORT(pdInstance) && (PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SINK_SUPPORT(pdInstance)))
                {
                    requestEvent = PD_DPM_EVENT_INVALID;
                    if (((uint8_t)PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SINK(pdInstance) == kAutoRequestProcess_Accept) &&
                        ((PD_PsmGetExternalPowerState(pdInstance)) ||
                         (!pdInstance->selfOrPartnerFirstSourcePDO.fixedPDO.externalPowered)))
                    {
                        pdInstance->commandEvaluateResult = kCommandResult_Accept;
                    }
                    else
                    {
                        pdInstance->commandEvaluateResult = kCommandResult_Reject;
                    }
                }
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
                if (requestEvent == PD_DPM_EVENT_INVALID)
                {
                    PD_PsmPrintAutoPolicyReplySwapRequestLog(pdInstance, kPD_MsgPrSwap);
                }
#endif
#endif
                PD_PsmEvaluateSwap(pdInstance, requestEvent, PSM_PE_PRS_SNK_SRC_ACCEPT_PR_SWAP,
                                   PSM_PE_PRS_SNK_SRC_REJECT_PR_SWAP);
                break;
            }

#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PE_FRS_SRC_SNK_Evaluate_Swap: /* C */
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
#endif
                PD_PsmEvaluateSwap(pdInstance, PD_DPM_FR_SWAP_REQUEST, PE_FRS_SRC_SNK_Accept_Swap, PSM_HARD_RESET);
                break;
#endif

            case PSM_PE_PRS_SRC_SNK_ACCEPT_PR_SWAP: /* (C) */
                PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, kPD_MsgAccept,
                                                                   PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF);
                break;
            case PSM_PE_PRS_SNK_SRC_ACCEPT_PR_SWAP: /* (C) */
                PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, kPD_MsgAccept,
                                                                   PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF);
                break;
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PE_FRS_SRC_SNK_Accept_Swap: /* C */
                PD_PsmSendControlTransitionWithTry(pdInstance, kPD_MsgAccept, PE_FRS_SRC_SNK_Transition_to_off,
                                                   PSM_HARD_RESET, PSM_HARD_RESET);
                break;
#endif

            case PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF: /* (C) */
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                if (pdInstance->frsEnabled)
                {
                    PD_FRSControl(pdInstance, 0);
                    pdInstance->frsEnabled = 0;
                }
#endif
                PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InPRSwap);

                /* pr swap transition to standby */
                /* 1. tSrcTransition */
                PD_PsmTimerWait(pdInstance, T_SRC_TRANSITION);
                /* 2. start enter to standby tSrcSwapStdby */
                pdInstance->callbackFns->PD_SrcTurnOffVbus(pdInstance->callbackParam, kVbusPower_InPRSwap);
                PD_DpmDischargeVbus(pdInstance, 1);
                break;
            case PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF: /* (C) */
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                if (pdInstance->frsEnabled)
                {
                    PD_FRSControl(pdInstance, 0);
                    pdInstance->frsEnabled = 0;
                }
#endif
                pdInstance->psmExplicitContractExisted = 0;
                PD_TimerStart(pdInstance, tPSSourceOffTimer, T_PS_SOURCE_OFF);
                PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InPRSwap);
                /* sink transition to standby. */
                pdInstance->callbackFns->PD_SnkStopDrawVbus(pdInstance->callbackParam, kVbusPower_InPRSwap);
                break;
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PE_FRS_SNK_SRC_Transition_to_off: /* C */
                /* uint8_t enable = 0; */
                /* PD_PhyControl(pdInstance, PD_PHY_CONTROL_FR_SWAP, &enable); */
                PD_TimerStart(pdInstance, tPSSourceOffTimer, T_PS_SOURCE_OFF);
                break;
            case PE_FRS_SRC_SNK_Transition_to_off: /* C */
                PD_FRSControl(pdInstance, 0);
                pdInstance->frsEnabled = 0;
                PD_TimerStart(pdInstance, timrFRSwapWaitPowerStable, T_FRSWAP_WAIT_POWER_STABLE);
                break;
#endif

            case PSM_PE_PRS_SRC_SNK_ASSERT_RD: /* (C) */
                PD_PsmPowerSwapAssertRdRp(pdInstance, kPD_PowerRoleSink, PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON);
                break;
            case PSM_PE_PRS_SNK_SRC_ASSERT_RP: /* (C) */
                PD_PsmPowerSwapAssertRdRp(pdInstance, kPD_PowerRoleSource, PSM_PE_PRS_SNK_SRC_SOURCE_ON);
                break;
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PE_FRS_SRC_SNK_Assert_Rd: /* C */
                PD_PsmPowerSwapAssertRdRp(pdInstance, kPD_PowerRoleSink, PE_FRS_SRC_SNK_Wait_Source_on);
                break;
            case PE_FRS_SNK_SRC_Assert_Rp: /* C */
                PD_PsmPowerSwapAssertRdRp(pdInstance, kPD_PowerRoleSource, PE_FRS_SNK_SRC_Source_on);
                break;
#endif

            case PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON: /* (C) */
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PE_FRS_SRC_SNK_Wait_Source_on: /* C */
#endif
                pdInstance->curPowerRole = kPD_PowerRoleSink;
                /* 0 Role stays as standby until we receive the PS_RDY message */
                PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                if (PE_FRS_SRC_SNK_Wait_Source_on == pdInstance->psmCurState)
                {
                    if (PD_PsmSendControlTransitionWithTry(pdInstance, kPD_MsgPsRdy, PE_FRS_SRC_SNK_Wait_Source_on,
                                                           PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY,
                                                           PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY))
                    {
                        PD_TimerStart(pdInstance, tPSSourceOnTimer, T_PS_SOURCE_ON);
                    }
                }
                else
#endif
                {
                    if (PD_PsmSendControlTransitionWithErrorRecovery(pdInstance, kPD_MsgPsRdy, PE_PSM_STATE_NO_CHANGE))
                    {
                        PD_TimerStart(pdInstance, tPSSourceOnTimer, T_PS_SOURCE_ON);
                    }
                }
                break;
            case PSM_PE_PRS_SNK_SRC_SOURCE_ON: /* (C) */
                pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam, kVbusPower_InPRSwap);
                break;
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PE_FRS_SNK_SRC_Source_on: /* C */
                PD_PsmPowerSwapSinkOpenVbus(pdInstance, PD_DPM_FR_SWAP_SUCCESS);
                break;
#endif

            case PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP: /* (C) */
            case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP: /* (C) */
                PD_PsmSendControlTransitionWithSendResponserTimeOut(pdInstance, kPD_MsgPrSwap, PE_PSM_STATE_NO_CHANGE,
                                                                    prevState, PSM_SEND_SOFT_RESET);
                break;
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PE_FRS_SNK_SRC_Send_Swap: /* C */
                pdInstance->fr5VOpened = 0;
                PD_FRSControl(pdInstance, 0);
                pdInstance->frsEnabled = 0;
                PD_PsmCheckFRS5V(pdInstance);
                if (PD_PsmSendControlTransitionWithSendResponserTimeOut(
                        pdInstance, kPD_MsgFrSwap, PE_PSM_STATE_NO_CHANGE, PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY,
                        PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY))
                {
                    PD_PsmStartCommand(pdInstance, PD_DPM_FAST_ROLE_SWAP, 0);
                }
                else
                {
                    pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY;
                }
                break;
#endif

            case PSM_PE_PRS_SRC_SNK_REJECT_PR_SWAP: /* (C) */
            case PSM_PE_PRS_SNK_SRC_REJECT_PR_SWAP: /* (C) */
            {
                PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(
                    pdInstance,
                    (uint8_t)((pdInstance->commandEvaluateResult == kCommandResult_Reject) ? kPD_MsgReject :
                                                                                             kPD_MsgWait),
                    PE_PSM_STATE_ROLE_RDY_STATE);
                uint8_t callbackResult = kCommandResult_Reject;
                PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_FAIL, &callbackResult, 1);
                break;
            }
#endif

#if defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
            case PSM_PE_DRS_EVALUATE_DR_SWAP: /* (C) */
            {
                uint8_t requestEvent           = PD_DPM_DR_SWAP_REQUEST;
                pdInstance->psmDrSwapPrevState = prevState;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
                if (PD_POLICY_SUPPORT(pdInstance) && (((pdInstance->curDataRole == kPD_DataRoleUFP) &&
                                                       PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_DFP_SUPPORT(pdInstance)) ||
                                                      ((pdInstance->curDataRole == kPD_DataRoleDFP) &&
                                                       PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_UFP_SUPPORT(pdInstance))))
                {
                    if (pdInstance->curDataRole == kPD_DataRoleUFP)
                    {
                        pdInstance->commandEvaluateResult =
                            (uint8_t)PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_DFP(pdInstance);
                    }
                    else
                    {
                        pdInstance->commandEvaluateResult =
                            (uint8_t)PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_UFP(pdInstance);
                    }
                    requestEvent = PD_DPM_EVENT_INVALID;
                }
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
                if (requestEvent == PD_DPM_EVENT_INVALID)
                {
                    PD_PsmPrintAutoPolicyReplySwapRequestLog(pdInstance, kPD_MsgDrSwap);
                }
#endif
#endif
                PD_PsmEvaluateSwap(pdInstance, requestEvent, PSM_PE_DRS_ACCEPT_DR_SWAP, PSM_PE_DRS_REJECT_DR_SWAP);
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                pdInstance->drSwapResult = pdInstance->commandEvaluateResult;
#endif
                break;
            }

            case PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_TIMER_TIME_OUT: /* (C) */
            case PSM_PE_PRS_EVALUATE_PR_SWAP_WAIT_TIMER_TIME_OUT: /* (C) */
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
            case PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_ALT_MODE_EXIT: /* (C) */
#endif
                break;

            case PSM_PE_DRS_REJECT_DR_SWAP: /* (C) */
            {
                PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(
                    pdInstance,
                    ((pdInstance->commandEvaluateResult == kCommandResult_Reject) ? kPD_MsgReject : kPD_MsgWait),
                    (pd_psm_state_t)pdInstance->psmDrSwapPrevState);
                if (pdInstance->commandEvaluateResult != kCommandResult_Reject)
                {
                    pdInstance->commandEvaluateResult = kCommandResult_Wait;
                }
                PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_FAIL, &pdInstance->commandEvaluateResult, 1);
                break;
            }

            case PSM_PE_DRS_SEND_DR_SWAP:                   /* (C) */
                pdInstance->psmDrSwapPrevState = prevState; /* snk_rdy or src_rdy */
                PD_PsmSendControlTransitionWithSendResponserTimeOut(pdInstance, kPD_MsgDrSwap, PE_PSM_STATE_NO_CHANGE,
                                                                    prevState, PSM_SEND_SOFT_RESET);
                break;

            case PSM_PE_DRS_ACCEPT_DR_SWAP: /* (C) */
            {
                PD_PsmSetPhyMessageHeader(
                    pdInstance, ((pdInstance->curDataRole == kPD_DataRoleUFP) ? kPD_DataRoleDFP : kPD_DataRoleUFP));
                PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, kPD_MsgAccept,
                                                                   PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP);
                if (pdInstance->psmNewState == PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP)
                {
                    pdInstance->curDataRole =
                        (pdInstance->curDataRole == kPD_DataRoleUFP) ? kPD_DataRoleDFP : kPD_DataRoleUFP;
                    pdInstance->sendingMsgHeader.bitFields.portDataRole = pdInstance->curDataRole;
                }
                break;
            }

            case PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP: /* (C) */
            {
                /* Exit any active alternate mode */

                pdInstance->pendingSOP = kPD_MsgSOPMask;
                if (pdInstance->curDataRole == kPD_DataRoleDFP)
                {
                    PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_SUCCESS, NULL, 1);
#if 0 /* even do the cable discovery identity, but cannot do SOP'/SOP'' communication after negotiation done */
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    pdInstance->pendingSOP = kPD_MsgSOPMask | kPD_MsgSOPpMask | kPD_MsgSOPppMask;
#endif
#endif
                }
                else
                {
                    PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_SUCCESS, NULL, 1);
                }

                /* Raise the interrupt */
                if ((pdInstance->curDataRole == kPD_DataRoleDFP) || (pdInstance->curPowerRole == kPD_PowerRoleSource))
                {
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    /* Cable plug will need a soft reset */
                    pdInstance->psmCablePlugResetNeeded = 1;
#endif
                }
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                pdInstance->drSwapResult = kCommandResult_Accept;
                PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
#endif
                /* return to previous state */
                pdInstance->psmNewState = pdInstance->psmDrSwapPrevState;
                break;
            }
#endif

#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
            case PSM_PE_VCS_SEND_SWAP: /* (C) */
                pdInstance->psmVconnSwapPrevState = prevState;
                PD_PsmSendControlTransitionWithSendResponserTimeOut(
                    pdInstance, kPD_MsgVconnSwap, PE_PSM_STATE_NO_CHANGE, pdInstance->psmVconnSwapPrevState,
                    PSM_SEND_SOFT_RESET);
                break;

            case PSM_PE_VCS_WAIT_FOR_VCONN: /* (C) */
                PD_TimerStart(pdInstance, tVconnOnTimer, T_VCONN_SOURCE_ON);
                break;

            case PSM_PE_VCS_TURN_OFF_VCONN: /* (C) */
                pdInstance->psmPresentlyVconnSource = kPD_NotVconnSource;
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                pdInstance->cableDiscoverIdentityCounter = 0;
#endif
                /* Inform the DPM */
                pdInstance->callbackFns->PD_ControlVconn(pdInstance->callbackParam, 0);
                PD_DpmAppCallback(pdInstance, PD_DPM_VCONN_SWAP_SUCCESS, NULL, 1);
                /* return to previous state */
                pdInstance->psmNewState = pdInstance->psmVconnSwapPrevState;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                pdInstance->vconnSwapResult = kCommandResult_Accept;
                PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
#endif
                break;

            case PSM_PE_VCS_TURN_ON_VCONN: /* (C) */
                pdInstance->psmPresentlyVconnSource = kPD_IsVconnSource;
                /* Inform the DPM */
                pdInstance->callbackFns->PD_ControlVconn(pdInstance->callbackParam, 1);
                PD_TimerStart(pdInstance, tVconnOnTimer, T_VCONN_SOURCE_ON / 2);
                break;

            case PSM_PE_VCS_SEND_PS_RDY: /* (C) */
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgPsRdy, pdInstance->psmVconnSwapPrevState,
                                                PSM_SEND_SOFT_RESET, PSM_SEND_SOFT_RESET))
                {
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    pdInstance->cableDiscoverIdentityCounter = 0;
#endif
                    PD_DpmAppCallback(pdInstance, PD_DPM_VCONN_SWAP_SUCCESS, NULL, 1);
                }
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                pdInstance->vconnSwapResult = kCommandResult_Accept;
                PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
#endif
                break;

            case PSM_PE_VCS_EVALUATE_SWAP: /* (C) */
            {
                uint8_t requestEvent              = PD_DPM_VCONN_SWAP_REQUEST;
                pdInstance->psmVconnSwapPrevState = prevState;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                PD_PsmSetAutoPolicyState(pdInstance, PSM_RDY_EVAL_INIT);
                if (PD_POLICY_SUPPORT(pdInstance) &&
                    (((pdInstance->psmPresentlyVconnSource == kPD_NotVconnSource) &&
                      PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_ON_SUPPORT(pdInstance)) ||
                     ((pdInstance->psmPresentlyVconnSource == kPD_IsVconnSource) &&
                      PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_OFF_SUPPORT(pdInstance))))
                {
                    if (pdInstance->psmPresentlyVconnSource == kPD_NotVconnSource)
                    {
                        pdInstance->commandEvaluateResult =
                            (uint8_t)PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_ON(pdInstance);
                    }
                    else
                    {
                        pdInstance->commandEvaluateResult = PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_OFF(pdInstance);
                    }
                    requestEvent = PD_DPM_EVENT_INVALID;
                }
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY_LOG) && (PD_CONFIG_ENABLE_AUTO_POLICY_LOG)
                if (requestEvent == PD_DPM_EVENT_INVALID)
                {
                    PD_PsmPrintAutoPolicyReplySwapRequestLog(pdInstance, kPD_MsgVconnSwap);
                }
#endif
#endif
                PD_PsmEvaluateSwap(pdInstance, requestEvent, PSM_PE_VCS_ACCEPT_SWAP, PSM_PE_VCS_REJECT_SWAP);
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                pdInstance->vconnSwapResult = pdInstance->commandEvaluateResult;
#endif
                break;
            }

            case PSM_PE_VCS_ACCEPT_SWAP: /* (C) */
                PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(
                    pdInstance, kPD_MsgAccept,
                    pdInstance->psmPresentlyVconnSource == kPD_IsVconnSource ? PSM_PE_VCS_WAIT_FOR_VCONN :
                                                                               PSM_PE_VCS_TURN_ON_VCONN);

                break;

            case PSM_PE_VCS_REJECT_SWAP: /* (C) */
                PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(
                    pdInstance,
                    (pdInstance->commandEvaluateResult == kCommandResult_Reject) ? kPD_MsgReject : kPD_MsgWait,
                    (pd_psm_state_t)pdInstance->psmVconnSwapPrevState);
                if (pdInstance->commandEvaluateResult != kCommandResult_Reject)
                {
                    pdInstance->commandEvaluateResult = kCommandResult_Wait;
                }
                PD_DpmAppCallback(pdInstance, PD_DPM_VCONN_SWAP_FAIL, &pdInstance->commandEvaluateResult, 1);
                break;
#endif

#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case PE_SNK_GET_SOURCE_CAP_EXT:    /* C */
            case PE_DR_SRC_GET_SOURCE_CAP_EXT: /* C */
                PD_PsmSendControlTransitionWithSendResponserTimeOut(
                    pdInstance, kPD_MsgGetSourceCapExtended, PE_PSM_STATE_NO_CHANGE, prevState, PSM_SEND_SOFT_RESET);
                break;
            case PE_SRC_Get_Sink_Status:   /* C */
            case PE_SNK_Get_Source_Status: /* C */
                PD_PsmSendControlTransitionWithSendResponserTimeOut(
                    pdInstance, kPD_MsgGetStatus, PE_PSM_STATE_NO_CHANGE, prevState, PSM_SEND_SOFT_RESET);
                break;
            case PE_Get_Battery_Cap:    /* C */
            case PE_Get_Battery_Status: /* C */
                if (PD_MsgSendExtTransition(pdInstance, kPD_MsgSOP,
                                            (PE_Get_Battery_Cap == pdInstance->psmCurState) ? kPD_MsgGetBatteryCap :
                                                                                              kPD_MsgGetBatteryStatus,
                                            1, &pdInstance->getBatteryCapDataBlock, PE_PSM_STATE_NO_CHANGE, prevState,
                                            PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;
            case PE_Get_Manufacturer_Info: /* C */
                if (PD_MsgSendExtTransition(pdInstance, (start_of_packet_t)pdInstance->commandExtParam.sop,
                                            kPD_MsgGetManufacturerInfo, 2, pdInstance->commandExtParam.dataBuffer,
                                            PE_PSM_STATE_NO_CHANGE, prevState, PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PE_SRC_Send_Source_Alert: /* C */
            case PE_SNK_Send_Sink_Alert:   /* C */
                if (PD_MsgSendDataTransition(pdInstance, kPD_MsgAlert, 1, &pdInstance->alertADO,
                                             PE_PSM_STATE_ROLE_RDY_STATE, prevState, PSM_SEND_SOFT_RESET))
                {
                    pdInstance->alertWaitReply = 1u;
                    PD_DpmAppCallback(pdInstance, PD_DPM_SEND_ALERT_SUCCESS, NULL, 1);
                }
                else
                {
                    uint32_t errorResult = kCommandResult_Error;
                    PD_DpmAppCallback(pdInstance, PD_DPM_SEND_ALERT_FAIL, &errorResult, 1);
                }
                break;

            case PE_SRC_GIVE_SOURCE_CAP_EXT:    /* C */
            case PE_DR_SNK_GIVE_SOURCE_CAP_EXT: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_GIVE_SRC_EXT_CAP, &pdInstance->commandExtParamCallback, 0);
                PD_StateSendReplyExtDataTransition(pdInstance, kPD_MsgSourceCapExtended);
                break;
            case PE_SRC_Give_Source_Status: /* C */
            case PE_SNK_Give_Sink_Status:   /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_GIVE_STATUS, &pdInstance->commandExtParamCallback, 0);
                PD_StateSendReplyExtDataTransition(pdInstance, kPD_MsgStatus);
                break;
            case PE_Give_Battery_Cap: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_GIVE_BATTERY_CAP, &pdInstance->commandExtParamCallback, 0);
                PD_StateSendReplyExtDataTransition(pdInstance, kPD_MsgBatteryCapabilities);
                break;
            case PE_Give_Battery_Status: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_GIVE_BATTERY_STATUS, &pdInstance->commandExtParamCallback, 0);
                PD_StateSendReplyExtDataTransition(pdInstance, kPD_MsgBatteryStatus);
                break;
            case PE_Give_Manufacturer_Info: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_GIVE_MANUFACTURER_INFO, &pdInstance->commandExtParamCallback, 0);
                PD_StateSendReplyExtDataTransition(pdInstance, kPD_MsgManufacturerInfo);
                break;

            case PE_SNK_Source_Alert_Received: /* C */
            case PE_SRC_Sink_Alert_Received:   /* C */
                pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;
                break;

#if 0
            case PE_Send_Security_Request: /* C */
            {
                pd_status_t sendStatus = kStatus_PD_Error;

                /* even this msg is chunked, it only needs one chunk. */
                if (!pdInstance->unchunkedFeature)
                {
                    sendStatus = PD_PsmChunkingLayerTXStateMachine(pdInstance,
                                                                   kPD_MsgSOP, kPD_MsgSecurityRequest,
                                                                   pdInstance->commandExtParam.dataBuffer,
                                                                   pdInstance->commandExtParam.dataLength, NULL);
                    if (sendStatus != kStatus_PD_Success)
                    {
                        uint8_t inCase32Tmp = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_SECURITY_REQUEST_FAIL, &inCase32Tmp, 1);
                        PD_PsmTransitionOnMsgSendError(pdInstance, prevState, PSM_SEND_SOFT_RESET);
                    }
                    else
                    {
                        pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;
                    }
                }
                else
                {
                    sendStatus = PD_MsgSendUnchunkedExtendedMsg(
                        pdInstance, (start_of_packet_t)pdInstance->commandExtParam.sop, kPD_MsgSecurityRequest,
                        pdInstance->commandExtParam.dataLength, pdInstance->commandExtParam.dataBuffer);
                    if ((sendStatus == kStatus_PD_Success) && PD_MsgWaitSendResult(pdInstance))
                    {
                        pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;
                    }
                    else
                    {
                        uint8_t inCase32Tmp = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_SECURITY_REQUEST_FAIL, &inCase32Tmp, 1);
                        PD_PsmTransitionOnMsgSendError(pdInstance, prevState, PSM_SEND_SOFT_RESET);
                    }
                }
                break;
            }

            case PE_Send_Security_Response: /* C */
            {
                pd_status_t sendStatus = kStatus_PD_Error;

                /* first time */
                PD_DpmAppCallback(pdInstance, PD_DPM_RESPONSE_SECURITY_REQUEST, &pdInstance->commandExtParamCallback,
                                  0);
                if ((pdInstance->commandExtParamCallback.resultStatus != kCommandResult_NotSupported) &&
                    (!pdInstance->unchunkedFeature))
                {
                    sendStatus = PD_PsmChunkingLayerTXStateMachine(pdInstance,
                                                                   kPD_MsgSOP, kPD_MsgSecurityResponse,
                                                                   pdInstance->commandExtParamCallback.dataBuffer,
                                                                   pdInstance->commandExtParamCallback.dataLength, NULL);
                    if (sendStatus != kStatus_PD_Success)
                    {
                        PD_PsmTransitionOnMsgSendError(pdInstance, PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                    }
                    else
                    {
                        if (PD_PsmChunkingLayerCheckSentDone(pdInstance))
                        {
                            pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;
                        }
                        else
                        {
                            /* wait chunk message sent done */
                        }
                    }
                }
                else
                {
                    if ((pdInstance->commandExtParamCallback.resultStatus != kCommandResult_NotSupported) &&
                        (pdInstance->unchunkedFeature))
                    {
                        sendStatus = PD_MsgSendUnchunkedExtendedMsg(
                            pdInstance, (start_of_packet_t)pdInstance->commandExtParam.sop, kPD_MsgSecurityResponse,
                            pdInstance->commandExtParamCallback.dataLength,
                            pdInstance->commandExtParamCallback.dataBuffer);
                    }
                    else if (pdInstance->commandExtParamCallback.resultStatus == kCommandResult_NotSupported)
                    {
                        PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, kPD_MsgNotSupported,
                                                           PE_PSM_STATE_ROLE_RDY_STATE);
                    }
                    else
                    {
                    }

                    if ((sendStatus == kStatus_PD_Success) && PD_MsgWaitSendResult(pdInstance))
                    {
                        pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;
                    }
                    else
                    {
                        PD_PsmTransitionOnMsgSendError(pdInstance, PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                    }
                }
                break;
            }

            case PE_Security_Response_Received: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_SECURITY_RESPONSE_RECEIVED, &pdInstance->commandExtParamCallback, 0);
                pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;
                break;
#endif
#endif

#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PE_FRS_SRC_SNK_CC_Signal: /* C */
            {
                PD_MsgReceive(pdInstance);
                PD_TimerStart(pdInstance, tFRSwapSignalTimer, 500);
                break;
            }

            case PE_FRS_SNK_SRC_Vbus_Applied: /* C */
                break;
#endif

            case PSM_PE_BIST_TEST_DATA_MODE: /* (C) */
                break;

            case PSM_PE_BIST_CARRIER_MODE_2: /* (C) */
            {
#if (defined PD_CONFIG_COMPLIANCE_TEST_ENABLE) && (PD_CONFIG_COMPLIANCE_TEST_ENABLE)
                PD_TimerStart(pdInstance, tBISTContModeTimer, T_BIST_CONT_MODE);
                /* Delay for 1ms to make sure any previous GoodCRC tranmission has ended. */
                PD_PsmTimerWait(pdInstance, mSec(1));
                uint8_t bistMode = kBIST_CarrierMode2;
                PD_PhyControl(pdInstance, PD_PHY_ENTER_BIST, &bistMode);
#endif
                break;
            }

            case PSM_CHECK_ASYNC_RX: /* (C) */
                break;

            case PSM_BYPASS: /* (C) */
                break;

            default:
                break;
        }

        if (returnState != kSM_None)
        {
            return returnState;
        }
        PD_PsmEndCommand(pdInstance);
        PD_PsmSetNormalPower(pdInstance);
    }

    PD_PsmProcessImportEventBeforeNextStateMachine(pdInstance);
    if (didNothingC == 1)
    {
        return kSM_WaitEvent;
    }
    else
    {
        return kSM_Continue;
    }
}

static uint8_t PD_PsmGetNewestEvent(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    uint32_t taskEventSet;
    uint8_t processMessage;

    if (USB_OsaEventCheck(pdInstance->taskEventHandle, 0xffu, &taskEventSet) == kStatus_USB_OSA_Success)
    {
        if (taskEventSet & PD_TASK_EVENT_TIME_OUT)
        {
            USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_TIME_OUT);
#if ((defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT))
            if (PD_PsmChunkingLayerRXTimerWaiting(pdInstance))
            {
                PD_PsmChunkingLayerRXStateMachine(pdInstance, NULL);
            }
#endif
        }
        else if (taskEventSet & PD_TASK_EVENT_RECEIVED_HARD_RESET)
        {
            USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_PD_MSG);
            USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_RECEIVED_HARD_RESET);
            if (pdInstance->psmCurState != PSM_PE_BIST_CARRIER_MODE_2)
            {
                /* Hard reset is ignored when in BIST carrier mode */
                pdInstance->hardResetReceived = 0;
                triggerInfo->triggerEvent     = PSM_TRIGGER_RECEIVE_HARD_RESET;
            }
        }
        else if (taskEventSet & PD_TASK_EVENT_FR_SWAP_SINGAL)
        {
            USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_FR_SWAP_SINGAL);
            if (pdInstance->curPowerRole == kPD_PowerRoleSink)
            {
                pdInstance->psmNewState = PE_FRS_SNK_SRC_Send_Swap;
            }
        }
        else if (taskEventSet & PD_TASK_EVENT_PD_MSG)
        {
            USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_PD_MSG);

            if (pdInstance->psmCurState != PSM_PE_BIST_TEST_DATA_MODE)
            {
                processMessage = 0;
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                if ((pdInstance->receivedSop == kPD_MsgSOPp) || (pdInstance->receivedSop == kPD_MsgSOPpp))
                {
                    PD_MsgGetReceiveResult(pdInstance);
                    PD_MsgReceive(pdInstance);
                    processMessage = 1;
                }
                else
#endif
                    if ((pdInstance->receivedSop == kPD_MsgSOP) && (PD_MsgGetReceiveResult(pdInstance)))
                {
                    processMessage = 1;
                }
                else
                {
                }

                if (processMessage)
                {
                    triggerInfo->msgHeader.msgHeaderVal = (uint16_t)(MSG_DATA_HEADER);
                    triggerInfo->pdMsgSop               = pdInstance->receivedSop;
                    triggerInfo->pdMsgDataBuffer        = (uint8_t *)MSG_DATA_BUFFER;
                    triggerInfo->pdMsgDataLength = ((MSG_DATA_HEADER & PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_MASK) >>
                                                    PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_POS);

                    if (!(triggerInfo->msgHeader.bitFields.extended))
                    {
                        if (triggerInfo->msgHeader.bitFields.NumOfDataObjs == 0)
                        {
                            triggerInfo->pdMsgType = triggerInfo->msgHeader.bitFields.messageType;
                            /* There are three places: two message will be sent continuously by one port.
                             * 1. fast role swap.
                             * 2. sink beginning power role swap.
                             * 3. rdo request in source.
                             * 4. after soft reset, the partner will send source_caps.
                             */
                            if (triggerInfo->pdMsgType == kPD_MsgAccept)
                            {
                                if (pdInstance->psmCurState != PSM_SEND_SOFT_RESET)
                                {
                                    PD_MsgReceive(pdInstance);
                                }
                            }
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                            if (triggerInfo->pdMsgType == kPD_MsgFrSwap)
                            {
                                pdInstance->frSwapReceived = 1;
                            }
#endif
                        }
                        else
                        {
                            triggerInfo->pdMsgType =
                                (triggerInfo->msgHeader.bitFields.messageType | PD_MSG_DATA_TYPE_MASK);
                            if (triggerInfo->pdMsgType == kPD_MsgRequest)
                            {
                                PD_PsmCheckRevision(pdInstance, triggerInfo->msgHeader);
                            }
                        }
                    }
                    else
                    {
                        triggerInfo->pdMsgType = (triggerInfo->msgHeader.bitFields.messageType | PD_MSG_EXT_TYPE_MASK);
                        if (!(triggerInfo->pdMsgDataBuffer[1] & 0x80u))
                        {
                            triggerInfo->pdExtMsgLength =
                                (triggerInfo->pdMsgDataBuffer[0] + (triggerInfo->pdMsgDataBuffer[1] & 0x01u) * 256);
                        }
                        else
                        {
                            triggerInfo->pdExtMsgLength =
                                (triggerInfo->pdMsgDataBuffer[0] + (triggerInfo->pdMsgDataBuffer[1] & 0x01u) * 16);
                        }
                    }
                    triggerInfo->triggerEvent = PSM_TRIGGER_PD_MSG;

                    if (triggerInfo->pdMsgType == kPD_MsgVendorDefined)
                    {
                        triggerInfo->vdmHeader.structuredVdmHeaderVal = (uint32_t)MSG_DATA_BUFFER[0];
                        /* it is interrupted */
                        PD_MsgReceive(pdInstance);
                        if (USB_OsaEventCheck(pdInstance->taskEventHandle, PD_TASK_EVENT_PD_MSG, &taskEventSet) ==
                            kStatus_USB_OSA_Success)
                        {
                            PD_PsmProcessImportEventBeforeNextStateMachine(pdInstance);
                            return kSM_Continue; /* process next message */
                        }
                    }
#if ((defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT))
                    if (!PD_PsmChunkingLayerRXStateMachine(pdInstance, triggerInfo))
                    {
                        return kSM_Continue;
                    }
#endif
                }
                else
                {
                }
            }
            else
            {
                /* Ignore everything except Hard Reset and disconnect */
            }
        }
        else if (taskEventSet & PD_TASK_EVENT_DPM_MSG)
        {
            uint8_t command;
            command             = PD_DpmGetMsg(pdInstance);
            triggerInfo->dpmMsg = 0;
            if (command)
            {
                triggerInfo->dpmMsg = command;
                switch (command)
                {
                    case PD_DPM_CONTROL_SOFT_RESET:
                    case PD_DPM_CONTROL_HARD_RESET:
                    case PD_DPM_FAST_ROLE_SWAP:
                        break;

                    default:
                        if ((pdInstance->psmCurState != PSM_PE_SNK_READY) &&
                            (pdInstance->psmCurState != PSM_PE_SRC_READY))
                        {
                            triggerInfo->dpmMsg = 0;
                        }
                        break;
                }
            }
            else
            {
                USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_DPM_MSG);
            }

            if (triggerInfo->dpmMsg != 0)
            {
                PD_DpmClearMsg(pdInstance, (pd_command_t)triggerInfo->dpmMsg);
                USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_DPM_MSG);
                triggerInfo->triggerEvent = PSM_TRIGGER_DPM_MSG;
            }
        }
        else if (taskEventSet & PD_TASK_EVENT_SEND_DONE)
        {
            USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_SEND_DONE);
        }
        else
        {
            PD_PortTaskEventProcess(pdInstance, taskEventSet);
            triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        }
    }
    return kSM_None;
}

static uint8_t PD_PsmCheckTimeOutHardReset(pd_instance_t *pdInstance)
{
    uint8_t needHardReset = 0;
    if ((pdInstance->psmHardResetCount <= N_HARD_RESET_COUNT) && (pdInstance->curPowerRole == kPD_PowerRoleSink))
    {
        if (PD_TimerCheckValidTimeOut(pdInstance, tSinkWaitCapTimer))
        {
            PD_TimerClear(pdInstance, tSinkWaitCapTimer);
            needHardReset = 1;
        }
        if (PD_TimerCheckValidTimeOut(pdInstance, tPSTransitionTimer))
        {
            PD_TimerClear(pdInstance, tPSTransitionTimer);
            needHardReset = 1;
        }
        if (PD_TimerCheckValidTimeOut(pdInstance, tNoResponseTimer))
        {
            PD_TimerClear(pdInstance, tNoResponseTimer);
            needHardReset = 1;
        }
    }

    return needHardReset;
}

static uint8_t PD_PsmProcessHardResetState(pd_instance_t *pdInstance)
{
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
    if (pdInstance->curPowerRole == kPD_PowerRoleSource)
    {
        /* source send hard_reset, 2. after tpstimer. */
        if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSHardResetTimer))
        {
            /* Hard reset is complete */
            pdInstance->psmNewState = PSM_PE_SRC_TRANSITION_TO_DEFAULT;
        }
        else
        {
            return 1;
        }
    }
    else
#endif
#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
        if (pdInstance->curPowerRole == kPD_PowerRoleSink)
    {
        /* sink send hard_reset */
        pdInstance->psmNewState = (pd_psm_state_t)PD_PsmSinkHardResetfunction(pdInstance);
    }
    else
#endif
    {
    }
    return 0;
}

static uint8_t PD_PsmProcessSendSoftResetState(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgSop == kPD_MsgSOP) &&
        (triggerInfo->pdMsgType == kPD_MsgAccept))
    {
        PD_TimerClear(pdInstance, tSenderResponseTimer);
        PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_SUCCESS, NULL,
                          (pdInstance->commandProcessing == PD_DPM_CONTROL_SOFT_RESET) ? 1 : 0);

        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
        {
            pdInstance->psmNewState = PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY;
        }
        else
        {
            pdInstance->psmNewState = PSM_PE_SNK_WAIT_FOR_CAPABILITIES;
        }
    }
    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
    {
        if (pdInstance->commandProcessing == PD_DPM_CONTROL_SOFT_RESET)
        {
            PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_FAIL, NULL, 1);
        }
        pdInstance->psmNewState = PSM_HARD_RESET;
    }
    else
    {
        return 1;
    }
    return 0;
}

static void PD_PsmCheckUnexpectedReceivedMsg(pd_instance_t *pdInstance, uint8_t triggerEvent, uint8_t *didNothingStepB)
{
    if (triggerEvent == PSM_TRIGGER_PD_MSG)
    {
        switch (pdInstance->psmCurState)
        {
            case PSM_PE_SRC_STARTUP:
            case PSM_PE_SRC_DISCOVERY:
                pdInstance->psmNewState = PSM_HARD_RESET;
                *didNothingStepB        = 0;
                break;
            case PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF:
            case PSM_PE_PRS_SNK_SRC_SOURCE_ON:
                pdInstance->psmNewState = PSM_CHECK_ASYNC_RX;
                *didNothingStepB        = 0;
                break;

            case PSM_PE_SNK_WAIT_FOR_CAPABILITIES:
                pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                *didNothingStepB        = 0;
                break;

            default:
                break;
        }
    }
}

static uint8_t PD_PsmProcessReadyState(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    uint8_t didNothingStepB = 0;
#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE))
    uint8_t didNothing = 1;

    didNothing &= Pd_PsmSecondaryStateHandler(pdInstance, 0, triggerInfo);

#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
    if ((pdInstance->psmSecondaryState[1] == PSM_IDLE) &&
        PD_TimerCheckValidTimeOut(pdInstance, tDiscoverIdentityTimer) &&
        (pdInstance->cableDiscoverIdentityCounter < N_DISCOVER_IDENTITY_COUNTER))
    {
        pdInstance->cableDiscoverIdentityTimeout = 1;
        PD_TimerClear(pdInstance, tDiscoverIdentityTimer);
        pdInstance->psmNewSecondaryState[1] = PSM_PE_SRC_VDM_IDENTITY_REQUEST;
    }

    didNothing &= Pd_PsmSecondaryStateHandler(pdInstance, 1, triggerInfo);
    didNothing &= Pd_PsmSecondaryStateHandler(pdInstance, 2, triggerInfo);

    /* second state is IDLE && don't get success && count is not max && DiscoverIdentityTimer is timeout */
    if ((pdInstance->psmSecondaryState[1] == PSM_IDLE) &&
        PD_CheckWhetherInitiateCableDiscoveryIdentityOrNot(pdInstance) && pdInstance->cableDiscoverIdentityTimeout)
    {
        pdInstance->cableDiscoverIdentityTimeout = 0;
        PD_TimerStart(pdInstance, tDiscoverIdentityTimer, T_DISCOVER_IDENTITY);
    }
    /* cable discover success || beyond maximum count */
    else if (((pdInstance->psmCableIdentitiesDataCount > 0) ||
              (pdInstance->cableDiscoverIdentityCounter >= N_DISCOVER_IDENTITY_COUNTER)) &&
             (pdInstance->pendingSOP != kPD_MsgSOPMask))
    {
        pdInstance->pendingSOP = kPD_MsgSOPMask;
        PD_PhyControl(pdInstance, PD_PHY_CANCEL_MSG_RX, NULL);
        pdInstance->receiveState = 0;
        PD_MsgReceive(pdInstance);
    }
#endif

    if ((pdInstance->psmSecondaryState[0] == PSM_IDLE) && (pdInstance->psmSecondaryState[1] == PSM_IDLE) &&
        (pdInstance->psmSecondaryState[2] == PSM_IDLE) && (didNothing))
    {
        didNothingStepB = 1;
    }
#endif

    if (triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG)
    {
        PD_PsmRdyProcessPdMessage(pdInstance, triggerInfo);
    }
    else if (triggerInfo->triggerEvent == PSM_TRIGGER_DPM_MSG)
    {
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
        if (pdInstance->psmCurState == PSM_PE_SRC_READY)
        {
            PD_PsmSourceRdyProcessDpmMessage(pdInstance, triggerInfo);
        }
#endif
#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
        if (pdInstance->psmCurState == PSM_PE_SNK_READY)
        {
            PD_PsmSinkRdyProcessDpmMessage(pdInstance, triggerInfo);
        }
#endif
    }
    else
    {
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
        didNothingStepB = PD_PsmReadyAutoPolicyProcess(pdInstance);
#else
        didNothingStepB         = 1;
#endif
    }

    return didNothingStepB;
}

#if (defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)) || \
    ((defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE))
static inline uint8_t PD_PsmGetTypeCCurrent(pd_instance_t *pdInstance)
{
    uint8_t typeCurrent;
    PD_PhyControl(pdInstance, PD_PHY_SNK_GET_TYPEC_CURRENT_CAP, &typeCurrent);
    return typeCurrent;
}
#endif

static uint8_t PD_PsmProcessState(pd_instance_t *pdInstance)
{
    psm_trigger_info_t triggerInfo;
    uint8_t didNothingStepA = 1;
    uint8_t didNothingStepB = 1;
    uint8_t msgReceived     = 0;

    if (pdInstance->psmCurState != PSM_UNKNOWN)
    {
        triggerInfo.triggerEvent    = PSM_TRIGGER_NONE;
        triggerInfo.pdMsgType       = kPD_MsgInvalid;
        triggerInfo.dpmMsg          = 0;
        triggerInfo.pdMsgDataBuffer = NULL;

        /* get the newest events */
        if (PD_PsmGetNewestEvent(pdInstance, &triggerInfo) == kSM_Continue)
        {
            return kSM_Continue;
        }
        if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
        {
            msgReceived = 1;
        }

        if (((triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo.pdMsgSop == kPD_MsgSOP)) ||
            (triggerInfo.triggerEvent == PSM_TRIGGER_RECEIVE_HARD_RESET))
        {
            didNothingStepA &=
                PD_PsmPrimaryStateProcessPdMsg(pdInstance, (uint8_t *)&pdInstance->psmNewState, &triggerInfo);
        }
        else if (triggerInfo.triggerEvent == PSM_TRIGGER_DPM_MSG)
        {
            didNothingStepA &=
                PD_PsmPrimaryStateProcessDpmMsg(pdInstance, (uint8_t *)&pdInstance->psmNewState, &triggerInfo);
        }
        else
        {
        }

        if (!didNothingStepA)
        {
            if (msgReceived && (PD_PsmCanPendingReceive(pdInstance)))
            {
                PD_MsgReceive(pdInstance); /* msg has been processed, receiving next msg */
            }
            PD_PsmProcessImportEventBeforeNextStateMachine(pdInstance);
            return kSM_Continue;
        }

        /* different timeout need hard_reset */
        if (PD_PsmCheckTimeOutHardReset(pdInstance))
        {
            didNothingStepA         = 0;
            pdInstance->psmNewState = PSM_HARD_RESET;
        }
        /* alternate mode tAMETimeoutTimer */
        else if (PD_TimerCheckValidTimeOut(pdInstance, tAMETimeoutTimer))
        {
            PD_TimerClear(pdInstance, tAMETimeoutTimer);
            PD_ConnectAltModeEnterFail(pdInstance, pdInstance->psmPresentlyPdConnected);

#ifdef USBPD_ENABLE_USB_BILLBOARD
            UsbbSetAltModeConfigResult(AMR_CONFIG_FAILED);
#endif
        }
        /* Check if we are waiting for an event for state transition */
        else if (pdInstance->psmNewState == pdInstance->psmCurState)
        {
            didNothingStepB = 0;
            switch (pdInstance->psmCurState)
            {
                case PSM_EXIT_TO_ERROR_RECOVERY:            /* (B) */
                case PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY: /* (B) */
                    return kSM_ErrorRecovery;

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
                case PSM_INTERRUPTED_REQUEST: /* (B) */
                    /* Transitioned back here after VDM handling, re attempt the request */
                    pdInstance->psmNewState         = pdInstance->psmInterruptedState;
                    pdInstance->psmInterruptedState = PSM_UNKNOWN;
                    break;
#endif

                /* do hard_reset actively do in (C) */
                case PSM_HARD_RESET: /* (B) */
                    didNothingStepB = PD_PsmProcessHardResetState(pdInstance);
                    break;

                case PSM_SEND_SOFT_RESET: /* (B) */
                    didNothingStepB = PD_PsmProcessSendSoftResetState(pdInstance, &triggerInfo);
                    break;

                case PSM_SOFT_RESET: /* (B) */
                    /* If we get here, then recover with a hard reset */
                    pdInstance->psmNewState = PSM_HARD_RESET;
                    break;

                case PSM_CHECK_ASYNC_RX: /* (B) */
                    /* The async conditions at the top of the loop have been evaluated */
                    /* continue to soft reset */
                    pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                    break;

                case PSM_BYPASS: /* (B) */
                    /* We do nothing here. */
                    didNothingStepB = 1;
                    break;

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
                case PSM_PE_SRC_STARTUP: /* (B) */
                {
                    if (PD_TimerCheckValidTimeOut(pdInstance, tSwapSourceStartTimer))
                    {
                        PD_TimerClear(pdInstance, tSwapSourceStartTimer);
                        pdInstance->psmNewState = PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
                }

                case PSM_PE_SRC_DISCOVERY: /* (B) */
                {
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSourceCapabilityTimer))
                    {
                        if ((pdInstance->psmSendCapsCounter > N_CAPS_COUNT))
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_DISABLED;
                        }
                        else
                        {
#if (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)) && \
    (defined(PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG))
                            pdInstance->pendingSOP = kPD_MsgSOPMask;
#endif
                            pdInstance->psmNewState = PSM_PE_SRC_SEND_CAPABILITIES;
                        }
                    }
#if (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)) && \
    (defined(PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG))
                    else if (PD_CheckWhetherInitiateCableDiscoveryIdentityOrNot(pdInstance))
                    {
                        /* cable plug didn't response data object on PE_SRC_STARTUP state */
                        Pd_PsmSecondaryStateHandler(pdInstance, 1, &triggerInfo);
                    }
#endif
                    else if (!PD_PsmNoResponseHardResetCountCheck(pdInstance))
                    {
                        didNothingStepB = 1;
                    }
                    else
                    {
                    }
                    break;
                }

                case PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY: /* (B) */
                {
#if (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)) && \
    (defined(PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG))
                    Pd_PsmSecondaryStateHandler(pdInstance, 1, &triggerInfo);
                    if (pdInstance->psmSecondaryState[1] == PSM_IDLE)
                    {
                        pdInstance->pendingSOP  = kPD_MsgSOPMask;
                        pdInstance->psmNewState = PSM_PE_SRC_SEND_CAPABILITIES;
                    }
#endif
                    break;
                }

                case PSM_PE_SRC_SEND_CAPABILITIES: /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        PD_TimerClear(pdInstance, tSenderResponseTimer);
                        if (triggerInfo.pdMsgType == kPD_MsgRequest)
                        {
                            pdInstance->partnerRdoRequest.rdoVal = *((uint32_t *)(&(triggerInfo.pdMsgDataBuffer[0])));
                            pdInstance->psmNewState              = PSM_PE_SRC_NEGOTIATE_CAPABILITY;
                        }
                        else
                        {
                            /* discard vendor defined message at here */
                            if (triggerInfo.pdMsgType != kPD_MsgVendorDefined)
                            {
                                pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                            }
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = PSM_HARD_RESET;
                    }
                    else if (!PD_PsmNoResponseHardResetCountCheck(pdInstance))
                    {
                        didNothingStepB = 1;
                    }
                    else
                    {
                    }
                    break;
                }

                case PSM_PE_SRC_TRANSITION_SUPPLY: /* (B) */
                {
                    PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, kPD_MsgPsRdy, PSM_PE_SRC_READY);
                    if (pdInstance->psmGotoMinTx)
                    {
                        pdInstance->psmGotoMinTx = 0;
                        PD_DpmAppCallback(pdInstance, PD_DPM_SRC_GOTOMIN_SUCCESS, NULL, 1);
                    }
                    else if (pdInstance->psmNewState == PSM_PE_SRC_READY)
                    {
                        PD_DpmAppCallback(pdInstance, PD_DPM_SRC_RDO_SUCCESS, NULL, 1);
                    }
                    else
                    {
                    }
                    break;
                }

                case PSM_PE_SRC_READY: /* (B) */
                {
                    if (PD_PsmProcessReadyState(pdInstance, &triggerInfo))
                    {
#if (defined PD_CONFIG_PD3_PPS_ENABLE) && (PD_CONFIG_PD3_PPS_ENABLE)
                        if ((PD_PsmSourceIsPPSRDO(pdInstance)) &&
                            (PD_TimerCheckValidTimeOut(pdInstance, tSourcePPSCommTimer)))
                        {
                            pdInstance->psmNewState = PSM_HARD_RESET;
                        }
                        else
#endif
                        {
                            didNothingStepB = 1;
                        }
                    }
                    break;
                }

                case PSM_PE_SRC_DISABLED: /* (B) */
                {
                    if (!(PD_PsmNoResponseHardResetCountCheck(pdInstance)))
                    {
                        didNothingStepB = 1;
                    }
                    break;
                }

                case PSM_PE_SRC_WAIT_NEW_CAPABILITIES: /* (B) */
                    /* The exit transition is handled by the global state transitions */
                    didNothingStepB = 1;
                    break;

                case PSM_PE_SRC_HARD_RESET_RECEIVED: /* (B) */
                {
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSHardResetTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_SRC_TRANSITION_TO_DEFAULT;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
                }

                case PSM_PE_SRC_TRANSITION_TO_DEFAULT: /* (B) */
                {
                    if (pdInstance->psmHardResetNeedsVSafe0V)
                    {
                        /* source send hard_reset, 4. after tSrcRecover, open vsafe5v. */
                        /* source receive hard_reset, 4. after tSrcRecover, open vsafe5v. */
                        /* 2. wait tSrcRecover */
                        if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSrcRecoverTimer))
                        {
                            pdInstance->psmHardResetNeedsVSafe0V = 0;
                            pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam,
                                                                           kVbusPower_InHardReset);
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
#ifdef USBPD_ENABLE_VCONN_DISCHARGE
                            PD_DpmDischargeVconn(pdInstance, 0);
#endif
                            PD_DpmSetVconn(pdInstance, 1);
#endif
                        }
                        else
                        {
                            didNothingStepB = 1;
                        }
                    }
                    /* source send hard_reset, 5. wait vsafe5v. */
                    /* source receive hard_reset, 5. wait vsafe5v. */
                    else if (PD_PsmCheckVbus5V(pdInstance))
                    {
                        PD_DpmAppCallback(pdInstance, PD_DPM_SRC_HARD_RESET_REQUEST, NULL, 1);
                        PD_TimerStart(pdInstance, tNoResponseTimer, T_NO_RESPONSE);
                        pdInstance->enterSrcFromSwap = 0;
                        pdInstance->psmNewState      = PSM_PE_SRC_STARTUP;
                    }
                    else
                    {
                        /* Wait until the power supply transitions to default. */
                        didNothingStepB = 0;
                    }
                    break;
                }

#endif

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
                case PSM_PE_SNK_DISCOVERY: /* (B) */
                {
                    if ((PD_PsmNoResponseHardResetCountCheck(pdInstance)))
                    {
                    }
                    else if (pdInstance->psmHardResetNeedsVSafe0V)
                    {
                        if ((PD_PsmCheckVsafe0V(pdInstance)) ||
                            (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSHardResetTimer)))
                        {
                            pdInstance->psmHardResetNeedsVSafe0V = 0;
                        }
                        else
                        {
                            /* No change in state, we should stay in low power mode */
                            didNothingStepB = 1;
                        }
                    }
                    else if (PD_PsmCheckOnlyVbus(pdInstance))
                    {
                        if (pdInstance->asmHardResetSnkProcessing)
                        {
                            pdInstance->asmHardResetSnkProcessing = 0;
                            pdInstance->callbackFns->PD_SnkDrawTypeCVbus(pdInstance->callbackParam,
                                                                         (uint8_t)PD_PsmGetTypeCCurrent(pdInstance),
                                                                         kVbusPower_InHardReset);
                            PD_DpmAppCallback(pdInstance, PD_DPM_SNK_HARD_RESET_REQUEST, NULL, 1);
                        }

                        /* Hard Reset is no longer in progress, allow VBus monitoring */
                        /* kVbusPower_InHardReset is end */
                        PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
                        pdInstance->psmHardResetNeedsVSafe0V = 0;

                        pdInstance->psmNewState = PSM_PE_SNK_WAIT_FOR_CAPABILITIES;
                    }
                    else
                    {
                        /* need check vbus constantly */
                        didNothingStepB = 0;
                    }
                    break;
                }

                case PSM_PE_SNK_WAIT_FOR_CAPABILITIES: /* (B) */
                {
                    if (!(PD_PsmNoResponseHardResetCountCheck(pdInstance)))
                    {
                        didNothingStepB = 1;
                    }
                    else
                    {
                    }
                    break;
                }

                case PSM_PE_SNK_SELECT_CAPABILITY: /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        uint8_t commandResultCallback = 0;
                        switch (triggerInfo.pdMsgType)
                        {
                            case kPD_MsgAccept:
                                pdInstance->psmNewState = PSM_PE_SNK_TRANSITION_SINK;
                                break;

                            case kPD_MsgWait:
                                pdInstance->psmNewState = PSM_PE_SNK_READY;
                                commandResultCallback   = kCommandResult_Wait;
                                PD_DpmAppCallback(pdInstance, PD_DPM_SNK_RDO_FAIL, &commandResultCallback, 1);
                                if (pdInstance->psmExplicitContractExisted)
                                {
                                    pdInstance->psmSnkReceiveRdoWaitRetry = 1;
                                    PD_TimerStart(pdInstance, tSinkRequestTimer, T_SINK_REQUEST);
                                }
                                else
                                {
                                    pdInstance->psmNewState = PSM_PE_SNK_WAIT_FOR_CAPABILITIES;
                                }
                                break;

                            case kPD_MsgReject:
                                pdInstance->psmNewState = PSM_PE_SNK_READY;
                                commandResultCallback   = kCommandResult_Reject;
                                PD_DpmAppCallback(pdInstance, PD_DPM_SNK_RDO_FAIL, &commandResultCallback, 1);
                                if (!pdInstance->psmExplicitContractExisted)
                                {
                                    pdInstance->psmNewState = PSM_PE_SNK_WAIT_FOR_CAPABILITIES;
                                }
                                break;

                            default:
                                if (triggerInfo.pdMsgType != kPD_MsgInvalid)
                                {
                                    /* SourceCapabilities and VendorDefined are handled in the global section. */
                                    /* soft reset other packets */
                                    pdInstance->psmNewState = PSM_CHECK_SINK_SOURCE_CAP_RX;
                                }
                                break;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = PSM_HARD_RESET;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                }
                break;

                case PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT: /* (B) */
                {
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSinkRequestTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_SNK_SELECT_CAPABILITY;
                    }
                    break;
                }

                case PSM_PE_SNK_TRANSITION_SINK: /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        /* SourceCapabilities and VendorDefined are handled in the global section. */
                        if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && (triggerInfo.pdMsgType == kPD_MsgPing))
                        {
                            /* Remain in the same state */
                            pdInstance->psmNewState = PSM_PE_SNK_TRANSITION_SINK;
                        }
                        else
                        {
                            PD_TimerClear(pdInstance, tPSTransitionTimer);
                            if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && (triggerInfo.pdMsgType == kPD_MsgPsRdy))
                            {
                                PD_MsgReceive(pdInstance);
                                if (pdInstance->psmGotoMinRx)
                                {
                                    pdInstance->psmGotoMinRx = 0;
                                    PD_DpmAppCallback(pdInstance, PD_DPM_SNK_GOTOMIN_SUCCESS, NULL, 1);
                                }
                                else
                                {
                                    pdInstance->callbackFns->PD_SnkDrawRequestVbus(pdInstance->callbackParam,
                                                                                   pdInstance->rdoRequest);
                                    PD_DpmAppCallback(pdInstance, PD_DPM_SNK_RDO_SUCCESS, &pdInstance->rdoRequest, 1);
                                }
                                pdInstance->psmNewState = PSM_PE_SNK_READY;
                            }
                            else
                            {
                                pdInstance->psmNewState = PSM_HARD_RESET;
                            }
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
                }

                case PSM_PE_SNK_READY: /* (B) */
                {
                    if (PD_PsmProcessReadyState(pdInstance, &triggerInfo))
                    {
                        if (PD_TimerCheckValidTimeOut(pdInstance, tSinkRequestTimer))
                        {
                            if (pdInstance->psmSnkReceiveRdoWaitRetry)
                            {
                                pdInstance->psmSnkReceiveRdoWaitRetry = 0;
                                pdInstance->psmNewState               = PSM_PE_SNK_SELECT_CAPABILITY;
                            }
                        }
#if (defined PD_CONFIG_PD3_PPS_ENABLE) && (PD_CONFIG_PD3_PPS_ENABLE)
                        else if ((PD_PsmSinkIsPPSRDO(pdInstance)) &&
                                 (PD_TimerCheckValidTimeOut(pdInstance, tSinkPPSPeriodicTimer)))
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_EVALUATE_CAPABILITY;
                        }
#endif
                        else
                        {
                            didNothingStepB = 1;
                        }
                    }
                    break;
                }

                case PSM_PE_SNK_TRANSITION_TO_DEFAULT: /* (B) */
                    break;
#endif

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                case PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP_GET_SOURCE_CAP: /* (B) */
#endif
                case PSM_PE_DR_SRC_GET_SOURCE_CAP: /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        PD_TimerClear(pdInstance, tSenderResponseTimer);
                        /* SourceCapabilities and VendorDefined are handled in the global section. */
                        if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && ((triggerInfo.pdMsgType == kPD_MsgReject) ||
                                                                     (triggerInfo.pdMsgType == kPD_MsgNotSupported)))
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
                            if (pdInstance->psmCurState == PSM_PE_DR_SRC_GET_SOURCE_CAP)
#endif
                            {
                                uint8_t commandResultCallback;
                                if (triggerInfo.pdMsgType == kPD_MsgReject)
                                {
                                    commandResultCallback = kCommandResult_Reject;
                                }
                                else
                                {
                                    commandResultCallback = kCommandResult_NotSupported;
                                }
                                PD_DpmAppCallback(pdInstance, PD_DPM_GET_PARTNER_SRC_CAP_FAIL, &commandResultCallback,
                                                  1);
                            }
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_SRC_READY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
                }

                case PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF: /* (B) */
                {
                    /* check source is standby */
                    if (PD_PsmCheckVsafe0V(pdInstance))
                    {
                        /* NOTE : DPM will actively discharge VBUS, and use a timer to ensure at least a min discharge
                         * time
                         */
                        /* have enter standby */
                        PD_DpmDischargeVbus(pdInstance, 0);
                        pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_ASSERT_RD;
                    }
                    else
                    {
                        didNothingStepB = 0; /* need check vbus constantly */
                    }
                    break;
                }
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                case PE_FRS_SRC_SNK_Transition_to_off: /* B */
                {
                    /* wait vbus reach vSafe5V */
                    if ((PD_PsmCheckOnlyVbus(pdInstance)) &&
                        (PD_TimerCheckInvalidOrTimeOut(pdInstance, timrFRSwapWaitPowerStable)))
                    {
                        pdInstance->psmNewState = PE_FRS_SRC_SNK_Assert_Rd;
                    }
                    else
                    {
                        /* need check vbus constantly */
                        didNothingStepB = 0;
                    }
                    break;
                }
#endif
                case PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF: /* (B) */
                    PD_PsmPowerSwapSinkSourceTransitionOff(pdInstance, &triggerInfo, PSM_PE_PRS_SNK_SRC_ASSERT_RP,
                                                           &didNothingStepB);
                    break;
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                case PE_FRS_SNK_SRC_Transition_to_off: /* B */
                    PD_PsmCheckFRS5V(pdInstance);
                    PD_PsmPowerSwapSinkSourceTransitionOff(pdInstance, &triggerInfo, PE_FRS_SNK_SRC_Vbus_Applied,
                                                           &didNothingStepB);
                    break;
#endif

                case PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON: /* (B) */
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                case PE_FRS_SRC_SNK_Wait_Source_on: /* B */
#endif
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && (triggerInfo.pdMsgType == kPD_MsgPsRdy))
                        {
#if 0
                            /* SIP code doesn't do this */
                            PD_FRSControl(pdInstance, 1);
#endif
                            PD_MsgStopReceive(pdInstance);
                            PD_TimerClear(pdInstance, tPSSourceOnTimer);
                            /* Swap from SOURCE to SINK */
                            /* pr swap end */
                            PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                            /* Cable plug will need a soft reset */
                            pdInstance->psmCablePlugResetNeeded = 1;
#endif

                            uint8_t vbusPowerState;
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                            if (pdInstance->psmCurState == PE_FRS_SRC_SNK_Wait_Source_on)
                            {
                                vbusPowerState = kVbusPower_InFRSwap;
                            }
                            else
#endif
                            {
                                vbusPowerState = kVbusPower_InPRSwap;
                            }
                            pdInstance->callbackFns->PD_SnkDrawTypeCVbus(
                                pdInstance->callbackParam, (uint8_t)PD_PsmGetTypeCCurrent(pdInstance), vbusPowerState);
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                            if (pdInstance->psmCurState == PE_FRS_SRC_SNK_Wait_Source_on)
                            {
                                vbusPowerState = PD_DPM_FR_SWAP_SUCCESS;
                            }
                            else
#endif
                            {
                                vbusPowerState = PD_DPM_PR_SWAP_SUCCESS;
                            }
                            PD_DpmAppCallback(pdInstance, vbusPowerState, NULL, 1);

                            pdInstance->psmNewState = PSM_PE_SNK_STARTUP;
                        }
                        else
                        {
                            /* A protocol error during power role swap triggers a Hard Reset */
                            pdInstance->psmNewState = PSM_HARD_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSSourceOnTimer))
                    {
                        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
                }

                case PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP: /* (B) */
                case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP: /* (B) */
                    PD_PsmStateWaitReply(pdInstance, &triggerInfo, &didNothingStepB);
                    break;
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                case PE_FRS_SNK_SRC_Send_Swap: /* B */
                {
                    PD_PsmCheckFRS5V(pdInstance);
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if (triggerInfo.pdMsgType == kPD_MsgAccept)
                        {
                            pdInstance->psmNewState = PE_FRS_SNK_SRC_Transition_to_off;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY;
                        }
                    }
                    else if (PD_TimerCheckValidTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
                }
#endif

                case PSM_PE_PRS_SNK_SRC_SOURCE_ON: /* (B) */
                {
                    if (PD_PsmCheckVbus5V(pdInstance))
                    {
                        PD_PsmPowerSwapSinkOpenVbus(pdInstance, PD_DPM_PR_SWAP_SUCCESS);
                    }
                    else
                    {
                        /* need check vbus constantly */
                        didNothingStepB = 0;
                    }
                    break;
                }

                case PSM_PE_PRS_EVALUATE_PR_SWAP_WAIT_TIMER_TIME_OUT: /* (B) */
                {
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPrSwapWaitTimer))
                    {
                        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                        {
                            pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP;
                        }
                    }
                    break;
                }

                case PSM_PE_DR_SNK_GET_SINK_CAP: /* (B) */
                case PSM_PE_SRC_GET_SINK_CAP:    /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;

                        /* SourceCapabilities and VendorDefined are handled in the global section. */
                        PD_TimerClear(pdInstance, tSenderResponseTimer);
                        if (triggerInfo.pdMsgType == kPD_MsgSinkCapabilities)
                        {
                            pd_capabilities_t sinkCapa;
                            /* Clear any pending message now that we have the latest */
                            (void)PD_DpmClearMsg(pdInstance, PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES);
                            /* Update the PDOs for the EC and send interrupt */
                            sinkCapa.capabilities      = MSG_DATA_BUFFER;
                            sinkCapa.capabilitiesCount = triggerInfo.pdMsgDataLength;
                            if (pdInstance->commandProcessing == PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES)
                            {
                                pdInstance->commandProcessing = 0;
                            }
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_PARTNER_SNK_CAP_SUCCESS, &sinkCapa, 1);
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                            pdInstance->portPartnerSinkPDO1.PDOValue = sinkCapa.capabilities[0];
#endif
#endif
                        }
                        else if ((triggerInfo.pdMsgType == kPD_MsgReject) ||
                                 (triggerInfo.pdMsgType == kPD_MsgNotSupported))
                        {
                            uint8_t commandResultCallback;
                            if (triggerInfo.pdMsgType == kPD_MsgReject)
                            {
                                commandResultCallback = kCommandResult_Reject;
                            }
                            else
                            {
                                commandResultCallback = kCommandResult_NotSupported;
                            }
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_PARTNER_SNK_CAP_FAIL, &commandResultCallback, 1);
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
                }
#endif

#if defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                case PSM_PE_DRS_SEND_DR_SWAP: /* (B) */
                    PD_PsmStateWaitReply(pdInstance, &triggerInfo, &didNothingStepB);
                    break;

                case PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_TIMER_TIME_OUT: /* (B) */
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tDrSwapWaitTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_DRS_SEND_DR_SWAP;
                    }
                    break;
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
                case PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_ALT_MODE_EXIT: /* (B) */
                {
                    uint8_t doSwap = 0;
                    if (PD_PsmCheckInAltMode(pdInstance, kPD_MsgSOP))
                    {
                        /* exit alt mode and wait the exit done */
                        if (PD_AltModeExitModeForDrSwap(pdInstance->altModeHandle) == 1)
                        {
                            doSwap = 1;
                        }
                    }
                    else
                    {
                        doSwap = 1;
                    }

                    if (doSwap)
                    {
                        PD_TimerClear(pdInstance, tDrSwapWaitTimer);
                        pdInstance->psmNewState = PSM_PE_DRS_SEND_DR_SWAP;
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tDrSwapWaitTimer))
                    {
                        doSwap = kCommandResult_Error;
                        /* the AMS is not started */
                        PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_FAIL, &doSwap, 0);
                    }
                    else
                    {
                    }
                    break;
                }
#endif
#endif

#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
                case PSM_PE_VCS_SEND_SWAP: /* (B) */
                    PD_PsmStateWaitReply(pdInstance, &triggerInfo, &didNothingStepB);
                    break;

                case PSM_PE_VCS_WAIT_FOR_VCONN: /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        /* SourceCapabilities and VendorDefined are handled in the global section. */
                        PD_TimerClear(pdInstance, tVconnOnTimer);
                        if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && (triggerInfo.pdMsgType == kPD_MsgPsRdy))
                        {
                            pdInstance->psmNewState = PSM_PE_VCS_TURN_OFF_VCONN;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tVconnOnTimer))
                    {
                        pdInstance->psmNewState = PSM_HARD_RESET;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
                }

                case PSM_PE_VCS_TURN_ON_VCONN: /* (B) */
                {
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tVconnOnTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_VCS_SEND_PS_RDY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
                }
#endif

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
                case PE_DR_SRC_GET_SOURCE_CAP_EXT: /* B */
                case PE_SNK_GET_SOURCE_CAP_EXT:    /* B */
                    PD_StateWaitReplyExtDataProcess(pdInstance, &triggerInfo, kPD_MsgSourceCapExtended,
                                                    PD_DPM_GET_SRC_EXT_CAP_SUCCESS, PD_DPM_GET_SRC_EXT_CAP_FAIL,
                                                    &didNothingStepB);
                    break;

                case PE_SRC_Get_Sink_Status:   /* B */
                case PE_SNK_Get_Source_Status: /* B */
                    PD_StateWaitReplyExtDataProcess(pdInstance, &triggerInfo, kPD_MsgStatus, PD_DPM_GET_STATUS_SUCCESS,
                                                    PD_DPM_GET_STATUS_FAIL, &didNothingStepB);
                    break;

                case PE_Get_Battery_Cap: /* B */
                    PD_StateWaitReplyExtDataProcess(pdInstance, &triggerInfo, kPD_MsgBatteryCapabilities,
                                                    PD_DPM_GET_BATTERY_CAP_SUCCESS, PD_DPM_GET_BATTERY_CAP_FAIL,
                                                    &didNothingStepB);
                    break;

                case PE_Get_Battery_Status: /* B */
                    PD_StateWaitReplyExtDataProcess(pdInstance, &triggerInfo, kPD_MsgBatteryStatus,
                                                    PD_DPM_GET_BATTERY_STATUS_SUCCESS, PD_DPM_GET_BATTERY_STATUS_FAIL,
                                                    &didNothingStepB);
                    break;

                case PE_Get_Manufacturer_Info: /* B */
                    PD_StateWaitReplyExtDataProcess(pdInstance, &triggerInfo, kPD_MsgManufacturerInfo,
                                                    PD_DPM_GET_MANUFACTURER_INFO_SUCCESS,
                                                    PD_DPM_GET_MANUFACTURER_INFO_FAIL, &didNothingStepB);
                    break;
#endif

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                case PE_FRS_SRC_SNK_CC_Signal: /* B */
                    didNothingStepB = 1;
                    if (((triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG) &&
                         (triggerInfo.msgHeader.bitFields.NumOfDataObjs == 0) &&
                         (triggerInfo.msgHeader.bitFields.messageType == kPD_MsgFrSwap)) ||
                        (pdInstance->frSwapReceived))
                    {
                        PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InFRSwap);
                        pdInstance->frSwapReceived = 0;
                        pdInstance->psmNewState    = PE_FRS_SRC_SNK_Evaluate_Swap;
                        didNothingStepB            = 0;
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tFRSwapSignalTimer))
                    {
                        /* In normal situation, this system cannot execute to here because the system has no power now
                         */
                        PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
                    }
                    else
                    {
                    }
                    break;

                case PE_FRS_SNK_SRC_Vbus_Applied: /* B */
                {
                    uint8_t checkVal;
                    PD_PsmCheckFRS5V(pdInstance);
                    PD_PhyControl(pdInstance, PD_PHY_FR_SWAP_CHECK_VBUS_APPLIED, &checkVal);
                    if ((pdInstance->fr5VOpened) || (checkVal))
                    {
                        pdInstance->fr5VOpened  = 1;
                        pdInstance->psmNewState = PE_FRS_SNK_SRC_Assert_Rp;
                    }
                    else
                    {
                        /* need check vbus constantly */
                        didNothingStepB = 0;
                    }
                    break;
                }
#endif
                case PSM_PE_BIST_TEST_DATA_MODE: /* (B) */
                    didNothingStepB = 1;
                    break;

                case PSM_PE_BIST_CARRIER_MODE_2: /* (B) */
#if (defined PD_CONFIG_COMPLIANCE_TEST_ENABLE) && (PD_CONFIG_COMPLIANCE_TEST_ENABLE)
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tBISTContModeTimer))
                    {
                        PD_PhyControl(pdInstance, PD_PHY_RESET_MSG_FUNCTION, NULL);
                        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_TRANSITION_TO_DEFAULT;
                        }
                        if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_TRANSITION_TO_DEFAULT;
                        }
                        /* PD_PhyControl(pdInstance, PD_PHY_EXIT_BIST, NULL); */
                    }
                    else
                    {
                        /* We do nothing here, it's all done by the hardware until the device is power cycled. */
                        didNothingStepB = 1;
                    }
#endif
                    break;

#if 0
                case PE_Send_Security_Response: /* (B) */
                {
                    if (PD_PsmChunkingLayerTXStateMachine(pdInstance,
                                                          kPD_MsgSOP, kPD_MsgSecurityResponse,
                                                          pdInstance->commandExtParam.dataBuffer,
                                                          pdInstance->commandExtParam.dataLength, &triggerInfo)
                        != kStatus_PD_Success)
                    {
                        PD_PsmTransitionOnMsgSendError(pdInstance, PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                    }
                    else
                    {
                        if (PD_PsmChunkingLayerCheckSentDone(pdInstance))
                        {
                            pdInstance->psmNewState = PE_PSM_STATE_ROLE_RDY_STATE;
                        }
                        else
                        {
                            /* wait chunk message sent done */
                        }
                    }
                    break;
                }
#endif

                default:
                    didNothingStepB = 1;
                    break;
            }
        }
        else
        {
        }

        /* put this type exception process codes here, then the normal process codes are simple */
        PD_PsmCheckUnexpectedReceivedMsg(pdInstance, triggerInfo.triggerEvent, &didNothingStepB);

        if (triggerInfo.triggerEvent != PSM_TRIGGER_NONE)
        {
            if (triggerInfo.triggerEvent == PSM_TRIGGER_DPM_MSG)
            {
                PD_PsmCommandFail(pdInstance, triggerInfo.dpmMsg);
            }
            triggerInfo.triggerEvent = PSM_TRIGGER_NONE;
        }
    }

    if ((msgReceived) && (PD_PsmCanPendingReceive(pdInstance)))
    {
        PD_MsgReceive(pdInstance); /* msg has been processed, receiving next msg */
    }

    PD_PsmProcessImportEventBeforeNextStateMachine(pdInstance);
    if ((!didNothingStepB) || (!didNothingStepA))
    {
        return kSM_Continue;
    }

    return kSM_WaitEvent;
}

/*! ***************************************************************************
   VMD process related functions
******************************************************************************/
#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)) || \
    (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE))
static void PD_PsmEndVdmCommand(pd_instance_t *pdInstance, uint8_t secondState)
{
    if (secondState != PSM_IDLE)
    {
        return;
    }

    if (pdInstance->commandProcessing <= PD_DPM_ALERT)
    {
        return;
    }
    else
    {
        PD_PsmCommandFail(pdInstance, pdInstance->commandProcessing);
    }
}
#endif

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)

static pd_psm_state_t PD_PsmVdmCheckkVDMBusyTimer(pd_instance_t *pdInstance, pd_psm_state_t newState)
{
    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tVDMBusyTimer))
    {
        return newState;
    }
    else
    {
        pdInstance->psmVDMBusyWaitDpmMsg = newState;
        return PSM_PE_DFP_UFP_VDM_VDM_BUSY_WAIT;
    }
}

static uint8_t PD_PsmVdmIdleProcessDpmMessage(pd_instance_t *pdInstance,
                                              uint8_t statIndex,
                                              psm_trigger_info_t *triggerInfo)
{
    pd_psm_state_t secondNewState = PSM_UNKNOWN;

    if (triggerInfo->dpmMsg >= PD_DPM_CONTROL_DISCOVERY_IDENTITY)
    {
        /* special process */
        if (pdInstance->structuredVdmCommandParameter.vdmSop != statIndex)
        {
            return PSM_UNKNOWN;
        }
        switch (triggerInfo->dpmMsg)
        {
            case PD_DPM_CONTROL_CABLE_RESET:
                if (statIndex == 0)
                {
                    return PSM_UNKNOWN;
                }
                break;

            default:
                break;
        }

        /* common process */
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        if (!PD_PsmStartCommand(pdInstance, triggerInfo->dpmMsg, 1))
        {
            PD_PsmCommandFail(pdInstance, triggerInfo->dpmMsg);
            return PSM_UNKNOWN;
        }
        switch (triggerInfo->dpmMsg)
        {
            case PD_DPM_CONTROL_CABLE_RESET:
            {
                secondNewState = PSM_PE_DFP_CBL_SEND_CABLE_RESET;
                break;
            }
            case PD_DPM_CONTROL_DISCOVERY_IDENTITY:
                secondNewState = PD_PsmVdmCheckkVDMBusyTimer(pdInstance, PSM_PE_DFP_UFP_VDM_IDENTITY_REQUEST);
                break;
            case PD_DPM_CONTROL_DISCOVERY_SVIDS:
                secondNewState = PD_PsmVdmCheckkVDMBusyTimer(pdInstance, PSM_PE_DFP_VDM_SVIDS_REQUEST);
                break;
            case PD_DPM_CONTROL_DISCOVERY_MODES:
                secondNewState = PD_PsmVdmCheckkVDMBusyTimer(pdInstance, PSM_PE_DFP_VDM_MODES_REQUEST);
                break;
            case PD_DPM_CONTROL_ENTER_MODE:
            {
                secondNewState = PSM_PE_DFP_VDM_MODE_ENTRY_REQUEST;
                break;
            }
            case PD_DPM_CONTROL_EXIT_MODE:
            {
                secondNewState = PSM_PE_DFP_VDM_MODE_EXIT_REQUEST;
                break;
            }
            case PD_DPM_CONTROL_SEND_ATTENTION:
            {
                secondNewState = PSM_PE_DFP_VDM_ATTENTION_REQUEST;
                break;
            }
            case PD_DPM_SEND_VENDOR_STRUCTURED_VDM:
            {
                secondNewState = PSM_PE_VENDOR_STRUCTURED_VDM_REQUEST;
                break;
            }
            case PD_DPM_SEND_UNSTRUCTURED_VDM:
            {
                secondNewState = PSM_PD_SEND_UNSTRUCTURED_VDM;
                break;
            }

            default:
                break;
        }
    }

    return secondNewState;
}

static uint8_t PDPsmVdmEnterExitModePdMsgProcess(pd_instance_t *pdInstance,
                                                 uint8_t statIndex,
                                                 psm_trigger_info_t *triggerInfo,
                                                 pd_svdm_command_request_t *commandVdmRequest)
{
    if (pdInstance->curDataRole == kPD_DataRoleDFP)
    {
        pd_structured_vdm_header_t reponseVdmHeader;
        reponseVdmHeader.structuredVdmHeaderVal = triggerInfo->vdmHeader.structuredVdmHeaderVal;
        /* not supported */
        reponseVdmHeader.bitFields.commandType = kVDM_ResponderNAK;
        PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex, reponseVdmHeader, 0, NULL);
    }
    else
    {
        commandVdmRequest->vdoSop                           = triggerInfo->pdMsgSop;
        commandVdmRequest->vdmHeader.structuredVdmHeaderVal = triggerInfo->vdmHeader.structuredVdmHeaderVal;
        if (triggerInfo->vdmHeader.bitFields.command == kVDM_EnterMode)
        {
            if (triggerInfo->msgHeader.bitFields.NumOfDataObjs >= 1)
            {
                commandVdmRequest->vdoCount = 1;
                commandVdmRequest->vdoData  = (uint32_t *)&triggerInfo->pdMsgDataBuffer[4];
            }
        }
        if (((statIndex == 0) && (pdInstance->curDataRole == kPD_DataRoleUFP)) || (statIndex >= 1))
        {
            PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, commandVdmRequest, 0);
            return 1;
        }
    }
    return 0;
}

static void PD_PsmVdmIdleProcessPdMessage(pd_instance_t *pdInstance, uint8_t statIndex, psm_trigger_info_t *triggerInfo)
{
    /* note: vdm message will be processed before this */
    if (triggerInfo->pdMsgType == kPD_MsgVendorDefined)
    {
        if (triggerInfo->pdMsgSop != statIndex)
        {
            return;
        }

        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;

        /* structured vdm */
        if (triggerInfo->vdmHeader.bitFields.vdmType)
        {
            if (triggerInfo->vdmHeader.bitFields.commandType == kVDM_Initiator)
            {
                pd_svdm_command_request_t commandVdmRequest;
                uint8_t needReply = 0;

                if ((triggerInfo->vdmHeader.bitFields.command >= kVDM_DiscoverIdentity) &&
                    (triggerInfo->vdmHeader.bitFields.command <= kVDM_ExitMode) &&
                    (pdInstance->curDataRole == kPD_DataRoleDFP))
                {
                    /* PD2.0 spec: If Structured VDMs are not supported,
                     * a Structured VDM Command received by a DFP or UFP
                     * Shall be Ignored */
                    if (pdInstance->revision < PD_SPEC_REVISION_30)
                    {
                        return; /* ignore these commands */
                    }
                    else
                    {
                        /* PD3.0 spec: If Structured VDMs are not supported,
                         * the DFP or UFP receiving a VDM Command Shall send
                         * a Not_Supported Message in response */
                        commandVdmRequest.requestResultStatus = kCommandResult_VDMNAK;
                        PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, PD_NOT_SUPPORT_REPLY_MSG,
                                                                           pdInstance->psmCurState);
                        return;
                    }
                }

                commandVdmRequest.vdoSop                           = triggerInfo->pdMsgSop;
                commandVdmRequest.vdmHeader.structuredVdmHeaderVal = triggerInfo->vdmHeader.structuredVdmHeaderVal;

                /* process the message and get the reply data */
                if (!needReply)
                {
                    /* special process */
                    switch (triggerInfo->vdmHeader.bitFields.command)
                    {
                        case kVDM_DiscoverIdentity:
                            if (triggerInfo->vdmHeader.bitFields.SVID == PD_STANDARD_ID)
                            {
                                needReply = 1;
                                PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                                for (uint8_t index = 0; index < commandVdmRequest.vdoCount; ++index)
                                {
                                    pdInstance->sendingData[2 + index] = commandVdmRequest.vdoData[index];
                                }
                                if (pdInstance->revision < PD_SPEC_REVISION_30)
                                {
                                    ((pd_id_header_vdo_t *)(&pdInstance->sendingData[2]))->bitFields.productTypeDFP = 0;
                                }
                            }
                            break;

                        case kVDM_DiscoverSVIDs:
                            if (triggerInfo->vdmHeader.bitFields.SVID == PD_STANDARD_ID)
                            {
                                needReply = 1;
                                PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                            }
                            break;

                        case kVDM_DiscoverModes:
                            needReply = 1;
                            PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                            break;

                        case kVDM_EnterMode:
                            needReply = PDPsmVdmEnterExitModePdMsgProcess(pdInstance, statIndex, triggerInfo,
                                                                          &commandVdmRequest);
                            break;

                        case kVDM_ExitMode:
                            pdInstance->psmVdmActiveModeValidMask &=
                                ~(uint32_t)(0x01u << triggerInfo->vdmHeader.bitFields.objPos);
                            pdInstance->vdmExitReceived[statIndex] = 0;
                            needReply = PDPsmVdmEnterExitModePdMsgProcess(pdInstance, statIndex, triggerInfo,
                                                                          &commandVdmRequest);
                            break;

                        case kVDM_Attention:
                            if (triggerInfo->msgHeader.bitFields.NumOfDataObjs >= 1)
                            {
                                commandVdmRequest.vdoCount = 1;
                                commandVdmRequest.vdoData  = (uint32_t *)&triggerInfo->pdMsgDataBuffer[4];
                            }
                            needReply = 0;
                            PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                            break;

                        default:
                        {
                            /* vendor structured vdm */
                            if (triggerInfo->vdmHeader.bitFields.command >= 16)
                            {
                                needReply                  = 1;
                                commandVdmRequest.vdoCount = (triggerInfo->pdMsgDataLength - 1);
                                commandVdmRequest.vdmHeader.structuredVdmHeaderVal =
                                    USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS((triggerInfo->pdMsgDataBuffer));
                                commandVdmRequest.vdoData = (uint32_t *)(triggerInfo->pdMsgDataBuffer + 4);
                                PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                            }
                            break;
                        }
                    }
                }

                /* reply response mseeage */
                if (needReply)
                {
                    pd_structured_vdm_header_t reponseVdmHeader;
                    reponseVdmHeader.structuredVdmHeaderVal = triggerInfo->vdmHeader.structuredVdmHeaderVal;
                    if (commandVdmRequest.requestResultStatus == kCommandResult_VDMACK)
                    {
                        reponseVdmHeader.bitFields.commandType = kVDM_ResponderACK;
                        if (PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex, reponseVdmHeader,
                                                           commandVdmRequest.vdoCount,
                                                           (uint32_t *)commandVdmRequest.vdoData))
                        {
                            if (triggerInfo->vdmHeader.bitFields.command == kVDM_EnterMode)
                            {
                                pdInstance->psmVdmActiveModeValidMask |=
                                    (uint32_t)(0x01u << triggerInfo->vdmHeader.bitFields.objPos);
                            }
                        }
                    }
                    else
                    {
                        switch (triggerInfo->vdmHeader.bitFields.command)
                        {
                            case kVDM_EnterMode:
                            case kVDM_ExitMode:
                                reponseVdmHeader.bitFields.commandType = kVDM_ResponderNAK;
                                break;

                            case kVDM_DiscoverIdentity:
                            case kVDM_DiscoverSVIDs:
                            case kVDM_DiscoverModes:
                            default:
                                if (commandVdmRequest.requestResultStatus == kCommandResult_VDMNAK)
                                {
                                    reponseVdmHeader.bitFields.commandType = kVDM_ResponderNAK;
                                }
                                else
                                {
                                    reponseVdmHeader.bitFields.commandType = kVDM_ResponderBUSY;
                                }
                                break;
                        }
                        PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex, reponseVdmHeader, 0,
                                                       NULL);
                    }
                }
            }
            else
            {
                /* should not reach here */
            }
        }
        else
        {
            /* unstructured vdm */
            pd_unstructured_vdm_command_param_t unstructuredVDMParam;
            unstructuredVDMParam.vdmSop                = triggerInfo->pdMsgSop;
            unstructuredVDMParam.vdmHeaderAndVDOsData  = (uint32_t *)triggerInfo->pdMsgDataBuffer;
            unstructuredVDMParam.vdmHeaderAndVDOsCount = (triggerInfo->pdMsgDataLength);
            unstructuredVDMParam.resultStatus          = kCommandResult_None;
            PD_DpmAppCallback(pdInstance, PD_DPM_UNSTRUCTURED_VDM_RECEIVED, &unstructuredVDMParam, 1);

#if defined(PD_CONFIG_REVISION) && (PD_CONFIG_REVISION >= PD_SPEC_REVISION_30)
            if (unstructuredVDMParam.resultStatus == kCommandResult_NotSupported)
            {
                PD_PsmSendControlTransitionWithAsyncRxAndSoftReset(pdInstance, kPD_MsgNotSupported,
                                                                   pdInstance->psmCurState);
            }
#endif
        }
    }
}
#endif

#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)) || \
    (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE))

/*! ***************************************************************************
   \brief Common Handler for structured VDM initiator requests
   \param reject_flags will perSopPartner[kPD_MsgSOP].flags on failure
   \retval PSM_IDLE on failure
   \retval org_state on success
   \note Only called from CLP task context
******************************************************************************/
static pd_psm_state_t PD_PsmStructuredVdmInitiatorRequest(pd_instance_t *pdInstance,
                                                          uint8_t sopIndex,
                                                          uint16_t svid,
                                                          uint8_t position,
                                                          pd_vdm_command_t command,
                                                          uint8_t vdo_count,
                                                          uint32_t *vdos)
{
    pd_structured_vdm_header_t vdmHeader;

    start_of_packet_t sop = (start_of_packet_t)sopIndex;

    vdmHeader.structuredVdmHeaderVal = 0;
    vdmHeader.bitFields.command      = command;
    vdmHeader.bitFields.commandType  = kVDM_Initiator;
    vdmHeader.bitFields.objPos       = position;
    vdmHeader.bitFields.SVID         = svid;
    vdmHeader.bitFields.vdmType      = 1;
    vdmHeader.bitFields.vdmVersion   = PD_CONFIG_STRUCTURED_VDM_VERSION;
    /* Send Discover Modes request */
    if (PD_MsgSendStructuredVDMAndWait(pdInstance, sop, vdmHeader, vdo_count, vdos))
    {
        if (command == kVDM_EnterMode)
        {
            PD_TimerStart(pdInstance, tVDMModeEntryTimer, T_VDM_WAIT_MODE_ENTRY);
        }
        else if (command == kVDM_ExitMode)
        {
            PD_TimerStart(pdInstance, tVDMModeExitTimer, T_VDM_WAIT_MODE_EXIT);
        }
        else
        {
            PD_TimerStart(pdInstance, tVDMResponseTimer, T_VDM_SENDER_RESPONSE);
        }
        /* Event driven -> wait for response packet or timeout */
        return pdInstance->psmSecondaryState[sopIndex];
    }
    else
    {
        return PSM_IDLE;
    }
}

static uint8_t PD_PsmVdmResponseHandler(pd_instance_t *pdInstance,
                                        uint8_t sop,
                                        uint8_t vdmMsgType,
                                        psm_trigger_info_t *triggerInfo,
                                        tTimer_t timr,
                                        uint8_t ackState,
                                        uint8_t nakState)
{
    uint8_t expected    = 0;
    uint8_t newStateTmp = PSM_UNKNOWN;
    if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgType == kPD_MsgVendorDefined) &&
        (triggerInfo->vdmHeader.bitFields.command == vdmMsgType) &&
        (triggerInfo->vdmHeader.bitFields.commandType != kVDM_Initiator) && (triggerInfo->pdMsgSop == sop))
    {
        if ((vdmMsgType == kVDM_DiscoverIdentity) || (vdmMsgType == kVDM_DiscoverSVIDs))
        {
            if (triggerInfo->vdmHeader.bitFields.SVID == PD_STANDARD_ID)
            {
                expected = 1;
            }
        }
        else
        {
            expected = 1;
        }
    }

    if (expected)
    {
        PD_TimerClear(pdInstance, timr);
        triggerInfo->triggerEvent  = PSM_TRIGGER_NONE;
        pdInstance->amsVdmReplyMsg = triggerInfo->vdmHeader.bitFields.commandType;
        switch (triggerInfo->vdmHeader.bitFields.commandType)
        {
            case kVDM_ResponderACK:
                newStateTmp = ackState;
                break;

            case kVDM_ResponderBUSY:
                newStateTmp = nakState;
                break;

            case kVDM_ResponderNAK:
                if (vdmMsgType == kVDM_ExitMode)
                {
                    newStateTmp = ackState;
                }
                else
                {
                    newStateTmp = nakState;
                }
                break;

            default:
                newStateTmp = nakState;
                break;
        }
    }
    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, timr))
    {
        pdInstance->amsVdmReplyMsg = 0xFFu;
        newStateTmp                = nakState;
    }
    else
    {
        /* TODO: AMS interrupt process */
    }

    return newStateTmp;
}

/*! ***************************************************************************
   \brief Meet the requirement to terminate the msgSopP and kPD_MsgSOPp communications on
******************************************************************************/
static void PD_PsmSecondaryStateHandlerTerminate(pd_instance_t *pdInstance, uint8_t sop)
{
    int i;

    if (sop == 0xffu)
    {
        for (i = 0; i < 3; i++)
        {
            if (pdInstance->psmSecondaryState[i] != PSM_IDLE)
            {
                pdInstance->psmNewSecondaryState[i] = PSM_IDLE;
            }
        }
    }
    else
    {
        if (pdInstance->psmSecondaryState[sop] != PSM_IDLE)
        {
            pdInstance->psmNewSecondaryState[sop] = PSM_IDLE;
        }
    }
}

/*! ***************************************************************************
   \brief
   \note Only called from CLP task context
******************************************************************************/
static uint8_t Pd_PsmSecondaryStateHandler(pd_instance_t *pdInstance,
                                           uint8_t statIndex,
                                           psm_trigger_info_t *triggerInfo)
{
    uint32_t commandResultCallback;
    uint8_t didNothingSecond      = 0;
    pd_psm_state_t secondNewState = PSM_UNKNOWN;
    uint8_t index;
    uint8_t sop = statIndex; /* kPD_MsgSOP = 0, kPD_MsgSOPp = 1, kPD_MsgSOPpp = 2*/

    while (1)
    {
        /* global transition */
        if (triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG)
        {
            /* sop has high priority */
            if (sop != kPD_MsgSOP)
            {
                if (
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    (pdInstance->psmNewSecondaryState[statIndex] != PSM_PE_DFP_CBL_SEND_CABLE_RESET) &&
                    (pdInstance->psmSecondaryState[statIndex] != PSM_PE_DFP_CBL_SEND_CABLE_RESET) &&
                    (pdInstance->psmSecondaryState[statIndex] != PSM_PE_DFP_CBL_SEND_SOFT_RESET) &&
#endif
                    ((pdInstance->psmNewSecondaryState[statIndex] != PSM_UNKNOWN) ||
                     (pdInstance->psmSecondaryState[statIndex] != PSM_IDLE)))
                {
                    /* During PD Connection (Explicit Contract): */
                    /* o The DFP can communicate with a Cable Plug, using SOP' Packets or SOP" Packets, at any time it
                     */
                    /* is not engaged in any other SOP Communications. */
                    /* o If SOP Packets are received by the DFP, during SOP' or SOP" Communication, the SOP' or SOP" */
                    /* Communication is immediately terminated (the Cable Plug times out and does not retry) */
                    /* o If the DFP needs to initiate an SOP Communication during an ongoing SOP' or SOP" Communication
                     */
                    /* (e.g.for a Capabilities change) then the SOP' or SOP" Communications will be interrupted. */

                    /* Check for pending message on SOP */
                    /* Primary state machine needs to handle kPD_MsgSOP */
                    if (triggerInfo->pdMsgSop == kPD_MsgSOP)
                    {
                        /* Abort the current action */
                        pdInstance->psmNewSecondaryState[statIndex] = PSM_IDLE;
                        didNothingSecond                            = 1;
                    }
                }
                /* Allow soft reset to also interrupt with priority over cable resets */
                if (triggerInfo->pdMsgType == kPD_MsgSoftReset)
                {
                    /* Abort the current action */
                    pdInstance->psmNewSecondaryState[statIndex] = PSM_IDLE;
                    didNothingSecond                            = 1;
                }

                if (didNothingSecond)
                {
                    return didNothingSecond;
                }
            }
        }

        /* global transition for dpm message */
        if ((triggerInfo->triggerEvent == PSM_TRIGGER_DPM_MSG) &&
            (triggerInfo->dpmMsg >= PD_DPM_CONTROL_DISCOVERY_IDENTITY))
        {
            if (((pdInstance->psmNewSecondaryState[statIndex] != PSM_UNKNOWN) ||
                 (pdInstance->psmSecondaryState[statIndex] != PSM_IDLE)) &&
                (pdInstance->structuredVdmCommandParameter.vdmSop == (sop)))
            {
                triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
                PD_PsmCommandFail(pdInstance, triggerInfo->dpmMsg);
            }
        }

        /* State C */
        if (pdInstance->psmNewSecondaryState[statIndex] != PSM_UNKNOWN)
        {
            pdInstance->psmSecondaryState[statIndex]    = pdInstance->psmNewSecondaryState[statIndex];
            pdInstance->psmNewSecondaryState[statIndex] = PSM_UNKNOWN;
            secondNewState                              = pdInstance->psmSecondaryState[statIndex];
            pd_svdm_command_result_t commandVdmResult;
            commandVdmResult.vdmHeader.structuredVdmHeaderVal =
                pdInstance->structuredVdmCommandParameter.vdmHeader.structuredVdmHeaderVal;
            commandVdmResult.vdoSop = triggerInfo->pdMsgSop;

            switch (pdInstance->psmSecondaryState[statIndex])
            {
                case PSM_IDLE: /* C */
                    /* dpm message will callback */
                    break;

                case PSM_PE_SRC_IMPLICIT_CABLE_SOFT_RESET: /* C */
                    if ((PD_MsgSend(pdInstance, kPD_MsgSOPp, kPD_MsgSoftReset, 2, NULL) == kStatus_PD_Success) &&
                        (PD_MsgWaitSendResult(pdInstance)))
                    {
                        PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                    }
                    else
                    {
                        secondNewState = PSM_IDLE;
                    }
                    break;

                case PSM_PE_SRC_VDM_IDENTITY_REQUEST: /* C , only for sopp */
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    pdInstance->cableDiscoverIdentityCounter++;
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(pdInstance, statIndex, PD_STANDARD_ID, 0,
                                                                         kVDM_DiscoverIdentity, 0, NULL);
#endif
                    break;

                case PSM_PE_DFP_VDM_SVIDS_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(pdInstance, statIndex, PD_STANDARD_ID, 0,
                                                                         kVDM_DiscoverSVIDs, 0, NULL);
                    break;

                case PSM_PE_DFP_VDM_MODES_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, statIndex, pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID, 0,
                        kVDM_DiscoverModes, 0, NULL);
                    break;

                case PSM_PE_DFP_VDM_MODE_ENTRY_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, statIndex, pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos, kVDM_EnterMode,
                        (pdInstance->structuredVdmCommandParameter.vdoCount) ? 1 : 0,
                        (pdInstance->structuredVdmCommandParameter.vdoCount) ?
                            (pdInstance->structuredVdmCommandParameter.vdoData) :
                            NULL);
                    break;

                case PSM_PE_DFP_VDM_MODE_EXIT_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, statIndex, pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos, kVDM_ExitMode, 0, NULL);
                    break;

                case PSM_PE_DFP_VDM_MODE_EXIT_HARD_RESET: /* C */
                    commandVdmResult.vdmCommand = kVDM_ExitMode;
                    commandVdmResult.vdoData    = NULL;
                    commandVdmResult.vdoCount   = 0;
                    if (pdInstance->amsVdmReplyMsg == kVDM_ResponderBUSY)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMBUSY;
                    }
                    else
                    {
                        /* time out or receive non-right message */
                        commandVdmResult.vdmCommandResult = kCommandResult_Error;
                    }
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    secondNewState          = PSM_IDLE;
                    pdInstance->psmNewState = PSM_HARD_RESET;
                    /* TODO: it may be CABLE. */
                    break;

                case PSM_PE_DFP_VDM_ATTENTION_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, statIndex, pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos, kVDM_Attention,
                        (pdInstance->structuredVdmCommandParameter.vdoCount) ? 1 : 0,
                        (pdInstance->structuredVdmCommandParameter.vdoCount) ?
                            (pdInstance->structuredVdmCommandParameter.vdoData) :
                            NULL);

                    commandVdmResult.vdmCommand = kVDM_Attention;
                    commandVdmResult.vdoData    = NULL;
                    commandVdmResult.vdoCount   = 0;
                    if (secondNewState == PSM_IDLE)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else
                    {
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    }
                    break;

                case PSM_PE_DFP_CBL_SEND_CABLE_RESET: /* C */
                    PD_PhyControl(pdInstance, PD_PHY_SEND_CABLE_RESET, NULL);
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_VENDOR_STRUCTURED_VDM_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, statIndex, pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos,
                        (pd_vdm_command_t)pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command,
                        pdInstance->structuredVdmCommandParameter.vdoCount,
                        pdInstance->structuredVdmCommandParameter.vdoData);

                    if (!(pdInstance->structuredVdmCommandParameter.vendorVDMNeedResponse))
                    {
                        commandVdmResult.vdmCommand =
                            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command;
                        commandVdmResult.vdoData  = NULL;
                        commandVdmResult.vdoCount = 0;
                        if (secondNewState == pdInstance->psmSecondaryState[statIndex])
                        {
                            PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                        }
                        else
                        {
                            commandVdmResult.vdmCommandResult = kCommandResult_Error;
                            PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                        }
                        secondNewState = PSM_IDLE;
                    }
                    break;

                case PSM_PE_SRC_VDM_IDENTITY_ACKED: /* C */
                {
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    pdInstance->psmCableIdentitiesDataCount = triggerInfo->pdMsgDataLength;
                    for (index = 0; index < triggerInfo->pdMsgDataLength; ++index)
                    {
                        pdInstance->psmCableIdentities[index] = ((uint32_t *)&triggerInfo->pdMsgDataBuffer[0])[index];
                    }
                    pd_id_header_vdo_t *headerVDO = (pd_id_header_vdo_t *)(&pdInstance->psmCableIdentities[1]);
                    if ((headerVDO->bitFields.productTypeUFPOrCablePlug == VDM_ID_HEADER_VDO_PASSIVE_CABLE_VAL) ||
                        (headerVDO->bitFields.productTypeUFPOrCablePlug == VDM_ID_HEADER_VDO_ACTIVE_CABLE_VAL))
                    {
                        if (pdInstance->psmCableIdentitiesDataCount > 4)
                        {
                            pd_passive_cable_vdo_vdm20_t *cableVDO =
                                (pd_passive_cable_vdo_vdm20_t *)(&pdInstance->psmCableIdentities[4]);
                            switch (cableVDO->bitFields.vbusCurrentHandlingCapability)
                            {
                                case 0x01u:
                                    pdInstance->dpmCableMaxCurrent = 3000 / 10; /* 3A */
                                    break;
                                case 0x02u:
                                    pdInstance->dpmCableMaxCurrent = 5000 / 10; /* 5A */
                                    break;
                                default:
                                    pdInstance->dpmCableMaxCurrent = 3000 / 10; /* 3A */
                                    break;
                            }
                        }
                    }
                    secondNewState = PSM_IDLE;
#endif
                    break;
                }

                case PSM_PE_SRC_VDM_IDENTITY_NAKED: /* C */
                    for (index = 0; index < 7; ++index)
                    {
                        pdInstance->psmCableIdentities[index] = 0;
                    }
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_UFP_VDM_IDENTITY_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(pdInstance, statIndex, PD_STANDARD_ID, 0,
                                                                         kVDM_DiscoverIdentity, 0, NULL);
                    break;

                case PSM_PE_DFP_UFP_VDM_IDENTITY_ACKED:  /* C */
                case PSM_PE_DFP_VDM_SVIDS_ACKED:         /* C */
                case PSM_PE_DFP_VDM_MODES_ACKED:         /* C */
                case PSM_PE_VENDOR_STRUCTURED_VDM_ACKED: /* C */
                    if (pdInstance->psmSecondaryState[statIndex] == PSM_PE_DFP_UFP_VDM_IDENTITY_ACKED)
                    {
                        commandVdmResult.vdmCommand = kVDM_DiscoverIdentity;
                    }
                    else if (pdInstance->psmSecondaryState[statIndex] == PSM_PE_DFP_VDM_SVIDS_ACKED)
                    {
                        commandVdmResult.vdmCommand = kVDM_DiscoverSVIDs;
                    }
                    else if (pdInstance->psmSecondaryState[statIndex] == PSM_PE_DFP_VDM_MODES_ACKED)
                    {
                        commandVdmResult.vdmCommand = kVDM_DiscoverModes;
                    }
                    else if (pdInstance->psmSecondaryState[statIndex] == PSM_PE_VENDOR_STRUCTURED_VDM_ACKED)
                    {
                        commandVdmResult.vdmCommand = commandVdmResult.vdmHeader.bitFields.command;
                    }
                    else
                    {
                    }
                    commandVdmResult.vdmHeader.structuredVdmHeaderVal = ((uint32_t *)triggerInfo->pdMsgDataBuffer)[0];
                    commandVdmResult.vdoData                          = (uint32_t *)(triggerInfo->pdMsgDataBuffer + 4);
                    commandVdmResult.vdoCount                         = triggerInfo->pdMsgDataLength - 1;
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_VDM_MODE_ENTRY_ACKED: /* C */
                    commandVdmResult.vdmHeader.structuredVdmHeaderVal = ((uint32_t *)triggerInfo->pdMsgDataBuffer)[0];
                    commandVdmResult.vdmCommand                       = kVDM_EnterMode;
                    commandVdmResult.vdoData                          = NULL;
                    commandVdmResult.vdoCount                         = 0;
                    pdInstance->psmVdmActiveModeValidMask |=
                        (uint8_t)(0x01u << ((((uint32_t *)(triggerInfo->pdMsgDataBuffer))[0] >> 8) & 0x07u));
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_VDM_MODE_EXIT_ACKED: /* C */
                    commandVdmResult.vdmCommand = kVDM_ExitMode;
                    commandVdmResult.vdoData    = NULL;
                    commandVdmResult.vdoCount   = 0;
                    pdInstance->psmVdmActiveModeValidMask &=
                        ~(uint32_t)(0x01 << pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos);
                    if (pdInstance->amsVdmReplyMsg == kVDM_ResponderACK)
                    {
                        commandVdmResult.vdmHeader.structuredVdmHeaderVal =
                            ((uint32_t *)triggerInfo->pdMsgDataBuffer)[0];
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    }
                    else
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMNAK;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_UFP_VDM_IDENTITY_NAKED:  /* C */
                case PSM_PE_DFP_VDM_SVIDS_NAKED:         /* C */
                case PSM_PE_DFP_VDM_MODES_NAKED:         /* C */
                case PSM_PE_DFP_VDM_MODE_ENTRY_NAKED:    /* C */
                case PSM_PE_VENDOR_STRUCTURED_VDM_NAKED: /* C */
                    switch (pdInstance->psmSecondaryState[statIndex])
                    {
                        case PSM_PE_DFP_UFP_VDM_IDENTITY_NAKED:
                            commandVdmResult.vdmCommand = kVDM_DiscoverIdentity;
                            break;
                        case PSM_PE_DFP_VDM_SVIDS_NAKED:
                            commandVdmResult.vdmCommand = kVDM_DiscoverSVIDs;
                            break;
                        case PSM_PE_DFP_VDM_MODES_NAKED:
                            commandVdmResult.vdmCommand = kVDM_DiscoverModes;
                            break;
                        case PSM_PE_DFP_VDM_MODE_ENTRY_NAKED:
                            commandVdmResult.vdmCommand = kVDM_EnterMode;
                            break;
                        case PSM_PE_VENDOR_STRUCTURED_VDM_NAKED:
                            commandVdmResult.vdmCommand =
                                pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command;
                            break;
                        default:
                            break;
                    }

                    commandVdmResult.vdoData  = NULL;
                    commandVdmResult.vdoCount = 0;
                    if (pdInstance->amsVdmReplyMsg == kVDM_ResponderNAK)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMNAK;
                    }
                    else if (pdInstance->amsVdmReplyMsg == kVDM_ResponderBUSY)
                    {
                        PD_TimerStart(pdInstance, tVDMBusyTimer, T_VDM_BUSY);
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMBUSY;
                    }
                    else
                    {
                        /* time out or receive non-right message */
                        commandVdmResult.vdmCommandResult = kCommandResult_Error;
                    }
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    secondNewState = PSM_IDLE;
                    break;

#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE))
                case PSM_PD_SEND_UNSTRUCTURED_VDM: /* C */
                {
                    pd_status_t status = PD_MsgSendUnstructuredVDM(
                        pdInstance, (start_of_packet_t)pdInstance->unstructuredVdmCommandParameter.vdmSop,
                        (uint8_t *)(pdInstance->unstructuredVdmCommandParameter.vdmHeaderAndVDOsData),
                        pdInstance->unstructuredVdmCommandParameter.vdmHeaderAndVDOsCount * 4 + 2);
                    if (status == kStatus_PD_Success)
                    {
                        PD_DpmAppCallback(pdInstance, PD_DPM_SEND_UNSTRUCTURED_VDM_SUCCESS, NULL, 1);
                    }
                    else
                    {
                        commandResultCallback = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_SEND_UNSTRUCTURED_VDM_FAIL, &commandResultCallback, 1);
                    }
                    secondNewState = PSM_IDLE;
                    break;
                }
#endif

                default:
                    break;
            }

            PD_PsmEndVdmCommand(pdInstance, pdInstance->psmSecondaryState[statIndex]);

            if (secondNewState != pdInstance->psmSecondaryState[statIndex])
            {
                pdInstance->psmNewSecondaryState[statIndex] = secondNewState;
                continue;
            }
        }

        /* state B */
        secondNewState = pdInstance->psmSecondaryState[statIndex];
        switch (pdInstance->psmSecondaryState[statIndex])
        {
            case PSM_IDLE: /* B */
                if (triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG)
                {
                    PD_PsmVdmIdleProcessPdMessage(pdInstance, statIndex, triggerInfo);
                }
                else if (triggerInfo->triggerEvent == PSM_TRIGGER_DPM_MSG)
                {
                    secondNewState = (pd_psm_state_t)PD_PsmVdmIdleProcessDpmMessage(pdInstance, statIndex, triggerInfo);
                }
                else
                {
                }

                if (pdInstance->vdmExitReceived[sop])
                {
                    pd_svdm_command_request_t commandVdmRequest;
                    pd_structured_vdm_header_t reponseVdmHeader;
                    commandVdmRequest.vdmHeader.structuredVdmHeaderVal = pdInstance->vdmExitReceived[sop];
                    pdInstance->vdmExitReceived[sop]                   = 0;
                    commandVdmRequest.vdoSop                           = pdInstance->vdmExitReceivedSOP[sop];
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);

                    reponseVdmHeader.structuredVdmHeaderVal = commandVdmRequest.vdmHeader.structuredVdmHeaderVal;
                    reponseVdmHeader.bitFields.objPos       = commandVdmRequest.vdoSop;
                    if (commandVdmRequest.requestResultStatus == kCommandResult_VDMACK)
                    {
                        reponseVdmHeader.bitFields.commandType = kVDM_ResponderACK;
                        PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex, reponseVdmHeader,
                                                       commandVdmRequest.vdoCount,
                                                       (uint32_t *)commandVdmRequest.vdoData);
                    }
                    else
                    {
                        reponseVdmHeader.bitFields.commandType = kVDM_ResponderNAK;
                        PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex, reponseVdmHeader, 0,
                                                       NULL);
                    }
                }
                break;

            case PSM_PE_DFP_UFP_VDM_VDM_BUSY_WAIT: /* B */
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tVDMBusyTimer))
                {
                    secondNewState = (pd_psm_state_t)pdInstance->psmVDMBusyWaitDpmMsg;
                }
                break;

            case PSM_PE_SRC_IMPLICIT_CABLE_SOFT_RESET: /* B */
                if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgSop == sop) &&
                    (triggerInfo->pdMsgType == kPD_MsgAccept))
                {
                    PD_TimerClear(pdInstance, tSenderResponseTimer);
                    secondNewState = PSM_PE_SRC_VDM_IDENTITY_REQUEST;
                }
                else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                {
                    secondNewState = PSM_IDLE;
                }
                else
                {
                    didNothingSecond = 1;
                }
                break;

            case PSM_PE_SRC_VDM_IDENTITY_REQUEST: /* B */
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_DiscoverIdentity, triggerInfo, tVDMResponseTimer,
                    PSM_PE_SRC_VDM_IDENTITY_ACKED, PSM_PE_SRC_VDM_IDENTITY_NAKED);
#endif
                break;

            case PSM_PE_DFP_UFP_VDM_IDENTITY_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_DiscoverIdentity, triggerInfo, tVDMResponseTimer,
                    PSM_PE_DFP_UFP_VDM_IDENTITY_ACKED, PSM_PE_DFP_UFP_VDM_IDENTITY_NAKED);
                break;

            case PSM_PE_DFP_VDM_SVIDS_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_DiscoverSVIDs, triggerInfo, tVDMResponseTimer, PSM_PE_DFP_VDM_SVIDS_ACKED,
                    PSM_PE_DFP_VDM_SVIDS_NAKED);
                break;

            case PSM_PE_DFP_VDM_MODES_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_DiscoverModes, triggerInfo, tVDMResponseTimer, PSM_PE_DFP_VDM_MODES_ACKED,
                    PSM_PE_DFP_VDM_MODES_NAKED);
                break;

            case PSM_PE_DFP_VDM_MODE_ENTRY_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_EnterMode, triggerInfo, tVDMModeEntryTimer, PSM_PE_DFP_VDM_MODE_ENTRY_ACKED,
                    PSM_PE_DFP_VDM_MODE_ENTRY_NAKED);
                break;

            /* The Responder shall not return a BUSY acknowledgement and shall
               only return a NAK acknowledgement to a request not containing an Active Mode (i.e. Invalid object
               position). An
               Initiator which fails to receive an ACK within tVDMWaitModeExit or receives a NAK or BUSY response shall
               exit its
               Active Mode. */
            case PSM_PE_DFP_VDM_MODE_EXIT_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_ExitMode, triggerInfo, tVDMModeExitTimer, PSM_PE_DFP_VDM_MODE_EXIT_ACKED,
                    PSM_PE_DFP_VDM_MODE_EXIT_HARD_RESET);
                break;

            case PSM_PE_DFP_VDM_ATTENTION_REQUEST: /* B */
                secondNewState = PSM_IDLE;
                break;

            case PSM_PE_VENDOR_STRUCTURED_VDM_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command, triggerInfo,
                    tVDMResponseTimer, PSM_PE_VENDOR_STRUCTURED_VDM_ACKED, PSM_PE_VENDOR_STRUCTURED_VDM_NAKED);
                break;

            default:
                break;
        }

        if ((secondNewState != PSM_UNKNOWN) && (secondNewState != pdInstance->psmSecondaryState[statIndex]))
        {
            pdInstance->psmNewSecondaryState[statIndex] = secondNewState;
            continue;
        }

        return didNothingSecond;
    }
}

#endif

static uint8_t PD_PsmStateMachine(pd_instance_t *pdInstance)
{
    uint8_t returnVal = kSM_Continue;

    switch (pdInstance->dpmStateMachine)
    {
        case 0:
            /* wait vbus charge */
            /* Neither Source nor Sink should enter PSM before VSafe5V is available */
            if (PD_PsmCheckOnlyVbus(pdInstance))
            {
                pdInstance->dpmStateMachine = 1;
                pdInstance->psmNewState     = PSM_IDLE;
                PD_PsmStateMachine(pdInstance);
            }
            break;

        case 1:
            /* Did we do anything in Step A or B or C */
            /* Step A: common process */
            /* Step B: state is not changed */
            /* Step C: change to new state */

            do
            {
                returnVal = PD_PsmEnterState(pdInstance);
                returnVal = PD_PsmDisconnectCheck(pdInstance, returnVal);
                if ((returnVal != kSM_ErrorRecovery) && (returnVal != kSM_Detach))
                {
                    returnVal = PD_PsmProcessState(pdInstance);
                    returnVal = PD_PsmDisconnectCheck(pdInstance, returnVal);
                }
            } while (((pdInstance->psmCurState != pdInstance->psmNewState) || (returnVal == kSM_Continue)) &&
                     (returnVal != kSM_ErrorRecovery) && (returnVal != kSM_Detach));

            if ((returnVal == kSM_ErrorRecovery) || (returnVal == kSM_Detach))
            {
                PD_TimerCancelAllTimers(pdInstance, _tStartTimer, _tMaxDpmTimer);
            }
            break;

        default:
            break;
    }

    return returnVal;
}

void PD_StackStateMachine(pd_instance_t *pdInstance)
{
    uint32_t taskEventSet = 0;
    TypeCState_t connectStatus;
    uint8_t smState = 0;
    uint8_t connected;

    if (pdInstance->initializeLabel == 0)
    {
        pdInstance->initializeLabel = 1;
        pdInstance->isConnected     = 0;
        /* We want to override any existing MTP-based connection */
        PD_ConnectInitRole(pdInstance, 0);
        pdInstance->connectedResult = PD_ConnectGetStateMachine(pdInstance);
        PD_DpmSetVconn(pdInstance, 0);
    }
#if (defined PD_CONFIG_COMMON_TASK) && (PD_CONFIG_COMMON_TASK)
    USB_OsaEventCheck(pdInstance->taskEventHandle, 0xffffu, &taskEventSet);
#else
    USB_OsaEventWait(pdInstance->taskEventHandle, 0xffffu, 0, pdInstance->taskWaitTime, &taskEventSet);
#endif
/* if waiting for ever and no event */
#if defined USB_STACK_BM
    if ((pdInstance->taskWaitTime == 0) && (taskEventSet == 0u))
    {
        return;
    }
#endif
    pdInstance->taskWaitTime = PD_WAIT_EVENT_TIME;
    PD_PortTaskEventProcess(pdInstance, taskEventSet);
    if (taskEventSet & PD_TASK_EVENT_TYPEC_STATE_PROCESS)
    {
        USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_TYPEC_STATE_PROCESS);
    }

    /* Process Type-C state change by PD stack */
    connectStatus = PD_ConnectGetStateMachine(pdInstance);
    if (connectStatus != pdInstance->connectedResult)
    {
        pdInstance->connectedResult = connectStatus;
    }

    /* Process Type-C state change by Type-C state machine */
    connected     = PD_ConnectCheck(pdInstance);
    connectStatus = PD_ConnectGetStateMachine(pdInstance);
    if (connected != kConnectState_NotStable)
    {
        if (connected == kConnectState_Connected)
        {
            connected = 1;
        }
        else
        {
            connected = 0;
        }
        /* connect state change */
        if ((connected != pdInstance->isConnected) ||
            ((pdInstance->isConnected) && (connectStatus != pdInstance->connectedResult)))
        {
            if (connected)
            {
                switch (connectStatus)
                {
                    case TYPEC_ATTACHED_SRC:
                    case TYPEC_ATTACHED_SNK:
                        pdInstance->pendingSOP = kPD_MsgSOPMask;
                        if (connectStatus == TYPEC_ATTACHED_SRC)
                        {
                            pdInstance->curPowerRole            = kPD_PowerRoleSource;
                            pdInstance->initialPowerRole        = kPD_PowerRoleSource;
                            pdInstance->curDataRole             = kPD_DataRoleDFP;
                            pdInstance->psmPresentlyVconnSource = kPD_IsVconnSource;
                        }
                        else
                        {
                            pdInstance->curPowerRole            = kPD_PowerRoleSink;
                            pdInstance->initialPowerRole        = kPD_PowerRoleSink;
                            pdInstance->curDataRole             = kPD_DataRoleUFP;
                            pdInstance->psmPresentlyVconnSource = kPD_NotVconnSource;
                        }
                        break;

                    case TYPEC_AUDIO_ACCESSORY:
                    case TYPEC_UNORIENTED_DEBUG_ACCESSORY_SRC:
                    case TYPEC_DEBUG_ACCESSORY_SNK:
                    case TYPEC_ORIENTED_DEBUG_ACCESSORY_SNK:
                    case TYPEC_POWERED_ACCESSORY:
                        /* do nothing */
                        /* pdInstance->curPowerRole = kPD_PowerRoleSource; */
                        /* pdInstance->curDataRole = kPD_DataRoleDFP; */
                        break;

#if (defined PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
                    case TYPEC_ORIENTED_DEBUG_ACCESSORY_SRC:
                        if (PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_NONE)
                        {
                            /* Wait for disconnection */
                        }
                        else
                        {
                            pdInstance->curPowerRole            = kPD_PowerRoleSource;
                            pdInstance->initialPowerRole        = kPD_PowerRoleSource;
                            pdInstance->curDataRole             = kPD_DataRoleDFP;
                            pdInstance->psmPresentlyVconnSource = kPD_IsVconnSource;
                        }
                        break;
#endif
                    default:
                        break;
                }

                /* change from disconnect to connect */
                if ((!(pdInstance->isConnected)) && (connected))
                {
                    PD_DpmAppCallback(pdInstance, PD_CONNECTED, NULL, 0);
                    PD_PsmConnect(pdInstance);
                }
                /* still connect, but role change. For example: Try.SRC */
                else if ((pdInstance->isConnected) && (connected) && (connectStatus != pdInstance->connectedResult))
                {
                    PD_DpmAppCallback(pdInstance, PD_CONNECT_ROLE_CHANGE, NULL, 0);
                }
                else
                {
                }
            }
            /* change from connect to disconnect */
            else if ((pdInstance->isConnected) && (!(connected)))
            {
                PD_DpmAppCallback(pdInstance, PD_DISCONNECTED, NULL, 0);
                PD_PsmDisconnect(pdInstance);
                pdInstance->initializeLabel = 0;
                PD_StackSetEvent(pdInstance, PD_TASK_EVENT_RESET_CONFIGURE);
            }
            else
            {
            }

            pdInstance->isConnected = connected;
        }
        else if (pdInstance->isConnected) /* connect result stable and connected */
        {
            pdInstance->noConnectButVBusExit = 0;
            smState                          = PD_PsmStateMachine(pdInstance);

            /* Force a partner disconnect if the PSM has requested a error recovery */
            if ((smState == kSM_ErrorRecovery) || (smState == kSM_Detach))
            {
                pdInstance->isConnected = 0;
                PD_DpmAppCallback(pdInstance, PD_DISCONNECTED, NULL, 0);
                PD_PsmDisconnect(pdInstance);
                if (smState == kSM_ErrorRecovery)
                {
                    PD_ConnectInitRole(pdInstance, 1);
                }
                pdInstance->initializeLabel = 0;
                PD_StackSetEvent(pdInstance, PD_TASK_EVENT_RESET_CONFIGURE);
                pdInstance->taskWaitTime = PD_WAIT_EVENT_TIME;
            }
            else if (smState == kSM_Continue)
            {
                pdInstance->taskWaitTime = PD_WAIT_EVENT_TIME;
            }
            else
            {
                pdInstance->taskWaitTime = 0; /* wait for ever */
            }
        }
        else
        {
        }

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        /* check FR_Swap signal */
        if (USB_OsaEventCheck(pdInstance->taskEventHandle, PD_TASK_EVENT_FR_SWAP_SINGAL, NULL) ==
            kStatus_USB_OSA_Success)
        {
            if (!(pdInstance->isConnected))
            {
                USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_FR_SWAP_SINGAL);

                /* when receiving FR_Swap signal, PTN5110 will open source vbus defaultly.  */
                if (pdInstance->pdConfig->phyType == kPD_PhyPTN5110)
                {
                    pdInstance->callbackFns->PD_SrcTurnOffVbus(pdInstance->callbackParam, kVbusPower_Stable);
                }
            }
        }

        /* check unreasonable situation */
        if (!(pdInstance->isConnected))
        {
            if (PD_PsmCheckVbus5V(pdInstance))
            {
                pdInstance->noConnectButVBusExit++;
                PD_PsmTurnOffVconnAndVbus(pdInstance, kVbusPower_Stable);
                if (pdInstance->noConnectButVBusExit >= 100)
                {
                    pd_phy_config_t phyConfig;

                    pdInstance->noConnectButVBusExit = 0;
                    pdInstance->phyInterface->pdPhyDeinit(pdInstance->pdPhyHandle);
                    phyConfig.interface      = pdInstance->pdConfig->phyInterface;
                    phyConfig.interfaceParam = pdInstance->pdConfig->interfaceParam;
                    pdInstance->phyInterface->pdPhyInit(pdInstance, &(pdInstance->pdPhyHandle), &phyConfig);
                }
            }
        }
        else
        {
            pdInstance->noConnectButVBusExit = 0;
        }
#endif
    }
    pdInstance->connectedResult = connectStatus;

#if 0
    /* unclear error recovery */
    connectStatus = PD_ConnectGetStateMachine(pdInstance);
    if (connectStatus == PD_ConnectGetInitRoleState(pdInstance))
    {
        pdInstance->typeCStateNeedRecovery++;
    }
    else
    {
        pdInstance->typeCStateNeedRecovery = 0;
    }
    if ((!(pdInstance->isConnected)) && (pdInstance->typeCStateNeedRecovery >= 100))
    {
        pdInstance->typeCStateNeedRecovery = 0;
        PD_ConnectInitRole(pdInstance, 1);
    }
#endif
}

void PD_DpmDischargeVbus(pd_instance_t *pdInstance, uint8_t enable)
{
    PD_PhyControl(pdInstance, PD_PHY_DISCHARGE_VBUS, &enable);
}

#ifdef USBPD_ENABLE_VCONN_DISCHARGE
void PD_DpmDischargeVconn(pd_instance_t *pdInstance, uint8_t enable)
{
    PD_PhyControl(pdInstance, PD_PHY_DISCHARGE_VCONN, &enable);
}

void PD_DpmSetVconn(pd_instance_t *pdInstance, uint8_t enable)
{
    if (pdInstance->pdPowerPortConfig->vconnSupported)
    {
        PD_PhyControl(pdInstance, PD_PHY_CONTROL_VCONN, &enable);
    }
}
#endif

/* change to command and is async */
pd_status_t PD_Command(pd_handle pdHandle, uint32_t command, void *param)
{
    pd_status_t status        = kStatus_PD_Success;
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    /* common process */
    switch (command)
    {
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
        case PD_DPM_CONTROL_DISCOVERY_IDENTITY:
        case PD_DPM_CONTROL_DISCOVERY_SVIDS:
        case PD_DPM_CONTROL_DISCOVERY_MODES:
        {
            pdInstance->structuredVdmCommandParameter = *((pd_svdm_command_param_t *)param);

            pdInstance->structuredVdmCommandParameter.vdoCount                        = 0;
            pdInstance->structuredVdmCommandParameter.vdoData                         = NULL;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.commandType = kVDM_Initiator;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos      = 0;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.vdmVersion = PD_CONFIG_STRUCTURED_VDM_VERSION;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.vdmType    = 1;
            if ((command == PD_DPM_CONTROL_DISCOVERY_IDENTITY) || (command == PD_DPM_CONTROL_DISCOVERY_SVIDS))
            {
                pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID = PD_STANDARD_ID;
            }
            break;
        }

        case PD_DPM_SEND_VENDOR_STRUCTURED_VDM:
        case PD_DPM_CONTROL_ENTER_MODE:
        case PD_DPM_CONTROL_EXIT_MODE:
        case PD_DPM_CONTROL_SEND_ATTENTION:
        {
            pdInstance->structuredVdmCommandParameter = *((pd_svdm_command_param_t *)param);

            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.commandType = kVDM_Initiator;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.vdmVersion = PD_CONFIG_STRUCTURED_VDM_VERSION;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.vdmType    = 1;
            if (command == PD_DPM_CONTROL_EXIT_MODE)
            {
                pdInstance->structuredVdmCommandParameter.vdoCount = 0;
                pdInstance->structuredVdmCommandParameter.vdoData  = NULL;
            }
            break;
        }
#endif

        default:
            break;
    }

    /* special process */
    switch (command)
    {
        case PD_DPM_CONTROL_POWER_NEGOTIATION:
        {
            if ((pdInstance->psmCurState != PSM_PE_SRC_READY) || (pdInstance->curPowerRole != kPD_PowerRoleSource))
            {
                status = kStatus_PD_Error;
            }
            break;
        }

        case PD_DPM_CONTROL_REQUEST:
        {
            if ((pdInstance->psmCurState == PSM_PE_SNK_READY) && (pdInstance->curPowerRole == kPD_PowerRoleSink))
            {
                pdInstance->rdoRequest.rdoVal = *((uint32_t *)param);
                status                        = kStatus_PD_Success;
            }
            else
            {
                status = kStatus_PD_Error;
            }
            break;
        }

        case PD_DPM_CONTROL_GOTO_MIN:
        {
            if (pdInstance->psmCurState != PSM_PE_SRC_READY)
            {
                status = kStatus_PD_Error;
            }
            break;
        }

#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
        case PD_DPM_CONTROL_PR_SWAP:
        {
            if (((pdInstance->psmCurState != PSM_PE_SNK_READY) && (pdInstance->psmCurState != PSM_PE_SRC_READY)) ||
                (!PD_PsmIsDualRole(pdInstance)))
            {
                status = kStatus_PD_Error;
            }
            break;
        }
#endif

#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
        case PD_DPM_CONTROL_DR_SWAP:
        {
            if (pdInstance->pdPowerPortConfig->dataFunction != kDataConfig_DRD)
            {
                status = kStatus_PD_Error;
            }
            break;
        }
#endif

#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
        case PD_DPM_CONTROL_VCONN_SWAP:
        {
            if (!(pdInstance->pdPowerPortConfig->vconnSupported))
            {
                status = kStatus_PD_Error;
            }
            break;
        }
#endif

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        case PD_DPM_GET_SRC_EXT_CAP:
#endif
        case PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES:
        {
            if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceOnly)
            {
                status = kStatus_PD_Error;
            }
            break;
        }

        case PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES:
        {
            if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkOnly)
            {
                status = kStatus_PD_Error;
            }
            break;
        }

        case PD_DPM_CONTROL_SOFT_RESET:
            pdInstance->psmSoftResetSop = *((uint8_t *)param);
            break;

        case PD_DPM_CONTROL_HARD_RESET:
            break;

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
        case PD_DPM_CONTROL_DISCOVERY_IDENTITY:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_DiscoverIdentity;
            break;
        }

        case PD_DPM_CONTROL_DISCOVERY_SVIDS:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_DiscoverSVIDs;
            break;
        }

        case PD_DPM_CONTROL_DISCOVERY_MODES:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_DiscoverModes;
            break;
        }

        case PD_DPM_CONTROL_SEND_ATTENTION:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_Attention;
            break;
        }

        case PD_DPM_SEND_VENDOR_STRUCTURED_VDM:
        {
            break;
        }

        case PD_DPM_CONTROL_ENTER_MODE:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_EnterMode;

            status = kStatus_PD_Success;
            if (pdInstance->curDataRole == kPD_DataRoleUFP)
            {
                status = kStatus_PD_Error;
            }
            break;
        }

        case PD_DPM_CONTROL_EXIT_MODE:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_ExitMode;

            status = kStatus_PD_Success;
            if (pdInstance->curDataRole == kPD_DataRoleUFP)
            {
                status = kStatus_PD_Error;
            }
            break;
        }

        case PD_DPM_SEND_UNSTRUCTURED_VDM:
        {
            pd_unstructured_vdm_header_t *unstructuredVDMHeader;
            pdInstance->unstructuredVdmCommandParameter = *((pd_unstructured_vdm_command_param_t *)param);
            unstructuredVDMHeader =
                (pd_unstructured_vdm_header_t *)&(pdInstance->unstructuredVdmCommandParameter.vdmHeaderAndVDOsData[0]);
            unstructuredVDMHeader->bitFields.vdmType = 0;
            break;
        }
#endif

        case PD_DPM_CONTROL_CABLE_RESET:
            break;

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        case PD_DPM_GET_STATUS:
            break;

        case PD_DPM_GET_BATTERY_CAP:
        case PD_DPM_GET_BATTERY_STATUS:
            pdInstance->getBatteryCapDataBlock = *((uint8_t *)param);
            break;

        case PD_DPM_GET_MANUFACTURER_INFO:
#if 0
        case PD_DPM_SECURITY_REQUEST:
        case PD_DPM_FIRMWARE_UPDATE_REQUEST:
#endif
            pdInstance->commandExtParam = *((pd_command_data_param_t *)param);
            break;

        case PD_DPM_ALERT:
            pdInstance->alertADO = *((uint32_t *)param);
            break;
#endif

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        case PD_DPM_FAST_ROLE_SWAP:
            if ((pdInstance->psmCurState != PSM_PE_SRC_READY) || (pdInstance->curPowerRole != kPD_PowerRoleSource) ||
                (!PD_PsmIsDualRole(pdInstance)))
            {
                status = kStatus_PD_Error;
            }
            break;
#endif

        default:
            status = kStatus_PD_Error;
            break;
    }

    if (status == kStatus_PD_Success)
    {
/* (1). PTN5110 need signal fr_swap before turn off vbus, otherwise signal will fail
 * (2). "enter the PE_FRS_SRC_SNK_CC_Signal state" after "signal fr_swap" need be atomic.
 *  the psmNewState need change in the same task context, it is the PD task. here
 *  will not change it. So the (2) is not satisfied.
 */
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        if (command == PD_DPM_FAST_ROLE_SWAP)
        {
            /* fr swap msg is received but the state may not be in PE_FRS_SRC_SNK_CC_Signal,
             * frSwapReceived avoid lost the fr swap msg if not in the PE_FRS_SRC_SNK_CC_Signal state */
            pdInstance->frSwapReceived = 0;
            /* frs signal is sent and wait the frs msg.
             * "(1) The PD_DPM_FAST_ROLE_SWAP is processed by PD task" and "(2) received the fr swap msg"
             * (1) may be ahead (2); (2) may be ahead (1).
             * frSignaledWaitFrSwap is used to enter PE_FRS_SRC_SNK_CC_Signal only in one place.
             */
            pdInstance->frSignaledWaitFrSwap = 1;
            PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InFRSwap);
            PD_PhyControl(pdInstance, PD_PHY_SIGNAL_FR_SWAP, NULL);
        }
#endif
        PD_DpmSendMsg(pdHandle, command);
    }

    return status;
}

pd_status_t PD_Control(pd_handle pdHandle, uint32_t controlCode, void *param)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    pd_status_t status        = kStatus_PD_Success;
    uint32_t caseValue        = 0;

    caseValue = caseValue;
    switch (controlCode)
    {
        case PD_CONTROL_GET_TYPEC_CURRENT_VALUE:
            *((uint8_t *)param) = pdInstance->pdPowerPortConfig->typecSrcCurrent;
            break;

        case PD_CONTROL_GET_PD_STATE:
            if (PD_TimerBusy(pdHandle) || PD_StackHasPendingEvent(pdInstance))
            {
                *((uint8_t *)param) = kState_Busy;
            }
            else
            {
                *((uint8_t *)param) = kState_Idle;
            }
            break;

        case PD_CONTROL_GET_TYPEC_ORIENTATION:
            if (pdInstance->ccUsed == kPD_CC1)
            {
                *((uint8_t *)param) = 0;
            }
            else
            {
                *((uint8_t *)param) = 1;
            }
            break;

        case PD_CONTROL_GET_POWER_ROLE:
            *((uint8_t *)param) = pdInstance->curPowerRole;
            break;

        case PD_CONTROL_GET_DATA_ROLE:
            *((uint8_t *)param) = pdInstance->curDataRole;
            break;

        case PD_CONTROL_GET_TYPEC_CONNECT_STATE:
            *((uint8_t *)param) = pdInstance->connectState;
            break;

        case PD_CONTROL_GET_VCONN_ROLE:
            *((uint8_t *)param) = pdInstance->psmPresentlyVconnSource;
            break;

        case PD_CONTROL_GET_SNK_TYPEC_CURRENT_CAP:
            PD_PhyControl(pdInstance, PD_PHY_SNK_GET_TYPEC_CURRENT_CAP, param);
            break;

        case PD_CONTROL_PHY_POWER_PIN:
            PD_PhyControl(pdInstance, PD_PHY_CONTROL_POWER_PIN, param);
            break;

        case PD_CONTROL_VCONN:
            PD_PhyControl(pdInstance, PD_PHY_CONTROL_VCONN, param);
            break;

        case PD_CONTROL_GET_PHY_LOW_POWER_STATE:
            status = PD_PhyControl(pdInstance, PD_PHY_GET_LOWPOWER_STATE, param);
            break;

        case PD_CONTROL_DISCHARGE_VBUS:
        {
            uint32_t tmp32Val = 1;
            PD_PhyControl(pdInstance, PD_PHY_DISCHARGE_VBUS, &tmp32Val);
#if (defined PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE) && (PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE)
            PD_TimerStart(pdInstance, tTypeCVbusMinDischargeTimer, T_MIN_VBUS_DISCHARGE);
            while (!PD_TimerCheckValidTimeOut(pdInstance, tTypeCVbusMinDischargeTimer))
#else
            PD_TimerStart(pdInstance, tTypeCVbusMaxDischargeTimer, T_MAX_VBUS_DISCHARGE);
            while (!PD_TimerCheckValidTimeOut(pdInstance, tTypeCVbusMaxDischargeTimer))
#endif
            {
                if ((PD_PsmCheckVsafe0V(pdInstance)))
                {
                    break;
                }
            }
            tmp32Val = 0;
            PD_PhyControl(pdInstance, PD_PHY_DISCHARGE_VBUS, &tmp32Val);

            break;
        }

        case PD_CONTROL_INFORM_VBUS_VOLTAGE_RANGE:
            caseValue = *((uint32_t *)param);
            PD_PhyControl(pdInstance, PD_PHY_INFORM_VBUS_VOLTAGE_RANGE, &caseValue);
            break;

        case PD_CONTROL_GET_CABLE_INFO:
        {
            if (param == NULL)
            {
                break;
            }
            pd_cable_plug_info_t *plugInfo = (pd_cable_plug_info_t *)param;
            if (pdInstance->psmCableIdentitiesDataCount)
            {
                pd_id_header_vdo_t *headerVDO = (pd_id_header_vdo_t *)(&pdInstance->psmCableIdentities[1]);
                pd_structured_vdm_header_t *vdmHeader =
                    (pd_structured_vdm_header_t *)(&pdInstance->psmCableIdentities[0]);
                plugInfo->vdmVersion = vdmHeader->bitFields.vdmVersion;
                if (headerVDO->bitFields.productTypeUFPOrCablePlug == VDM_ID_HEADER_VDO_PASSIVE_CABLE_VAL)
                {
                    plugInfo->cableType = kCableType_PassiveCable;
                }

                if (headerVDO->bitFields.productTypeUFPOrCablePlug == VDM_ID_HEADER_VDO_ACTIVE_CABLE_VAL)
                {
                    plugInfo->cableType = kCableType_ActiveCable;
                }
                plugInfo->vdoValue = pdInstance->psmCableIdentities[4];
            }
            else
            {
                plugInfo->cableType = kCableType_Invalid;
            }
            break;
        }

#if 1
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
        case PD_CONTROL_INFORM_EXTERNAL_POWER_STATE:
            if (pdInstance->rdySeenExtPower != *((uint8_t *)param))
            {
                PD_PsmExternalPowerChange(pdInstance);
            }
            break;
#endif
#endif
#if (defined PD_CONFIG_ALT_MODE_DP_SUPPORT) && (PD_CONFIG_ALT_MODE_DP_SUPPORT)
        case PD_CONTROL_ALT_MODE:
            PD_AltModeControl(pdInstance->altModeHandle, param);
            break;
#endif

        default:
            break;
    }

    return status;
}

static pd_status_t PD_DpmAppCallback(pd_instance_t *pdInstance, uint32_t event, void *param, uint8_t done)
{
    if (done)
    {
        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
        {
            PD_MsgSrcEndCommand(pdInstance);
        }

/* in case the power is not reset to normal operation */
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        if (pdInstance->inProgress != kVbusPower_InFRSwap)
#endif
        {
            PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
        }
        pdInstance->commandProcessing = 0;
    }
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
    PD_AltModeCallback(pdInstance, event, param);
#endif
    return pdInstance->pdCallback(pdInstance->callbackParam, event, param);
}

void PD_DpmAltModeCallback(pd_handle pdHandle, uint32_t event, void *param)
{
    ((pd_instance_t *)(pdHandle))->pdCallback(((pd_instance_t *)(pdHandle))->callbackParam, event, param);
}

#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
uint8_t PD_CheckWhetherInitiateCableDiscoveryIdentityOrNot(pd_instance_t *pdInstance)
{
    if (((pdInstance->raPresent) || (pdInstance->initialPowerRole == kPD_PowerRoleSink)) &&
        (pdInstance->psmPresentlyVconnSource) && (pdInstance->psmCableIdentitiesDataCount == 0) &&
        (pdInstance->cableDiscoverIdentityCounter < N_DISCOVER_IDENTITY_COUNTER))
    {
        return 1;
    }
    return 0;
}
#endif
