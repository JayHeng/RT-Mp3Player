/*
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PD_INTERFACE_H__
#define __PD_INTERFACE_H__

#include "usb_osa.h"
#include "usb_cmsis_wrapper.h"
#include "usb_pd_spec.h"

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

typedef enum
{
    CONFIG_DEBUG_ACCESSORY_NONE = 0,
    CONFIG_DEBUG_ACCESSORY_TS   = 1,
    CONFIG_DEBUG_ACCESSORY_DTS  = 2,
} pd_debug_acc_role_t;

#define PSM_SECONDARY_STATE_COUNT 3

#define PD_CONFIG_DEBUG_ACCESSORY_ROLE (CONFIG_DEBUG_ACCESSORY_DTS)

#define PD_WAIT_EVENT_TIME (0)

#define PD_TRY_GET_CABLE_INFO_COUNT (3)

/* private */
typedef enum _pd_task_event_type
{
    PD_TASK_EVENT_RECEIVED_HARD_RESET   = 0x01u,
    PD_TASK_EVENT_PD_MSG                = 0x02u,
    PD_TASK_EVENT_DPM_MSG               = 0x04u,
    PD_TASK_EVENT_SEND_DONE             = 0x08u,
    PD_TASK_EVENT_TIME_OUT              = 0x10u,
    PD_TASK_EVENT_PHY_STATE_CHAGNE      = 0x20u,
    PD_TASK_EVENT_OTHER                 = 0x40u,
    PD_TASK_EVENT_FR_SWAP_SINGAL        = 0x80u,
    PD_TASK_EVENT_TYPEC_STATE_PROCESS   = 0x100u,
    PD_TASK_EVENT_RESET_CONFIGURE       = 0x200u,
    PD_TASK_EVENT_EXTERNAL_POWER_CHANGE = 0x400u,
} pd_task_event_type_t;

typedef enum _pd_auto_policy_state
{
    PSM_RDY_EVAL_INIT = 0,
    PSM_RDY_EVAL_GET_SNK_CAP,
    PSM_RDY_EVAL_CHECK_PARTNER_CAP,
    PSM_RDY_EVAL_SWAP_TO_SRC,
    PSM_RDY_EVAL_CHECK_SWAP_TO_SRC,
    PSM_RDY_EVAL_SWAP_TO_SNK,
    PSM_RDY_EVAL_CHECK_SWAP_TO_SNK,
    PSM_RDY_EVAL_DR_SWAP,
    PSM_RDY_EVAL_CHECK_DR_SWAP,
    PSM_RDY_EVAL_VCONN_SWAP,
    PSM_RDY_EVAL_CHECK_VCONN_SWAP,
    PSM_RDY_EVAL_IDLE,
    PSM_RDY_DELAY_FLAG = 0x80, // Delay before each step.
} pd_auto_policy_state_t;

typedef enum _pd_connect_state
{
    kConnectState_NotStable,
    kConnectState_Connected,
    kConnectState_Disconnected,
} pd_connect_state_t;

typedef struct _pd_instance
{
    pd_phy_handle pdPhyHandle;
#if ((defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT))
    void *altModeHandle;
#endif
    pd_instance_config_t *pdConfig;
    pd_power_port_config_t *pdPowerPortConfig;
    const pd_phy_api_interface_t *phyInterface;
    pd_stack_callback_t pdCallback;
    pd_power_handle_callback_t *callbackFns;
    void *callbackParam;
    usb_osa_event_handle taskEventHandle;

    /* PD state machine */
    pd_rdo_t rdoRequest;
    pd_rdo_t partnerRdoRequest;
    pd_source_pdo_t selfOrPartnerFirstSourcePDO;
    pd_source_pdo_t partnerSourcePDOs[7];
    pd_phy_vendor_info_t phyInfo;
    uint32_t psmCableIdentities[7]; /* e-marked cable */
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    pd_sink_pdo_t portPartnerSinkPDO1;
#endif
    /* timr */
    volatile uint32_t timrsRunningState[(PD_MAX_TIMER_COUNT + 31) / 32];
    volatile uint32_t timrsTimeOutState[(PD_MAX_TIMER_COUNT + 31) / 32];
    /* pd msg process */
    uint8_t *receivingDataBuffer;
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    uint32_t receivingData[67];
    uint32_t receivingChunkedData[8];
#else
    uint32_t receivingData[8];
#endif
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    uint32_t sendingData[67];
#else
    uint32_t sendingData[8];
#endif
    volatile uint32_t *receivedData;
    volatile uint32_t receivedLength;
    /* DPM commands */
    uint32_t dpmMsgBits;
#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE))
    pd_svdm_command_param_t structuredVdmCommandParameter;
    pd_unstructured_vdm_command_param_t unstructuredVdmCommandParameter;
    uint32_t vdmExitReceived[3];
#endif
#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    pd_command_data_param_t commandExtParam;         /* command initiator's parameter */
    pd_command_data_param_t commandExtParamCallback; /* command callback parameter */
    uint32_t alertADO;
#endif

    /* PD state machine */
    uint16_t dpmCableMaxCurrent;
    volatile uint16_t taskWaitTime;
#if (defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT)
    uint16_t chunkNumBytesReceived;
#endif
    /* timr */
    volatile uint16_t timrsTimeValue[PD_MAX_TIMER_COUNT];

    /* PD state machine */
    TypeCState_t curConnectState; /* Type-C state machine */
    pd_psm_state_t psmCurState;
    pd_psm_state_t psmNewState;
    pd_psm_state_t psmInterruptedState;
    pd_psm_state_t psmDrSwapPrevState;
    pd_psm_state_t psmVconnSwapPrevState;
#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE))
    pd_psm_state_t psmSecondaryState[3];
    pd_psm_state_t psmNewSecondaryState[3];
#endif
#if (defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT)
    pd_chunking_layer_state_t chunkingRXState;
    pd_chunking_layer_state_t chunkingTXState;
#endif
    /* pd msg process */
    volatile pd_msg_header_t sendingMsgHeader;
    volatile pd_status_t receiveResult;
    volatile pd_status_t sendingResult;

    /* PD state machine */
    volatile uint8_t partnerSourcePDOsCount;
    uint8_t commandEvaluateResult;
    volatile uint8_t amsVdmReplyMsg;
    uint8_t vdmExitReceivedSOP[3];
    volatile uint8_t connectedResult;
    uint8_t dpmStateMachine;
    uint8_t connectState;
    uint8_t initialPowerRole;
    uint8_t curPowerRole;
    uint8_t curDataRole;
    uint8_t commandProcessing;
    uint8_t ccUsed;
    uint8_t inProgress; /* power progress state */
    uint8_t vbusDischargeInProgress;
    volatile uint8_t pendingSOP;
    uint8_t raPresent;
    uint8_t psmPresentlyVconnSource;
    uint8_t psmVdmActiveModeValidMask;
    uint8_t psmHardResetCount;
    uint8_t psmSendCapsCounter;
    uint8_t psmSoftResetSop;
    uint8_t revision;
    uint8_t psmCableIdentitiesDataCount;
    volatile uint8_t noConnectButVBusExit;
    /* 1. When Type-C connect to U-Disk, the attached.src check VBUS fail sometimes
     *    if the TrunONVbus application return too quick.
     * 2. if add delay in the TurnONVbus application function,
     *    it may influence the ellisys compliance test.
     * 3. Add this delay to avoid the VBUS (5V) not stable.
     */
    volatile uint8_t enterAttachedSRCFromTypeCStateMachine;
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
    uint8_t cableDiscoverIdentityCounter;
#endif
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    uint8_t autoPolicyState;
    uint8_t vconnSwapResult;
    uint8_t drSwapResult;
#endif
#if (defined PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT) && (PD_CONFIG_EXTENDED_CHUNK_LAYER_SUPPORT)
    uint8_t chunkNumberToSend;
    uint8_t chunkSentDone;
    uint8_t chunkNumberExpected;
#endif

    /* pd msg process */
    volatile uint8_t receiveState; /* 0 - no pending receive; 1 - receiving; 2 - received data */
    volatile uint8_t receivedSop;
    volatile uint8_t sendingState; /* 0 - no pending send; 1 - sending; 2 - send callback done */
    volatile uint8_t occupied;

/* DPM commands */
#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    uint8_t getBatteryCapDataBlock;
#endif
#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE))
    uint8_t psmVDMBusyWaitDpmMsg;
#endif
    volatile uint8_t commandIsInitiator;

#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
    uint8_t cableDiscoverIdentityTimeout : 1;
#endif
    /* msg process */
    volatile uint8_t hardResetReceived : 1;
    /* pd state machine */
    volatile uint8_t initializeLabel : 1;
    volatile uint8_t isConnected : 1;
    uint8_t trySNKState : 1;
    uint8_t psmGotoMinTx : 1;
    uint8_t psmGotoMinRx : 1;
    uint8_t enterTrySNKFromPoweredAcc : 1;
    uint8_t cc1Monitor : 1;
    uint8_t cc2Monitor : 1;
    uint8_t psmHardResetNeedsVSafe0V : 1;
    uint8_t psmPresentlyPdConnected : 1;
    uint8_t psmPreviouslyPdConnected : 1;
    uint8_t psmCablePlugResetNeeded : 1;
    uint8_t psmSnkReceiveRdoWaitRetry : 1;
    uint8_t psmExplicitContractExisted : 1;
    volatile uint8_t asmHardResetSnkProcessing : 1;
    uint8_t unchunkedFeature : 1;
    volatile uint8_t commandSrcOwner : 1;
    volatile uint8_t alertWaitReply : 1;
    volatile uint8_t fr5VOpened : 1;
    volatile uint8_t enableReceive : 1;
    volatile uint8_t enterSrcFromSwap : 1;
/* 1. fr_swap CC signal is sent && state is not in PE_FRS_SRC_SNK_CC_Signa.
 * 2. in case the message is not proccessed
 *    when the state machine is not in the source rdy state.
 */
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
    volatile uint8_t frSwapReceived : 1;
    /* need wait the fr swap msg after the fr signal */
    volatile uint8_t frSignaledWaitFrSwap : 1;
    volatile uint8_t frsEnabled : 1;
#endif
#if (defined PD_CONFIG_ENABLE_AUTO_POLICY) && (PD_CONFIG_ENABLE_AUTO_POLICY)
    uint8_t swapToSnkSrcCapReceived : 1;
    volatile uint8_t rdySeenExtPower : 1;
    volatile uint8_t portPartnerVconnSwapToOffRejected : 1;
    volatile uint8_t portPartnerVconnSwapToOnRejected : 1;
    volatile uint8_t portPartnerDrSwapToUFPRejected : 1;
    volatile uint8_t portPartnerDrSwapToDFPRejected : 1;
    volatile uint8_t portPartnerPrSwapToSinkRejected : 1;
    volatile uint8_t portPartnerPrSwapToSourceRejected : 1;
#endif
} pd_instance_t;

/*******************************************************************************
 * API
 ******************************************************************************/

void PD_WaitUsec(uint32_t us);

#endif
