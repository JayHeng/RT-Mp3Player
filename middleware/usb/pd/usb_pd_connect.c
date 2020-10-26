/*
 * Copyright 2015 - 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_timer.h"
#include "usb_pd_interface.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define AUDIO_ACC_DISCONNECT_DEBOUNCE 1
#define ACC_DISCONNECT_FIGURE 1

#define T_DRP_SRC_DUTY 60
#define T_DRP_SNK_DUTY (100 - (T_DRP_SRC_DUTY))
#define T_PD_DEBOUNCE mSec(12)      /* 10 ~ 20 ms */
#define T_CC_DEBOUNCE mSec(105)     /* 100 ~ 200 ms */
#define T_DRP mSec(70)              /* 50 ~ 100 ms */
#define T_DRP_TRY mSec(100)         /* 75ms ~ 150ms */
#define T_DRP_TRY_FOR_TRY_SNK (150) /* 75ms ~ 150ms */
#define T_DRP_TRY_WAIT mSec(310)    /* Use 300 as min in sim, Needs to have margin for HW CC debounce (real value) */
#define T_SINK_DISCONNECT mSec(1)   /* Select a small time to keep comptability with regression disconnect timing. */
#define T_DRP_TOGGLE_SRC (((T_DRP) * (T_DRP_SRC_DUTY)) / 100)
#define T_DRP_TOGGLE_SNK (((T_DRP) * (T_DRP_SNK_DUTY)) / 100)

typedef enum _vbus_discharge
{
    kVbus_NoDischarge = 0,
    kVbus_ApplyTypecDischarge,
    kVbus_TypecDischarge,
} vbus_discharge_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void PD_DpmDischargeVbus(pd_instance_t *pdInstance, uint8_t enable);
pd_status_t PD_PhyControl(pd_instance_t *pdInstance, uint32_t control, void *param);
static uint8_t PD_ConnectState(pd_instance_t *pdInstance);
void PD_StackSetEvent(pd_instance_t *pdInstance, uint32_t event);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static void PD_ConnectSetMonitorCC(pd_instance_t *pdInstance)
{
    if (pdInstance->ccUsed == kPD_CCInvalid)
    {
        pdInstance->cc1Monitor = 1;
        pdInstance->cc2Monitor = 1;
    }
    else if (pdInstance->ccUsed == kPD_CC1)
    {
        pdInstance->cc1Monitor = 1;
        pdInstance->cc2Monitor = 0;
    }
    else
    {
        pdInstance->cc1Monitor = 0;
        pdInstance->cc2Monitor = 1;
    }
}

static void PD_ConnectSetAttachDetect(pd_instance_t *pdInstance, uint8_t isDRP, uint8_t powerRole)
{
    pd_attach_detection_param_t attachDetecParam;

    attachDetecParam.isDRP        = isDRP;
    attachDetecParam.powerRole    = powerRole;
    attachDetecParam.deviceType   = pdInstance->pdConfig->deviceType;
    attachDetecParam.srcRpCurrent = pdInstance->pdPowerPortConfig->typecSrcCurrent;
    PD_PhyControl(pdInstance, PD_PHY_CONFIG_ATTACH_DETECTION, &attachDetecParam);
}

static void PD_ConnectSetDetachDetect(pd_instance_t *pdInstance,
                                      pd_detach_detection_param_t *detachDetecParam,
                                      uint8_t powerRole,
                                      uint8_t usedCC)
{
    detachDetecParam->powerRole             = powerRole;
    detachDetecParam->typecConnectState     = pdInstance->connectState;
    detachDetecParam->usedCC                = usedCC;
    detachDetecParam->srcRpCurrent          = pdInstance->pdPowerPortConfig->typecSrcCurrent;
    detachDetecParam->snkDetachDetectCCOpen = 0;
    PD_PhyControl(pdInstance, PD_PHY_CONFIG_DETACH_DETECTION, detachDetecParam);
}

static void PD_ConnectCheckRa(pd_instance_t *pdInstance, pd_phy_cc_state_t cc1State, pd_phy_cc_state_t cc2State)
{
    if (cc1State == kCCState_SrcRa)
    {
        pdInstance->raPresent = 1;
    }
    else if (cc2State == kCCState_SrcRa)
    {
        pdInstance->raPresent = 2;
    }
    else
    {
        pdInstance->raPresent = 0;
    }
}

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
static void PD_ConnectProcessVconn(pd_instance_t *pdInstance)
{
    if (pdInstance->raPresent)
    {
        if (pdInstance->callbackFns->PD_ControlVconn != NULL)
        {
            pdInstance->callbackFns->PD_ControlVconn(pdInstance->callbackParam, 1);
        }
    }
}
#endif

static void PD_ConnectSetupNewState(pd_instance_t *pdInstance)
{
    uint8_t writeRegsNeeded = 1;
    uint8_t controlVal;
    enum
    {
        VCONN_NO_CHANGE,
        VCONN_FORCE_OFF,
        VCONN_FORCE_ON,
    } vconn_en = VCONN_FORCE_OFF;
    pd_detach_detection_param_t detachDetecParam;

    /* Stop sourcing Vconn before asserting Rd */
    switch (pdInstance->curConnectState)
    {
        case TYPEC_ERROR_RECOVERY:
        case TYPEC_DISABLED:
        case TYPEC_UNATTACHED_SRC:
        case TYPEC_UNATTACHED_SNK:
        case TYPEC_UNATTACHED_ACCESSORY:
        case TYPEC_TRY_WAIT_SNK:
        case TYPEC_TRY_SNK:
        case TYPEC_TRY_SRC:
        case TYPEC_TRY_WAIT_SRC:
        case TYPEC_TOGGLE_SRC_FIRST:
        case TYPEC_TOGGLE_SNK_FIRST:
            if (pdInstance->vbusDischargeInProgress == kVbus_ApplyTypecDischarge)
            {
                if (pdInstance->callbackFns->PD_ControlVconn != NULL)
                {
                    pdInstance->callbackFns->PD_ControlVconn(pdInstance->callbackParam, 0);
                }
            }
            break;
        default:
            break;
    }

    /* State specific actions */
    switch (pdInstance->curConnectState)
    {
        case TYPEC_ERROR_RECOVERY:
        case TYPEC_DISABLED:
            PD_PhyControl(pdInstance, PD_PHY_RESET_CONNECT_DETECTION, NULL);
            break;

#if ((defined PD_CONFIG_SINK_ACCESSORY_SUPPORT) && (PD_CONFIG_SINK_ACCESSORY_SUPPORT)) || \
    ((defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE))
        case TYPEC_UNATTACHED_ACCESSORY:
        case TYPEC_UNATTACHED_SRC:
        case TYPEC_TOGGLE_SRC_FIRST:
        {
            if (pdInstance->curConnectState == TYPEC_TOGGLE_SRC_FIRST)
            {
#if (defined PD_CONFIG_PTN5100_PORT) && (PD_CONFIG_PTN5100_PORT)
                if (pdInstance->pdConfig->phyType == kPD_PhyPTN5100)
                {
                    pdInstance->curConnectState = TYPEC_UNATTACHED_SRC;
                    PD_ConnectSetupNewState(pdInstance);
                    return;
                }
#endif
            }
            else
            {
                if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) &&
                    (pdInstance->pdConfig->phyType == kPD_PhyPTN5110))
                {
                    pdInstance->curConnectState = TYPEC_TOGGLE_SRC_FIRST;
                    PD_ConnectSetupNewState(pdInstance);
                    return;
                }
            }

            PD_ConnectSetAttachDetect(pdInstance,
                                      (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling),
                                      kPD_PowerRoleSource);
            if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) ||
                (pdInstance->curConnectState == TYPEC_UNATTACHED_ACCESSORY))
            {
                PD_TimerStart(pdInstance, tDRPToggleTimer, T_DRP_TOGGLE_SRC);
            }
            break;
        }
#endif

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
        case TYPEC_ATTACH_WAIT_SRC:
            controlVal = 1;
            /* Preserve discharge in this state - exit condition is dependent on VSafe0V */
            PD_PhyControl(pdInstance, PD_PHY_CONTROL_VBUS_DETECT, &controlVal);
            PD_TimerStart(pdInstance, tCCDebounceTimer, T_CC_DEBOUNCE);
            /* vconn_en = VCONN_NO_CHANGE; */
            writeRegsNeeded = 0;
            break;

        case TYPEC_ATTACHED_SRC:
            controlVal = (pdInstance->ccUsed);
            PD_PhyControl(pdInstance, PD_PHY_CONNECT_SET_CC, &controlVal);
            /* if put blow set PD_PHY_CONFIG_DETACH_DETECTION, the ex350 TD4.10.4 cannot pass */
            if ((pdInstance->inProgress != kVbusPower_InPRSwap) && (pdInstance->inProgress != kVbusPower_InFRSwap))
            {
                pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam, pdInstance->inProgress);
            }
            pdInstance->enterAttachedSRCFromTypeCStateMachine = 3;
            PD_TimerStart(pdInstance, tVBUSNotStableTimer, T_VBUS_NOT_STABLE);
            if ((pdInstance->inProgress != kVbusPower_InPRSwap) && (pdInstance->inProgress != kVbusPower_InFRSwap))
            {
                /* vconn need be powered up after 2ms of vbus is powered up */
                pdInstance->connectState = kTYPEC_ConnectSource;
                if (((pdInstance->ccUsed == kPD_CC2) && (pdInstance->raPresent == 1)) ||
                    ((pdInstance->ccUsed == kPD_CC1) && (pdInstance->raPresent == 2)))
                {
                    if (pdInstance->pdPowerPortConfig->vconnSupported)
                    {
                        controlVal = 1;
                        PD_PhyControl(pdInstance, PD_PHY_CONTROL_VCONN, &controlVal);
                    }
                }
            }
            vconn_en                 = VCONN_NO_CHANGE;
            pdInstance->connectState = kTYPEC_ConnectSource;
            PD_ConnectSetDetachDetect(pdInstance, &detachDetecParam, kPD_PowerRoleSource, (pdInstance->ccUsed));
            if ((pdInstance->inProgress != kVbusPower_InPRSwap) && (pdInstance->inProgress != kVbusPower_InFRSwap))
            {
                PD_ConnectProcessVconn(pdInstance);
            }
            break;
#endif

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
        case TYPEC_DEAD_BATTERY_SNK:
        {
            writeRegsNeeded = 0;
            break;
        }

        case TYPEC_TOGGLE_SNK_FIRST:
        case TYPEC_UNATTACHED_SNK:
        {
            if (pdInstance->curConnectState == TYPEC_TOGGLE_SNK_FIRST)
            {
#if (defined PD_CONFIG_PTN5100_PORT) && (PD_CONFIG_PTN5100_PORT)
                if (pdInstance->pdConfig->phyType == kPD_PhyPTN5100)
                {
                    pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
                    PD_ConnectSetupNewState(pdInstance);
                    return;
                }
#endif
            }
            else
            {
                if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) &&
                    (pdInstance->pdConfig->phyType == kPD_PhyPTN5110))
                {
                    pdInstance->curConnectState = TYPEC_TOGGLE_SNK_FIRST;
                    PD_ConnectSetupNewState(pdInstance);
                    return;
                }
            }

            PD_ConnectSetAttachDetect(
                pdInstance, (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling), kPD_PowerRoleSink);
#if (defined PD_CONFIG_PTN5100_PORT) && (PD_CONFIG_PTN5100_PORT)
            /* drp toggle and sink accessory toggle */
            if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) ||
                (PD_CONFIG_SINK_ACCESSORY_SUPPORT))
            {
                PD_TimerStart(pdInstance, tDRPToggleTimer, T_DRP_TOGGLE_SNK);
            }
#endif
            break;
        }

        case TYPEC_ATTACH_WAIT_SNK:
            controlVal = 1;
            /* Preserve discharge in this state - exit condition is dependent on VSafe0V */
            PD_PhyControl(pdInstance, PD_PHY_CONTROL_VBUS_DETECT, &controlVal);
            /* Set the CC debounce time once at state entry, will not be reset on CC change */
            PD_TimerStart(pdInstance, tCCDebounceTimer, T_CC_DEBOUNCE);
            writeRegsNeeded = 0;
            break;

        case TYPEC_ATTACHED_SNK:
            pdInstance->connectState = kTYPEC_ConnectSink;
            controlVal               = (pdInstance->ccUsed);
            PD_PhyControl(pdInstance, PD_PHY_CONNECT_SET_CC, &controlVal);
            PD_ConnectSetDetachDetect(pdInstance, &detachDetecParam, kPD_PowerRoleSink, (pdInstance->ccUsed));
            if ((pdInstance->inProgress == kVbusPower_InPRSwap) || (pdInstance->inProgress == kVbusPower_InFRSwap))
            {
                /* Don't change ra_present or vconn */
                vconn_en = VCONN_NO_CHANGE;
            }
            PD_PhyControl(pdInstance, PD_PHY_SNK_GET_TYPEC_CURRENT_CAP, &controlVal);
            if ((pdInstance->inProgress != kVbusPower_InPRSwap) && (pdInstance->inProgress != kVbusPower_InFRSwap))
            {
                pdInstance->callbackFns->PD_SnkDrawTypeCVbus(pdInstance->callbackParam, controlVal, kVbusPower_Stable);
            }
            break;
#endif

#if (defined PD_CONFIG_TRY_SNK_SUPPORT) && (PD_CONFIG_TRY_SNK_SUPPORT)
        case TYPEC_TRY_SNK:
            PD_ConnectSetAttachDetect(pdInstance, 0, kPD_PowerRoleSink);
            pdInstance->trySNKState = 0;
            PD_TimerStart(pdInstance, tDRPTryTimer, T_DRP_TRY_FOR_TRY_SNK);
            break;
#endif

#if (defined PD_CONFIG_AUDIO_ACCESSORY_SUPPORT) && (PD_CONFIG_AUDIO_ACCESSORY_SUPPORT)
        case TYPEC_AUDIO_ACCESSORY:
            pdInstance->connectState = kTYPEC_ConnectAudioAccessory;
            controlVal               = (pdInstance->ccUsed);
            PD_PhyControl(pdInstance, PD_PHY_CONNECT_SET_CC, &controlVal);

            PD_ConnectSetDetachDetect(pdInstance, &detachDetecParam, kPD_PowerRoleSource, (kPD_CC1));
#if AUDIO_ACC_DISCONNECT_DEBOUNCE
            PD_TimerStart(pdInstance, tCCDebounceTimer, T_CC_DEBOUNCE);
#endif
            break;
#endif

#if (defined PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
        case TYPEC_UNORIENTED_DEBUG_ACCESSORY_SRC:
            pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam, kVbusPower_Stable);
            pdInstance->connectState = kTYPEC_ConnectDebugAccessory;
            if (PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_DTS)
            {
                detachDetecParam.debugDTS        = 1;
                detachDetecParam.debugUnoriented = 1;
            }
            else
            {
                detachDetecParam.debugDTS        = 0;
                detachDetecParam.debugUnoriented = 1;
            }
            PD_ConnectSetDetachDetect(pdInstance, &detachDetecParam, kPD_PowerRoleSource, (kPD_CCInvalid));
            break;

        case TYPEC_ORIENTED_DEBUG_ACCESSORY_SRC:
            pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam, kVbusPower_Stable);
            pdInstance->connectState = kTYPEC_ConnectDebugAccessory;
            if (PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_DTS)
            {
                detachDetecParam.debugDTS        = 1;
                detachDetecParam.debugUnoriented = 0;
            }
            else
            {
                detachDetecParam.debugDTS        = 0;
                detachDetecParam.debugUnoriented = 0;
            }
            PD_ConnectSetDetachDetect(pdInstance, &detachDetecParam, kPD_PowerRoleSource, (kPD_CCInvalid));
            break;

        case TYPEC_DEBUG_ACCESSORY_SNK:
            pdInstance->connectState = kTYPEC_ConnectDebugAccessory;
            if (PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_DTS)
            {
                /* The DTS controls the orientation */
                detachDetecParam.debugDTS        = 1;
                detachDetecParam.debugUnoriented = 1;
                PD_ConnectSetDetachDetect(pdInstance, &detachDetecParam, kPD_PowerRoleSink, (kPD_CCInvalid));
            }
            /* Wait PDDebounce before checking orientation */
            PD_TimerStart(pdInstance, tPDDebounceTimer, T_PD_DEBOUNCE);
            writeRegsNeeded = 0;
            break;

        case TYPEC_ORIENTED_DEBUG_ACCESSORY_SNK:
            pdInstance->connectState = kTYPEC_ConnectDebugAccessory;
            if (PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_DTS)
            {
                detachDetecParam.debugDTS        = 1;
                detachDetecParam.debugUnoriented = 0;
            }
            else
            {
                detachDetecParam.debugDTS        = 0;
                detachDetecParam.debugUnoriented = 0;
            }
            PD_ConnectSetDetachDetect(pdInstance, &detachDetecParam, kPD_PowerRoleSink, (kPD_CCInvalid));
            break;
#endif

#if (defined PD_CONFIG_SINK_ACCESSORY_SUPPORT) && (PD_CONFIG_SINK_ACCESSORY_SUPPORT)
        case TYPEC_UNSUPPORTED_ACCESSORY:
            /* Only one of the CC1 or CC2 pins shall be in the SRC.Rd state. The port shall advertise Default
               USB Power (see Table 4-15) on this CC pin and monitor its voltage. */
            pdInstance->connectState = kTYPEC_ConnectVconnPoweredAccessory;
            controlVal               = (pdInstance->ccUsed);
            PD_PhyControl(pdInstance, PD_PHY_CONNECT_SET_CC, &controlVal);

            PD_ConnectSetDetachDetect(pdInstance, &detachDetecParam, kPD_PowerRoleSource, (pdInstance->ccUsed));
            break;

        case TYPEC_ATTACH_WAIT_ACCESSORY:
            PD_TimerStart(pdInstance, tCCDebounceTimer, T_CC_DEBOUNCE);
            break;

        case TYPEC_POWERED_ACCESSORY:
            /* When in the PoweredAccessory state, the port is powering a VCONN-powered Accessory. */
            PD_TimerStart(pdInstance, tAMETimeoutTimer, T_AME_TIMEOUT);

            pdInstance->connectState = kTYPEC_ConnectSource;
            controlVal               = (pdInstance->ccUsed);
            PD_PhyControl(pdInstance, PD_PHY_CONNECT_SET_CC, &controlVal);

            PD_ConnectSetDetachDetect(pdInstance, &detachDetecParam, kPD_PowerRoleSource, (pdInstance->ccUsed));

            vconn_en = VCONN_FORCE_ON;
            break;
#endif

#if ((defined PD_CONFIG_TRY_SRC_SUPPORT) && (PD_CONFIG_TRY_SRC_SUPPORT)) || \
    ((defined PD_CONFIG_TRY_SNK_SUPPORT) && (PD_CONFIG_TRY_SNK_SUPPORT))
        case TYPEC_TRY_SRC:
        case TYPEC_TRY_WAIT_SRC:
            PD_ConnectSetAttachDetect(pdInstance, 0, kPD_PowerRoleSource);
            PD_TimerStart(pdInstance, tDRPTryTimer, T_DRP_TRY);
            PD_TimerStart(pdInstance, tPDDebounceTimer, T_PD_DEBOUNCE);
            break;
#endif

#if (defined PD_CONFIG_TRY_SRC_SUPPORT) && (PD_CONFIG_TRY_SRC_SUPPORT)
        case TYPEC_TRY_WAIT_SNK:
            PD_ConnectSetAttachDetect(pdInstance, 0, kPD_PowerRoleSink);
            PD_TimerStart(pdInstance, tPDDebounceTimer, T_PD_DEBOUNCE);
            PD_TimerStart(pdInstance, tCCDebounceTimer, T_CC_DEBOUNCE);
            break;
#endif

        default:
            writeRegsNeeded = 0;
            break;
    }

    /* common process, if put before State specific actions, the TD.4.6.5 Try.SRC Connect Sink cannot pass */
    switch (pdInstance->curConnectState)
    {
        case TYPEC_ERROR_RECOVERY:
        case TYPEC_DISABLED:
        case TYPEC_UNATTACHED_SRC:
        case TYPEC_UNATTACHED_SNK:
        case TYPEC_UNATTACHED_ACCESSORY:
        case TYPEC_TRY_WAIT_SNK:
        case TYPEC_TRY_SNK:
        case TYPEC_TRY_SRC:
        case TYPEC_TRY_WAIT_SRC:
        case TYPEC_TOGGLE_SRC_FIRST:
        case TYPEC_TOGGLE_SNK_FIRST:
            pdInstance->ccUsed = kPD_CCInvalid;

            if (pdInstance->vbusDischargeInProgress == kVbus_ApplyTypecDischarge)
            {
                controlVal                          = 1;
                pdInstance->vbusDischargeInProgress = kVbus_TypecDischarge;
                if (pdInstance->callbackFns->PD_ControlVconn != NULL)
                {
                    pdInstance->callbackFns->PD_ControlVconn(pdInstance->callbackParam, 0);
                }
                if (pdInstance->callbackFns->PD_SrcTurnOffVbus != NULL)
                {
                    pdInstance->callbackFns->PD_SrcTurnOffVbus(pdInstance->callbackParam, kVbusPower_Stable);
                }
                PD_PhyControl(pdInstance, PD_PHY_DISCHARGE_VBUS, &controlVal);
#if (defined PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE) && (PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE)
                PD_TimerStart(pdInstance, tTypeCVbusMinDischargeTimer, T_MIN_VBUS_DISCHARGE);
#else
                PD_TimerStart(pdInstance, tTypeCVbusMaxDischargeTimer, T_MAX_VBUS_DISCHARGE);
#endif
            }
            /* HalVbusDisableMonitorAndDetect must be excuted after HalResetFetControl (discharge) operation */
            PD_PhyControl(pdInstance, PD_PHY_RESET_MSG_FUNCTION, NULL);
            break;

        case TYPEC_ATTACH_WAIT_SRC:
        case TYPEC_ATTACH_WAIT_SNK:
        case TYPEC_DEAD_BATTERY_SNK:
            break;

        default:
            /* Ensure discharge does not continue across state change. */
            if (pdInstance->vbusDischargeInProgress != kVbus_NoDischarge)
            {
                pdInstance->vbusDischargeInProgress = kVbus_NoDischarge;
                controlVal                          = 0;
                PD_PhyControl(pdInstance, PD_PHY_DISCHARGE_VBUS, &controlVal);
            }
            break;
    }

    PD_ConnectSetMonitorCC(pdInstance);

    if (writeRegsNeeded)
    {
        if ((pdInstance->curConnectState != TYPEC_ATTACHED_SRC) && (pdInstance->curConnectState != TYPEC_ATTACHED_SNK))
        {
            controlVal = kPD_CCInvalid;
            PD_PhyControl(pdInstance, PD_PHY_CONNECT_SET_CC, &controlVal);
        }

        if ((pdInstance->pdPowerPortConfig->vconnSupported) && (vconn_en != VCONN_NO_CHANGE))
        {
            controlVal = (vconn_en == VCONN_FORCE_ON);
            PD_PhyControl(pdInstance, PD_PHY_CONTROL_VCONN, &controlVal);
        }
    }
}

static uint8_t PD_ConnectCheckVBusPresence(pd_instance_t *pdInstance)
{
    uint32_t infoVal = PD_VBUS_POWER_STATE_VSAFE5V_MASK | PD_VBUS_POWER_STATE_VBUS_MASK;
    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &infoVal);
    infoVal = ((infoVal & PD_VBUS_POWER_STATE_VSAFE5V_MASK) || (infoVal & PD_VBUS_POWER_STATE_VBUS_MASK));
    return infoVal;
}

#if (defined PD_CONFIG_AUDIO_ACCESSORY_SUPPORT) && (PD_CONFIG_AUDIO_ACCESSORY_SUPPORT)
static void PD_ConnectStateSrcAudioAccessoryProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
/*  If the port is a Sink, the port shall transition to Unattached.SNK when the state of the
    monitored CC1 or CC2 pin(s) is SRC.Open for at least tCCDebounce.
    If the port is a Source or DRP, the port shall transition to Unattached.SRC when the state of
    the monitored CC1 or CC2 pin(s) is SRC.Open for at least tCCDebounce. */
#if AUDIO_ACC_DISCONNECT_DEBOUNCE
    if (cc1State != kCCState_SrcOpen)
    {
        /* re-start timer */
        PD_TimerStart(pdInstance, tCCDebounceTimer, T_CC_DEBOUNCE);
    }

    if (PD_TimerCheckValidTimeOut(pdInstance, tCCDebounceTimer))
#else
    if (cc1State == kCCState_SrcOpen)
#endif
    {
        PD_TimerClear(pdInstance, tCCDebounceTimer);
        if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
        {
            pdInstance->curConnectState = TYPEC_TOGGLE_SRC_FIRST;
        }
        else
        {
            if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkOnly) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault))
            {
                pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
            }
            else
            {
                pdInstance->curConnectState = TYPEC_UNATTACHED_SRC;
            }
        }
    }

    if (pdInstance->curConnectState != TYPEC_AUDIO_ACCESSORY)
    {
        PD_TimerClear(pdInstance, tCCDebounceTimer);
    }
}
#endif

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
static void PD_ConnectStateSrcUnattachedSrcProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    if ((cc1State == kCCState_SrcRd) || (cc2State == kCCState_SrcRd) ||
        ((cc1State == kCCState_SrcRa) && (cc2State == kCCState_SrcRa)))
    {
        pdInstance->curConnectState = TYPEC_ATTACH_WAIT_SRC;

#if defined(PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
        PD_TimerStart(pdInstance, tCCDebounceTimer1, T_CC_DEBOUNCE);
#endif
#if defined(PD_CONFIG_AUDIO_ACCESSORY_SUPPORT) && (PD_CONFIG_AUDIO_ACCESSORY_SUPPORT)
        PD_TimerStart(pdInstance, tCCDebounceTimer2, T_CC_DEBOUNCE);
#endif
    }
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
    else if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
    {
        if (PD_TimerCheckValidTimeOut(pdInstance, tDRPToggleTimer))
        {
            PD_TimerStart(pdInstance, tDRPToggleTimer, T_DRP_TOGGLE_SNK);
            pdInstance->curConnectState = TYPEC_TOGGLE_SNK_FIRST;
        }
    }
#endif
    else
    {
    }
}

static void PD_ConnectStateSrcAttachWaitSrcProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    uint32_t infoVal;

    infoVal = PD_VBUS_POWER_STATE_VSAFE0V_MASK;
    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &infoVal);

    if (!(infoVal & PD_VBUS_POWER_STATE_VSAFE0V_MASK))
    {
        PD_TimerStart(pdInstance, tCCDebounceTimer, T_CC_DEBOUNCE);
    }
    else if (((cc1State == kCCState_SrcOpen) && (cc2State == kCCState_SrcOpen)) ||
             ((cc1State == kCCState_SrcRa) && (cc2State == kCCState_SrcOpen)) ||
             ((cc1State == kCCState_SrcOpen) && (cc2State == kCCState_SrcRa)))
    {
        if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
        {
            pdInstance->curConnectState = TYPEC_TOGGLE_SNK_FIRST;
        }
        else
        {
            pdInstance->curConnectState = TYPEC_UNATTACHED_SRC;
        }
    }
#if defined(PD_CONFIG_TRY_SNK_SUPPORT) && (PD_CONFIG_TRY_SNK_SUPPORT)
    else if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) &&
             (pdInstance->pdPowerPortConfig->drpTryFunction == kTypecTry_Snk) &&
             (((cc1State == kCCState_SrcRd) && (cc2State != kCCState_SrcRd)) ||
              ((cc1State != kCCState_SrcRd) && (cc2State == kCCState_SrcRd))) &&
             (infoVal & PD_VBUS_POWER_STATE_VSAFE0V_MASK) && (PD_TimerCheckValidTimeOut(pdInstance, tCCDebounceTimer)))
    {
        pdInstance->enterTrySNKFromPoweredAcc = 0;
        pdInstance->curConnectState           = TYPEC_TRY_SNK;
    }
#endif
    else if ((cc1State == kCCState_SrcRd) && (cc2State != kCCState_SrcRd) &&
             ((infoVal & PD_VBUS_POWER_STATE_VSAFE0V_MASK)) &&
             (PD_TimerCheckValidTimeOut(pdInstance, tCCDebounceTimer)))
    {
        pdInstance->ccUsed          = kPD_CC1;
        pdInstance->curConnectState = TYPEC_ATTACHED_SRC;
    }
    else if ((cc2State == kCCState_SrcRd) && (cc1State != kCCState_SrcRd) &&
             (infoVal & PD_VBUS_POWER_STATE_VSAFE0V_MASK) && (PD_TimerCheckValidTimeOut(pdInstance, tCCDebounceTimer)))
    {
        pdInstance->ccUsed          = kPD_CC2;
        pdInstance->curConnectState = TYPEC_ATTACHED_SRC;
    }
#if (defined PD_CONFIG_AUDIO_ACCESSORY_SUPPORT) && (PD_CONFIG_AUDIO_ACCESSORY_SUPPORT)
    else if ((cc1State == kCCState_SrcRa) && (cc2State == kCCState_SrcRa) &&
             (PD_TimerCheckValidTimeOut(pdInstance, tCCDebounceTimer2)))
    {
        pdInstance->ccUsed          = kPD_CC1;
        pdInstance->curConnectState = TYPEC_AUDIO_ACCESSORY;
    }
#else
#if (defined PD_CONFIG_PTN5110_PORT) && (PD_CONFIG_PTN5110_PORT > 0)
#if defined(USBPD_ENABLE_PTN5110_A0R1_WORKAROUNDS) && (USBPD_ENABLE_PTN5110_A0R1_WORKAROUNDS)
    else if ((pdInstance->phyInfo.vendorID == PD_VENDOR_ID_NXP) &&
             (pdInstance->phyInfo.productID == PRODUCT_ID_PTN5110) &&
             (pdInstance->phyInfo.deviceID < DEVICE_ID_PTN5110_A0R1))
    {
        if ((cc1State == kCCState_SrcRa) && (cc2State == kCCState_SrcRa) &&
            (infoVal & PD_VBUS_POWER_STATE_VSAFE0V_MASK))
        {
            pdInstance->ccUsed          = kPD_CCInvalid;
            pdInstance->curConnectState = TYPEC_ATTACHED_SRC;
        }
    }
#endif
#endif
#endif
#if defined(PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
    else if ((cc1State == kCCState_SrcRd) && (cc2State == kCCState_SrcRd) &&
             (PD_TimerCheckValidTimeOut(pdInstance, tCCDebounceTimer1)))
    {
        pdInstance->ccUsed          = kPD_CCInvalid;
        pdInstance->curConnectState = TYPEC_UNORIENTED_DEBUG_ACCESSORY_SRC;
    }
#else
#if (defined PD_CONFIG_PTN5110_PORT) && (PD_CONFIG_PTN5110_PORT > 0)
#if defined(USBPD_ENABLE_PTN5110_A0R1_WORKAROUNDS) && (USBPD_ENABLE_PTN5110_A0R1_WORKAROUNDS)
    else if ((pdInstance->phyInfo.vendorID == PD_VENDOR_ID_NXP) &&
             (pdInstance->phyInfo.productID == PRODUCT_ID_PTN5110) &&
             (pdInstance->phyInfo.deviceID < DEVICE_ID_PTN5110_A0R1))
    {
        if ((cc1State == kCCState_SrcRd) && (cc2State == kCCState_SrcRd) &&
            (infoVal & PD_VBUS_POWER_STATE_VSAFE0V_MASK))
        {
            pdInstance->ccUsed          = kPD_CCInvalid;
            pdInstance->curConnectState = TYPEC_ATTACHED_SRC;
        }
    }
#endif
#endif
#endif
    else
    {
    }

    /* exit process */
    if (pdInstance->curConnectState != TYPEC_ATTACH_WAIT_SRC)
    {
        /* 1. start the timer when enter TYPEC_ATTACH_WAIT_SRC; 2. end the tiemr when exit TYPEC_ATTACH_WAIT_SRC */
        PD_TimerClear(pdInstance, tCCDebounceTimer);
    }
    else
    {
#if defined(PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
        if ((cc1State != kCCState_SrcRd) && (cc2State != kCCState_SrcRd))
        {
            PD_TimerStart(pdInstance, tCCDebounceTimer1, T_CC_DEBOUNCE);
        }
#endif
#if defined(PD_CONFIG_AUDIO_ACCESSORY_SUPPORT) && (PD_CONFIG_AUDIO_ACCESSORY_SUPPORT)
        if ((cc1State != kCCState_SrcRa) || (cc2State != kCCState_SrcRa))
        {
            PD_TimerStart(pdInstance, tCCDebounceTimer2, T_CC_DEBOUNCE);
        }
#endif
    }
}

static void PD_ConnectStateSrcAttachedSrcProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    if ((pdInstance->inProgress != kVbusPower_InPRSwap) && (pdInstance->inProgress != kVbusPower_InFRSwap))
    {
        /* When the SRC.Open state is detected on the monitored CC pin, a DRP shall transition to
           Unattached.SNK unless it strongly prefers the Source role. In that case, it shall transition to
           TryWait.SNK. This transition to TryWait.SNK is needed so that two devices that both prefer
           the Source role do not loop endlessly between Source and Sink. In other words, a DRP that
           would enter Try.SRC from AttachWait.SNK shall enter TryWait.SNK for a Sink detach from
           Attached.SRC. */

        /* when partner set CC as open then set CC as Rd quickly,
           then source will disable en_src and remove vubs quickly because auto discharge,
           but check CC states they still are Rd  */
        uint32_t infoVal = PD_ConnectCheckVBusPresence(pdInstance);

        if (pdInstance->enterAttachedSRCFromTypeCStateMachine)
        {
            pdInstance->enterAttachedSRCFromTypeCStateMachine--;
            if ((infoVal) || (pdInstance->inProgress != kVbusPower_Stable))
            {
                pdInstance->enterAttachedSRCFromTypeCStateMachine = 0;
            }
            if (pdInstance->enterAttachedSRCFromTypeCStateMachine > 0)
            {
                PD_TimerStart(pdInstance, tVBUSNotStableTimer, T_VBUS_NOT_STABLE);
            }
        }

        if (((pdInstance->cc1Monitor) && ((cc1State == kCCState_SrcOpen) || (cc1State == kCCState_Unknown))) ||
            ((pdInstance->cc2Monitor) && ((cc2State == kCCState_SrcOpen) || (cc2State == kCCState_Unknown))) ||
            ((!infoVal) && (pdInstance->enterAttachedSRCFromTypeCStateMachine == 0) &&
             (pdInstance->inProgress == kVbusPower_Stable)))
        {
            pdInstance->enterAttachedSRCFromTypeCStateMachine = 0;
            if (pdInstance->pdPowerPortConfig->drpTryFunction == kTypecTry_Src)
            {
                pdInstance->curConnectState = TYPEC_TRY_WAIT_SNK;
            }
            else
            {
                if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
                {
                    pdInstance->curConnectState = TYPEC_TOGGLE_SNK_FIRST;
                }
                else if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault)
                {
                    pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
                }
                else
                {
                    pdInstance->curConnectState = TYPEC_UNATTACHED_SRC;
                }
            }
            pdInstance->vbusDischargeInProgress = kVbus_ApplyTypecDischarge;
        }
    }
}
#endif

#if (defined PD_CONFIG_TRY_SNK_SUPPORT) && (PD_CONFIG_TRY_SNK_SUPPORT)
static void PD_ConnectStateSrcTrySnkProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    uint32_t infoVal;

    /* The port shall wait for tDRPTry and only then begin monitoring the CC1 and CC2 pins for the SNK.Rp state. */
    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tDRPTryTimer) && (pdInstance->trySNKState == 0))
    {
        pdInstance->trySNKState = 1;
        PD_TimerClear(pdInstance, tPDDebounceTimer);
        PD_TimerClear(pdInstance, tPDDebounce2Timer);
        PD_TimerClear(pdInstance, tDRPTryWaitTimer);
    }
    if (pdInstance->trySNKState == 1)
    {
        infoVal = PD_VBUS_POWER_STATE_VBUS_MASK;
        PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &infoVal);
        /* source detected and VBUS detected */
        if ((cc1State == kCCState_SnkRp) || (cc2State == kCCState_SnkRp))
        {
            PD_TimerClear(pdInstance, tPDDebounce2Timer);
            PD_TimerClear(pdInstance, tDRPTryWaitTimer);
            /* tPDDebounceTimer not run and not timeout */
            if (!PD_TimerCheckValidTimeOut(pdInstance, tPDDebounceTimer) &&
                PD_TimerCheckInvalidOrTimeOut(pdInstance, tPDDebounceTimer))
            {
                PD_TimerStart(pdInstance, tPDDebounceTimer, T_PD_DEBOUNCE);
            }

            /* The port shall then transition to Attached.SNK when the SNK.Rp state is detected on exactly
               one of the CC1 or CC2 pins for at least tPDDebounce and VBUS is detected.  */
            if ((infoVal & PD_VBUS_POWER_STATE_VBUS_MASK) && PD_TimerCheckValidTimeOut(pdInstance, tPDDebounceTimer))
            {
                if (cc2State == kCCState_SnkRp)
                {
                    pdInstance->ccUsed = kPD_CC2;
                }
                else
                {
                    pdInstance->ccUsed = kPD_CC1;
                }
                pdInstance->curConnectState = TYPEC_ATTACHED_SNK;
            }
        }
        else
        {
            PD_TimerClear(pdInstance, tPDDebounceTimer);
            /* tPDDebounce2Timer not run and not timeout */
            if (!PD_TimerCheckValidTimeOut(pdInstance, tPDDebounce2Timer) &&
                PD_TimerCheckInvalidOrTimeOut(pdInstance, tPDDebounce2Timer))
            {
                PD_TimerStart(pdInstance, tPDDebounce2Timer, T_PD_DEBOUNCE);
            }
            /* tDRPTryWaitTimer not run and not timeout */
            if (!PD_TimerCheckValidTimeOut(pdInstance, tDRPTryWaitTimer) &&
                PD_TimerCheckInvalidOrTimeOut(pdInstance, tDRPTryWaitTimer))
            {
                PD_TimerStart(pdInstance, tDRPTryWaitTimer, T_PD_DEBOUNCE);
            }

            if (pdInstance->enterTrySNKFromPoweredAcc)
            {
                /* A Sink with Accessory Support shall transition to Unsupported.Accessory if
                   SNK.Rp state is not detected for tDRPTryWait. */
                if (PD_TimerCheckValidTimeOut(pdInstance, tDRPTryWaitTimer))
                {
                    pdInstance->curConnectState = TYPEC_UNSUPPORTED_ACCESSORY;
                }
                else
                {
                    /* There is no else here because a Sink with Accessory Support */
                    /* can only go to UnsupportedAccessory, not to TryWait.SRC */
                }
            }
            else if (PD_TimerCheckValidTimeOut(pdInstance, tPDDebounce2Timer))
            {
                /* the port shall transition to TryWait.SRC if SNK.Rp state is not detected for
                   tPDDebounce. */
                pdInstance->curConnectState = TYPEC_TRY_WAIT_SRC;
            }
            else
            {
            }
        }
    }

    if (pdInstance->curConnectState != TYPEC_TRY_SNK)
    {
        PD_TimerClear(pdInstance, tDRPTryTimer);
        PD_TimerClear(pdInstance, tDRPTryWaitTimer);
        PD_TimerClear(pdInstance, tPDDebounceTimer);
        PD_TimerClear(pdInstance, tPDDebounce2Timer);
    }
}

static void PD_ConnectStateSrcTryWaitSrcProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    uint32_t infoVal;
    /* The port shall transition to Attached.SRC when VBUS is at vSafe0V and the SRC.Rd state is
       detected on exactly one of the CC pins for at least tCCDebounce. */
    if ((cc1State == kCCState_SrcRd) || (cc2State == kCCState_SrcRd))
    {
        /* re-start */
        PD_TimerStart(pdInstance, tDRPTryTimer, T_DRP_TRY);
        infoVal = PD_VBUS_POWER_STATE_VSAFE0V_MASK;
        PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &infoVal);
        if ((infoVal & PD_VBUS_POWER_STATE_VSAFE0V_MASK) &&
            (((cc1State == kCCState_SrcRd) && (cc2State != kCCState_SrcRd)) ||
             ((cc2State == kCCState_SrcRd) && (cc1State != kCCState_SrcRd))) &&
            (PD_TimerCheckValidTimeOut(pdInstance, tPDDebounceTimer)))
        {
            if (cc1State == kCCState_SrcRd)
            {
                pdInstance->ccUsed = kPD_CC1;
            }
            else
            {
                pdInstance->ccUsed = kPD_CC2;
            }
            pdInstance->curConnectState = TYPEC_ATTACHED_SRC;
        }
    }
    else
    {
        /* re-start */
        PD_TimerStart(pdInstance, tPDDebounceTimer, T_PD_DEBOUNCE);

        /* The port shall transition to Unattached.SNK after tDRPTry if neither of the CC1 or CC2 pins
           are in the SRC.Rd state. */
        if (PD_TimerCheckValidTimeOut(pdInstance, tDRPTryTimer))
        {
            if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
            {
                pdInstance->curConnectState = TYPEC_TOGGLE_SNK_FIRST;
            }
            else
            {
                pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
            }
        }
    }

    if (pdInstance->curConnectState != TYPEC_TRY_WAIT_SRC)
    {
        PD_TimerClear(pdInstance, tDRPTryTimer);
        PD_TimerClear(pdInstance, tPDDebounceTimer);
    }
}
#endif

#if (defined PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
static void PD_ConnectStateSrcUnorientedDebugAccSrcProcess(pd_instance_t *pdInstance,
                                                           uint8_t cc1State,
                                                           uint8_t cc2State)
{
    if ((pdInstance->inProgress != kVbusPower_InPRSwap) && (pdInstance->inProgress != kVbusPower_InFRSwap) &&
        ((cc1State == kCCState_SrcOpen) || (cc2State == kCCState_SrcOpen)))
    {
        if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
        {
            pdInstance->curConnectState = TYPEC_TOGGLE_SNK_FIRST;
        }
        else
        {
            if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkOnly) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault))
            {
                pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
            }
            else
            {
                pdInstance->curConnectState = TYPEC_UNATTACHED_SRC;
            }
        }
        pdInstance->vbusDischargeInProgress = kVbus_ApplyTypecDischarge;
    }
    else if (PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_DTS)
    {
        /* We drive the orientation of a DTS */
        pdInstance->ccUsed          = kPD_CC1;
        pdInstance->curConnectState = TYPEC_ORIENTED_DEBUG_ACCESSORY_SRC;
    }
    else if ((PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_TS) && (cc1State != kCCState_Unknown) &&
             (cc2State != kCCState_Unknown) && (cc1State != kCCState_Unstable) && (cc2State != kCCState_Unstable))
    {
        /* Detect cc_orient as per Usb Typec spec 1.2 B.2.6.1.2 */
        /* The CC pin with the greater voltage is the plug CC pin, */
        /* which establishes the orientation of the DTS plug in the TS receptacle */
        /* and also indicates the USB-PD CC communication wire. */
        /* (SRC_RD should be the correct cc line, the condition cc1_detect < cc2_detect */
        /* relies on the definition that that CC_SRC_RA > CC_SRC_RD) */
        if (cc2State > cc1State)
        {
            pdInstance->ccUsed = kPD_CC1;
        }
        else if (cc2State == cc1State)
        {
            pdInstance->ccUsed = kPD_CCInvalid;
        }
        else
        {
            pdInstance->ccUsed = kPD_CC2;
        }

        if (pdInstance->ccUsed != kPD_CCInvalid)
        {
            pdInstance->curConnectState = TYPEC_ORIENTED_DEBUG_ACCESSORY_SRC;
        }
    }
}

static void PD_ConnectStateSrcOrientedDebugAccSrcProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    if ((pdInstance->inProgress != kVbusPower_InPRSwap) && (pdInstance->inProgress != kVbusPower_InFRSwap) &&
        ((cc1State == kCCState_SrcOpen) || (cc2State == kCCState_SrcOpen)))
    {
        if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
        {
            pdInstance->curConnectState = TYPEC_TOGGLE_SNK_FIRST;
        }
        else
        {
            if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkOnly) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault))
            {
                pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
            }
            else
            {
                pdInstance->curConnectState = TYPEC_UNATTACHED_SRC;
            }
        }
        pdInstance->vbusDischargeInProgress = kVbus_ApplyTypecDischarge;
    }
}

static void PD_ConnectStateSnkDebugAccSnkProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    /* The port shall transition to Unattached.SNK when VBUS is no longer present. */
    if ((!(PD_ConnectCheckVBusPresence(pdInstance))) && (pdInstance->inProgress == kVbusPower_Stable))
    {
        pdInstance->curConnectState         = TYPEC_UNATTACHED_SNK;
        pdInstance->vbusDischargeInProgress = kVbus_ApplyTypecDischarge;
    }
    else if (PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_DTS)
    {
        /* We drive the orientation of a DTS */
        pdInstance->ccUsed          = kPD_CC1;
        pdInstance->curConnectState = TYPEC_ORIENTED_DEBUG_ACCESSORY_SNK;
    }
    else if ((PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_TS) && (cc1State != kCCState_Unknown) &&
             (cc2State != kCCState_Unknown) && (cc1State != kCCState_Unstable) && (cc2State != kCCState_Unstable))
    {
        /* Detect cc_orient as per Usb Typec spec 1.2 B.2.6.1.2 */
        /* The CC pin with the greater voltage is the plug CC pin, */
        /* which establishes the orientation of the DTS plug in the TS receptacle */
        /* and also indicates the USB-PD CC communication wire. */
        /* (SRC_RD should be the correct cc line, the condition cc1_detect < cc2_detect */
        /* relies on the definition that that CC_SRC_RA > CC_SRC_RD) */
        if (cc1State > cc2State)
        {
            pdInstance->ccUsed = kPD_CC1;
        }
        else if (cc2State == cc1State)
        {
            pdInstance->ccUsed = kPD_CCInvalid;
        }
        else
        {
            pdInstance->ccUsed = kPD_CC2;
        }

        if (pdInstance->ccUsed != kPD_CCInvalid)
        {
            pdInstance->curConnectState = TYPEC_ORIENTED_DEBUG_ACCESSORY_SNK;
        }
    }
}

static void PD_ConnectStateSnkOrientedDebugAccSnkProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    if ((!(PD_ConnectCheckVBusPresence(pdInstance))) && (pdInstance->inProgress == kVbusPower_Stable))
    {
        pdInstance->curConnectState         = TYPEC_UNATTACHED_SNK;
        pdInstance->vbusDischargeInProgress = kVbus_ApplyTypecDischarge;
    }
}
#endif

#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
static void PD_ConnectStateDrpToggleSrcOrSnkFirstProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    uint8_t temp8Val = pdInstance->curConnectState;

    if ((cc1State == kCCState_SrcRd) || (cc2State == kCCState_SrcRd) ||
        ((cc1State == kCCState_SrcRa) && (cc2State == kCCState_SrcRa)))
    {
        pdInstance->curConnectState = TYPEC_ATTACH_WAIT_SRC;
    }
    else if ((cc1State == kCCState_SnkRp) || (cc2State == kCCState_SnkRp))
    {
        pdInstance->curConnectState = TYPEC_ATTACH_WAIT_SNK;
    }
    else
    {
    }

    if (temp8Val == pdInstance->curConnectState)
    {
        PD_PhyControl(pdInstance, PD_PHY_GET_LOOK4_CONNECTION_STATE, &temp8Val);
        /* looking for looking 4 connection
           note: TCPC will stop toggling when some bogus connection are found, TCPM will transit to
           one temp state, and use the cc_debounce to filter these bogus connection, TCPM will transit back to
           original TYPEC_TOGGLE_SRC_FIRST or TYPEC_TOGGLE_SNK_FIRST and restart DRP toggling */
        if (temp8Val == kLook4ConnState_AssertRd)
        {
            /* assert Rd */
            pdInstance->curConnectState = TYPEC_ATTACH_WAIT_SNK;
        }
        else if (temp8Val == kLook4ConnState_AssertRp)
        {
            pdInstance->curConnectState = TYPEC_ATTACH_WAIT_SRC;
        }
        else
        {
        }
    }
}
#endif

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
static void PD_ConnectStateSnkDeadBatteryProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    uint8_t vbusPresent = 0;

    {
        vbusPresent = PD_ConnectCheckVBusPresence(pdInstance);

        if ((cc1State == kCCState_SnkRp) && (cc2State != kCCState_SnkRp) && (vbusPresent))
        {
            /* The port shall transition to Attached.SNK after the state of only one of the CC1 or CC2 pins is
               SNK.Rp for at least tCCDebounce and VBUS is detected. */
            pdInstance->ccUsed          = kPD_CC1;
            pdInstance->curConnectState = TYPEC_ATTACHED_SNK;
        }
        else if ((cc1State != kCCState_SnkRp) && (cc2State == kCCState_SnkRp) && (vbusPresent))
        {
            pdInstance->ccUsed          = kPD_CC2;
            pdInstance->curConnectState = TYPEC_ATTACHED_SNK;
        }
        else
        {
        }
    }
}

static void PD_ConnectStateSnkUnattachedSnkProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    if ((cc1State == kCCState_SnkRp) || (cc2State == kCCState_SnkRp))
    {
        pdInstance->curConnectState = TYPEC_ATTACH_WAIT_SNK;
    }
    else if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
    {
        if (PD_TimerCheckValidTimeOut(pdInstance, tDRPToggleTimer))
        {
            PD_TimerStart(pdInstance, tDRPToggleTimer, T_DRP_TOGGLE_SRC);
            pdInstance->curConnectState = TYPEC_UNATTACHED_SRC;
        }
    }
    else if (PD_CONFIG_SINK_ACCESSORY_SUPPORT)
    {
        /* This state is functionally equivalent to the Unattached.SRC state in a DRP, except that
        Attached.SRC is not supported.
        The port shall provide an Rp */
        if (PD_TimerCheckValidTimeOut(pdInstance, tDRPToggleTimer))
        {
            PD_TimerStart(pdInstance, tDRPToggleTimer, T_DRP_TOGGLE_SRC);
            pdInstance->curConnectState = TYPEC_UNATTACHED_ACCESSORY;
        }
    }
    else
    {
    }
}

static void PD_ConnectStateSnkAttachWaitSnkProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    uint8_t vbusPresent = 0;

    /* all open */
    if ((cc1State != kCCState_SnkRp) && (cc2State != kCCState_SnkRp))
    {
        /* restart the attach detection timer */
        PD_TimerStart(pdInstance, tCCDebounceTimer, T_CC_DEBOUNCE);

        if (!PD_TimerCheckStarted(pdInstance, tPDDebounceTimer))
        {
            PD_TimerStart(pdInstance, tPDDebounceTimer, T_PD_DEBOUNCE);
        }
        /* A Sink shall transition to Unattached.SNK when the state of both the CC1 and CC2 pins is
           SNK.Open for at least tPDDebounce.
           A DRP shall transition to Unattached.SRC when the state of both the CC1 and CC2 pins is
           SNK.Open for at least tPDDebounce. */
        if (PD_TimerCheckValidTimeOut(pdInstance, tPDDebounceTimer))
        {
            if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
            {
                pdInstance->curConnectState = TYPEC_TOGGLE_SRC_FIRST;
            }
            else
            {
                pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
            }
        }
    }
    else
    {
        /* restart detach detection timer */
        vbusPresent = PD_ConnectCheckVBusPresence(pdInstance);

        /* A DRP that strongly prefers the Source role may optionally transition to Try.SRC instead of
           Attached.SNK when the state of only one CC pin has been SNK.Rp for at least tCCDebounce
           and VBUS is detected. */
        if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) &&
            (pdInstance->pdPowerPortConfig->drpTryFunction == kTypecTry_Src) &&
            (((cc1State == kCCState_SnkRp) && (cc2State != kCCState_SnkRp)) ||
             ((cc1State != kCCState_SnkRp) && (cc2State == kCCState_SnkRp))) &&
            (PD_TimerCheckInvalidOrTimeOut(pdInstance, tCCDebounceTimer)) && (vbusPresent))
        {
            pdInstance->curConnectState = TYPEC_TRY_SRC;
        }
        else if (((cc1State == kCCState_SnkRp) && (cc2State != kCCState_SnkRp)) && (vbusPresent) &&
                 (PD_TimerCheckInvalidOrTimeOut(pdInstance, tCCDebounceTimer)))
        {
            /* The port shall transition to Attached.SNK after the state of only one of the CC1 or CC2 pins is
               SNK.Rp for at least tCCDebounce and VBUS is detected. */
            pdInstance->ccUsed          = kPD_CC1;
            pdInstance->curConnectState = TYPEC_ATTACHED_SNK;
        }
        else if (((cc1State != kCCState_SnkRp) && (cc2State == kCCState_SnkRp)) && (vbusPresent) &&
                 (PD_TimerCheckInvalidOrTimeOut(pdInstance, tCCDebounceTimer)))
        {
            pdInstance->ccUsed          = kPD_CC2;
            pdInstance->curConnectState = TYPEC_ATTACHED_SNK;
        }
#if defined(PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
        else if ((pdInstance->pdConfig->deviceType == kDeviceType_DebugAccDevice) && (cc1State == kCCState_SnkRp) &&
                 (cc2State == kCCState_SnkRp) && (vbusPresent) &&
                 (PD_TimerCheckInvalidOrTimeOut(pdInstance, tCCDebounceTimer)))
        {
            /* If the port supports Debug Accessory Mode, the port shall transition to DebugAccessory.SNK
               if the state of both the CC1 and CC2 pins is SNK.Rp for at least tCCDebounce and VBUS is
               detected.  */
            pdInstance->ccUsed          = kPD_CCInvalid;
            pdInstance->curConnectState = TYPEC_DEBUG_ACCESSORY_SNK;
        }
#endif
        else
        {
        }
    }

    if (pdInstance->curConnectState != TYPEC_ATTACH_WAIT_SNK)
    {
        PD_TimerClear(pdInstance, tCCDebounceTimer);
        PD_TimerClear(pdInstance, tPDDebounceTimer);
    }
}

static void PD_ConnectStateSnkAttachedSnkProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    uint32_t infoVal;
    /* A port that is not in the process of a USB PD PR_Swap or a USB PD Hard Reset shall transition
    to Unattached.SNK when VBUS falls below 3.67 V. Note if VBUS has been adjusted by USB PD
    to operate above 5 V, then the port shall transition to Unattached.SNK when VBUS falls below
    80% of the negotiated value. If supplying VCONN, the port shall cease to supply it within
    tVCONNOFF of exiting Attached.SNK. */

    infoVal = PD_ConnectCheckVBusPresence(pdInstance);
    if (
#if (PD_CONFIG_SINK_DETACH_DETECT_WAY & PD_SINK_DETACH_ON_VBUS_ABSENT)
        ((!(infoVal)) && (pdInstance->inProgress == kVbusPower_Stable)) ||
        ((pdInstance->inProgress == kVbusPower_InHardReset) && (!(infoVal)) && (cc1State != kCCState_SnkRp) &&
         (cc2State != kCCState_SnkRp)) ||
#endif
#if (PD_CONFIG_SINK_DETACH_DETECT_WAY & PD_SINK_DETACH_ON_CC_OPEN)
        // Refer to "USB Type-C ECR Exit_from_Attached.SNK_state.doc"
        // The *_detect values are debounced with tPDDebounce
        // %%% Need to also exclude fast role swap in progress %%%
        ((pdInstance->inProgress == kVbusPower_Stable) &&
         (((pdInstance->cc1Monitor) && (cc1State != kCCState_SnkRp)) ||
          ((pdInstance->cc2Monitor) && (cc2State != kCCState_SnkRp)))) ||
#endif
        0)
    {
        if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
        {
            pdInstance->curConnectState = TYPEC_TOGGLE_SNK_FIRST;
        }
        else if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceDefault)
        {
            pdInstance->curConnectState = TYPEC_UNATTACHED_SRC;
        }
        else
        {
            pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
        }
        pdInstance->vbusDischargeInProgress = kVbus_ApplyTypecDischarge;
    }
}
#endif

#if (defined PD_CONFIG_TRY_SRC_SUPPORT) && (PD_CONFIG_TRY_SRC_SUPPORT)
static void PD_ConnectStateSnkTrySrcProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    /* The port shall transition to Attached.SRC when the SRC.Rd state is detected on exactly one of
       the CC1 or CC2 pins for at least tPDDebounce. */
    if ((cc1State == kCCState_SrcRd) || (cc2State == kCCState_SrcRd))
    {
        /* re-start */
        PD_TimerStart(pdInstance, tDRPTryTimer, T_DRP_TRY);
        if ((cc1State == kCCState_SrcRd) && (cc2State != kCCState_SrcRd) &&
            (PD_TimerCheckValidTimeOut(pdInstance, tPDDebounceTimer)))
        {
            pdInstance->ccUsed          = kPD_CC1;
            pdInstance->curConnectState = TYPEC_ATTACHED_SRC;
        }
        else if ((cc2State == kCCState_SrcRd) && (cc1State != kCCState_SrcRd) &&
                 (PD_TimerCheckValidTimeOut(pdInstance, tPDDebounceTimer)))
        {
            pdInstance->ccUsed          = kPD_CC2;
            pdInstance->curConnectState = TYPEC_ATTACHED_SRC;
        }
    }
    else
    {
        /* re-start */
        PD_TimerStart(pdInstance, tPDDebounceTimer, T_PD_DEBOUNCE);
        if (PD_TimerCheckValidTimeOut(pdInstance, tDRPTryTimer))
        {
            pdInstance->curConnectState = TYPEC_TRY_WAIT_SNK;
        }
    }

    if (pdInstance->curConnectState != TYPEC_TRY_SRC)
    {
        PD_TimerClear(pdInstance, tDRPTryTimer);
        PD_TimerClear(pdInstance, tPDDebounceTimer);
    }
}

static void PD_ConnectStateSnkTryWaitSnkProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    uint32_t infoVal;
    /* The port shall transition to Unattached.SNK when the state of both of the CC1 and CC2 pins
       is SNK.Open for at least tPDDebounce. */
    if (((cc1State == kCCState_SnkRp) && (cc2State != kCCState_SnkRp)) ||
        ((cc2State == kCCState_SnkRp) && (cc1State != kCCState_SnkRp)))
    {
        PD_TimerStart(pdInstance, tPDDebounceTimer, T_PD_DEBOUNCE);
        infoVal = PD_VBUS_POWER_STATE_VBUS_MASK;
        PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &infoVal);
        if ((infoVal & PD_VBUS_POWER_STATE_VBUS_MASK) && (PD_TimerCheckValidTimeOut(pdInstance, tCCDebounceTimer)))
        {
            if (cc1State == kCCState_SnkRp)
            {
                pdInstance->ccUsed = kPD_CC1;
            }
            else
            {
                pdInstance->ccUsed = kPD_CC2;
            }
            pdInstance->curConnectState = TYPEC_ATTACHED_SNK;
        }
    }
    else
    {
        PD_TimerStart(pdInstance, tCCDebounceTimer, T_CC_DEBOUNCE);

        if (PD_TimerCheckValidTimeOut(pdInstance, tPDDebounceTimer))
        {
            if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)
            {
                pdInstance->curConnectState = TYPEC_TOGGLE_SNK_FIRST;
            }
            else
            {
                pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
            }
        }
    }

    if (pdInstance->curConnectState != TYPEC_TRY_WAIT_SNK)
    {
        PD_TimerClear(pdInstance, tCCDebounceTimer);
        PD_TimerClear(pdInstance, tPDDebounceTimer);
    }
}
#endif

#if (defined PD_CONFIG_SINK_ACCESSORY_SUPPORT) && (PD_CONFIG_SINK_ACCESSORY_SUPPORT)
static void PD_ConnectStateSnkUnattachedAccessory(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    /* The port shall transition to AttachWait.Accessory when the state of both the CC1 and CC2
    pins is SRC.Ra or SRC.Rd. */
    if ((cc1State != kCCState_Unknown) && (cc2State != kCCState_Unknown) && (cc1State != kCCState_SrcOpen) &&
        (cc2State != kCCState_SrcOpen))
    {
        pdInstance->curConnectState = TYPEC_ATTACH_WAIT_ACCESSORY;
    }
    else if (PD_TimerCheckValidTimeOut(pdInstance, tDRPToggleTimer))
    {
        PD_TimerStart(pdInstance, tDRPToggleTimer, T_DRP_TOGGLE_SNK);

        pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
    }
}

static void PD_ConnectStateSnkAttachWaitAccessoryProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
#if defined(PD_CONFIG_AUDIO_ACCESSORY_SUPPORT) && (PD_CONFIG_AUDIO_ACCESSORY_SUPPORT)
    if ((cc1State == kCCState_SrcRa) && (cc2State == kCCState_SrcRa))
    {
        pdInstance->ccUsed          = kPD_CC1;
        pdInstance->curConnectState = TYPEC_AUDIO_ACCESSORY;
    }
#endif
#if defined(USBPD_ENABLE_POWERED_ACCESSORY_SUPPORT)
    if ((cc1State == kCCState_SrcRa) && (cc2State == kCCState_SrcRd))
    {
        pdInstance->ccUsed          = kPD_CC2;
        pdInstance->curConnectState = TYPEC_POWERED_ACCESSORY;
    }
    else if ((cc1State == kCCState_SrcRd) && (cc2State == kCCState_SrcRa))
    {
        pdInstance->ccUsed          = kPD_CC1;
        pdInstance->curConnectState = TYPEC_POWERED_ACCESSORY;
    }
#endif
    if ((cc1State == kCCState_SrcOpen) || (cc2State == kCCState_SrcOpen))
    {
        pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
    }
}

static void PD_ConnectStateSnkPoweredAccessoryProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    /* The port shall transition to Unattached.SNK when the SRC.Open state is detected on the
    monitored CC pin */
    if (((pdInstance->cc1Monitor) && (cc1State == kCCState_SrcOpen)) ||
        ((pdInstance->cc2Monitor) && (cc2State == kCCState_SrcOpen)))
    {
        pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
    }

    if (PD_TimerCheckValidTimeOut(pdInstance, tAMETimeoutTimer))
    {
        if (pdInstance->psmPresentlyPdConnected)
        {
            pdInstance->curConnectState = TYPEC_UNSUPPORTED_ACCESSORY;
        }
        else
        {
            pdInstance->enterTrySNKFromPoweredAcc = 1;
            pdInstance->curConnectState           = TYPEC_TRY_SNK;
        }
    }

    if (pdInstance->curConnectState != TYPEC_POWERED_ACCESSORY)
    {
        PD_TimerClear(pdInstance, tAMETimeoutTimer);
    }
}

static void PD_ConnectStateSrcUnsupportedAccProcess(pd_instance_t *pdInstance, uint8_t cc1State, uint8_t cc2State)
{
    if (((pdInstance->cc1Monitor) && (cc1State == kCCState_SrcOpen)) ||
        ((pdInstance->cc2Monitor) && (cc2State == kCCState_SrcOpen)))
    {
        pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
    }
}
#endif

static uint8_t PD_ConnectState(pd_instance_t *pdInstance)
{
    uint8_t reVal;

    switch (pdInstance->curConnectState)
    {
        case TYPEC_ATTACHED_SRC:
        case TYPEC_ATTACHED_SNK:
        case TYPEC_AUDIO_ACCESSORY:
        case TYPEC_POWERED_ACCESSORY:
        case TYPEC_UNSUPPORTED_ACCESSORY:
        case TYPEC_UNORIENTED_DEBUG_ACCESSORY_SRC:
        case TYPEC_ORIENTED_DEBUG_ACCESSORY_SRC:
        case TYPEC_DEBUG_ACCESSORY_SNK:
        case TYPEC_ORIENTED_DEBUG_ACCESSORY_SNK:
            reVal = kConnectState_Connected;
            break;

        case TYPEC_TRY_SRC:
        case TYPEC_TRY_WAIT_SNK:
        case TYPEC_TRY_SNK:
        case TYPEC_TRY_WAIT_SRC:
        case TYPEC_ATTACH_WAIT_SRC:
        case TYPEC_ATTACH_WAIT_SNK:
            reVal = kConnectState_NotStable;
            break;

        default:
            reVal = kConnectState_Disconnected;
            break;
    }

    return reVal;
}

static TypeCState_t PD_ConnectStateMachine(pd_instance_t *pdInstance)
{
    uint8_t preState;
    uint32_t infoVal;
    pd_phy_cc_state_t cc1State;
    pd_phy_cc_state_t cc2State;
    pd_phy_get_cc_state_t ccState;

    /* This loop will repeat while the state is changed */

    PD_PhyControl(pdInstance, PD_PHY_GET_CC_LINE_STATE, &ccState);
    cc1State = (pd_phy_cc_state_t)ccState.cc1State;
    cc2State = (pd_phy_cc_state_t)ccState.cc2State;
    if ((cc1State == kCCState_SnkRpDefault) || (cc1State == kCCState_SnkRp1_5) || (cc1State == kCCState_SnkRp3_0))
    {
        cc1State = kCCState_SnkRp;
    }
    if ((cc2State == kCCState_SnkRpDefault) || (cc2State == kCCState_SnkRp1_5) || (cc2State == kCCState_SnkRp3_0))
    {
        cc2State = kCCState_SnkRp;
    }

    preState = pdInstance->curConnectState;
    switch (pdInstance->curConnectState)
    {
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
        case TYPEC_TOGGLE_SRC_FIRST:
        case TYPEC_TOGGLE_SNK_FIRST:
            PD_ConnectStateDrpToggleSrcOrSnkFirstProcess(pdInstance, cc1State, cc2State);
            break;
#endif

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
        case TYPEC_UNATTACHED_SRC:
            PD_ConnectStateSrcUnattachedSrcProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_ATTACH_WAIT_SRC:
            PD_ConnectStateSrcAttachWaitSrcProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_ATTACHED_SRC:
            PD_ConnectStateSrcAttachedSrcProcess(pdInstance, cc1State, cc2State);
            break;
#endif

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
        case TYPEC_DEAD_BATTERY_SNK:
            PD_ConnectStateSnkDeadBatteryProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_UNATTACHED_SNK:
            PD_ConnectStateSnkUnattachedSnkProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_ATTACH_WAIT_SNK:
            PD_ConnectStateSnkAttachWaitSnkProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_ATTACHED_SNK:
            PD_ConnectStateSnkAttachedSnkProcess(pdInstance, cc1State, cc2State);
            break;
#endif

#if (defined PD_CONFIG_AUDIO_ACCESSORY_SUPPORT) && (PD_CONFIG_AUDIO_ACCESSORY_SUPPORT)
        case TYPEC_AUDIO_ACCESSORY:
            PD_ConnectStateSrcAudioAccessoryProcess(pdInstance, cc1State, cc2State);
            break;
#endif

#if (defined PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
        case TYPEC_UNORIENTED_DEBUG_ACCESSORY_SRC:
            PD_ConnectStateSrcUnorientedDebugAccSrcProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_ORIENTED_DEBUG_ACCESSORY_SRC:
            PD_ConnectStateSrcOrientedDebugAccSrcProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_DEBUG_ACCESSORY_SNK:
            PD_ConnectStateSnkDebugAccSnkProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_ORIENTED_DEBUG_ACCESSORY_SNK:
            PD_ConnectStateSnkOrientedDebugAccSnkProcess(pdInstance, cc1State, cc2State);
            break;
#endif

#if (defined PD_CONFIG_TRY_SNK_SUPPORT) && (PD_CONFIG_TRY_SNK_SUPPORT)
        case TYPEC_TRY_SNK:
            PD_ConnectStateSrcTrySnkProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_TRY_WAIT_SRC:
            PD_ConnectStateSrcTryWaitSrcProcess(pdInstance, cc1State, cc2State);
            break;
#endif

#if (defined PD_CONFIG_TRY_SRC_SUPPORT) && (PD_CONFIG_TRY_SRC_SUPPORT)
        case TYPEC_TRY_SRC:
            PD_ConnectStateSnkTrySrcProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_TRY_WAIT_SNK:
            PD_ConnectStateSnkTryWaitSnkProcess(pdInstance, cc1State, cc2State);
            break;
#endif

#if (defined PD_CONFIG_SINK_ACCESSORY_SUPPORT) && (PD_CONFIG_SINK_ACCESSORY_SUPPORT)
        case TYPEC_UNATTACHED_ACCESSORY:
            PD_ConnectStateSnkUnattachedAccessory(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_ATTACH_WAIT_ACCESSORY:
            PD_ConnectStateSnkAttachWaitAccessoryProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_POWERED_ACCESSORY:
            PD_ConnectStateSnkPoweredAccessoryProcess(pdInstance, cc1State, cc2State);
            break;

        case TYPEC_UNSUPPORTED_ACCESSORY:
            PD_ConnectStateSrcUnsupportedAccProcess(pdInstance, cc1State, cc2State);
            break;
#endif

        default:
            break;
    }

    if (pdInstance->vbusDischargeInProgress == kVbus_TypecDischarge)
    {
        infoVal = PD_VBUS_POWER_STATE_VSAFE0V_MASK;
        PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &infoVal);
        if (
#if (defined PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE) && (PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE)
            (PD_TimerCheckValidTimeOut(pdInstance, tTypeCVbusMinDischargeTimer)) &&
            (infoVal & PD_VBUS_POWER_STATE_VSAFE0V_MASK)
#else
            (infoVal & PD_VBUS_POWER_STATE_VSAFE0V_MASK)
#endif
        )
        {
            pdInstance->vbusDischargeInProgress = kVbus_NoDischarge;
            PD_DpmDischargeVbus(pdInstance, 0);
            PD_TimerClear(pdInstance, tTypeCVbusMaxDischargeTimer);
            PD_TimerClear(pdInstance, tTypeCVbusMinDischargeTimer);
        }
        else if (PD_TimerCheckValidTimeOut(pdInstance, tTypeCVbusMaxDischargeTimer))
        {
            pdInstance->vbusDischargeInProgress = kVbus_NoDischarge;
            PD_DpmDischargeVbus(pdInstance, 0);
        }
        else
        {
        }
    }

    if (pdInstance->curConnectState != preState)
    {
        PD_ConnectCheckRa(pdInstance, cc1State, cc2State);
        PD_StackSetEvent(pdInstance, PD_TASK_EVENT_OTHER);
        PD_ConnectSetupNewState(pdInstance);
    }
    return pdInstance->curConnectState;
}

static uint8_t PD_ConnectDeadBatteryCheck(pd_instance_t *pdInstance)
{
    pd_phy_get_cc_state_t ccState;
    uint32_t infoVal;

    PD_PhyControl(pdInstance, PD_PHY_GET_CC_LINE_STATE, &ccState);
    infoVal = PD_VBUS_POWER_STATE_VBUS_MASK;
    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &infoVal);

    /* Check for an existing sink connection */
    if (infoVal & PD_VBUS_POWER_STATE_VBUS_MASK)
    {
        /* CC_STATUS should be valid for dead battery bootup, so it is not dead battery if CC_STATUS shows no connection
         */
        if ((ccState.cc1State == kCCState_SrcOpen) && (ccState.cc2State == kCCState_SrcOpen))
        {
            return 0;
        }

        /* Check for an existing oriented debug accessory sink connection */
        if (((ccState.cc1State == kCCState_SnkRpDefault) || (ccState.cc1State == kCCState_SnkRp1_5) ||
             (ccState.cc1State == kCCState_SnkRp3_0)) &&
            ((ccState.cc2State == kCCState_SnkRpDefault) || (ccState.cc2State == kCCState_SnkRp1_5) ||
             (ccState.cc2State == kCCState_SnkRp3_0)) &&
            (ccState.cc1State != ccState.cc2State))
        {
            return 0;
        }

        if ((ccState.cc1State == kCCState_SnkRpDefault) || (ccState.cc1State == kCCState_SnkRp1_5) ||
            (ccState.cc1State == kCCState_SnkRp3_0) || (ccState.cc2State == kCCState_SnkRpDefault) ||
            (ccState.cc2State == kCCState_SnkRp1_5) || (ccState.cc2State == kCCState_SnkRp3_0))
        {
            return 1;
        }
    }
    return 0;
}

void PD_ConnectSetPowerProgress(pd_instance_t *pdInstance, uint8_t state)
{
    if ((state != kVbusPower_Invalid) && (pdInstance->inProgress != state))
    {
        pdInstance->inProgress = state;
        PD_PhyControl(pdInstance, PD_PHY_SET_VBUS_TRANSFORM_STATE, &state);
    }
}

TypeCState_t PD_ConnectGetInitRoleState(pd_instance_t *pdInstance)
{
    TypeCState_t newState = TYPEC_DISABLED;

    if (pdInstance->pdPowerPortConfig->typecRole != kPowerConfig_SourceOnly)
    {
        if (PD_ConnectDeadBatteryCheck(pdInstance))
        {
            return TYPEC_DEAD_BATTERY_SNK;
        }
    }

    switch (pdInstance->pdPowerPortConfig->typecRole)
    {
        case kPowerConfig_SourceOnly:
            newState = TYPEC_UNATTACHED_SRC;
            break;

        case kPowerConfig_SourceDefault:
            /* check dead battery */
            newState = TYPEC_UNATTACHED_SRC;
            break;

        case kPowerConfig_SinkOnly:
        case kPowerConfig_SinkDefault:
            /* check dead battery */
            newState = TYPEC_UNATTACHED_SNK;
            break;

        case kPowerConfig_DRPToggling:
        case kPowerConfig_DRPSourcingDevice:
        case kPowerConfig_DRPSinkingHost:
            if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) &&
                (pdInstance->pdPowerPortConfig->drpTryFunction == kTypecTry_Src))
            {
                newState = TYPEC_TOGGLE_SRC_FIRST;
            }
            else
            {
                newState = TYPEC_TOGGLE_SNK_FIRST;
            }
            break;

        default:
            break; /* error */
    }

    return newState;
}

void PD_ConnectInitRole(pd_instance_t *pdInstance, uint8_t errorRecovery)
{
    TypeCState_t newState;
    uint32_t infoVal;

    /*
    The ErrorRecovery state is where the port removes the terminations from the CC1 and CC2
    pins for tErrorRecovery followed by transitioning to the appropriate Unattached.SNK or
    Unattached.SRC state based on port type. This is the equivalent of forcing a detach event
    and looking for a new attach.
    */
    if (errorRecovery)
    {
        if (pdInstance->curConnectState != TYPEC_ERROR_RECOVERY)
        {
            pdInstance->ccUsed = kPD_CCInvalid;
            PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
            pdInstance->curConnectState = TYPEC_ERROR_RECOVERY;
            PD_ConnectSetupNewState(pdInstance);
        }

        PD_TimerStart(pdInstance, tDelayTimer, T_ERROR_RECOVERY);

        while (!PD_TimerCheckValidTimeOut(pdInstance, tDelayTimer))
        {
        }
    }

    PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
    pdInstance->vbusDischargeInProgress               = kVbus_NoDischarge;
    pdInstance->ccUsed                                = kPD_CCInvalid;
    pdInstance->enterAttachedSRCFromTypeCStateMachine = 0;

    newState = PD_ConnectGetInitRoleState(pdInstance);
    if (newState != pdInstance->curConnectState)
    {
        /* In the process of connecting, just change the role */
        if (((newState == TYPEC_UNATTACHED_SNK) && (pdInstance->curConnectState == TYPEC_ATTACH_WAIT_SNK)) ||
            ((newState == TYPEC_UNATTACHED_SRC) && (pdInstance->curConnectState == TYPEC_ATTACH_WAIT_SRC)))
        {
            /* continue the original state machine */
        }
        /* Not connected */
        else
        {
            if ((newState != TYPEC_DISABLED) && (newState != TYPEC_DEAD_BATTERY_SNK))
            {
                infoVal = PD_VBUS_POWER_STATE_VSYS_MASK;
                PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &infoVal);
                if (infoVal & PD_VBUS_POWER_STATE_VSYS_MASK)
                {
                    infoVal = PD_VBUS_POWER_STATE_VSAFE0V_MASK;
                    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &infoVal);
                    if (!(infoVal & PD_VBUS_POWER_STATE_VSAFE0V_MASK))
                    {
                        /* VBUS Discharge:
                           If state is not disabled, and we are not in dead battery, enable vbus discharge on startup */
                        pdInstance->vbusDischargeInProgress = kVbus_ApplyTypecDischarge;
                    }
                }
            }

            pdInstance->curConnectState = newState;
            pdInstance->ccUsed          = kPD_CCInvalid;
            PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
            PD_ConnectSetupNewState(pdInstance);
        }
    }
}

void PD_ConnectSetPRSwapRole(pd_instance_t *pdInstance, uint8_t powerRole)
{
    TypeCState_t newState;

    newState = pdInstance->curConnectState;
    if ((newState == TYPEC_ATTACHED_SRC) && (powerRole == kPD_PowerRoleSink))
    {
        newState = TYPEC_ATTACHED_SNK;
    }
    else if ((newState == TYPEC_ATTACHED_SNK) && (powerRole == kPD_PowerRoleSource))
    {
        newState = TYPEC_ATTACHED_SRC;
    }
    else
    {
        return; /* ERROR */
    }
    if (newState != pdInstance->curConnectState)
    {
        pdInstance->curConnectState = newState;
        PD_ConnectSetupNewState(pdInstance);
    }
}

void PD_ConnectAltModeEnterFail(pd_instance_t *pdInstance, uint8_t pdConnected)
{
    if (pdConnected)
    {
        if (pdInstance->curConnectState == TYPEC_POWERED_ACCESSORY)
        {
            pdInstance->curConnectState = TYPEC_UNSUPPORTED_ACCESSORY;
            PD_ConnectSetupNewState(pdInstance);
        }
    }
    else
    {
        if (pdInstance->curConnectState == TYPEC_POWERED_ACCESSORY)
        {
            pdInstance->curConnectState = TYPEC_UNATTACHED_SNK;
            PD_ConnectSetupNewState(pdInstance);
        }
    }
}

uint8_t PD_ConnectCheck(pd_instance_t *pdInstance)
{
    PD_ConnectStateMachine(pdInstance);
    return PD_ConnectState(pdInstance);
}

TypeCState_t PD_ConnectGetStateMachine(pd_instance_t *pdInstance)
{
    return pdInstance->curConnectState;
}
