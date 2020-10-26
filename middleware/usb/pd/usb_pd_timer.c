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
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
#include "usb_pd_alt_mode.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void PD_StackSetEvent(pd_instance_t *pdInstance, uint32_t event);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*
  1. timr has started & timr time out: timrsTimeOutState == 1;
  2. timr not start || timr time out: timrsRunningState == 0;
*/

static void PD_TimerCallback(pd_instance_t *pdInstance, uint8_t timrName)
{
    switch (timrName)
    {
        case tMsgHardResetCompleteTimer:
            /* do nothing */
            break;
#if 0
        /* if has USB function */
        case tUSBSuspendTimer:
        case tUSBTimer:
            break;
#endif

        default:
            /* do nothing, code will use _PD_TimerCheckInvalidOrTimeOut to check the timr */
            PD_StackSetEvent(pdInstance, PD_TASK_EVENT_TIME_OUT);
            break;
    }
}

static uint8_t PD_TimerCheckBit(volatile uint32_t *dataArray, uint8_t timrName)
{
    if (timrName >= PD_MAX_TIMER_COUNT)
    {
        return 0;
    }

    /* timr is not running */
    if (dataArray[(timrName / 32)] & (0x00000001uL << (timrName & 0x1Fu)))
    {
        return 1;
    }
    return 0;
}

pd_status_t PD_TimerClear(pd_handle pdHandle, uint8_t timrName)
{
    uint32_t bitMape;
    USB_OSA_SR_ALLOC();
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    if (timrName >= PD_MAX_TIMER_COUNT)
    {
        return kStatus_PD_Error;
    }

    bitMape = (uint32_t)((uint32_t)0x00000001uL << (timrName & 0x1Fu));

    USB_OSA_ENTER_CRITICAL();
    pdInstance->timrsRunningState[(timrName / 32)] &= (~bitMape);
    pdInstance->timrsTimeOutState[(timrName / 32)] &= (~bitMape);
    USB_OSA_EXIT_CRITICAL();

    return kStatus_PD_Success;
}

uint8_t PD_TimerCheckStarted(pd_handle pdHandle, uint8_t timrName)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    return PD_TimerCheckBit(&pdInstance->timrsRunningState[0], timrName) ||
           PD_TimerCheckBit(&pdInstance->timrsTimeOutState[0], timrName);
}

pd_status_t PD_TimerStart(pd_handle pdHandle, uint8_t timrName, uint16_t timrTime)
{
    uint32_t bitMape;
    USB_OSA_SR_ALLOC();
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    if (timrName >= PD_MAX_TIMER_COUNT)
    {
        return kStatus_PD_Error;
    }
    if (timrTime == 0)
    {
        return kStatus_PD_Error;
    }

    bitMape = (uint32_t)((uint32_t)0x00000001uL << (timrName & 0x1Fu));

    USB_OSA_ENTER_CRITICAL();
    pdInstance->timrsTimeValue[timrName] = timrTime;
    pdInstance->timrsTimeOutState[(timrName / 32)] &= (~(bitMape));
    pdInstance->timrsRunningState[(timrName / 32)] |= bitMape;
    USB_OSA_EXIT_CRITICAL();
    return kStatus_PD_Success;
}

uint8_t PD_TimerCheckInvalidOrTimeOut(pd_handle pdHandle, uint8_t timrName)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    return !PD_TimerCheckBit(&pdInstance->timrsRunningState[0], timrName);
}

uint8_t PD_TimerCheckValidTimeOut(pd_handle pdHandle, uint8_t timrName)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    return PD_TimerCheckBit(&pdInstance->timrsTimeOutState[0], timrName);
}

void PD_TimerCancelAllTimers(pd_handle pdHandle, uint8_t begin, uint8_t end)
{
    uint32_t index;
    uint32_t bitMape;
    USB_OSA_SR_ALLOC();
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    if ((begin >= PD_MAX_TIMER_COUNT) || (end >= PD_MAX_TIMER_COUNT))
    {
        return;
    }
    USB_OSA_ENTER_CRITICAL();
    for (index = begin; index <= end; ++index)
    {
        bitMape = (uint32_t)((uint32_t)0x00000001uL << (index & 0x1Fu));
        pdInstance->timrsRunningState[(index / 32)] &= (~bitMape);
        pdInstance->timrsTimeOutState[(index / 32)] &= (~bitMape);
    }
    USB_OSA_EXIT_CRITICAL();
}

void PD_TimerIsrFunction(pd_handle pdHandle)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    uint32_t index32;
    uint8_t index8;
    uint32_t bitMape;
    uint8_t currentIndex;

    if (pdHandle == NULL)
    {
        return;
    }

    for (index32 = 0; index32 < ((tTimerCount + 31) / 32); ++index32)
    {
        if (pdInstance->timrsRunningState[index32] != 0)
        {
            for (index8 = 0; index8 < 32; ++index8)
            {
                bitMape      = (uint32_t)((uint32_t)0x00000001uL << index8);
                currentIndex = (index32 * 32) + index8;
                if (currentIndex < tTimerCount)
                {
                    if (pdInstance->timrsRunningState[index32] & bitMape)
                    {
                        if (((pdInstance->timrsTimeValue[currentIndex])--) == 0)
                        {
                            PD_TimerCallback(pdInstance, currentIndex);
                            pdInstance->timrsTimeValue[currentIndex] = 0;
                            pdInstance->timrsRunningState[index32] &= (~(bitMape));
                            pdInstance->timrsTimeOutState[index32] |= (bitMape);
                        }
                    }
                }
            }
        }
    }
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
    PD_AltModeTimer1msISR();
#endif
}

void PD_TimerInit(pd_handle pdHandle)
{
    uint32_t index32;
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    for (index32 = 0; index32 < ((PD_MAX_TIMER_COUNT + 31) / 32); ++index32)
    {
        pdInstance->timrsRunningState[index32] = 0;
        pdInstance->timrsTimeOutState[index32] = 0;
    }
    for (index32 = 0; index32 < PD_MAX_TIMER_COUNT; ++index32)
    {
        pdInstance->timrsTimeValue[index32] = 0;
    }
}

uint8_t PD_TimerBusy(pd_handle pdHandle)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    for (uint32_t index32 = 0; index32 < ((tTimerCount + 31) / 32); ++index32)
    {
        if (pdInstance->timrsRunningState[index32] != 0)
        {
            return 1;
        }
    }
    return 0;
}
