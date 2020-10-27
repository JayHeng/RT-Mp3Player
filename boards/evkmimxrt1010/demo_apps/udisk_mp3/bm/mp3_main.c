/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "usb_host_msd.h"
#include "board.h"
#include "host_msd_fatfs.h"
#include "fsl_common.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
#include "app.h"
#include "mp3_config.h"
#include "board.h"
#include "GUI_FontIntern.h"

#if ((!USB_HOST_CONFIG_KHCI) && (!USB_HOST_CONFIG_EHCI) && (!USB_HOST_CONFIG_OHCI) && (!USB_HOST_CONFIG_IP3516HS))
#error Please enable USB_HOST_CONFIG_KHCI, USB_HOST_CONFIG_EHCI, USB_HOST_CONFIG_OHCI, or USB_HOST_CONFIG_IP3516HS in file usb_host_config.
#endif

#include "pin_mux.h"
#include "usb_phy.h"
#include "clock_config.h"

#include "sai.h"
#include "diskio.h"
#include "fsl_wm8960.h"
#include "ff.h"

/* SAI instance and clock */
#define DEMO_CODEC_WM8960
#define DEMO_SAI SAI1
#define DEMO_SAI_CHANNEL (0)
#define DEMO_SAI_BITWIDTH (kSAI_WordWidth16bits)
#define DEMO_SAI_IRQ SAI1_IRQn
#define SAI_UserIRQHandler SAI1_IRQHandler

/* IRQ */
#define DEMO_SAI_TX_IRQ SAI1_IRQn
#define DEMO_SAI_RX_IRQ SAI1_IRQn

/* DMA */
#define EXAMPLE_DMA DMA0
#define EXAMPLE_DMAMUX DMAMUX
#define EXAMPLE_TX_CHANNEL (0U)
#define EXAMPLE_RX_CHANNEL (1U)
#define EXAMPLE_SAI_TX_SOURCE kDmaRequestMuxSai1Tx
#define EXAMPLE_SAI_RX_SOURCE kDmaRequestMuxSai1Rx

/* Select Audio/Video PLL (786.48 MHz) as sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_SELECT (2U)
/* Clock pre divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER (0U)
/* Clock divider for sai1 clock source */
#define DEMO_SAI1_CLOCK_SOURCE_DIVIDER (63U)
/* Get frequency of sai1 clock */
#define DEMO_SAI_CLK_FREQ                                                        \
    (CLOCK_GetFreq(kCLOCK_AudioPllClk) / (DEMO_SAI1_CLOCK_SOURCE_DIVIDER + 1U) / \
     (DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER + 1U))

/* I2C instance and clock */
#define DEMO_I2C LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define DEMO_LPI2C_CLOCK_SOURCE_DIVIDER (5U)
/* Get frequency of lpi2c clock */
#define DEMO_I2C_CLK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (DEMO_LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

      
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t txHandle) = {0};
edma_handle_t dmaTxHandle = {0};
AT_NONCACHEABLE_SECTION_INIT(sai_edma_handle_t rxHandle) = {0};
edma_handle_t dmaRxHandle = {0};
sai_transfer_format_t format = {0};
AT_NONCACHEABLE_SECTION_ALIGN(uint8_t audioBuff[BUFFER_SIZE * BUFFER_NUM], 4);
codec_handle_t codecHandle = {0};
extern codec_config_t boardCodecConfig;
volatile bool istxFinished = false;
volatile bool isrxFinished = false;

/* static values for fatfs */
AT_NONCACHEABLE_SECTION(FATFS g_fileSystem); /* File system object */
AT_NONCACHEABLE_SECTION(FIL g_fileObject);   /* File object */
AT_NONCACHEABLE_SECTION(BYTE work[FF_MAX_SS]);

#include "GUI.h"
#include "BUTTON.h"
#include "CHECKBOX.h"
#include "SLIDER.h"
#include "DROPDOWN.h"
#include "RADIO.h"
#include "MULTIPAGE.h"


#ifndef GUI_NORMAL_FONT
#define GUI_NORMAL_FONT (&GUI_Font16_ASCII)
#endif

#ifndef GUI_LARGE_FONT
#define GUI_LARGE_FONT (&GUI_Font16B_ASCII)
#endif

#ifndef GUI_SCALE_FACTOR
#define GUI_SCALE_FACTOR 1
#endif

#ifndef GUI_SCALE_FACTOR_X
#define GUI_SCALE_FACTOR_X GUI_SCALE_FACTOR
#endif

#ifndef GUI_SCALE_FACTOR_Y
#define GUI_SCALE_FACTOR_Y GUI_SCALE_FACTOR
#endif

#define GUI_SCALE(a) ((int)((a) * (GUI_SCALE_FACTOR)))
#define GUI_SCALE_X(x) ((int)((x) * (GUI_SCALE_FACTOR_X)))
#define GUI_SCALE_Y(y) ((int)((y) * (GUI_SCALE_FACTOR_Y)))
#define GUI_SCALE_COORDS(x, y) GUI_SCALE_X(x), GUI_SCALE_Y(y)
#define GUI_SCALE_RECT(x0, y0, xs, ys) GUI_SCALE_X(x0), GUI_SCALE_Y(y0), GUI_SCALE_X(xs), GUI_SCALE_Y(ys)

#define GUI_ID_DRAWAREA (GUI_ID_USER + 0)
#define GUI_ID_PAGEWIN1 (GUI_ID_USER + 1)
#define GUI_ID_PAGEWIN2 (GUI_ID_USER + 2)

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief host callback function.
 *
 * device attach/detach callback function.
 *
 * @param deviceHandle        device handle.
 * @param configurationHandle attached device's configuration descriptor information.
 * @param eventCode           callback event code, please reference to enumeration host_event_t.
 *
 * @retval kStatus_USB_Success              The host is initialized successfully.
 * @retval kStatus_USB_NotSupported         The application don't support the configuration.
 */
static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode);

/*!
 * @brief app initialization.
 */
static void USB_HostApplicationInit(void);

extern void USB_HostClockInit(void);
extern void USB_HostIsrEnable(void);
extern void USB_HostTaskFn(void *param);
void BOARD_InitHardware(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief USB host msd fatfs instance global variable */
extern usb_host_msd_fatfs_instance_t g_MsdFatfsInstance;
usb_host_handle g_HostHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*
 * AUDIO PLL setting: Frequency = Fref * (DIV_SELECT + NUM / DENOM)
 *                              = 24 * (32 + 77/100)
 *                              = 786.48 MHz
 */
const clock_audio_pll_config_t audioPllConfig = {
    .loopDivider = 32,  /* PLL loop divider. Valid range for DIV_SELECT divider value: 27~54. */
    .postDivider = 1,   /* Divider after the PLL, should only be 1, 2, 4, 8, 16. */
    .numerator = 77,    /* 30 bit numerator of fractional loop divider. */
    .denominator = 100, /* 30 bit denominator of fractional loop divider */
};


void BOARD_EnableSaiMclkOutput(bool enable)
{
    if (enable)
    {
        IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK;
    }
    else
    {
        IOMUXC_GPR->GPR1 &= (~IOMUXC_GPR_GPR1_SAI1_MCLK_DIR_MASK);
    }
}
//uint8_t buf_decode[2304*2];
uint8_t mp3_decode_one_frame(uint8_t * buf_out);
#define BLOCK_SIZE (2304*2)
#define BLOCK_NUM (2)

uint8_t audio_buf[BLOCK_SIZE*BLOCK_NUM];
int buf_index = 0;
uint8_t audio_buf_dummy[BLOCK_SIZE] = {0};

void SAI_send_audio(uint8_t * buf, uint32_t size)
{

}
static void tx_send_dummy(void)
{
    sai_transfer_t xfer = {0};
    xfer.data           = audio_buf_dummy;
    xfer.dataSize       = BLOCK_SIZE;
    SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
    SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
    SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
    SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
}

static int flag_sai_tx = 0;
static uint8_t task_audio_tx(void)
{
    uint8_t RES = 0;
    if(flag_sai_tx)
    {
        flag_sai_tx = 0;
        uint8_t * buf;
        buf = audio_buf + buf_index*BLOCK_SIZE;
        buf_index ^= 1;
        //GPIO_PinWrite(GPIO3, 21U, 0U);
        RES = mp3_decode_one_frame(buf);
        //GPIO_PinWrite(GPIO3, 21U, 1U);
        if(0 == RES)
          return RES;
        sai_transfer_t xfer;
        xfer.data           = buf;
        xfer.dataSize       = BLOCK_SIZE;
        SAI_TransferSendEDMA(DEMO_SAI, &txHandle, &xfer);
    }
    return 1;
}


static void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    flag_sai_tx = 1;   
    //task_audio_tx();
/*
    sendCount++;
    emptyBlock++;

    if (sendCount == beginCount)
    {
        istxFinished = true;
        SAI_TransferTerminateSendEDMA(base, handle);
        sendCount = 0;
    }
*/
}


static void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
//    receiveCount++;
//    fullBlock++;
//
//    if (receiveCount == beginCount)
//    {
//        isrxFinished = true;
//        SAI_TransferTerminateReceiveEDMA(base, handle);
//        receiveCount = 0;
//    }
}


void USB_OTG1_IRQHandler(void)
{
    USB_HostEhciIsrFunction(g_HostHandle);
}

void USB_HostClockInit(void)
{
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };

    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
    CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
}

void USB_HostIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbHOSTEhciIrq[] = USBHS_IRQS;
    irqNumber                = usbHOSTEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];
/* USB_HOST_CONFIG_EHCI */

/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
#endif
    EnableIRQ((IRQn_Type)irqNumber);
}

void USB_HostTaskFn(void *param)
{
    USB_HostEhciTaskFunction(param);
}

/*!
 * @brief USB isr function.
 */

#if ((defined USB_HOST_CONFIG_COMPLIANCE_TEST) && (USB_HOST_CONFIG_COMPLIANCE_TEST))
extern usb_status_t USB_HostTestEvent(usb_device_handle deviceHandle,
                                      usb_host_configuration_handle configurationHandle,
                                      uint32_t eventCode);
#endif

static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode)
{
#if ((defined USB_HOST_CONFIG_COMPLIANCE_TEST) && (USB_HOST_CONFIG_COMPLIANCE_TEST))
    usb_host_configuration_t *configuration;
    usb_status_t status1;
    usb_status_t status2;
    uint8_t interfaceIndex = 0;
#endif
    usb_status_t status = kStatus_USB_Success;
    switch (eventCode)
    {
        case kUSB_HostEventAttach:
#if ((defined USB_HOST_CONFIG_COMPLIANCE_TEST) && (USB_HOST_CONFIG_COMPLIANCE_TEST))
            status1 = USB_HostTestEvent(deviceHandle, configurationHandle, eventCode);
            status2 = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            if ((status1 == kStatus_USB_NotSupported) && (status2 == kStatus_USB_NotSupported))
            {
                status = kStatus_USB_NotSupported;
            }
#else
            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
#endif
            break;

        case kUSB_HostEventNotSupported:
#if ((defined USB_HOST_CONFIG_COMPLIANCE_TEST) && (USB_HOST_CONFIG_COMPLIANCE_TEST))
            configuration = (usb_host_configuration_t *)configurationHandle;
            for (interfaceIndex = 0; interfaceIndex < configuration->interfaceCount; ++interfaceIndex)
            {
                if (((usb_descriptor_interface_t *)configuration->interfaceList[interfaceIndex].interfaceDesc)
                        ->bInterfaceClass == 9U) /* 9U is hub class code */
                {
                    break;
                }
            }

            if (interfaceIndex < configuration->interfaceCount)
            {
                usb_echo("unsupported hub\r\n");
            }
            else
            {
                usb_echo("Unsupported Device\r\n");
            }
#else
            usb_echo("Unsupported Device\r\n");
#endif
            break;

        case kUSB_HostEventEnumerationDone:
#if ((defined USB_HOST_CONFIG_COMPLIANCE_TEST) && (USB_HOST_CONFIG_COMPLIANCE_TEST))
            status1 = USB_HostTestEvent(deviceHandle, configurationHandle, eventCode);
            status2 = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            if ((status1 != kStatus_USB_Success) && (status2 != kStatus_USB_Success))
            {
                status = kStatus_USB_Error;
            }
#else
            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
#endif
            break;

        case kUSB_HostEventDetach:
#if ((defined USB_HOST_CONFIG_COMPLIANCE_TEST) && (USB_HOST_CONFIG_COMPLIANCE_TEST))
            status1 = USB_HostTestEvent(deviceHandle, configurationHandle, eventCode);
            status2 = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            if ((status1 != kStatus_USB_Success) && (status2 != kStatus_USB_Success))
            {
                status = kStatus_USB_Error;
            }
#else
            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
#endif
            break;

        default:
            break;
    }
    return status;
}

static void USB_HostApplicationInit(void)
{
    usb_status_t status = kStatus_USB_Success;

    USB_HostClockInit();

#if ((defined FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    status = USB_HostInit(CONTROLLER_ID, &g_HostHandle, USB_HostEvent);
    if (status != kStatus_USB_Success)
    {
        usb_echo("host init error\r\n");
        return;
    }
    USB_HostIsrEnable();

    usb_echo("host init done\r\n");
}

uint32_t LPSPI1_GetFreq(void)
{
   return CLOCK_GetFreq(kCLOCK_OscClk) / 3;
}

uint32_t LPI2C1_GetFreq(void)
{
    return CLOCK_GetFreq(kCLOCK_OscClk) / 2;
}

static DROPDOWN_Handle hDropdown0;
static RADIO_Handle hRadio0;
static CHECKBOX_Handle hCheck0;
static WM_HWIN hDrawArea;

//static 
SLIDER_Handle hSlider0;
//static 
SLIDER_Handle hSlider1;
TEXT_Handle hTEXT0;
TEXT_Handle hTEXT1;
TEXT_Handle hTEXT2;
TEXT_Handle hTEXT3;
TEXT_Handle hTEXT4;
TEXT_Handle hTEXT5;
TEXT_Handle hTEXT6;
TEXT_Handle hTEXT7;
TEXT_Handle hTEXT8;

static SPINBOX_Handle hSpinbox0;
static PROGBAR_Handle hProgbar0;

static const GUI_COLOR color_list[]      = {GUI_BLACK,   GUI_YELLOW, GUI_ORANGE, GUI_RED,
                                       GUI_MAGENTA, GUI_BLUE,   GUI_CYAN,   GUI_GREEN};
static const GUI_POINT triangle_points[] = {
    {GUI_SCALE(0), GUI_SCALE(0)}, {GUI_SCALE(-50), GUI_SCALE(100)}, {GUI_SCALE(50), GUI_SCALE(100)}};

static void cbDrawArea(WM_MESSAGE *pMsg)
{
    switch (pMsg->MsgId)
    {
        case WM_PAINT:
            GUI_SetColor(color_list[DROPDOWN_GetSel(hDropdown0)]);
            switch (RADIO_GetValue(hRadio0))
            {
                case 0:
                    if (CHECKBOX_GetState(hCheck0))
                    {
                        GUI_FillRect(GUI_SCALE_X(70) - GUI_SCALE(50), GUI_SCALE_Y(70) - GUI_SCALE(50),
                                     GUI_SCALE_X(70) + GUI_SCALE(50), GUI_SCALE_Y(70) + GUI_SCALE(50));
                    }
                    else
                    {
                        GUI_DrawRect(GUI_SCALE_X(70) - GUI_SCALE(50), GUI_SCALE_Y(70) - GUI_SCALE(50),
                                     GUI_SCALE_X(70) + GUI_SCALE(50), GUI_SCALE_Y(70) + GUI_SCALE(50));
                    }
                    break;
                case 1:
                    if (CHECKBOX_GetState(hCheck0))
                    {
                        GUI_FillPolygon(triangle_points, 3, GUI_SCALE_COORDS(70, 20));
                    }
                    else
                    {
                        GUI_DrawPolygon(triangle_points, 3, GUI_SCALE_COORDS(70, 20));
                    }
                    break;
                case 2:
                    if (CHECKBOX_GetState(hCheck0))
                    {
                        GUI_FillEllipse(GUI_SCALE_COORDS(70, 70), GUI_SCALE(50), GUI_SCALE(50));
                    }
                    else
                    {
                        GUI_DrawEllipse(GUI_SCALE_COORDS(70, 70), GUI_SCALE(50), GUI_SCALE(50));
                    }
                    break;
            }
            break;
        default:
            WM_DefaultProc(pMsg);
            break;
    }
}

static void cbPageWin1(WM_MESSAGE *pMsg)
{
    int NCode;
    int Id;

    switch (pMsg->MsgId)
    {
        case WM_NOTIFY_PARENT:
            Id    = WM_GetId(pMsg->hWinSrc);
            NCode = pMsg->Data.v;

            switch (Id)
            {
                case GUI_ID_RADIO0:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_VALUE_CHANGED:
                            WM_InvalidateWindow(hDrawArea);
                            break;
                    }
                    break;

                case GUI_ID_DROPDOWN0:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_SEL_CHANGED:
                            WM_InvalidateWindow(hDrawArea);
                            break;
                    }
                    break;

                case GUI_ID_CHECK0:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_VALUE_CHANGED:
                            WM_InvalidateWindow(hDrawArea);
                            break;
                    }
                    break;
            }
            break;

        default:
            WM_DefaultProc(pMsg);
            break;
    }
}

static void cbPageWin2(WM_MESSAGE *pMsg)
{
    int NCode;
    int Id;

    switch (pMsg->MsgId)
    {
        case WM_NOTIFY_PARENT:
            Id    = WM_GetId(pMsg->hWinSrc);
            NCode = pMsg->Data.v;

            switch (Id)
            {
                case GUI_ID_SLIDER0:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_VALUE_CHANGED:
                            PROGBAR_SetValue(hProgbar0, SLIDER_GetValue(hSlider0));
                            break;
                    }
                    break;

                case GUI_ID_SLIDER1:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_VALUE_CHANGED:
                            SPINBOX_SetValue(hSpinbox0, SLIDER_GetValue(hSlider1));
                            break;
                    }
                    break;

                case GUI_ID_SPINBOX0:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_VALUE_CHANGED:
                            SLIDER_SetValue(hSlider1, SPINBOX_GetValue(hSpinbox0));
                            break;
                    }
                    break;
            }
            break;

        default:
            WM_DefaultProc(pMsg);
            break;
    }
}
int Speed0 = 0;
uint8_t Button0 = 0;
uint8_t Button1 = 0;
uint8_t Button2 = 0;


#define AUDIO 1
int main(void)
{
    BOARD_ConfigMPU();
    
#if AUDIO
    BOARD_InitPins_Audio();
#else
    BOARD_InitPins();
#endif
    
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    USB_HostApplicationInit();

#if AUDIO
    sai_config_t config;
    uint32_t mclkSourceClockHz = 0U, masterClockHz = 0U;
    edma_config_t dmaConfig = {0};
    CLOCK_InitAudioPll(&audioPllConfig);
        /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, DEMO_LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, DEMO_LPI2C_CLOCK_SOURCE_DIVIDER);

    /*Clock setting for SAI1*/
    CLOCK_SetMux(kCLOCK_Sai1Mux, DEMO_SAI1_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Sai1PreDiv, DEMO_SAI1_CLOCK_SOURCE_PRE_DIVIDER);
    CLOCK_SetDiv(kCLOCK_Sai1Div, DEMO_SAI1_CLOCK_SOURCE_DIVIDER);

    /*Enable MCLK clock*/
    BOARD_EnableSaiMclkOutput(true); 
    BOARD_Codec_I2C_Init();

    PRINTF("SAI Demo started!\n\r");
    //gp_timer_init();

    /* Create EDMA handle */
    /*
     * dmaConfig.enableRoundRobinArbitration = false;
     * dmaConfig.enableHaltOnError = true;
     * dmaConfig.enableContinuousLinkMode = false;
     * dmaConfig.enableDebugMode = false;
     */
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(EXAMPLE_DMA, &dmaConfig);
    EDMA_CreateHandle(&dmaTxHandle, EXAMPLE_DMA, EXAMPLE_TX_CHANNEL);
    EDMA_CreateHandle(&dmaRxHandle, EXAMPLE_DMA, EXAMPLE_RX_CHANNEL);

    DMAMUX_Init(EXAMPLE_DMAMUX);
    DMAMUX_SetSource(EXAMPLE_DMAMUX, EXAMPLE_TX_CHANNEL, (uint8_t)EXAMPLE_SAI_TX_SOURCE);
    DMAMUX_EnableChannel(EXAMPLE_DMAMUX, EXAMPLE_TX_CHANNEL);
    DMAMUX_SetSource(EXAMPLE_DMAMUX, EXAMPLE_RX_CHANNEL, (uint8_t)EXAMPLE_SAI_RX_SOURCE);
    DMAMUX_EnableChannel(EXAMPLE_DMAMUX, EXAMPLE_RX_CHANNEL);

    /* Init SAI module */
    /*
     * config.masterSlave = kSAI_Master;
     * config.mclkSource = kSAI_MclkSourceSysclk;
     * config.protocol = kSAI_BusLeftJustified;
     * config.syncMode = kSAI_ModeAsync;
     * config.mclkOutputEnable = true;
     */
    SAI_TxGetDefaultConfig(&config);
    SAI_TxInit(DEMO_SAI, &config);

    /* Initialize SAI Rx */
    SAI_RxGetDefaultConfig(&config);
    SAI_RxInit(DEMO_SAI, &config);

    /* Configure the audio format */
    format.bitWidth = kSAI_WordWidth16bits;
    format.channel = 0U;
    format.sampleRate_Hz = SAMPLE_RATE;
#if (defined FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER && FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) || \
    (defined FSL_FEATURE_PCC_HAS_SAI_DIVIDER && FSL_FEATURE_PCC_HAS_SAI_DIVIDER)
    masterClockHz = OVER_SAMPLE_RATE * format.sampleRate_Hz;
#else
    masterClockHz = DEMO_SAI_CLK_FREQ;
#endif
    format.protocol = config.protocol;
    format.stereo = kSAI_Stereo;
    format.isFrameSyncCompact = true;
#if defined(FSL_FEATURE_SAI_FIFO_COUNT) && (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    format.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;
#endif

    /* Use default setting to init codec */
    CODEC_Init(&codecHandle, &boardCodecConfig);
    CODEC_SetFormat(&codecHandle, masterClockHz, format.sampleRate_Hz, format.bitWidth);
#if defined CODEC_USER_CONFIG
    BOARD_Codec_Config(&codecHandle);
#endif

    SAI_TransferTxCreateHandleEDMA(DEMO_SAI, &txHandle, txCallback, NULL, &dmaTxHandle);
    SAI_TransferRxCreateHandleEDMA(DEMO_SAI, &rxHandle, rxCallback, NULL, &dmaRxHandle);

    mclkSourceClockHz = DEMO_SAI_CLK_FREQ;
    SAI_TransferTxSetFormatEDMA(DEMO_SAI, &txHandle, &format, mclkSourceClockHz, masterClockHz);
    SAI_TransferRxSetFormatEDMA(DEMO_SAI, &rxHandle, &format, mclkSourceClockHz, masterClockHz);

    /* Enable interrupt to handle FIFO error */
    SAI_TxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
    SAI_RxEnableInterrupts(DEMO_SAI, kSAI_FIFOErrorInterruptEnable);
    EnableIRQ(DEMO_SAI_TX_IRQ);
    EnableIRQ(DEMO_SAI_RX_IRQ);
#endif

     CLOCK_SetDiv(kCLOCK_LpspiDiv, 1);
    /* Set Lpspi clock source. */
    CLOCK_SetMux(kCLOCK_LpspiMux, 0);
    
    GUI_Init();
    
    MULTIPAGE_SetDefaultFont(GUI_LARGE_FONT);

    TEXT_SetDefaultFont(&GUI_Font13B_ASCII);

#if 1
    /* Create multipage widget */
    MULTIPAGE_Handle hMultipage0;
    hMultipage0 = MULTIPAGE_CreateEx(GUI_SCALE_RECT(10, 10, 300, 220), 0, WM_CF_SHOW, 0, GUI_ID_MULTIPAGE0);

    WM_HWIN hPageWin;
    /* Create window for page 2 and add it */
    hPageWin = WINDOW_CreateEx(GUI_SCALE_RECT(0, 0, 300, 200), WM_HBKWIN, 0, 0, GUI_ID_PAGEWIN2, cbPageWin2);
    MULTIPAGE_AddPage(hMultipage0, hPageWin, "Audio Box based on RT1010");
    hTEXT0 = TEXT_CreateEx(115, 20, 30, 25, hPageWin, WM_CF_SHOW, 0, GUI_ID_TEXT0, "N");
    TEXT_SetTextColor(hTEXT0,GUI_YELLOW);
    TEXT_SetFont(hTEXT0,&GUI_Font32B_ASCII);
    hTEXT1 = TEXT_CreateEx(145, 20, 30, 25, hPageWin, WM_CF_SHOW, 0, GUI_ID_TEXT0, "X");
    TEXT_SetTextColor(hTEXT1,GUI_BLUE);
    TEXT_SetFont(hTEXT1,&GUI_Font32B_ASCII);
    hTEXT2 = TEXT_CreateEx(175, 20, 30, 25, hPageWin, WM_CF_SHOW, 0, GUI_ID_TEXT0, "P");
    TEXT_SetTextColor(hTEXT2,GUI_GREEN);
    TEXT_SetFont(hTEXT2,&GUI_Font32B_ASCII);
    
#if 0
    /* Create widgets on page 2 */
    hSlider0 = SLIDER_CreateEx(GUI_SCALE_RECT(100, 60, 190, 30), hPageWin, WM_CF_SHOW, 0, GUI_ID_SLIDER0);
    SLIDER_SetWidth(hSlider0, GUI_SCALE(10));
    SLIDER_SetRange(hSlider0, -29000, 29000);
    SLIDER_SetValue(hSlider0, 0);
#endif

    hTEXT3 = TEXT_CreateEx(10, 60, 300, 15, hPageWin, WM_CF_SHOW, 0, GUI_ID_TEXT3, "NXP RT1010 Inside");
    hTEXT4 = TEXT_CreateEx(10, 80, 300, 15, hPageWin, WM_CF_SHOW, 0, GUI_ID_TEXT4, "High performing Arm Cortex-M7");
    hTEXT5 = TEXT_CreateEx(10, 100, 300, 15, hPageWin, WM_CF_SHOW, 0, GUI_ID_TEXT5, "2517 CoreMark @ 500 MHz");
    hTEXT6 = TEXT_CreateEx(10, 120, 300, 15, hPageWin, WM_CF_SHOW, 0, GUI_ID_TEXT6, "Up to 128KB Tightly Coupled Memory (TCM)");
    hTEXT7 = TEXT_CreateEx(10, 140, 300, 15, hPageWin, WM_CF_SHOW, 0, GUI_ID_TEXT7, "80 LQFP packages for low-cost PCB designs");
    hTEXT8 = TEXT_CreateEx(10, 160, 300, 15, hPageWin, WM_CF_SHOW, 0, GUI_ID_TEXT8, "Supported by MCUXpresso SDK, IDE and Config Tools");
    
#if 0   
    PROGBAR_SKINFLEX_PROPS pProps = {0};
    PROGBAR_GetSkinFlexProps(&pProps, 0);
    pProps.ColorText = GUI_BLACK;
    PROGBAR_SetSkinFlexProps(&pProps, 0);

    hButton1 = 
        BUTTON_CreateEx(GUI_SCALE_RECT(5, 25, 90, 30), hPageWin, WM_CF_SHOW, PROGBAR_CF_HORIZONTAL, GUI_ID_BUTTON1);
    BUTTON_SetFont(hButton1, GUI_LARGE_FONT);
    BUTTON_SetText(hButton1 , "Stop");
    
    hButton2 = 
        BUTTON_CreateEx(GUI_SCALE_RECT(5, 60, 90, 30), hPageWin, WM_CF_SHOW, PROGBAR_CF_HORIZONTAL, GUI_ID_BUTTON2);
    BUTTON_SetFont(hButton2, GUI_LARGE_FONT);
    BUTTON_SetText(hButton2 , "Demo Mode");
#endif
#endif
    WM_SetDesktopColor(GUI_WHITE);//GUI_YELLOW GUI_WHITE
    WM_Exec();
    
    while (1)
    {
        USB_HostTaskFn(g_HostHandle);
        USB_HostMsdTask(&g_MsdFatfsInstance);
    }
}

int USBDISK_FatFsInit()
{
    /* If there is SDCard, Initialize SDcard and Fatfs */
    FRESULT error;

    const uint8_t driverNumberBuffer[3U] = {USBDISK + '0', ':', '/'};

    if (f_mount(&g_fileSystem, driverNumberBuffer, 0U))
    {
        PRINTF("Mount volume failed.\r\n");
        return -1;
    }
}

void Audio_task()
{
    uint8_t RES = 0;
    char mp3_play_song(char* fname);
    /* time delay */
    for (uint32_t freeClusterNumber = 0; freeClusterNumber < 10000; ++freeClusterNumber)
    {
        __ASM("nop");
    }
    USBDISK_FatFsInit();
    mp3_play_song(MP3_FILENAME);
    tx_send_dummy();
    while (1)
    {
        RES = task_audio_tx();
        if(RES == 0)
        {
          mp3_play_song(MP3_FILENAME);
          tx_send_dummy();
        }
        
#if 0
        /* Poll touch controller for update */
        if (BOARD_Touch_Poll())
        {
          Speed0 = SLIDER_GetValue(hSlider0);
#if 0
          Button0 = BUTTON_IsPressed(hButton0);
#endif
          Button1 = BUTTON_IsPressed(hButton1);
          Button2 = BUTTON_IsPressed(hButton2);
          //TEXT_SetDec(hTEXT8, Speed0, 5, 0, 1, 5);

#ifdef GUI_BUFFERS
            GUI_MULTIBUF_Begin();
#endif
            GUI_Exec();
#ifdef GUI_BUFFERS
            GUI_MULTIBUF_End();
#endif
        }
#endif
    }
}