                                                                                                                                                                                                                                /*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017,2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "usb_host_cdc.h"
#include "fsl_debug_console.h"
#include "host_cdc.h"
#include "fsl_common.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */
#include "app.h"
#include "fsl_component_serial_manager.h"
#if ((!USB_HOST_CONFIG_KHCI) && (!USB_HOST_CONFIG_EHCI) && (!USB_HOST_CONFIG_OHCI) && (!USB_HOST_CONFIG_IP3516HS))
#error Please enable USB_HOST_CONFIG_KHCI, USB_HOST_CONFIG_EHCI, USB_HOST_CONFIG_OHCI, or USB_HOST_CONFIG_IP3516HS in file usb_host_config.
#endif

#include "usb_phy.h"
#include "usb_host_ehci.h"
#include "usb_host_hci.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void USB_HostClockInit(void);
extern void USB_HostIsrEnable(void);
extern void USB_HostTaskFn(void *param);
void BOARD_InitHardware(void);
extern void UART_UserRxCallback(void *callbackParam,
                                serial_manager_callback_message_t *message,
                                serial_manager_status_t status);
extern void UART_UserTxCallback(void *callbackParam,
                                serial_manager_callback_message_t *message,
                                serial_manager_status_t status);
/*******************************************************************************
 * Variables
 ******************************************************************************/

usb_host_handle g_hostHandle;
volatile uint8_t g_AttachFlag;
USB_RAM_ADDRESS_ALIGNMENT(4) static uint8_t s_serialWriteHandleBuffer[SERIAL_MANAGER_WRITE_HANDLE_SIZE];
USB_RAM_ADDRESS_ALIGNMENT(4) static uint8_t s_serialReadHandleBuffer[SERIAL_MANAGER_READ_HANDLE_SIZE];
serial_write_handle_t g_UartTxHandle;
serial_write_handle_t g_UartRxHandle;

extern char usbRecvUart[USB_HOST_CDC_UART_RX_MAX_LEN];

/*******************************************************************************
 * Code
 ******************************************************************************/

void USB_OTG1_IRQHandlerx(void)
{
    USB_HostEhciIsrFunction(g_hostHandle);
}

void USB_OTG2_IRQHandlerx(void)
{
    USB_HostEhciIsrFunction(g_hostHandle);
}

void USB_HostClockInit(void)
{
    uint32_t usbClockFreq;
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };
    usbClockFreq = 24000000;
    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, usbClockFreq);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, usbClockFreq);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, usbClockFreq);
    }
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

/*!
 * @brief host callback function.
 *
 * device attach/detach callback function.
 *
 * @param deviceHandle           device handle.
 * @param configurationHandle attached device's configuration descriptor information.
 * @param event_code           callback event code, please reference to enumeration host_event_t.
 *
 * @retval kStatus_USB_Success              The host is initialized successfully.
 * @retval kStatus_USB_NotSupported         The application don't support the configuration.
 */
usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                           usb_host_configuration_handle configurationHandle,
                           uint32_t event_code)
{
    usb_status_t status;
    status = kStatus_USB_Success;

    switch (event_code & 0x0000FFFFU)
    {
        case kUSB_HostEventAttach:
            status = USB_HostCdcEvent(deviceHandle, configurationHandle, event_code);
            break;
        case kUSB_HostEventNotSupported:
            usb_echo("device not supported.\r\n");
            break;

        case kUSB_HostEventEnumerationDone:
            status = USB_HostCdcEvent(deviceHandle, configurationHandle, event_code);
            break;

        case kUSB_HostEventDetach:
            status = USB_HostCdcEvent(deviceHandle, configurationHandle, event_code);
            break;

        case kUSB_HostEventEnumerationFail:
            usb_echo("enumeration failed\r\n");
            break;

        default:
            break;
    }
    return status;
}

/*!
 * @brief app initialization.
 */
void APP_init(void)
{
    status_t status = (status_t)kStatus_SerialManager_Error;
    g_UartTxHandle  = (serial_write_handle_t)&s_serialWriteHandleBuffer[0];
    g_UartRxHandle  = (serial_read_handle_t)&s_serialReadHandleBuffer[0];
    status          = (status_t)SerialManager_OpenWriteHandle(g_serialHandle, g_UartTxHandle);
    assert(kStatus_SerialManager_Success == status);
    //(void)SerialManager_InstallTxCallback(g_UartTxHandle, UART_UserTxCallback, &g_UartTxHandle);

    status = (status_t)SerialManager_OpenReadHandle(g_serialHandle, g_UartRxHandle);
    assert(kStatus_SerialManager_Success == status);
    //(void)SerialManager_InstallRxCallback(g_UartRxHandle, UART_UserRxCallback, &g_UartRxHandle);

    //SerialManager_ReadNonBlocking(g_UartRxHandle, (uint8_t *)&usbRecvUart[0], USB_HOST_CDC_UART_RX_MAX_LEN);

    g_AttachFlag = 0;

    USB_HostCdcInitBuffer();

    USB_HostClockInit();

#if ((defined FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    status = USB_HostInit(CONTROLLER_ID, &g_hostHandle, USB_HostEvent);
    if (status != kStatus_USB_Success)
    {
        usb_echo("host init error\r\n");
        return;
    }
    USB_HostIsrEnable();

    usb_echo("host init done\r\n");
    usb_echo("This example requires that the CDC device uses Hardware flow\r\n");
    usb_echo(
        "if the device does't support it, please set USB_HOST_UART_SUPPORT_HW_FLOW to zero and rebuild this "
        "project\r\n");
    usb_echo("Type strings, then the string\r\n");
    usb_echo("will be echoed back from the device\r\n");
}
static void usb_call_back_test(void *param, usb_host_transfer_t *transfer, usb_status_t status)
{

}


void low_level_tt(void);

int main(void)
{
    BOARD_ConfigMPU();

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    low_level_tt();

    while(1)
        ;


#if 0
    APP_init();

    while (1)
    {
        USB_HostTaskFn(g_hostHandle);
        // USB_HostCdcTask(&g_cdc);
    }
#endif
}



#include "MIMXRT1176_cm7.h"
#include "fsl_iomuxc.h"
#include "ehci.h"

void usb1_host_pw_en(void)
{
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_09_USBPHY1_OTG_ID,0U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_10_USB_OTG1_PWR,0U);
}
const char desc[8] = {0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00};
void usb_soc_init(void)
{
    // SOC level init
    // usb1_pll_init();
    // phy_utmi_init();
    USB_HostClockInit();
    usb1_host_pw_en();
    EnableIRQ(USB_OTG1_IRQn);
}






























typedef enum
{
    s_idle,
    s_reset_done,
    s_begin_enumeration0,
    s_begin_enumeration1,
}state_usb_t;



// This global variable is not necessary if RTOS exists.


static char buf_in[64];
static ehci_control_transfer_t transfer;
state_usb_t state_usb1 = s_idle;
static ehci_callback_event_t event_usb1 = event_none;
static ehci_handle_t usb1_handle;

/*
    Call back from USB ISR.
    Just record event here.
    Do not call any ECHI API in call back.
*/
static void usb1_callback(ehci_callback_event_t event)
{
    // if rtos, use mail box to communicate with task.
    event_usb1 = event;
}

static void enumerate(void)
{
    uint32_t speed;
    speed = ehci_get_speed(&usb1_handle);
    
    transfer.usb       = USB_OTG1;
    transfer.addr      = 0;
    transfer.ep        = 0;
    transfer.speed     = speed;
    transfer.buf_setup = (char *)desc;
    transfer.len_setup = sizeof(desc);
    transfer.buf_in    = buf_in;
    transfer.len_in    = 64;
    ehci_control_in(&usb1_handle, &transfer);
}
static void delay_for_stable(void)
{
    int i;

    for(i=0; i<1000*1000*10; i++)
        ;
}
static void usb1_event_handler_port_attach(void)
{
    delay_for_stable();

    switch(state_usb1)
    {
        case s_idle:
            usb_printf("s_idle.\n");
            ehci_reset(&usb1_handle);
            state_usb1 = s_reset_done;
            break;

        case s_reset_done:
            usb_printf("s_reset_done0.\n");
            enumerate();
            state_usb1 = s_begin_enumeration0;
            break;
    }
    
    usb_printf("*exit\r\n");
}
static void usb1_event_handler_port_detach(void)
{
    state_usb1 = s_idle;
}
static void usb1_event_handler_transfer_done(void)
{
    usb_printf("-----------usb1_event_handler_transfer_done.\n");
    usb_printf("state_usb1 = %d.\n", state_usb1);

    switch(state_usb1)
    {
        case s_begin_enumeration0:
            usb_printf(" s_begin_enumeration0 \r\n");
            enumerate();
            state_usb1 = s_begin_enumeration1;
            break;
        case s_begin_enumeration1:
            usb_printf(" s_begin_enumeration1 \r\n");

            // now try to setup iso schedule.
            break;            
    }
}

static void usb1_task(void)
{
    switch(event_usb1)
    {
        case event_port_attach:
            __disable_irq();
            usb1_event_handler_port_attach(); 
            event_usb1 = event_none;
            __enable_irq();
            break;

        case event_port_detach:
            usb1_event_handler_port_detach();
            break;

        case event_transfer_done:
            __disable_irq();
            usb1_event_handler_transfer_done();
            event_usb1 = event_none;
            __enable_irq();
            break;
    }
}

void low_level_tt(void)
{
    usb1_handle.usb = USB_OTG1;
    usb1_handle.phy = USBPHY1;
    
    usb_printf("low_level_tt %s, %s --------------------------\n", __DATE__, __TIME__);
    usb_soc_init();

    ehci_init(&usb1_handle, ECHI_MODE_HOST);
    
    while(1)
    {
        usb1_task();
    }
}

void USB_OTG1_IRQHandler(void)
{
    ehci_isr(&usb1_handle, usb1_callback);
}

void USB_OTG2_IRQHandler(void)
{
    
}






#define mem_dump_printf PRINTF
/*
*   addr: memory address to begin dump
*   len:  in bytes.
*/
void mem_dump_32(void * addr, uint32_t len)
{
    uint32_t * p;
    uint32_t i;
    p = (uint32_t*)addr;
    i = 0;
    
    while(len >= 16)
    {
        mem_dump_printf("0x%08x:  %08x  %08x  %08x  %08x\r\n", &p[i], p[i], p[i+1], p[i+2], p[i+3]);
        i   += 4;
        len -= 16;        
    }
    
    if(len == 12)     mem_dump_printf("0x%x:  %08x  %08x  %08x  --------\r\n",         &p[i], p[i], p[i+1], p[i+2]);
    else if(len == 8) mem_dump_printf("0x%x:  %08x  %08x  --------  --------\r\n",     &p[i], p[i], p[i+1]);
    else if(len == 4) mem_dump_printf("0x%x:  %08x  --------  --------  --------\r\n", &p[i], p[i]);
}

void mem_dump_8(void * addr, uint32_t len) 
{
    uint8_t * p;
    uint32_t i;
    uint32_t left;
    p = (uint8_t*)addr;
    i = 0;
    
    while(len >= 8)
    {
        mem_dump_printf("0x%08x: %02x %02x %02x %02x %02x %02x %02x %02x \r\n", 
                        &p[i], p[i], p[i+1], p[i+2], p[i+3], p[i+4], p[i+5], p[i+6], p[i+7]);
        i   += 8;
        len -= 8;
    }
    
    if(len)
    {
        mem_dump_printf("0x%08x: ");
        left = 8;
        
        while(len)
        {
            mem_dump_printf("%02x ", p[i]);
            len--;
            i++;
            left--;
        }
        
        while(left)
        {
            mem_dump_printf("-- ");
            i++;
            left--;
        }
        
        mem_dump_printf("\r\n");
    }
}



