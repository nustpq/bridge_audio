/*
*********************************************************************************************************
*                               iSAM TEST BENCH USB AUDIO APP PACKAGE
*
*                            (c) Copyright 2013 - 2016; Fortemedia Inc.; Nanjing, China
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            APP PACKAGE
*
*                                          Atmel SAM3U4C
*                                               on the
*                                      iSAM Audio Bridge Board
*
* Filename      : main.c
* Version       : V1.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#include <stdbool.h>
#include <tc/tc.h>
#include <utility/trace.h>
#include <usart/usart.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include "kfifo.h"
#include <usb.h>
#include <app.h>

#include <irq/irq.h>

int main( void )
{

    TRACE_CONFIGURE( DBGU_STANDARD, 115200, BOARD_MCK );
    
    printf("\r\n----------  USB Flood  --------------\r\n");
    printf("-- iSAM Audio Bridge Project  --\r\n");
    printf("-- HW version: %s --\r\n", BOARD_NAME);
    printf("-- SW version: %s --\r\n", fw_version);
    printf("   BOARD_MCK = %dMHz\r\n",BOARD_MCK/1000000);
    printf("   USBEndpointSize = %d B, PlayPreBuffer = %d ms\r\n", USBDATAEPSIZE, 1<<PLAY_BUF_DLY_N);
    printf("-- Compiled: %s %s by PQ--\r\n", __DATE__, __TIME__);
    printf("--------------------------------------------------\r\n");
    
    Timer0_Init();     
    SysTick_Init(); 
    UART_Init();
#if(1)    
#else
    USART_Write( AT91C_BASE_US0, 0, 0 ); //Send ACK
#endif
    USB_Init();
    I2S_Init();    
    printf("\r\n");
    
    while(1) {   
      
        Debug_Info();              
        Audio_State_Control(); 
      
    }
    
}
