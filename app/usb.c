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
*                                          USB APP PACKAGE
*
*                                         Atmel  SAM3U4C
*                                             on the
*                                      iSAM Audio Bridge Board
*
* Filename      : usb.c
* Version       : V1.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#include <pio/pio.h>
#include <pio/pio_it.h>
#include <usart/usart.h>
#include <dbgu/dbgu.h>
#include <irq/irq.h>
#include <board.h>
#include <string.h>
#include <stdbool.h>
#include <utility/led.h>
#include <usb/device/core/USBD.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <utility/trace.h>
#include "kfifo.h"
#include <ssc/ssc.h>
#include <app.h>



/// Use for power management
#define STATE_IDLE    0
/// The USB device is in suspend state
#define STATE_SUSPEND 4
/// The USB device is in resume state
#define STATE_RESUME  5

//------------------------------------------------------------------------------
//      Internal variables
//------------------------------------------------------------------------------
/// State of USB, for suspend and resume
unsigned char USBState = STATE_IDLE;

unsigned long long total_received = 0 ;
unsigned long long total_transmit = 0 ;
unsigned int error_bulkout_full  = 0 ;
unsigned int error_bulkout_empt  = 0 ;
unsigned int error_bulkin_full   = 0 ;
unsigned int error_bulkin_empt   = 0 ;


//------------------------------------------------------------------------------
//         VBus monitoring (optional)
//------------------------------------------------------------------------------
#if defined(PIN_USB_VBUS)

#define VBUS_CONFIGURE()  VBus_Configure()

/// VBus pin instance.
static const Pin pinVbus = PIN_USB_VBUS;


/*
*********************************************************************************************************
*                                    ISR_Vbus()
*
* Description :  VBUS interruption service. Check USB attach detection GPIO status and init USB module
* Argument(s) : *pPin: pointer to the USB attach detection GPIO.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static void ISR_Vbus(const Pin *pPin)
{
    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {

        TRACE_INFO("VBUS conn\r\n");
        USBD_Connect();
    }
    else {

        TRACE_INFO("VBUS discon\r\n");
        USBD_Disconnect();
    }
}


/*
*********************************************************************************************************
*                                    VBus_Configure()
*
* Description :  Configures the VBus pin to trigger an interrupt when the level on that pin changes.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static void VBus_Configure( void )
{
    TRACE_INFO("VBus configuration\r\n");

    // Configure PIO
    PIO_Configure(&pinVbus, 1);
    PIO_ConfigureIt(&pinVbus, ISR_Vbus);
    PIO_EnableIt(&pinVbus);

    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {

        // if VBUS present, force the connect
        TRACE_INFO("VBUS conn\r\n");
        USBD_Connect();
    }
    else {
        USBD_Disconnect();
    }           
}

#else
    #define VBUS_CONFIGURE()    USBD_Connect()
#endif //#if defined(PIN_USB_VBUS)


/*
//------------------------------------------------------------------------------
//         Callbacks re-implementation
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Invoked when the USB device leaves the Suspended state. By default,
/// configures the LEDs.
//------------------------------------------------------------------------------

void USBDCallbacks_Resumed(void)
{
    // Initialize LEDs
    LED_Configure(USBD_LEDPOWER);
    LED_Set(USBD_LEDPOWER);
    LED_Configure(USBD_LEDUSB);
    LED_Clear(USBD_LEDUSB);
    USBState = STATE_RESUME;
}

//------------------------------------------------------------------------------
/// Invoked when the USB device gets suspended. By default, turns off all LEDs.
//------------------------------------------------------------------------------
void USBDCallbacks_Suspended(void)
{
    // Turn off LEDs
    LED_Clear(USBD_LEDPOWER);
    LED_Clear(USBD_LEDUSB);
    
    if (USBD_GetState() >= USBD_STATE_CONFIGURED)
        USBState = STATE_SUSPEND;
}
*/




////////////////////////////////////////////////////////////////////////////////
/*
const unsigned char test_vec[] = 
{
  '1','2','3','4','5','6','7','8'  
};

static unsigned int check_bulkout_data( unsigned char *pBuf, unsigned int length )
{  
    unsigned int i;    
    unsigned int *pInt = (unsigned int*)pBuf;
    unsigned int *pVec = (unsigned int*)test_vec;
    
    for( i = 0; i < (length>>3); i++ ) {      
      if( *pInt++ != *pVec ) { //1234
        return 1;
      }   
      if( *pInt++ != *(pVec+1) ) {//5678
        return 1;
      }          
    }
    return 0;  
  
}
*/



void Init_Bulk_FIFO( void )
{
    kfifo_t *pfifo;
    
    pfifo = &bulkout_fifo;
    kfifo_init_static(pfifo, FIFOBufferBulkOut, USB_OUT_BUFFER_SIZE);
    pfifo = &bulkin_fifo;
    kfifo_init_static(pfifo, FIFOBufferBulkIn, USB_IN_BUFFER_SIZE);

}  


/*
*********************************************************************************************************
*                                    UsbDataReceived()
*
* Description :  USB bulk out process call back subroutine
* Argument(s) :  unused    : not used argument.
*                status    : curren USB data process status, must be SUCCESS to operation
*                received  : received data in buffer
*                remaining : remaining data in buffer
*
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void UsbDataReceived(  unsigned int unused,
                              unsigned char status,
                              unsigned int received,
                              unsigned int remaining )
{
    TRACE_INFO_NEW_WP("\r\n#BO:") ;
    //Play
    if ( ! bulkout_enable ) {
        printf("\r\nstatus %d, bulkout_enable %d\r\n",status, bulkout_enable);
        return ;
    }
    if ( status == USBD_STATUS_SUCCESS ) {     
        //while( received > kfifo_get_free_space( &bulkout_fifo ) ) ; //wait if buf is full   
        kfifo_put(&bulkout_fifo, usbBufferBulkOut, received);
//        if( check_buf_debug(usbBufferBulkOut, received) ) {
//                printf("\r\n Check USB BI buf err : \r\n");
//                dump_buf_debug(usbBufferBulkOut, received );
//                while(1){ DBGUART_Service();};
//            }
        
        if ( USBDATAEPSIZE <= kfifo_get_free_space( &bulkout_fifo ) ) { //enouth free buffer                      
            CDCDSerialDriver_Read(    usbBufferBulkOut,
                                      USBDATAEPSIZE,
                                      (TransferCallback) UsbDataReceived,
                                      0);        
        } else { //usb out too fast                     
            bulkout_start  = true ; 
            //printf("\r\n");                   
        }     
        total_received += received ; 
     
    }  else {      
        printf( "\r\nERROR : UsbDataReceived: Transfer error\r\n" ); 
        
    }
    
    
}


/*
*********************************************************************************************************
*                                    UsbDataTransmit()
*
* Description :  USB bulk in process call back subroutine
* Argument(s) :  unused    : not used argument.
*                status    : curren USB data process status, must be SUCCESS to operation
*                received  : received data in buffer
*                remaining : remaining data in buffer
*
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void UsbDataTransmit(  unsigned int unused,
                              unsigned char status,
                              unsigned int transmit,
                              unsigned int remaining )
{
    TRACE_INFO_NEW_WP("\r\n#BI:") ;
    //printf("BI ");
    //Record    
    if ( ! bulkin_enable ) {
        printf("\r\nstatus %d, bulkin_enable %d\r\n",status, bulkin_enable);
        return ;
    }
    
    if ( status == USBD_STATUS_SUCCESS  ) {       
       
        if ( USBDATAEPSIZE <= kfifo_get_data_size(  &bulkin_fifo )  ) { //enouth data to send to PC
           
            kfifo_get(&bulkin_fifo, usbBufferBulkIn, USBDATAEPSIZE);              
            CDCDSerialDriver_Write( usbBufferBulkIn,
                                    USBDATAEPSIZE,
                                    (TransferCallback) UsbDataTransmit,
                                    0);       
        } else {                    
            bulkin_start  = true ; 
            //printf("*");
        }              
        total_transmit += transmit ; 
     
    }  else {
        CDCDSerialDriver_Write( usbBufferBulkIn,
                                USBDATAEPSIZE,
                                (TransferCallback) UsbDataTransmit,
                                0);  
        TRACE_WARNING( "\r\nERROR : UsbDataTransmit: Rr-transfer hit\r\n" );           
    }    
    
}


/*
*********************************************************************************************************
*                                    USB_Init()
*
* Description :  Initialize USB function.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void USB_Init(void)
{
  
    printf("\r\nInit USB ...");
  
   // If there is on board power, switch it off 
#ifdef PIN_USB_POWER_ENB
    const Pin pinUsbPwr = PIN_USB_POWER_ENB;
    PIO_Configure(&pinUsbPwr, 1); 
#endif 
    
  //external pull up seems not working now
#ifdef PIN_USB_PULLUP
    const Pin pinPullUp = PIN_USB_PULLUP;
    PIO_Configure(&pinPullUp,1);
#endif  
  
    LED_Configure(USBD_LEDPOWER);
    LED_Configure(USBD_LEDUSB);
    
    // BOT driver initialization
    CDCDSerialDriver_Initialize();
    
    // connect if needed
    VBUS_CONFIGURE();

    // Connect pull-up, wait for configuration
    USBD_Connect();

    Init_Bulk_FIFO();
    
    //while ( USBD_GetState() < USBD_STATE_CONFIGURED );
    
    // Start receiving data on the USB
    
    /*
    CDCDSerialDriver_Read(  usbBufferBulkOut,
                            USBDATAEPSIZE,
                            (TransferCallback) UsbDataReceived,
                            0);    
    
    CDCDSerialDriver_Write( usbBufferBulkIn,
                            504,//USBDATAEPSIZE,
                            (TransferCallback) UsbDataTransmit,
                            0);
    */
 
    LED_Clear(USBD_LEDUDATA); 
    printf("Done\r\n");
    
}

