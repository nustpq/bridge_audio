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
*                                           APP PACKAGE
*
*                                         Atmel  SAM3U4C
*                                               on the
*                                      iSAM Audio Bridge Board
*
* Filename      : i2s.c
* Version       : V1.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <ssc/ssc.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <stdbool.h>
#include <string.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <dmad/dmad.h>
#include <dma/dma.h>
#include "kfifo.h"
#include "app.h"
#include "usb.h"


const Pin SSC_Pins       = PINS_SSC_TX;
const Pin SSC_Sync_Pin   = PIN_SSC_RF;
//void Demo_Sine_Gen( unsigned char *pdata, unsigned int size, unsigned int REC_SR_Set );


/*
*********************************************************************************************************
*                                    Init_I2S_Buffer()
*
* Description :  Initialize I2S data buffers.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void Init_I2S_Buffer( void )
{
  
#if( false )   //debug use
    unsigned int i;
    unsigned short  *pInt;
    pInt = (unsigned short *)I2SBuffersOut[0] ;
    for( i = 0; i< (I2S_BUFFER_SIZE>>1);  ) {        
       *(pInt+i++) = 0x1111 ;      
       *(pInt+i++) = 0x3333 ;
       *(pInt+i++) = 0x5555 ;
       *(pInt+i++) = 0x7777 ;     
       *(pInt+i++) = 0xcccc ;
       *(pInt+i++) = 0x0000 ;   
    }   
    pInt = (unsigned short *)I2SBuffersOut[1] ;
    for( i = 0; i< (I2S_BUFFER_SIZE>>1);  ) {        
       *(pInt+i++) = 0x1111 ;      
       *(pInt+i++) = 0x3333 ;
       *(pInt+i++) = 0x5555 ;
       *(pInt+i++) = 0x7777 ;     
       *(pInt+i++) = 0xcccc ;
       *(pInt+i++) = 0x0000 ;   
    }   
    
#else  
    
    //Demo_Sine_Gen(I2SBuffersIn[0], I2S_BUFFER_SIZE, 48000);    
    memset((unsigned char *)I2SBuffersOut, 0, I2S_BUFFER_SIZE<<1); 
   
    
#endif
    
}  


/*
*********************************************************************************************************
*                                    fill_buf_debug()
*
* Description :  Debug use. fill buffer with speicifed data .
* Argument(s) :  *pChar : pointer to buffer
*                 size  : buffer size
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static unsigned char test_a = 0;
static unsigned int test_b = 0;
void fill_buf_debug( unsigned char *pChar, unsigned int size) 
{
    unsigned int i;
    unsigned short  *pInt;
    pInt = (unsigned short *)pChar;
//    for( i = 0; i< (size>>1);  ) { 
//       *(pInt+i++) =  0x1100 + test_a;      
//       *(pInt+i++) =  0x2200 + test_a; 
//       *(pInt+i++) =  0x3300 + test_a; 
//       *(pInt+i++) =  0x4400 + test_a;     
//       *(pInt+i++) =  0x5500 + test_a; 
//       *(pInt+i++) =  0x6600 + test_a;  
//       *(pInt+i++) =  0x7700 + test_a; 
//       *(pInt+i++) =  0x8800 + test_a; 
//       *(pInt+i++) =  0x9900 + test_a;      
//       *(pInt+i++) =  0xaa00 + test_a; 
//       *(pInt+i++) =  0xbb00 + test_a; 
//       *(pInt+i++) =  0xcc00 + test_a;     
//       *(pInt+i++) =  0xdd00 + test_a; 
//       *(pInt+i++) =  0xee00 + test_a;  
//       *(pInt+i++) =  0xff00 + test_a; 
//       *(pInt+i++) =  0x0000 + test_a; 
//    }
// //test_a++;
    for( i = 0; i< (size>>1); i++ ) { 
       *(pInt+i) =   test_a + test_b++; 
    }
    
    
}

unsigned char check_buf_debug( unsigned char *pChar, unsigned int size) 
{
    
#if( false )
    unsigned int i;
    unsigned short  *pInt;
    pInt = (unsigned short *)pChar;
    
    for( i = 0; i< (size>>1);  ) { 
//        if( *(pInt+i++) !=  0x1111 ) {
//           return 1 ;  
//        } 
        i++;
        if( *(pInt+i++) !=  0x2211 ) {
           return 2 ;  
        }        
//        if( *(pInt+i++) !=  0x55AA ) {
//           return 1 ;  
//        }   
        i++;
//        if( *(pInt+i++) !=  0x4433 ) {
//           return 1 ;  
//        }  
        i++;
    
    } 
#endif
    
    return 0;
  
}

void dump_buf_debug( unsigned char *pChar, unsigned int size) 
{
    unsigned int i;
    unsigned short  *pInt;
    pInt = (unsigned short *)pChar;
    
    for( i = 0; i< (size>>1);  ) { 
        printf(" 0x%04X",*(pInt+i++));
        if( i%16 == 0 ) {
            printf("\n\r");
        }   
    
    } 
  
}
/*
*********************************************************************************************************
*                                    HDMA_IrqHandler()
*
* Description :  SSC HDMA interruption service subroutine.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

void HDMA_IrqHandler(void)
{
    unsigned int status;  
    unsigned int temp;

    status = DMA_GetMaskedStatus();   

    if( status & ( 1 << BOARD_SSC_IN_DMA_CHANNEL) ) { //record
    
        TRACE_INFO_NEW_WP("-SI-") ;      
        //fill_buf_debug( (unsigned char *)I2SBuffersIn[i2s_buffer_in_index],i2s_rec_buffer_size);   //bulkin tes data for debug 
        if ( i2s_rec_buffer_size > kfifo_get_free_space( &bulkin_fifo ) ) { //if rec fifo buf full    
            kfifo_release(&bulkin_fifo, i2s_rec_buffer_size);       //discard oldest data for newest data  
            error_bulkin_full++; //bulkin fifo full        
        }
        if( error_bulkin_full ) {//force record data to fixed line to alert user record error...  
             memset((unsigned char *)I2SBuffersIn[i2s_buffer_in_index],0x10,i2s_rec_buffer_size);  
        }
//        if( test_dump++ == 0 ) {
//            memset((unsigned char *)I2SBuffersIn[i2s_buffer_in_index],0x20,i2s_rec_buffer_size); 
//        }
        //fill_buf_debug( (unsigned char *)I2SBuffersIn[i2s_buffer_in_index],i2s_rec_buffer_size);
        kfifo_put(&bulkin_fifo, (unsigned char *)I2SBuffersIn[i2s_buffer_in_index], i2s_rec_buffer_size) ;
        
        SSC_ReadBuffer(AT91C_BASE_SSC0, (void *)I2SBuffersIn[i2s_buffer_in_index], i2s_buffer_in_index, flag_stop ? 0 : i2s_rec_buffer_size);                      
        i2s_buffer_in_index ^= 1; 
        
        if ( bulkin_enable && bulkin_start && (!flag_stop) && ( (USBDATAEPSIZE<<1) <= kfifo_get_data_size(&bulkin_fifo)) ) {
            TRACE_INFO_NEW_WP("-LBI-") ;  
            bulkin_start = false ;
            error_bulkin_empt++;
            kfifo_get(&bulkin_fifo, usbBufferBulkIn, USBDATAEPSIZE); 
            CDCDSerialDriver_Write(  usbBufferBulkIn,
                                     USBDATAEPSIZE,
                                    (TransferCallback) UsbDataTransmit,
                                     0); 
        }
                           
    } 

////////////////////////////////////////////////////////////////////////////////

    if( status & ( 1 << BOARD_SSC_OUT_DMA_CHANNEL) ) {   //play 
       
        TRACE_INFO_NEW_WP("-SO-") ;              
        temp = kfifo_get_data_size(&bulkout_fifo);        
        TRACE_INFO_NEW_WP("\n\r[%d, %d]",temp,error_bulkout_empt);
        
        if( (i2s_play_buffer_size<<PLAY_BUF_DLY_N) <= (temp<<1)) { //play buffer delay (2^PLAY_BUF_DLY_N) ms       
            bulkout_trigger = true; //1st buffered enough data will trigger SSC Out           
        }        

        if ( (i2s_play_buffer_size <= temp) && bulkout_trigger ) { //play until buf have enough data  
            if( bulkout_empt ) {
                TRACE_INFO_NEW_WP( "\r\n ##IN1: %d, OUT: %d",bulkout_fifo.in, bulkout_fifo.out);
                if( bulkout_empt > bulkout_fifo.size ) {
                    bulkout_empt -= bulkout_fifo.size;
                    kfifo_release(&bulkout_fifo, bulkout_fifo.size);
                    memset((unsigned char *)I2SBuffersOut[i2s_buffer_out_index],0x00,i2s_play_buffer_size); //can pop sound gene          
                    bulkout_empt++;
                    error_bulkout_empt++;
                } else {
                    kfifo_release(&bulkout_fifo, bulkout_empt);
                    bulkout_empt = 0;
                }
                TRACE_INFO_NEW_WP( "\r\n ##IN2: %d, OUT: %d",bulkout_fifo.in, bulkout_fifo.out);
            }                        
             
            kfifo_get(&bulkout_fifo, (unsigned char *)I2SBuffersOut[i2s_buffer_out_index], i2s_play_buffer_size) ;
            TRACE_INFO_NEW_WP( "\r\n ##IN: %d, OUT: %d",bulkout_fifo.in, bulkout_fifo.out);

             
       } else {  //play buf empty , send silence : 0x00
            memset((unsigned char *)I2SBuffersOut[i2s_buffer_out_index],0x00,i2s_play_buffer_size); //can pop sound gene          
            error_bulkout_empt++; //bulkout fifo empty error                
            if( bulkout_trigger ) {               
                bulkout_empt++;           
            }
        }    
//     if(test_dump++ == 1000) {
//        dump_buf_debug((void *)I2SBuffersOut[i2s_buffer_out_index],i2s_play_buffer_size);   
//     }
        SSC_WriteBuffer(AT91C_BASE_SSC0, (void *)I2SBuffersOut[i2s_buffer_out_index], i2s_buffer_out_index, flag_stop ? 0 : i2s_play_buffer_size);             
        i2s_buffer_out_index ^= 1;     
        
        if ( bulkout_enable && bulkout_start && (!flag_stop) && ((USBDATAEPSIZE<<1) <= kfifo_get_free_space(&bulkout_fifo)) ) { //
            TRACE_INFO_NEW_WP("-LBO-") ;         
            bulkout_start = false ;
            error_bulkout_full++;
            CDCDSerialDriver_Read(   usbBufferBulkOut,
                                     USBDATAEPSIZE,
                                     (TransferCallback) UsbDataReceived,
                                     0);
        }
      
    } 
    
 
}


/*
*********************************************************************************************************
*                                    SSC_Play_Start()
*
* Description :  Start SSC for playing(OUT).
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void SSC_Play_Start(void)
{

    i2s_buffer_out_index = 0 ;    
    //Start transmitting WAV file to SSC   
    //disable BTC and CBTC int 
    DMA_DisableIt( 1 << (BOARD_SSC_OUT_DMA_CHANNEL + 0) );
    DMA_DisableChannel(BOARD_SSC_OUT_DMA_CHANNEL);    
    //Fill DMA buffer
 
    SSC_WriteBuffer_Start(AT91C_BASE_SSC0, (void *)I2SBuffersOut[0], (void *)I2SBuffersOut[1], i2s_play_buffer_size);
    //i2s_buffer_out_index ^= 1;     
    
    DMA_EnableIt( 1 << (BOARD_SSC_OUT_DMA_CHANNEL + 0)  );
    DMA_EnableChannel(BOARD_SSC_OUT_DMA_CHANNEL); 
      
}


/*
*********************************************************************************************************
*                                    SSC_Record_Start()
*
* Description :  Start SSC for recording(IN).
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void SSC_Record_Start(void)
{ 

    i2s_buffer_in_index = 0 ;    
    // Start transmitting WAV file to SSC   
    //disable BTC and CBTC int
    DMA_DisableIt( 1 << (BOARD_SSC_IN_DMA_CHANNEL + 0) );
    DMA_DisableChannel(BOARD_SSC_IN_DMA_CHANNEL);    
    // Fill DMA buffer
    
    SSC_ReadBuffer_Start(AT91C_BASE_SSC0, (void *)I2SBuffersIn[0], (void *)I2SBuffersIn[1], i2s_rec_buffer_size);
    //i2s_buffer_in_index ^= 1;    
    
    DMA_EnableIt( 1 << (BOARD_SSC_IN_DMA_CHANNEL + 0)  );
    DMA_EnableChannel(BOARD_SSC_IN_DMA_CHANNEL);    
     
}


/*
*********************************************************************************************************
*                                    SSC_Play_Stop()
*
* Description :  Stop SSC for playing(OUT).
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void SSC_Play_Stop(void)
{
    
    SSC_DisableTransmitter(AT91C_BASE_SSC0);   
    DMA_DisableIt( 1 << (BOARD_SSC_OUT_DMA_CHANNEL + 0) ); 
    DMA_DisableChannel(BOARD_SSC_OUT_DMA_CHANNEL);        
    DMA_ClearAutoMode(BOARD_SSC_OUT_DMA_CHANNEL);

}


/*
*********************************************************************************************************
*                                    SSC_Record_Stop()
*
* Description :  Stop SSC for recording(IN).
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void SSC_Record_Stop(void)
{  
    
    SSC_DisableReceiver(AT91C_BASE_SSC0);    
    DMA_DisableIt( 1 << (BOARD_SSC_IN_DMA_CHANNEL + 0) );  
    DMA_DisableChannel(BOARD_SSC_IN_DMA_CHANNEL);      
    DMA_ClearAutoMode(BOARD_SSC_IN_DMA_CHANNEL);
// test_a++;
// test_b=0;
}


/*
*********************************************************************************************************
*                                    I2S_Init()
*
* Description :  Initialize I2S module.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/


void I2S_Init( void )
{  
    
    printf("\r\nInit I2S ..."); 
    
    PIO_Configure(&SSC_Pins, 1);    
    PIO_Configure(&SSC_Sync_Pin, 1) ;  
    
    IRQ_DisableIT(BOARD_AT73C213_SSC_ID);
    IRQ_DisableIT(AT91C_ID_HDMA);
    
    SSC_Init( MCK );  
    
    // Initialize DMA controller.    
    DMAD_Initialize(BOARD_SSC_IN_DMA_CHANNEL);
    DMAD_Initialize(BOARD_SSC_OUT_DMA_CHANNEL);   

    // Configure and enable the SSC interrupt
    IRQ_ConfigureIT(AT91C_ID_HDMA, HDMA_PRIORITY, HDMA_IrqHandler);
    IRQ_EnableIT(AT91C_ID_HDMA);
    
    printf("Done\r\n");
    
    
}



