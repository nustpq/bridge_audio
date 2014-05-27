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
*                                          APP PACKAGE
*
*                                         Atmel  SAM3U4C
*                                               on the
*                                      iSAM Audio Bridge Board
*
* Filename      : app.c
* Version       : V1.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#include <board.h>
#include <stdbool.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <tc/tc.h>
#include <usart/usart.h>
#include <ssc/ssc.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <pmc/pmc.h>
#include "kfifo.h"
#include "usb.h"
#include "app.h"


#ifndef AT91C_ID_TC0
    #define AT91C_ID_TC0 AT91C_ID_TC
#endif

////////////////////////////////////////////////////////////////////////////////
const char fw_version[] = "[FW:A:V2.8]";
////////////////////////////////////////////////////////////////////////////////

//Buffer Level 1:  USB data stream buffer : 512 B
unsigned char usbBufferBulkOut[USBDATAEPSIZE];
unsigned char usbBufferBulkIn[USBDATAEPSIZE]; 

//Buffer Level 2:  FIFO Loop Data Buffer : 16384 B
unsigned char FIFOBufferBulkOut[USB_OUT_BUFFER_SIZE];
unsigned char FIFOBufferBulkIn[USB_IN_BUFFER_SIZE];  

//Buffer Level 3:  Double-buffer for I2S data : MAX 48*2*6=576 B
unsigned char I2SBuffersOut[2][I2S_BUFFER_SIZE]; // Play
unsigned char I2SBuffersIn[2][I2S_BUFFER_SIZE];  // Record
// Current I2S buffer index.
unsigned char i2s_buffer_out_index = 0;
unsigned char i2s_buffer_in_index  = 0;

AUDIO_CFG  Audio_Configure[2]; //[0]: rec config. [1]: play config.
unsigned char audio_cmd_index = AUDIO_CMD_IDLE ; 

kfifo_t bulkout_fifo;
kfifo_t bulkin_fifo;

volatile unsigned int i2s_play_buffer_size ; //real i2s paly buffer
volatile unsigned int i2s_rec_buffer_size ;  //real i2s record buffer

unsigned short sample_rate ; //not support 44.1khz now
unsigned char  channels_play;
unsigned char  channels_rec;

volatile bool bulkout_start     = false ;
volatile bool bulkin_start      = false ;
volatile bool bulkin_enable     = true;
volatile bool bulkout_enable    = true;
volatile bool bulkout_kk        = false ;


extern unsigned int debug_stall_counter;
extern unsigned int debug_trans_counter1,debug_trans_counter2;

static unsigned int BO_free_size_max = 0;
static unsigned int BI_free_size_min = 100; 
static unsigned int Stop_CMD_Miss_Counter = 0;

unsigned int counter_play = 0;
unsigned int counter_rec  = 0;

/*
*********************************************************************************************************
*                                    Init_GPIO()
*
* Description :  Initialize LED GPIOs.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void Init_GPIO( void )
{
    PIO_InitializeInterrupts( PIO_PRIORITY );     
    LED_Configure(USBD_LEDPOWER);
    LED_Configure(USBD_LEDUDATA);    
    LED_Set(USBD_LEDPOWER); 
    LED_Set(USBD_LEDUDATA); 
  
}


/*
*********************************************************************************************************
*                                    Init_Play_Setting()
*
* Description :  Initialize USB bulk out (play) settings.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static void Init_Play_Setting( void )
{
    channels_play = Audio_Configure[1].channel_num ;
    sample_rate   = Audio_Configure[1].sample_rate ;
    printf( "\r\nStart Play[%dCH - %dHz] ...\r\n",channels_play,sample_rate);  
    i2s_play_buffer_size = sample_rate / 1000 * channels_play * 2;  
    SSC_Channel_Set( channels_play, 0 );  
  
}


/*
*********************************************************************************************************
*                                    Init_Rec_Setting()
*
* Description :  Initialize USB bulk in (record) settings.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static void Init_Rec_Setting( void )
{
    channels_rec = Audio_Configure[0].channel_num ;
    sample_rate  = Audio_Configure[0].sample_rate ; 
    printf( "\r\nStart Rec [%dCH - %dHz]...\r\n",channels_rec,sample_rate);     
    i2s_rec_buffer_size  = sample_rate / 1000 * channels_rec  * 2; 
    SSC_Channel_Set( 0, channels_rec ); 
}


/*
*********************************************************************************************************
*                                    Audio_Start_Rec()
*
* Description :  Start USB data transfer for recording.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static void Audio_Start_Rec( void )
{  
    Init_Rec_Setting();
    SSC_Record_Start();   
    bulkin_start  = true ;        
    SSC_EnableReceiver(AT91C_BASE_SSC0);    //enable aAT91C_SSC_RXEN    
}


/*
*********************************************************************************************************
*                                    Audio_Start_Play()
*
* Description :  Start USB data transfer for playing.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static void Audio_Start_Play( void )
{  
    Init_I2S_Buffer();   
    Init_Play_Setting();   
    SSC_Play_Start();
    bulkout_start  = true ;
    SSC_EnableTransmitter(AT91C_BASE_SSC0); //enable aAT91C_SSC_TXEN       
}


/*
*********************************************************************************************************
*                                    Audio_Start_Play_Rec()
*
* Description :  Start USB data transfer for both playing & recording.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static void Audio_Start_Play_Rec( void )
{  
    Init_I2S_Buffer(); 
    Init_Rec_Setting();
    Init_Play_Setting();
    SSC_Play_Start();
    SSC_Record_Start(); 
     
    bulkin_start   = true ; 
    bulkout_start  = true ;
    SSC_EnableBoth(AT91C_BASE_SSC0); //enable aAT91C_SSC_TXEN aAT91C_SSC_RXEN   
    
}


/*
*********************************************************************************************************
*                                    Audio_Stop()
*
* Description :  Stop USB data transfer.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static void Audio_Stop( void )
{  
#if( 1 )              
    SSC_Record_Stop();
    SSC_Play_Stop();          
    bulkin_start    = false ;
    bulkout_start   = false ;        
    bulkin_enable   = true ; 
    bulkout_enable  = true ;
    bulkout_kk      = false ; 
    //delay_ms(100);  //???????????? useless
    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;   
    delay_ms(100);    
    AT91C_BASE_UDPHS->UDPHS_EPTRST = (1<<CDCDSerialDriverDescriptors_DATAIN | 1<<CDCDSerialDriverDescriptors_DATAOUT);
    delay_ms(100);
    Reset_USBHS_HDMA( CDCDSerialDriverDescriptors_DATAIN );
    I2S_Init();            
    Init_Bulk_FIFO(); //???
    LED_Clear(USBD_LEDUDATA);
    printf( "\r\nStop Play&Rec...\r\n"); 
#else            
    printf("\r\n Got command to reset MCU...MCM_RESET_CMD");                                   
    while(1) {
        AT91C_BASE_RSTC->RSTC_RCR = 0xa5000005 ; //reset MCU     
    }            
#endif            
    //reset debug counters
    BO_free_size_max    = 0 ;
    BI_free_size_min    = 100 ; 
    total_received      = 0 ;
    total_transmit      = 0 ;
    error_bulkout_full  = 0 ;
    error_bulkout_empt  = 0 ;
    error_bulkin_full   = 0 ;
    error_bulkin_empt   = 0 ;
  
}



/*
*********************************************************************************************************
*                                    Audio_State_Control()
*
* Description : Process command from Host MCU via UART.
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static unsigned char state_check  = 0; //avoid re-start issue in case of not stop previous start 

void Audio_State_Control( void )
{
    
    unsigned char i, err = 0 ;
    unsigned int  temp ;
    
    if( audio_cmd_index == AUDIO_CMD_IDLE ) {
        return;
    }
    
    if( USBD_GetState() < USBD_STATE_CONFIGURED && 
        audio_cmd_index != AUDIO_CMD_VERSION ) {
        err = ERR_USB_STATE;
        
    } else {
    
        switch( audio_cmd_index ) {
            
            case AUDIO_CMD_START_REC :                
                if( state_check != 0 ) {
                    Audio_Stop(); 
                    Stop_CMD_Miss_Counter++;
                } 
                state_check = 1;
                Audio_Start_Rec();                
            break;

            case AUDIO_CMD_START_PLAY :                
                if( state_check != 0 ) {
                    Audio_Stop(); 
                    Stop_CMD_Miss_Counter++;
                } 
                state_check = 2;     
                Audio_Start_Play();
            break;
            
            case AUDIO_CMD_START_PALYREC :                
                if( state_check != 0 ) {
                    Audio_Stop(); 
                    Stop_CMD_Miss_Counter++;
                } 
                state_check = 3;        
                Audio_Start_Play_Rec();
            break;

            case AUDIO_CMD_STOP :   
                state_check = 0;  
                Audio_Stop();                
            break;   
        
            case AUDIO_CMD_CFG:
                temp = Audio_Configure[1].sample_rate / 1000 *  Audio_Configure[1].channel_num * 2;
                if( (temp << PLAY_BUF_DLY_N) > USB_OUT_BUFFER_SIZE ) { //play pre-buffer must not exceed whole play buffer
                    err = ERR_AUD_CFG;
                }
            break;
            
            case AUDIO_CMD_VERSION:              
            break;         
            
            default:         
                err = ERR_CMD_TYPE;
            break;
        
        }
        
     }   
    
     if( audio_cmd_index == AUDIO_CMD_VERSION ) {        
            //USART_WriteBuffer( AT91C_BASE_US0, (void*)&fw_version[0], sizeof(fw_version) );         
            for( i = 0; i< sizeof(fw_version) ; i++ ) {
                USART_Write( AT91C_BASE_US0, fw_version[i], 0 );
            }
            
     } else {          
            USART_Write( AT91C_BASE_US0, err, 0 );
            
     }
        
     audio_cmd_index = AUDIO_CMD_IDLE ;     
    
    
}


/*
*********************************************************************************************************
*                                    Debug_Info()
*
* Description : Print debug infomation via UART port.
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

void Debug_Info( void )
{
  
    unsigned int BO_free_size ;
    unsigned int BI_free_size ;     
    static unsigned int counter;
    
    if( !(bulkout_start || bulkin_start) ) { 
        printf("\rWaitting for USB trans start...[Miss Stop = %d]",Stop_CMD_Miss_Counter);
        return ; 
    }
    
    //start print debug_after USB trans started
    if( (total_received < 2048) && (total_transmit < 2048) ){  
        return;
    }  
         
    BO_free_size = kfifo_get_free_space(&bulkout_fifo) ;
    BO_free_size = BO_free_size * 100 / USB_OUT_BUFFER_SIZE;
    
    BI_free_size = kfifo_get_free_space(&bulkin_fifo) ;  
    BI_free_size = BI_free_size * 100 / USB_IN_BUFFER_SIZE; 
    
    BO_free_size_max = BO_free_size > BO_free_size_max ? BO_free_size : BO_free_size_max ;
    BI_free_size_min = BI_free_size < BI_free_size_min ? BI_free_size : BI_free_size_min ;
    
    if( Check_SysTick_State() == 0 ){ 
        return;
    }     

    if( counter++ % 20 == 0 ) { //20*100ms = 1s 
        printf("\r\n");        
    } 
    
    
    
//    if( (total_received>>1) > total_transmit ) {
//        printf( "\rbulkin_start = %d , bulkin_enable = %d, bulkin_fifo data size = %d ",                
//                            bulkin_start,
//                            bulkin_enable,
//                            kfifo_get_data_size(&bulkin_fifo));
//        return;
//      
//    }
    //if(total_transmit >5000000 ) {  error_bulkin_full++; } //simulate bulkin fifo full error 
    //printf("\r\nPLAY %d, REC %d",counter_play++,counter_rec++); 
    
//    printf( "\rBO:[%4.6f]MB-BI:[%4.6f]MB. eBI_F[%u]-eBI_E[%u]-[%3u%](%3u%). eBO_F[%u]-eBO_E[%u]-[%3u%](%3u%)",
//               total_received/1000000.0,               
//               total_transmit/1000000.0,
//               
//               error_bulkin_full,
//               error_bulkin_empt,
//               BI_free_size,
//               BI_free_size_min,
//               
//               error_bulkout_full,
//               error_bulkout_empt,
//               BO_free_size,
//               BO_free_size_max            
//               );
   
//      printf( "\rIN[Size:%10uB,Full:%u,Empty:%u,FreeSize:%3u%>%3u%] OUT[Size:%10uB,Full:%u,Empty:%u,FreeSize:%3u%<%3u%]",
//                             
//               total_transmit,               
//               error_bulkin_full,
//               error_bulkin_empt,
//               BI_free_size,
//               BI_free_size_min,               
//               
//               total_received,
//               error_bulkout_full,
//               error_bulkout_empt,
//               BO_free_size,
//               BO_free_size_max   
//                   
//               ); 
      
//      printf( "\rIN[Size:%6.6fMB,Full:%u,Empty:%u,FreeSize:%3u%>%3u%] OUT[Size:%6.6fMB,Full:%u,Empty:%u,FreeSize:%3u%<%3u%]",
//                             
//               total_transmit/1000000.0,               
//               error_bulkin_full,
//               error_bulkin_empt,
//               BI_free_size,
//               BI_free_size_min,               
//               
//               total_received/1000000.0,
//               error_bulkout_full,
//               error_bulkout_empt,
//               BO_free_size,
//               BO_free_size_max   
//                   
//               ); 
      
            printf( "\rIN[Size:%uMB,Full:%u,Empty:%u,FreeSize:%3u%>%3u%] OUT[Size:%uMB,Full:%u,Empty:%u,FreeSize:%3u%<%3u%]",
                             
               (unsigned int)(total_transmit>>20),               
               error_bulkin_full,
               error_bulkin_empt,
               BI_free_size,
               BI_free_size_min,               
               
               (unsigned int)(total_received>>20), 
               error_bulkout_full,
               error_bulkout_empt,
               BO_free_size,
               BO_free_size_max   
                   
               ); 
}



