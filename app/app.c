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
char fw_version[] = "[FW:A:V3.6c]";
////////////////////////////////////////////////////////////////////////////////

//Buffer Level 1:  USB data stream buffer : 512 B
unsigned char usbBufferBulkOut[USBDATAEPSIZE];
unsigned char usbBufferBulkIn[USBDATAEPSIZE]; 

//Buffer Level 2:  FIFO Loop Data Buffer : 16384 B
unsigned char FIFOBufferBulkOut[USB_OUT_BUFFER_SIZE];
unsigned char FIFOBufferBulkIn[USB_IN_BUFFER_SIZE];  

//Buffer Level 3:  Double-buffer for I2S data : MAX 48*2*8*2 = 1536 B
unsigned char I2SBuffersOut[2][I2S_BUFFER_SIZE]; // Play
unsigned char I2SBuffersIn[2][I2S_BUFFER_SIZE];  // Record
// Current I2S buffer index.
volatile unsigned char i2s_buffer_out_index = 0;
volatile unsigned char i2s_buffer_in_index  = 0;

AUDIO_CFG  Audio_Configure[2]; //[0]: rec config. [1]: play config.
unsigned char audio_cmd_index     = AUDIO_CMD_IDLE ; 
 

kfifo_t bulkout_fifo;
kfifo_t bulkin_fifo;

volatile unsigned int i2s_play_buffer_size ; //real i2s paly buffer
volatile unsigned int i2s_rec_buffer_size ;  //real i2s record buffer


volatile bool bulkout_enable   = false ;
volatile bool bulkin_enable    = false ;
volatile bool bulkin_start     = true;
volatile bool bulkout_start    = true;
volatile bool bulkout_trigger  = false ;
volatile bool flag_stop        = false ;

volatile unsigned int bulkout_empt = 0;
volatile unsigned int debug_trans_counter1 = 0 ;
volatile unsigned int debug_trans_counter2 = 0 ;  
volatile unsigned int debug_trans_counter3 = 0 ;
volatile unsigned int debug_trans_counter4 = 0 ;  
volatile unsigned int debug_usb_dma_IN = 0;
volatile unsigned int debug_usb_dma_OUT = 0;

extern unsigned int debug_stall_counter;
//extern unsigned int debug_trans_counter1,debug_trans_counter2;
extern kfifo_t dbguart_fifo;

static unsigned int BO_free_size_max = 0;
static unsigned int BI_free_size_min = 100; 
static unsigned int DBGUART_free_size_min = 100; 

static unsigned int Stop_CMD_Miss_Counter = 0;

unsigned int counter_play = 0;
unsigned int counter_rec  = 0;

unsigned int test_dump = 0 ;



void Init_Bus_Matix( void )
{
     *(unsigned int*)0x400E03E4 = 0x4D415400 ; 
     *(unsigned int*)0x400E0240 = 0x011200FF ; 
     *(unsigned int*)0x400E0244 = 0x011200FF ; 
     *(unsigned int*)0x400E0280 = 0x30000 ; 
     *(unsigned int*)0x400E0288 = 0x30000 ;  
     printf("\r\nBUS PROTECT: %08X",  *(unsigned int*)0x400E03E8);
     printf("\r\nBUS  : %08X",  *(unsigned int*)0x400E0240);
     printf("\r\nBUS  : %08X",  *(unsigned int*)0x400E0244);
}



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
    //PIO_InitializeInterrupts( PIO_PRIORITY );     
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
    unsigned short sample_rate ; //not support 44.1khz now
    unsigned char  channels_play;

    channels_play = Audio_Configure[1].channel_num ;
    sample_rate   = Audio_Configure[1].sample_rate ;
    printf( "\r\nStart [%dth]Play[%dCH - %dHz] ...\r\n",counter_play++,channels_play,sample_rate);  
    i2s_play_buffer_size = sample_rate / 1000 * channels_play * 2 * 2;  
    SSC_Channel_Set_Tx( channels_play );  
    
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
    unsigned short sample_rate ; //not support 44.1kHz now
    unsigned char  channels_rec;
    
    channels_rec = Audio_Configure[0].channel_num ;
    sample_rate  = Audio_Configure[0].sample_rate ; 
    printf( "\r\nStart [%dth]Rec [%dCH - %dHz]...\r\n",counter_rec++,channels_rec,sample_rate);     
    i2s_rec_buffer_size  = sample_rate / 1000 * channels_rec  * 2 * 2; 
    SSC_Channel_Set_Rx( channels_rec ); 
     
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
    bulkin_start   = true ;      
    bulkin_enable  = true ;
    SSC_EnableReceiver(AT91C_BASE_SSC0);    //enable AT91C_SSC_RXEN 
      
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
    bulkout_enable  = true ;
    SSC_EnableTransmitter(AT91C_BASE_SSC0); //enable AT91C_SSC_TXEN  
       
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
    Init_Play_Setting();
    Init_Rec_Setting(); 

    SSC_Play_Start();
    SSC_Record_Start();
      
    bulkin_enable   = true ; 
    bulkout_enable  = true ;
  
    SSC_EnableBoth(AT91C_BASE_SSC0); //enable AT91C_SSC_TXEN aAT91C_SSC_RXEN   
    
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
    
#ifdef METHOD_BY_RESET_MCU             
    printf("\r\n Got command to reset MCU...MCM_RESET_CMD");                                   
    while(1) {
        AT91C_BASE_RSTC->RSTC_RCR = 0xA5000005 ; //reset MCU     
    }       
#endif  
     
    printf( "\r\nStop Play & Rec...\r\n"); 
    flag_stop        = true ;   
    delay_ms(50); //wait until DMA interruption done.
    //printf( "\r\nflag_stop Done\r\n");    
    bulkin_enable    = false ;
    bulkout_enable   = false ;

    SSC_Play_Stop();  
    SSC_Record_Stop();  
    delay_ms(50);
   
    printf("\r\nReset USB EP...");                    

    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAOUT].UDPHS_EPTCLRSTA = 0xFFFF; //AT91C_UDPHS_NAK_OUT | AT91C_UDPHS_TOGGLESQ | AT91C_UDPHS_FRCESTALL;                  
    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTCLRSTA  = 0xFFFF;//AT91C_UDPHS_TOGGLESQ | AT91C_UDPHS_FRCESTALL;
    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;
    delay_ms(10);

    //Reset Endpoint Fifos
    AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_DATAOUT;
    AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_DATAIN; 
   
    delay_ms(50);
    
//    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;  
//    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTCLRSTA  = AT91C_UDPHS_TOGGLESQ ;
//    delay_ms(50);
// 
//    AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_DATAIN ;
//    delay_ms(50); 
    //Reset_USBHS_HDMA( CDCDSerialDriverDescriptors_DATAIN );
//    
    //AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAOUT].UDPHS_EPTCLRSTA  = AT91C_UDPHS_TOGGLESQ ;
//   
//    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAOUT].UDPHS_EPTCLRSTA  = AT91C_UDPHS_NAK_OUT ;
//    delay_ms(50);    
//    AT91C_BASE_UDPHS->UDPHS_EPTRST =  1<<CDCDSerialDriverDescriptors_DATAOUT;
          
    //Reset_USBHS_HDMA( CDCDSerialDriverDescriptors_DATAOUT);   
        
    I2S_Init();  
    //SSC_Reset(); 
    
    delay_ms(50); 
    
    Init_Bulk_FIFO();    
    LED_Clear( USBD_LEDUDATA );
    
    bulkin_start    = true ; 
    bulkout_start   = true ;    
    bulkout_trigger = false ;     
    flag_stop       = false ;
    bulkout_empt    = 0;  
    
    //reset debug counters
    BO_free_size_max      = 0 ;
    BI_free_size_min      = 100 ; 
    total_received        = 0 ;
    total_transmit        = 0 ;
    error_bulkout_full    = 0 ;
    error_bulkout_empt    = 0 ;
    error_bulkin_full     = 0 ;
    error_bulkin_empt     = 0 ;    
    debug_trans_counter1  = 0 ;
    debug_trans_counter2  = 0 ;
    debug_trans_counter3  = 0 ;
    debug_trans_counter4  = 0 ; 
    debug_usb_dma_IN      = 0 ;
    debug_usb_dma_OUT     = 0 ;
    
    test_dump = 0 ;
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
    unsigned char err ;
    unsigned int  temp ;    
    
    if( audio_cmd_index == AUDIO_CMD_IDLE ) {
        return;
    }
    
    err = 0 ;
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
                //Audio_Start_Play_Rec();                 
                Audio_Start_Play();
                delay_ms(1);                
                Audio_Start_Rec(); 
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
                USART_WriteBuffer( AT91C_BASE_US0,(void *)fw_version, sizeof(fw_version) );  //Version string, no ACK 
            break;         
            
            case AUDIO_CMD_RESET:                 
                printf("\r\nReset USB EP...");                    
                //Reset Endpoint Fifos
                AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_DATAOUT;
                AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_DATAIN; 
                delay_ms(5);
                AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAOUT].UDPHS_EPTCLRSTA = 0xFFFF; //AT91C_UDPHS_NAK_OUT | AT91C_UDPHS_TOGGLESQ | AT91C_UDPHS_FRCESTALL;                  
                AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTCLRSTA  = 0xFFFF;//AT91C_UDPHS_TOGGLESQ | AT91C_UDPHS_FRCESTALL;
                AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;
                printf("Done.\r\n");
                
            break;  
            
            default:         
                err = ERR_CMD_TYPE;
            break;
        
        }
        
     }   
    
     if( audio_cmd_index != AUDIO_CMD_VERSION ) {       
         USART_Write( AT91C_BASE_US0, err, 0 ); //ACK
            
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
    unsigned int DBGUART_free_size ;
    
    static unsigned int counter;
     
    if( !(bulkout_enable || bulkin_enable) ) { 
        if( Check_SysTick_State() == 0 ) { 
              return;
        }
        printf("\rWaitting for USB trans start [lost %d stop]...",Stop_CMD_Miss_Counter);
        return ; 
    }
    
    //start print debug_after USB trans started
    if( (total_received < 102400) && (total_transmit < 102400) ){  
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
//        printf( "\rbulkin_start = %d , bulkin_start = %d, bulkin_fifo data size = %d ",                
//                            bulkin_enable,
//                            bulkin_start,
//                            kfifo_get_data_size(&bulkin_fifo));
//        return;
//    }
    //if(total_transmit >5000000 ) {  error_bulkin_full++; } //simulate bulkin fifo full error 
    //printf("\r\nPLAY %d, REC %d",counter_play++,counter_rec++); 
      
      printf("\rIN[Size:%6.6f MB, Full:%u, Empty:%u, FreeSize:%3u%>%3u%]  OUT[Size:%6.6f MB, Full:%u, Empty:%u, FreeSize:%3u%<%3u%]",
                         
               total_transmit/1000000.0,               
               error_bulkin_full,
               error_bulkin_empt,
               BI_free_size,
               BI_free_size_min,               
               
               total_received/1000000.0,
               error_bulkout_full,
               error_bulkout_empt,
               BO_free_size,           
               BO_free_size_max 
                   
            
             ); 
    
//     DBGUART_free_size = kfifo_get_free_space(&dbguart_fifo) ;
//     DBGUART_free_size = DBGUART_free_size * 100 / DBGUART_FIFO_SIZE;  
//     DBGUART_free_size_min = DBGUART_free_size < DBGUART_free_size_min ? DBGUART_free_size : DBGUART_free_size_min ;
//     printf( " [DBGUART:%3u%>%3u%]", DBGUART_free_size, DBGUART_free_size_min );                         
     
}



