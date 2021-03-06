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
* Version       : V2.0.0
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
char fw_version[] = "[FW:A:V4.7]";
////////////////////////////////////////////////////////////////////////////////

//Buffer Level 1:  USB data stream buffer : 512 B
unsigned char usbBufferBulkOut[USBDATAEPSIZE];
unsigned char usbBufferBulkIn[USBDATAEPSIZE]; 

//Buffer Level 2:  FIFO Loop Data Buffer : 16384 B
unsigned char FIFOBufferBulkOut[USB_OUT_BUFFER_SIZE];
unsigned char FIFOBufferBulkIn[USB_IN_BUFFER_SIZE];  

//Buffer Level 3:  Double-buffer for I2S data : MAX 48*2*8*2 = 1536 B
unsigned char I2SBuffersOut[2][I2S_OUT_BUFFER_SIZE]; // Play
unsigned char I2SBuffersIn[2][I2S_IN_BUFFER_SIZE];  // Record
// Current I2S buffer index.
volatile unsigned char i2s_buffer_out_index = 0;
volatile unsigned char i2s_buffer_in_index  = 0;

AUDIO_CFG  Audio_Configure[2]; //[0]: rec config. [1]: play config.
unsigned char audio_cmd_index     = AUDIO_CMD_IDLE ; 
unsigned char usb_data_padding    = 0; //add for usb BI/BO padding for first package


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
volatile bool bulkout_padding_ok  = false ;
volatile unsigned char Toggle_PID_BI    = 0;

volatile unsigned int bulkout_empt = 0;
volatile bool         flag_bulkout_empt = false;
volatile unsigned int debug_trans_counter1 = 0 ;
volatile unsigned int debug_trans_counter2 = 0 ;  
volatile unsigned int debug_trans_counter3 = 0 ;
volatile unsigned int debug_trans_counter4 = 0 ;  
volatile unsigned int debug_usb_dma_IN = 0;
volatile unsigned int debug_usb_dma_OUT = 0;

extern unsigned int debug_stall_counter;
//extern unsigned int debug_trans_counter1,debug_trans_counter2;
extern kfifo_t dbguart_fifo;
extern const Pin SSC_Sync_Pin;

static unsigned char audio_state_check    = 0; //avoid re-start issue in case of not stop previous start 
static unsigned int BO_free_size_max      = 0;
static unsigned int BI_free_size_min      = 100; 
static unsigned int DBGUART_free_size_min = 100; 
static unsigned int Stop_CMD_Miss_Counter = 0;

unsigned int counter_play    = 0;
unsigned int counter_rec     = 0;
unsigned int test_dump       = 0;
unsigned int time_start_test = 0;

extern unsigned char Check_Toggle_State( void );


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
*                                  First_Pack_Check_BO()
*
* Description :  Check if first USB bulk out package is same as padding data.
* Argument(s) :  None.
* Return(s)   :  true -- check ok.
*                false -- check failed.
*
* Note(s)     :  None.
*********************************************************************************************************
*/
static bool bo_check_sync = false;
__ramfunc bool First_Pack_Check_BO( unsigned int size )
{    
    unsigned int i;
    
    for( i = 0; i < size ; i++ )   {
        if( usb_data_padding != usbBufferBulkOut[i]) {
            return false;
        }
    }
    bo_check_sync = true;
    //printf("\r\nSync\r\n");
    return true; 

}

/*
*********************************************************************************************************
*                                First_Pack_Padding_BI()
*
* Description :  Padding the first USB bulk in package.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     :  Must be called after reset FIFO and before start audio.
*********************************************************************************************************
*/
static void First_Pack_Padding_BI( void )
{
    memset( (unsigned char *)I2SBuffersIn[0], usb_data_padding, USBDATAEPSIZE );
    kfifo_put(&bulkin_fifo, (unsigned char *)I2SBuffersIn[0], USBDATAEPSIZE) ; 
    kfifo_put(&bulkin_fifo, (unsigned char *)I2SBuffersIn[0], USBDATAEPSIZE) ;//2 package incase of PID error
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
static unsigned char Init_Play_Setting( void )
{
    unsigned short sample_rate ; //not support 44.1khz now
    unsigned char  channels_play;
    
    channels_play = Audio_Configure[1].channel_num ;
    if( channels_play == 0 ) { 
        return ERR_CH_ZERO ; 
    }    
    sample_rate   = Audio_Configure[1].sample_rate ;
    printf( "\r\nStart [%dth]Play[%dCH - %dHz] ...\r\n",counter_play++,channels_play,sample_rate);  
    i2s_play_buffer_size = sample_rate / 1000 * channels_play * 2 * 2;  
    SSC_Channel_Set_Tx( channels_play ); 
    
    return 0;
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
static unsigned char Init_Rec_Setting( void )
{
    unsigned short sample_rate ; //not support 44.1kHz now
    unsigned char  channels_rec;
    
    channels_rec = Audio_Configure[0].channel_num ;
    if( channels_rec == 0 ) { 
        return ERR_CH_ZERO ; 
    }  
    sample_rate  = Audio_Configure[0].sample_rate ; 
    printf( "\r\nStart [%dth]Rec [%dCH - %dHz]...\r\n",counter_rec++,channels_rec,sample_rate);     
    i2s_rec_buffer_size  = sample_rate / 1000 * channels_rec  * 2 * 2; 
    SSC_Channel_Set_Rx( channels_rec ); 
    
    First_Pack_Padding_BI();
    
    return 0;
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
static unsigned char Audio_Start_Rec( void )
{  
    unsigned char err;  
    
//   if( Toggle_PID_BI ) { //send padding package if PC Driver expect DATA1 Token          
//         kfifo_put(&bulkin_fifo, (unsigned char *)I2SBuffersIn[0], USBDATAEPSIZE) ;
//    }  
    err = Init_Rec_Setting();
    if( err != 0 ) {
        return err;
    }
    SSC_Record_Start(); 
    bulkin_enable  = true ;
    
    while( !PIO_Get(&SSC_Sync_Pin) ) ;
    while(  PIO_Get(&SSC_Sync_Pin) ) ;     
    SSC_EnableReceiver(AT91C_BASE_SSC0);    //enable AT91C_SSC_RXEN
    
    return 0;  
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
static unsigned char Audio_Start_Play( void )
{  
    unsigned char err;  
    Init_I2S_Buffer();
    err = Init_Play_Setting(); 
    if( err != 0 ) {
        return err;
    }
    SSC_Play_Start(); 
    bulkout_enable  = true ;
    
    while( !PIO_Get(&SSC_Sync_Pin) ) ;
    while(  PIO_Get(&SSC_Sync_Pin) ) ;    
    SSC_EnableTransmitter(AT91C_BASE_SSC0); //enable AT91C_SSC_TXEN  
  
    return 0;
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
    delay_ms(20); //wait until DMA interruption done. 
    bulkin_enable    = false ;
    bulkout_enable   = false ;    
    delay_ms(10);             
    SSC_Play_Stop();  
    SSC_Record_Stop();     
    delay_ms(10);   
    
    printf("\r\nReset USB EP...");
//    if( audio_state_check != 0 ) { //in case of error from repeat Stop CMD 
//        Toggle_PID_BI =  Check_Toggle_State();
//    }
    //Reset Endpoint Fifos
    AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_DATAOUT;
    AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_DATAIN; 
    delay_ms(50);
    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAOUT].UDPHS_EPTCLRSTA = 0xFFFF; //AT91C_UDPHS_NAK_OUT | AT91C_UDPHS_FRCESTALL;                  
    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTCLRSTA  = 0xFFFF; //AT91C_UDPHS_TOGGLESQ | AT91C_UDPHS_FRCESTALL;
    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;
    delay_ms(50);
    
////////////////////////////////////////////////////////////////////////////////    
//    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;  
//    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTCLRSTA  = AT91C_UDPHS_TOGGLESQ ;
//    delay_ms(50);
//    AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_DATAIN ;
//    delay_ms(50); 
    //Reset_USBHS_HDMA( CDCDSerialDriverDescriptors_DATAIN );   
    //AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAOUT].UDPHS_EPTCLRSTA  = AT91C_UDPHS_TOGGLESQ ;
//   
//    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAOUT].UDPHS_EPTCLRSTA  = AT91C_UDPHS_NAK_OUT ;
//    delay_ms(50);    
//    AT91C_BASE_UDPHS->UDPHS_EPTRST =  1<<CDCDSerialDriverDescriptors_DATAOUT;          
    //Reset_USBHS_HDMA( CDCDSerialDriverDescriptors_DATAOUT);   
////////////////////////////////////////////////////////////////////////////////
    
    SSC_Reset(); //I2S_Init();    
    
    Init_Bulk_FIFO();    
    LED_Clear( USBD_LEDUDATA );
    
    bulkin_start      = true ; 
    bulkout_start     = true ;    
    bulkout_trigger   = false ;     
    flag_stop         = false ;
    flag_bulkout_empt = false;
    bulkout_empt      = 0;  
    bulkout_padding_ok  = false ;
    
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
    test_dump             = 0 ;

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
                if( audio_state_check != 0 ) {
                    Audio_Stop(); 
                    Stop_CMD_Miss_Counter++;
                }                 
                bulkout_trigger = true; //trigger paly&rec sync
                err = Audio_Start_Rec();  
                time_start_test = second_counter ;
                audio_state_check = 1;
            break;

            case AUDIO_CMD_START_PLAY :                
                if( audio_state_check != 0 ) {
                    Audio_Stop(); 
                    Stop_CMD_Miss_Counter++;
                }                     
                err = Audio_Start_Play();  
                time_start_test = second_counter ;
                audio_state_check = 2; 
            break;
            
            case AUDIO_CMD_START_PALYREC :                
                if( audio_state_check != 0 ) {
                    Audio_Stop(); 
                    Stop_CMD_Miss_Counter++;
                }                                         
                err = Audio_Start_Play();
                if( err == 0 ) {                    
                  delay_ms(1);  //make sure play and rec enter interruption in turns 2ms              
                  err = Audio_Start_Rec(); 
                }
                time_start_test = second_counter ;
                audio_state_check = 3; 
            break;

            case AUDIO_CMD_STOP :                               
                Audio_Stop(); 
                printf("\r\nThis cycle test time cost: ");
                Get_Run_Time(second_counter - time_start_test);   
                printf("\r\n\r\n");
                time_start_test = 0 ;
                audio_state_check = 0; 
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
            
//            case AUDIO_CMD_RESET:                 
//                printf("\r\nReset USB EP...");   
//                if( audio_state_check != 0 ) { //in case of error from repeat Stop CMD 
//                    Toggle_PID_BI =  Check_Toggle_State();
//                }
//                //Reset Endpoint Fifos
//                AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_DATAOUT;
//                AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_DATAIN; 
//                delay_ms(10);
//                AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAOUT].UDPHS_EPTCLRSTA = 0xFFFF; //AT91C_UDPHS_NAK_OUT | AT91C_UDPHS_TOGGLESQ | AT91C_UDPHS_FRCESTALL;                  
//                AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTCLRSTA  = 0xFFFF;//AT91C_UDPHS_TOGGLESQ | AT91C_UDPHS_FRCESTALL;
//                AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_DATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;
//                printf("Done.\r\n");
//                delay_ms(10); 
//                printf("\r\nReset USB EP...");
    
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
        printf("\r");
        Get_Run_Time(second_counter);
        printf(" [LostStop: %d][LastPadding: 0x%X]  Wait for starting...",Stop_CMD_Miss_Counter,usb_data_padding);
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

    if( counter++ % 20 == 0 ) { //100ms * 20 = 2s 
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
    //printf("\rIN[Size:%6.6f MB, Full:%u, Empty:%u, FreeSize:%3u%>%3u%]  OUT[Size:%6.6f MB, Full:%u, Empty:%u, FreeSize:%3u%<%3u%]",
      
    if( bo_check_sync ) {
        bo_check_sync = false;
        printf("\r\nReceived USB Sync package.\r\n");
    }
    
    printf("\rIN[%6.6f MB, Full:%u, Empty:%u, Free:%3u%>%3u%]  OUT[%6.6f MB, Full:%u, Empty:%u, Free:%3u%<%3u%]. ",
                         
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
     
    Get_Run_Time(second_counter - time_start_test);
     
//     DBGUART_free_size = kfifo_get_free_space(&dbguart_fifo) ;
//     DBGUART_free_size = DBGUART_free_size * 100 / DBGUART_FIFO_SIZE;  
//     DBGUART_free_size_min = DBGUART_free_size < DBGUART_free_size_min ? DBGUART_free_size : DBGUART_free_size_min ;
//     printf( " [DBGUART:%3u%>%3u%]", DBGUART_free_size, DBGUART_free_size_min );                         
     
}


void Get_Run_Time( unsigned int time )
{
    unsigned char  msec, sec, min, hour;
    unsigned int   day;

    msec = time % 10;
    sec  = time /10 % 60 ;
    min  = time / 600 %60 ;
    hour = time / 36000 %24 ; 
    day  = time / 36000 /24 ;
    printf("[%d:%02d:%02d:%02d.%d]", day, hour, min, sec, msec ); 
   
    
}

