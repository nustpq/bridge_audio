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

#ifndef _APP_INC_
#define _APP_INC_

//Softpack Version
#define MCK                  BOARD_MCK
#define I2S_BUFFER_SIZE      1536//768   //audio data transfered per frame, Max 48kHz:   48*2*8=768
#define USBDATAEPSIZE        BOARD_USB_ENDPOINTS_MAXPACKETSIZE( CDCDSerialDriverDescriptors_DATAIN ) //512
#define USB_OUT_BUFFER_SIZE  16384  //2^14=16384  //USB audio data, size MUST be 2^n .
#define USB_IN_BUFFER_SIZE   16384  //2^14=16384  //USB audio data, size MUST be 2^n .

#define PLAY_BUF_DLY_N       5  //delay 2^6=64 ms
// A programmable priority level of 0-15 for each interrupt. A higher level corresponds to a lower 
// priority, so level 0 is the highest interrupt priority
//
//#define PIO_PRIORITY        7
//#define TIMER_PRIORITY      6
#define UART_PRIORITY       4
#define USB_PRIORITY        3
#define HDMA_PRIORITY       2 //SSC must have highest priority



#define  AUDIO_CMD_IDLE                 0x00
#define  AUDIO_CMD_START_REC            0x01
#define  AUDIO_CMD_START_PLAY           0x02
#define  AUDIO_CMD_START_PALYREC        0x03
#define  AUDIO_CMD_STOP                 0x04
#define  AUDIO_CMD_CFG                  0x05
#define  AUDIO_CMD_VERSION              0x06

#define   ERR_USB_STATE                 0xFD
#define   ERR_AUD_CFG                   0xFE
#define   ERR_CMD_TYPE                  0xFF


typedef struct {
  
  unsigned char type;//Rec: =0x00, Play: =0x01
  unsigned char channel_num; //1~6
  unsigned short int sample_rate;  
  
}AUDIO_CFG;

extern AUDIO_CFG  Audio_Configure[];
extern unsigned char audio_cmd_index;
extern unsigned char audio_cmd_ack;

extern unsigned char usbBufferBulkOut[];
extern unsigned char usbBufferBulkIn[];

extern unsigned char FIFOBufferBulkOut[];
extern unsigned char FIFOBufferBulkIn[]; 

extern unsigned char I2SBuffersOut[2][I2S_BUFFER_SIZE]; 
extern unsigned char I2SBuffersIn[2][I2S_BUFFER_SIZE]; 

extern volatile unsigned char i2s_buffer_out_index;
extern volatile unsigned char i2s_buffer_in_index;


extern volatile unsigned int i2s_play_buffer_size ;
extern volatile unsigned int i2s_rec_buffer_size ;


extern unsigned char  sample_index;

extern volatile bool bulkin_start;
extern volatile bool bulkout_start;
extern volatile bool bulkout_enable;
extern volatile bool bulkin_enable;
extern volatile bool bulkout_trigger;
extern volatile bool flag_stop;
extern volatile unsigned int testc;
extern kfifo_t bulkout_fifo;
extern kfifo_t bulkin_fifo;

extern volatile unsigned int bulkout_empt;

extern void Debug_Info( void );
extern void Usb_Init(void);
extern void Init_Buffer( void );
extern void Audio_State_Control( void );

void UART_Init( void );
void Check_UART_CMD( void );
void USB_Init( void );
void I2S_Init( void );
void I2S_ReInit( void );
void SSC_Play_Start(void);
void SSC_Record_Start(void);
void SSC_Play_Stop(void);
void SSC_Record_Stop(void);
void Init_I2S_Buffer( void );

extern char fw_version[];

extern unsigned int counter_play ;
extern unsigned int counter_rec  ;




extern volatile unsigned int debug_trans_counter1 ;
extern volatile unsigned int debug_trans_counter2 ;  
extern volatile unsigned int debug_trans_counter3 ;
extern volatile unsigned int debug_trans_counter4 ;  
extern volatile unsigned int debug_usb_dma_enterhandler;
extern volatile unsigned int debug_usb_dma_IN ;
extern volatile unsigned int debug_usb_dma_OUT;
#endif //#ifndef APP_H
