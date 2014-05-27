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
*                                          UART PACKAGE
*
*                                         Atmel  SAM3U4C
*                                             on the
*                                      iSAM Audio Bridge Board
*
* Filename      : uart.c
* Version       : V1.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/


#include <pio/pio.h>
#include <usart/usart.h>
#include <dbgu/dbgu.h>
#include <utility/trace.h>
#include <irq/irq.h>
#include <board.h>
#include <stdbool.h>
#include "kfifo.h"
#include "app.h"


#define  RULER_CMD_SET_AUDIO_CFG        0x01
#define  RULER_CMD_START_AUDIO          0x02
#define  RULER_CMD_STOP_AUDIO           0x03
#define  RULER_CMD_GET_AUDIO_VERSION    0x0B

#define CMD_STAT_SYNC1     0
#define CMD_STAT_SYNC2     1
#define CMD_STAT_FLAG      2
#define CMD_STAT_CMD1      3
#define CMD_STAT_CMD2      4
#define CMD_STAT_DATA      5

#define CMD_DATA_SYNC1     0xEB
#define CMD_DATA_SYNC2     0x90

#define UART_TIMEOUT_BIT     ( 5 * 10 ) // 50=5*10  timeout in 5 Bytes' time  
#define UART_BUFFER_SIZE     ( 128    ) // uart buf size
#define UART_BAUD            ( 115200 )



volatile unsigned char usartBuffers[2][UART_BUFFER_SIZE] ;
volatile unsigned char usartCurrentBuffer = 0 ;


static const Pin Uart_Pins[] = {  

    PIN_USART0_RXD,
    PIN_USART0_TXD
      
};



unsigned char UART_CMD_Buffer[4];
unsigned char state_mac      = CMD_STAT_SYNC1 ;
unsigned char PcCmdCounter   = 0; 


/*
*********************************************************************************************************
*                                    pcInt()
*
* Description : Process command data byte by byte and decode beased on communication protocol between USB AUDIO MCU and HOST MCU.
* Argument(s) : ch : the one byte data.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void pcInt(  unsigned char ch )
{    
    unsigned char *pChar = UART_CMD_Buffer;
    
    switch( state_mac ) {   
        
        case CMD_STAT_SYNC1 :        
            if(ch == CMD_DATA_SYNC1)  {
                state_mac = CMD_STAT_SYNC2 ;
            }
        break;
        
        case CMD_STAT_SYNC2 :
            if(ch == CMD_DATA_SYNC2)  {           
                 state_mac     =  CMD_STAT_FLAG;
                 PcCmdCounter  = 0 ; 
            } else {              
                state_mac = CMD_STAT_SYNC1;                
            }
        break ;
        
        case CMD_STAT_FLAG :   
       
            switch( ch )  {
                case RULER_CMD_SET_AUDIO_CFG : 
                    state_mac =  CMD_STAT_CMD1 ;
                    break ;
                case RULER_CMD_START_AUDIO :
                    state_mac =  CMD_STAT_CMD2 ;
                    break ;                
                case RULER_CMD_STOP_AUDIO :
                    audio_cmd_index = AUDIO_CMD_STOP ; 
                    state_mac = CMD_STAT_SYNC1; 
                    break ;  
                case RULER_CMD_GET_AUDIO_VERSION  :
                    audio_cmd_index = AUDIO_CMD_VERSION ; 
                    state_mac = CMD_STAT_SYNC1; 
                break ;  
                        
                default :
                    break ;                        
            }
         
        break ;
        
        case CMD_STAT_CMD1 :            
             UART_CMD_Buffer[PcCmdCounter++] = ch;       
             state_mac  = CMD_STAT_DATA ;
          
        break ;
        
        case CMD_STAT_CMD2 :  
             audio_cmd_index = ch;  //
             PcCmdCounter    = 0;
             state_mac       = CMD_STAT_SYNC1 ;
          
        break ;
        
        case CMD_STAT_DATA :
            *(pChar+PcCmdCounter) = ch; 
            PcCmdCounter++;
            if( PcCmdCounter > 3 ) { //check verflow
               Audio_Configure[(*pChar)&0x01] = *(AUDIO_CFG *)pChar;                
               audio_cmd_index = AUDIO_CMD_CFG ; 
               PcCmdCounter = 0 ;        
               state_mac = CMD_STAT_SYNC1;                
            } 
        break ;
                
        default :
            state_mac     = CMD_STAT_SYNC1;
            PcCmdCounter  = 0 ;
        break ;
        
    } 
    
    
}


/*
*********************************************************************************************************
*                                    USART0_IrqHandler()
*
* Description : UART0 interruption service routine. Handles interrupts coming from USART #0.
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void USART0_IrqHandler( void )
{
  
    unsigned int    i ;
    unsigned int    status ;
    unsigned int    data_received;
  
    status        = AT91C_BASE_US0->US_CSR;
    data_received = UART_BUFFER_SIZE - ( AT91C_BASE_US0->US_RCR );
    
    // Buffer has been read successfully
    if ( (status & AT91C_US_ENDRX) != 0 ) {         
        usartCurrentBuffer = 1 - usartCurrentBuffer; //pingpong buffer index
        USART_ReadBuffer( AT91C_BASE_US0,(void *)usartBuffers[usartCurrentBuffer], UART_BUFFER_SIZE); // Restart read on buffer
        for( i = 0; i < data_received; i++)  { //analyze the data          
            pcInt( usartBuffers[1-usartCurrentBuffer][i] ) ;
        }         
    }
    
    if ( (status & AT91C_US_TIMEOUT) != 0 ) {  
        AT91C_BASE_US0->US_RCR = 0;   
        usartCurrentBuffer = 1 - usartCurrentBuffer;
        USART_ReadBuffer(AT91C_BASE_US0,(void *)usartBuffers[usartCurrentBuffer], UART_BUFFER_SIZE);    // Restart read on buffer
        AT91C_BASE_US0->US_CR   = AT91C_US_STTTO; //restart timeout counter
        AT91C_BASE_US0->US_RTOR = UART_TIMEOUT_BIT;
        for( i = 0; i< data_received ; i++)  {          
            pcInt(usartBuffers[1-usartCurrentBuffer][i]) ;
        } 
     
    }
    
    /*
    // Buffer has been sent
    if ((status & AT91C_US_TXBUFE) != 0) {
        AT91C_BASE_US0->US_IDR = AT91C_US_TXBUFE;
    }
    */
    
    
    
}


/*
*********************************************************************************************************
*                                    UART_Init()
*
* Description : init UART port : UART0
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void UART_Init( void )
{
      
    PIO_Configure(Uart_Pins, PIO_LISTSIZE(Uart_Pins));    
    usartCurrentBuffer = 0 ;
    
    // Configure USART0¡¡¡¡//Comm between DP/DC and HOST mcu    
    AT91C_BASE_PMC->PMC_PCER    = 1 << AT91C_ID_US0;
    AT91C_BASE_US0->US_IDR      = 0xFFFFFFFF;
    AT91C_BASE_US0->US_IER      =  AT91C_US_ENDRX | AT91C_US_TIMEOUT; 
    USART_Configure(AT91C_BASE_US0,USART_MODE_ASYNCHRONOUS,UART_BAUD, MCK);
    USART_SetTransmitterEnabled(AT91C_BASE_US0, 1);
    USART_SetReceiverEnabled(AT91C_BASE_US0, 1);
    IRQ_ConfigureIT(AT91C_ID_US0, UART_PRIORITY, USART0_IrqHandler); //priority chaned to max?
    
    AT91C_BASE_US0->US_CR   = AT91C_US_STTTO; //restart timeout counter
    AT91C_BASE_US0->US_RTOR = UART_TIMEOUT_BIT;
    
    USART_ReadBuffer(AT91C_BASE_US0,(void *)usartBuffers[usartCurrentBuffer],UART_BUFFER_SIZE);    
    IRQ_EnableIT(AT91C_ID_US0);   
      
    
}



