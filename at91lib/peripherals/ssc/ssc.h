/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
/// \dir
/// !Purpose
///
/// Set of functions and definition for using a SSC peripheral.
///
/// !Usage
///
/// -# Enable the SSC interface pins (see pio & board.h).
/// -# Configure the SSC to operate at a specific frequency by calling
///    SSC_Configure(). This function enables the peripheral clock of the SSC,
///    but not its PIOs.
/// -# Configure the transmitter and/or the receiver using the
///    SSC_ConfigureTransmitter() and SSC_ConfigureEmitter() functions.
/// -# Enable the PIOs or the transmitter and/or the received.
/// -# Enable the transmitter and/or the receiver using SSC_EnableTransmitter()
///    and SSC_EnableReceiver()
/// -# Send data through the transmitter using SSC_Write() and SSC_WriteBuffer()
/// -# Receive data from the receiver using SSC_Read() and SSC_ReadBuffer()
/// -# Disable the transmitter and/or the receiver using SSC_DisableTransmitter()
///    and SSC_DisableReceiver()
//------------------------------------------------------------------------------

#ifndef SSC_H
#define SSC_H

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// \page "SSC configuration macros"
/// This page lists several macros which are used when configuring a SSC
/// peripheral.
/// 
/// !Macros
/// - SSC_STTDLY
/// - SSC_PERIOD
/// - SSC_DATLEN
/// - SSC_DATNB
/// - SSC_FSLEN

/// Calculates the value of the STTDLY field given the number of clock cycles
/// before the first bit of a new frame is transmitted.
#define SSC_STTDLY(bits)        (bits << 16)

/// Calculates the value of the PERIOD field of the Transmit Clock Mode Register
/// of an SSC interface, given the desired clock divider.
#define SSC_PERIOD(divider)     (((divider / 2) - 1) << 24)

/// Calculates the value of the DATLEN field of the Transmit Frame Mode Register
/// of an SSC interface, given the number of bits in one sample.
#define SSC_DATLEN(bits)        (bits - 1)

/// Calculates the value of the DATNB field of the Transmit Frame Mode Register
/// of an SSC interface, given the number of samples in one frame.
#define SSC_DATNB(samples)      ((samples -1) << 8)

/// Calculates the value of the FSLEN field of the Transmit Frame Mode Register
/// of an SSC interface, given the number of transmit clock periods that the 
/// frame sync signal should take.
#define SSC_FSLEN(periods)      ((periods - 1) << 16)

/// SSC DMA Fifo size
#define BOARD_SSC_DMA_FIFO_SIZE         (4*1024)

/// SSC Linked list size
#define MAX_SSC_LLI_SIZE                    2

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------
extern void SSC_Configure(AT91S_SSC *ssc,
                                 unsigned int id,
                                 unsigned int bitRate,
                                 unsigned int masterClock);
extern void SSC_ConfigureTransmitter(AT91S_SSC *ssc,
                                            unsigned int tcmr,
                                            unsigned int tfmr);
extern void SSC_ConfigureReceiver(AT91S_SSC *ssc,
                                         unsigned int rcmr,
                                         unsigned int rfmr);

extern void SSC_EnableTransmitter(AT91S_SSC *ssc);
extern void SSC_DisableTransmitter(AT91S_SSC *ssc);
extern void SSC_EnableBoth( AT91S_SSC *ssc );
extern void SSC_EnableReceiver(AT91S_SSC *ssc);
extern void SSC_DisableReceiver(AT91S_SSC *ssc);

extern void SSC_EnableInterrupts(AT91S_SSC *ssc, unsigned int sources);
extern void SSC_DisableInterrupts(AT91S_SSC *ssc, unsigned int sources);

extern void SSC_Write(AT91S_SSC *ssc, unsigned int frame);
extern unsigned char SSC_WriteBuffer(AT91S_SSC *ssc,
                                            void *buffer,
                                            unsigned char buffer_index,
                                            unsigned int length);
unsigned char SSC_WriteBuffer_Start(  AT91S_SSC *ssc,
                                     void *buffer,
                                     void *buffer_next,
                                     unsigned int length);
extern unsigned int SSC_Read(AT91S_SSC *ssc);
extern unsigned char SSC_ReadBuffer(AT91S_SSC *ssc,
                                           void *buffer,
                                           unsigned char buffer_index,
                                           unsigned int length);
extern unsigned char SSC_ReadBuffer_Start(  AT91S_SSC *ssc,
                                     void *buffer,
                                     void *buffer_next,
                                     unsigned int length);
extern void SSC_Init( unsigned int mclk );
extern void SSC_Reset( void );
extern void SSC_Channel_Set(unsigned char tx_ch_num, unsigned char rx_ch_num );
extern void SSC_Channel_Set_Rx( unsigned char rx_ch_num );
extern void SSC_Channel_Set_Tx( unsigned char tx_ch_num );


typedef union __TCMR
{
    struct 
    {
      unsigned int  cks     : 2 ;
      unsigned int  cko     : 3 ;
      unsigned int  cki     : 1 ;
      unsigned int  ckg     : 2 ;
      unsigned int  start   : 4 ;
      unsigned int  rsv     : 4 ;
      unsigned int  sttdly  : 8 ;
      unsigned int  period  : 8 ;
    };
    unsigned int value;
}TCMR ;

typedef union __TFMR
{
    struct 
    {
        unsigned int datlen : 5 ;
        unsigned int datdef : 1 ;
        unsigned int rst    : 1 ;
        unsigned int msbf   : 1 ;
        unsigned int datnb  : 4 ;
        unsigned int rst2   : 4 ;
        unsigned int fslen  : 4 ;
        unsigned int fsos   : 3 ;
        unsigned int fsden  : 1 ;
        unsigned int fsedge : 1 ;
        unsigned int rst3   : 7 ;
    };
    unsigned int value;
}TFMR ;

typedef union __RCMR
{
    struct
    {
        unsigned int cks    : 2 ;
        unsigned int cko    : 3 ;
        unsigned int cki    : 1 ;
        unsigned int ckg    : 2 ;
        unsigned int start  : 4 ;
        unsigned int stop   : 1 ;
        unsigned int rst    : 3 ;
        unsigned int sttdly : 8 ;
        unsigned int period : 8 ;
    };
    unsigned int value ;
}RCMR ;

typedef union __RFMR
{
    struct 
    {
      unsigned int datlen   : 5 ;
      unsigned int loop     : 1 ;
      unsigned int rst      : 1 ;
      unsigned int msbf     : 1 ;
      unsigned int datnb    : 4 ;
      unsigned int rst2     : 4 ;
      unsigned int fslen    : 4 ;
      unsigned int fsos     : 3 ;
      unsigned int rst3     : 1 ;
      unsigned int fsedge   : 1 ;
      unsigned int rst4     : 7 ;
    } ;
    unsigned int  value ;
}RFMR ;

typedef union _SSCMODE
{
    struct 
    {
        unsigned int fslen  : 4 ;
        unsigned int start  : 4 ;
        unsigned int delay  : 2 ;
        unsigned int period : 6 ;
        unsigned int fsos   : 3 ;
        unsigned int cko    : 1 ;
        unsigned int RxLr   : 1 ;  // 0 : fclk =1 left, fclk =0 right ;      1:  flck = 0 left, flck =1 right
        unsigned int TxLr   : 1 ;  // 0 : fclk =1 left, fclk =0 right ;      1:  flck = 0 left, flck =1 right
        unsigned int cki    : 1 ;
        unsigned int loop   : 1 ;
    } ;
    unsigned int value ;
}SSCMODE ;




#endif

