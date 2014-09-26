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
//         Headers
//------------------------------------------------------------------------------


#include <irq/irq.h>
#include <utility/led.h>
#include <stdbool.h>
#include "kfifo.h"
#include "tc.h"
#include "app.h"
//------------------------------------------------------------------------------
//         Global Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Configures a Timer Counter to operate in the given mode. Timer is stopped
/// after configuration and must be restarted with TC_Start(). All the
/// interrupts of the timer are also disabled.
/// \param pTc  Pointer to an AT91S_TC instance.
/// \param mode  Operating mode (TC_CMR value).
//------------------------------------------------------------------------------
void TC_Configure(AT91S_TC *pTc, unsigned int mode)
{
    // Disable TC clock
    pTc->TC_CCR = AT91C_TC_CLKDIS;

    // Disable interrupts
    pTc->TC_IDR = 0xFFFFFFFF;

    // Clear status register
    pTc->TC_SR;

    // Set mode
    pTc->TC_CMR = mode;
}

//------------------------------------------------------------------------------
/// Enables the timer clock and performs a software reset to start the counting.
/// \param pTc  Pointer to an AT91S_TC instance.
//------------------------------------------------------------------------------
void TC_Start(AT91S_TC *pTc)
{
    pTc->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
}

//------------------------------------------------------------------------------
/// Disables the timer clock, stopping the counting.
/// \param pTc  Pointer to an AT91S_TC instance.
//------------------------------------------------------------------------------
void TC_Stop(AT91S_TC *pTc)
{
    pTc->TC_CCR = AT91C_TC_CLKDIS;
}

//------------------------------------------------------------------------------
/// Finds the best MCK divisor given the timer frequency and MCK. The result
/// is guaranteed to satisfy the following equation:
/// \pre
///   (MCK / (DIV * 65536)) <= freq <= (MCK / DIV)
/// \endpre
/// with DIV being the highest possible value.
/// \param freq  Desired timer frequency.
/// \param mck  Master clock frequency.
/// \param div  Divisor value.
/// \param tcclks  TCCLKS field value for divisor.
/// \return 1 if a proper divisor has been found; otherwise 0.
//------------------------------------------------------------------------------
unsigned char TC_FindMckDivisor(
    unsigned int freq,
    unsigned int mck,
    unsigned int *div,
    unsigned int *tcclks)
{
    unsigned int index = 0;
    unsigned int divisors[5] = {2, 8, 32, 128,
#if defined(at91sam9260) || defined(at91sam9261) || defined(at91sam9g10) || defined(at91sam9263) \
    || defined(at91sam9xe) || defined(at91sam9rl64) || defined(at91cap9) \
    || defined(at91sam9m10) || defined(at91sam9m11) || defined(at91sam3u4)
        0};
    divisors[4] = mck / 32768;
#else
        1024};
#endif

    // Satisfy lower bound
    while (freq < ((mck / divisors[index]) / 65536)) {

        index++;

        // If no divisor can be found, return 0
        if (index == 5) {

            return 0;
        }
    }

    // Try to maximise DIV while satisfying upper bound
    while (index < 4) {

        if (freq > (mck / divisors[index + 1])) {

            break;
        }
        index++;
    }

    // Store results
    if (div) {

        *div = divisors[index];
    }
    if (tcclks) {

        *tcclks = index;
    }

    return 1;
}




                                                            

//------------------------------------------------------------------------------
///  Timer #2  :  For test time record
//------------------------------------------------------------------------------
volatile unsigned int second_counter = 0 ;
    
void TC2_IrqHandler( void )
{    
    unsigned int status;
    static unsigned int timer2_counter = 0;
    status = AT91C_BASE_TC2->TC_SR;
    
    if ((status & AT91C_TC_CPCS) != 0) { 
        AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
        if( ++timer2_counter == 10 ) { // 10ms*10 = 100ms
            timer2_counter = 0 ;
            second_counter++;             
        }
        //LED_TOGGLE_DATA ; //test
    }
    
}

void Timer2_Init( void )
{
    unsigned int counter; 
    
    counter =  MCK / 128 / 100; //10ms time 
    counter = (counter & 0xFFFF0000) == 0 ? counter : 0xFFFF ;    

    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TC2);
    AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC2->TC_IDR = 0xFFFFFFFF;
    AT91C_BASE_TC2->TC_CMR = AT91C_TC_CLKS_TIMER_DIV4_CLOCK //choose 1/128
                             | AT91C_TC_CPCSTOP
                             | AT91C_TC_CPCDIS
                             | AT91C_TC_WAVESEL_UP_AUTO
                             | AT91C_TC_WAVE;
    AT91C_BASE_TC2->TC_RC = counter ;
    AT91C_BASE_TC2->TC_IER = AT91C_TC_CPCS;
    IRQ_ConfigureIT(AT91C_ID_TC2, TIMER_PRIORITY, TC2_IrqHandler);
    IRQ_EnableIT(AT91C_ID_TC2);
    AT91C_BASE_TC2->TC_CCR = AT91C_TC_CLKEN +  AT91C_TC_SWTRG ;
    
}




//------------------------------------------------------------------------------
///  Timer #0  :  For delay_ms()
//------------------------------------------------------------------------------
void Timer0_Init( void )
{
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TC0);
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC0->TC_IDR = 0xFFFFFFFF;
    AT91C_BASE_TC0->TC_CMR = AT91C_TC_CLKS_TIMER_DIV4_CLOCK //choose 1/128 
                             | AT91C_TC_CPCSTOP
                             | AT91C_TC_CPCDIS
                             | AT91C_TC_WAVESEL_UP_AUTO
                             | AT91C_TC_WAVE;   
    
}

//void  delay_ms( unsigned int delay)
//{
//
//    timer0_counter = 0;
//    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;   //start    
//    while ( timer0_counter < delay );
//    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS; //stop
//
//}
void  delay_ms( unsigned int delay_ms ) //
{   
    unsigned int counter_top;
    unsigned int counter_cycle;
    
    // MCK / (1000) / ( DIV[timer_div] ) * delay_ms ; = 96000000/1000/128 *delay_ms = 750 * delay_ms    
    counter_cycle   = delay_ms / 87 ;
    counter_top     = delay_ms - counter_cycle * 87 ;
    
    if( counter_cycle > 0 ) {     
        while( counter_cycle-- >0 ) { 
            AT91C_BASE_TC0->TC_RC  = 87 * 750 ;
            AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN +  AT91C_TC_SWTRG ;
            while( !(AT91C_BASE_TC0->TC_SR & AT91C_TC_CPCS) ) ;  
        }
        
    }             
    counter_top =  (counter_top - 0) * 750 ;   
    AT91C_BASE_TC0->TC_RC  = counter_top;
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN +  AT91C_TC_SWTRG ;
    
    while( !(AT91C_BASE_TC0->TC_SR & AT91C_TC_CPCS) ) ;     
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;     
     
}




//------------------------------------------------------------------------------
///  Timer #1  :  For delay_us()
//------------------------------------------------------------------------------
void Timer1_Init( void )
{
    
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TC1);
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC1->TC_IDR = 0xFFFFFFFF;
    AT91C_BASE_TC1->TC_CMR = AT91C_TC_CLKS_TIMER_DIV1_CLOCK //choose 1/2
                             | AT91C_TC_CPCSTOP
                             | AT91C_TC_CPCDIS
                             | AT91C_TC_WAVESEL_UP_AUTO
                             | AT91C_TC_WAVE;
    
}


void  __ramfunc delay_us(unsigned int delay_us)  
{   
   unsigned int counter_top;   

   // MCK / (1000*1000) / ( DIV[timer_div] ) * delay_us ; = 96000000/1000000/2 *delay_us = 48*  *delay_us   
    counter_top =  (delay_us - 1) * 48 ;               
    if( counter_top & 0xFFFF0000 ) {
        counter_top = 0; // if exceeds TC_RC ...       
    } 
    
     AT91C_BASE_TC1->TC_RC  = counter_top;// if exceeds TC_RC ...
     AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN +  AT91C_TC_SWTRG ;
     while( !( AT91C_BASE_TC1->TC_SR & AT91C_TC_CPCS) ) ;  
     AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKDIS;     
    
}




//------------------------------------------------------------------------------
///  SysTick Timer  :  For debug info refresh
//------------------------------------------------------------------------------

#define DEBUG_INFO_FRESH_INTERVAL 100  //100ms

// Configure sys tick timer for debug_info()
void SysTick_Init( void )
{
  
   unsigned int value ;
   unsigned int counter; 
    
   counter =  MCK / 8 / 1000 * DEBUG_INFO_FRESH_INTERVAL;  
   counter = (counter & 0xFF000000) == 0 ? counter : 0x00FFFFFF ;
   
   value  = AT91C_NVIC_STICKENABLE ;
   value &= ~(unsigned int)AT91C_NVIC_STICKCLKSOURCE ; 
   AT91C_BASE_NVIC->NVIC_STICKRVR = counter ;
   AT91C_BASE_NVIC->NVIC_STICKCSR = value ;   
  
}


unsigned char Check_SysTick_State( void )
{
  
    unsigned int status = AT91C_BASE_NVIC->NVIC_STICKCSR;

    if ( status & AT91C_NVIC_STICKCOUNTFLAG ) {   
        //LED_TOGGLE_DATA ;  
        return 1;
        
    } else {
        return 0;
        
    }
    
}








