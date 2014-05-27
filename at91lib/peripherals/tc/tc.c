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
/// Handles interrupts coming from Timer #0.
//------------------------------------------------------------------------------
static volatile unsigned int timer0_counter = 0 ;
    
void TC0_IrqHandler( void )
{  
  
    unsigned int status = AT91C_BASE_TC0->TC_SR;
    if ((status & AT91C_TC_CPCS) != 0) { 
        AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
        timer0_counter++;
        //LED_TOGGLE_DATA ; 
  
    }
    
}


// Configure timer 0 for delay_ms()
void Timer0_Init( void )
{
    unsigned int counter; 
    
    counter =  MCK / 8 / 1000; //1ms time 
    counter = (counter & 0xFFFF0000) == 0 ? counter : 0xFFFF ;

    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TC0);
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC0->TC_IDR = 0xFFFFFFFF;
    AT91C_BASE_TC0->TC_CMR = AT91C_TC_CLKS_TIMER_DIV2_CLOCK //choose 1/8 
                             | AT91C_TC_CPCSTOP
                             | AT91C_TC_CPCDIS
                             | AT91C_TC_WAVESEL_UP_AUTO
                             | AT91C_TC_WAVE;
    AT91C_BASE_TC0->TC_RC = counter ;
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;
    IRQ_ConfigureIT(AT91C_ID_TC0, TIMER_PRIORITY, TC0_IrqHandler);
    IRQ_EnableIT(AT91C_ID_TC0);
    
}



void  delay_ms( unsigned int delay)
{

    timer0_counter = 0;
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;   //start    
    while ( timer0_counter < delay );
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS; //stop

}



#define DEBUG_INFO_FRESH_INTERVAL 100 //100ms

void Timer1_Init( void )
{
    unsigned int counter; 
    
    counter =  MCK / 128 / 1000 * DEBUG_INFO_FRESH_INTERVAL; //100ms time
    counter = (counter & 0xFFFF0000) == 0 ? counter : 0xFFFF ;

    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TC1);
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC1->TC_IDR = 0xFFFFFFFF;
    AT91C_BASE_TC1->TC_CMR = AT91C_TC_CLKS_TIMER_DIV4_CLOCK //choose 1/128 
                             | AT91C_TC_CPCSTOP
                             | AT91C_TC_CPCDIS
                             | AT91C_TC_WAVESEL_UP_AUTO
                             | AT91C_TC_WAVE;
    AT91C_BASE_TC1->TC_RC = counter;
    AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;   //start
    
}


unsigned char Check_Timer1_State( void )
{
    unsigned int status = AT91C_BASE_TC1->TC_SR;

    if ((status & AT91C_TC_CPCS) != 0) { 
        //AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKDIS;
        AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;        
        //LED_TOGGLE_DATA ;   
        return 1;
    } else {
        return 0;
        
    }
    
}

// Configure timer 1 for debug_info()
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






