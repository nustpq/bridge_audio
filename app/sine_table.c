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


#include "sine_table.h"

#define SINE_1KHZ_TABLE_POINT  (360*2)  //0.5 degree precision

/*
*********************************************************************************************************
*                                    Demo_Sine_Gen()
*
* Description : Generate a golden 1kHz sine wave data for USB audio recording for demo.
* Argument(s) :  *pdata      : pointer to the buffer adress where the generate data is stored.
*                 size       : buffer size in bytes
*                 REC_SR_Set : sample rate : 
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void Demo_Sine_Gen( unsigned char *pdata, unsigned int size, unsigned int REC_SR_Set )
{
    
    unsigned int    sample_per_ms;
    unsigned int    table_lookup_step;   
    unsigned int    i, index;
    unsigned int    sample_index;

    const unsigned short  *pVal;

    index             = 0; 
    sample_index      = 0;
    sample_per_ms     = REC_SR_Set / 1000 ;
    table_lookup_step = SINE_1KHZ_TABLE_POINT / sample_per_ms;    
    pVal              = sine_table; 
      

    while( sample_index < (size>>1) ) {     
        
      for( i = 0; i < 6; i++ ) {
        
            *((unsigned short *)pdata + sample_index++) =  *(pVal+index) ;
      }
      index = ( index + table_lookup_step ) % SINE_1KHZ_TABLE_POINT ;
      
    }  
    
   
} 

