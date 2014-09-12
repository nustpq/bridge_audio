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


#include "alert_table.h"

#define  ALERT_TABLE_SR     (48)  //48k source table
#define  ALERT_TABLE_POINT  (48000)  //48k source table: 1s length
/*
*********************************************************************************************************
*                                    Alert_Sound_Gen()
*
* Description : Generate a alert predefined sound for USB audio for error reminder.
* Argument(s) :  *pdata      : pointer to the buffer adress where the generate data is stored.
*                 size       : buffer size in bytes
*                 REC_SR_Set : sample rate : 
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void Alert_Sound_Gen( unsigned char *pdata, unsigned int size, unsigned int REC_SR_Set )
{
    
    unsigned int     sample_per_ms;
    unsigned int     table_lookup_step;     
    unsigned short  *pDest;
    unsigned int     sample_index = 0;
    unsigned short   temp;
    
    static unsigned int    index=0;    
    const unsigned short  *pVal;
    
    sample_per_ms     = REC_SR_Set / 1000 ;
    table_lookup_step = ALERT_TABLE_SR / sample_per_ms;    
    pVal              = (const unsigned short  *)alert_table; 
    pDest             = ( unsigned short  *)pdata; 
    
        
    while( sample_index < (size>>3) ) {  //4CH      
       temp =   *(pVal+index) ;
       *( pDest )   =  temp;
       *( pDest+1 ) =  temp;
       *( pDest+2 ) =  temp;
       *( pDest+3 ) =  temp;     
       sample_index++;
       pDest+=4;   
       index = ( index + table_lookup_step ) % ALERT_TABLE_POINT ;
    
    }  
    
//      while( sample_index < (size>>1) ) {     
//        
//         for( unsigned int i = 0; i < 4; i++ ) {
//        
//            *(pDest + sample_index++) =  *(pVal+index) ;
//      }
//      index = ( index + table_lookup_step ) % ALERT_TABLE_POINT ;
//      
//    }  
    
   
} 

