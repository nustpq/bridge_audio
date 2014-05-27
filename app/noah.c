/******************************Copyright (c)************************************
**                          fortemedia NJ SQA  
**                                              
*******************************************************************************/
#define PCCMD_C

#include "noah.h"
#include "app.h"
#include <string.h>
#include <board.h>
#include <usart/usart.h>
#include <dbgu/dbgu.h>
#include <tc/tc.h>
#include <usb/device/audio-speaker/AUDDSpeakerDriver.h>
#include <utility/trace.h>


unsigned char PcCmdRxdBuf[BUFNUM+1][MAXBUFLEN+1] ;
unsigned char PcCmdBufNum = 0 ;
unsigned char pccmdbufin  = 0 ;
unsigned char pccmdbufout = 0 ;
unsigned char *pIntRxd ;
unsigned char *pExtBuf ;
unsigned int PcCmdPtr =0;  
unsigned int rxdatalen ;
                                                                   
unsigned char pcCmdSendBuf[PCSENDBUFNUM+1][40] ;   
unsigned int  pcCmdSendLen ;
unsigned char SendCmdTail ;            
unsigned char SendCmdHead ;     

unsigned char PcCmdLen ;        
unsigned char PcCmdFlag  = CMD_STAT_SYNC1 ;        
unsigned char PcCmdCheckSum ;  
unsigned char PcCmdRxId ;     
unsigned char PcCmdTxId ;   

volatile unsigned char pcCmdRcv = 0 ;

static unsigned char datbuf[50] ;
static unsigned char buflen ;
unsigned char const estbuf[] = {0xeb,0x90,0x3f,0x3f} ;
unsigned char const nakbuf[] = {0xeb,0x90,0x01,0x01} ;
unsigned char hstbuf[10] ;
unsigned char hstbuflen ;
RAW_READ_BUF RawReadBuf ;





unsigned int delay_time_us ;


// Noah protocol parsing :  PC UART data  
void pcInt(unsigned char ch)
{
     
    switch( PcCmdFlag )
    {
        case CMD_STAT_SYNC1 :
            if(ch == CMD_DATA_SYNC1)
            {
                PcCmdFlag = CMD_STAT_SYNC2 ; 
                
            }
        break ;
        case CMD_STAT_SYNC2 :
            if(ch == CMD_DATA_SYNC2)
            {
                PcCmdFlag =  CMD_STAT_FLAG ;
                PcCmdPtr  = 0 ;
                pIntRxd = PcCmdRxdBuf[PcCmdBufNum] ;
            }
            else
            {
                PcCmdFlag = CMD_STAT_SYNC1 ;          
            }
        break ;
        case CMD_STAT_FLAG :            
            pIntRxd[PcCmdPtr] = ch;
            if(PcCmdPtr++ >= MAXBUFLEN)
            {
               PcCmdPtr = 0;         
            }
            switch(ch&0x3f)
            {
                case FRAM_TYPE_DATA :
                    PcCmdFlag =  CMD_STAT_LENTH ;
                break ;
                case FRAM_TYPE_ACK :
                case FRAM_TYPE_NAK :
                case FRAM_TYPE_EST :
                case FRAM_TYPE_ESTA :
                    PcCmdLen = 0 ;
                    PcCmdFlag =  CMD_STAT_CHECKSUM ;
                break;
                default :
                    PcCmdFlag = CMD_STAT_SYNC1 ;
                    PcCmdPtr  = 0 ;
                break ;                        
            }
        break ;
        case CMD_STAT_LENTH :
            pIntRxd[PcCmdPtr] = ch ;
            if(PcCmdPtr++ >= MAXBUFLEN)
            {
                PcCmdPtr = 0 ;           
            }
            PcCmdLen = ch ;
            if(PcCmdLen >= MAXBUFLEN-2)
            {
                PcCmdLen =  MAXBUFLEN-2 ;
            }
            PcCmdFlag =  CMD_STAT_DATA ;
        break ;
        case CMD_STAT_DATA :
            pIntRxd[PcCmdPtr] = ch ;
            if(PcCmdPtr++ >= MAXBUFLEN)
            {
                PcCmdPtr = 0;         
            }
            if(PcCmdPtr >= PcCmdLen + 2)
            {  
                PcCmdFlag =  CMD_STAT_CHECKSUM ;
            }
        break ;
        case CMD_STAT_CHECKSUM :
            PcCmdCheckSum = ch ;
            pIntRxd[PcCmdPtr] = ch ;
            if(PcCmdPtr++ >= MAXBUFLEN)
            {
                PcCmdPtr = 0;           
            }
            pExtBuf = pIntRxd ;

            if(PcCmdBufNum++ >= BUFNUM-1)
            {
                PcCmdBufNum = 0;
            }
            rxdatalen = PcCmdPtr ;
            PcCmdFlag = CMD_STAT_SYNC1 ; 
            pcCmdRcv_Task();//pcCmdRcv  = 1 ; 
        break ;
        
        //case CMD_STAT_FRAM :   
        //break;
        
        default :
            PcCmdFlag = CMD_STAT_SYNC1 ;         
            PcCmdPtr  = 0 ;
        break ;
    }
}

 
unsigned char checksum(unsigned char Initdat,unsigned char *pt,unsigned int len)
{
    unsigned int n ;
    unsigned char checksum ;
    
    checksum = Initdat ;
    
    for(n=0; n<len; n++) 
    {
	    if (checksum & 0x01)
        {
      	    checksum = (checksum >> 1) + 0x80;
        }
        else
        {
            checksum >>= 1;
        }
	    checksum += pt[n];
    }
    return(checksum) ;
}

void SendAck(unsigned char rxId)
{
    datbuf[0] = 0xeb ;
    datbuf[1] = 0x90 ;
    datbuf[2] = rxId ;
    datbuf[3] = rxId ;
    buflen = 4 ;
}
void SendDak(unsigned char txId)
{
    datbuf[buflen+0] = 0xeb ;
    datbuf[buflen+1] = 0x90 ;
    datbuf[buflen+2] = txId +0x02;
    datbuf[buflen+3] = 0x02 ;
    datbuf[buflen+4] = 0xff ;
    datbuf[buflen+5] = 0x00 ;
    datbuf[buflen+6] = checksum(0,&datbuf[buflen+2], 4) ;
    buflen += 7 ;
}


void SendReport(unsigned char txId)
{
    datbuf[buflen+0] = 0xeb ;
    datbuf[buflen+1] = 0x90 ;
    datbuf[buflen+2] = txId + 0x02 ; 
    datbuf[buflen+3] = 0x02 ;
    datbuf[buflen+4] = 0xfe ;
    datbuf[buflen+5] = 0x00 ;
    datbuf[buflen+6] = checksum(0,&datbuf[buflen+2], 4) ;
    buflen += 7 ;
}

void SendDat(unsigned char txId,unsigned char CmdId,unsigned char Len,unsigned char *pBuf)
{
    unsigned char i = 0 ;
    datbuf[buflen+0] = 0xeb ;
    datbuf[buflen+1] = 0x90 ;
    datbuf[buflen+2] = txId + 0x02 ; 
    datbuf[buflen+3] = Len + 2 ;
    datbuf[buflen+4] = CmdId ;
    datbuf[buflen+5] = Len ;
    for(i=0;i<Len;i++)
    {
       datbuf[buflen+6+i] = *(pBuf+i) ;
    }
    datbuf[buflen+6+Len] = checksum(0,&datbuf[buflen+2], Len+4) ;
    buflen += (7 + Len) ;
}





 
unsigned char CmdDn(unsigned char *pCmdDat, unsigned datalen) ;
unsigned char CheckCmd(unsigned char *pCmdDat, unsigned datalen) ;
 
unsigned char rxId ;
unsigned char cmdtxId = 0 ;
unsigned int Reset_CMD_Flag  = 0 ;
unsigned int CmdDn_Hook_Flag = 0;

void pcCmdRcv_Task(void)
{
  
    PCCMD *pPcCmd ;
    unsigned char *pCmdBuf ;
    unsigned char d_err ;
    unsigned char sum ;
    unsigned char FramType ; 

   // if( pcCmdRcv == 1 )   {   // Got PC cmd
      
        pcCmdRcv = 0 ;
        pCmdBuf = pExtBuf ;

        pPcCmd = (PCCMD *)pCmdBuf ;
        rxId = pPcCmd->Fram.FramId & 0xc0 ;
        //rxId = 0x20 ;
        FramType = pPcCmd->Fram.FramType & 0x3f ;         
        //printf("\r\n)_(:Got RXD\r\n"); //debug 
        
        switch(FramType)  {
          
            case FRAM_TYPE_DATA :    
                sum = checksum(0,pCmdBuf, PcCmdLen + 2) ;               
               
                if((sum == PcCmdCheckSum)||(PcCmdCheckSum == 0))
                {                      
                    SendAck(rxId) ;                   
                    rxId = 0x20 ;
                    if(PcCmdRxId == rxId)  {  // duplicated cmd
                        //printf("\r\n)_(:Got resend CMD\r\n"); //debug 
                    }  else  {                       
                        //printf("\r\n)_(:Got CMD\r\n"); //debug 
                        
                        PcCmdRxId   = (pPcCmd->Fram.FramId & 0xc0) ;                        
                        d_err       = CmdDn(pPcCmd->Data,pPcCmd->DataLen); // cmd parsing                        
                        SendDak(cmdtxId) ;
                        cmdtxId += 0x40 ;
                        if(d_err == 0x40)  {
                            SendDat(cmdtxId,RAW_READ_ACK,RawReadBuf.datLen,RawReadBuf.datbuf);
                            
                        } else {
                            SendReport(cmdtxId);
                            
                        }
                        cmdtxId += 0x40 ;
                    } 
                    while( ! USART_WriteBuffer(AT91C_BASE_US0, (void *)hstbuf, hstbuflen ) ) ; //give ack response to host 
                
                } else {
                   //printf("\r\n)_(:checksum error: 0x%x : 0x%x\r\n",sum, PcCmdCheckSum); //debug 
                  
                }  
                ////while(!DBGU_WriteBuffer(AT91C_BASE_DBGU, (void *)datbuf, buflen)) ;
                /*
                if( Reset_CMD_Flag == 1) //do rest after send the data
                {
                    delay_ms() ; //wait for ack sent
                    AT91C_BASE_RSTC->RSTC_RCR = 0xa5000005 ;
                }
                */
            break ;
            
            case FRAM_TYPE_EST :
                PcCmdRxId = 0x40 ;
                PcCmdTxId = rxId ;
                cmdtxId = 0 ;             
                //while (!DBGU_WriteBuffer(AT91C_BASE_DBGU, (void *)estbuf, 4));
            break ;
            
            case FRAM_TYPE_ESTA :
                PcCmdRxId = 0x40 ;
                PcCmdTxId = rxId ;             
            break ;
            
            case FRAM_TYPE_ACK :            
            break ;
            
            case FRAM_TYPE_NAK :               
            break;
            
            default :              
            break ;              
        }       
  //  }
    

}



#define CMD_NOT_SURRPORT  0x19 
// eb 90 02 09 24 07 01 02 03 04 05 06 07 00  
// eb 90 02              -- head
// 09                    -- len
// 24                    -- raw_write
// 07                    -- len
// 01 02 03 04 05 06 07  -- data
// 00                    -- check sum 

#define INSTALL_TIME_OUT (15*1000) //15s timeout

SETSSC CmdDn_Flag;

unsigned char CmdDn(unsigned char *pCmdDat, unsigned datalen)
{

    PCCMDDATA *pPcCmdData = (PCCMDDATA *)pCmdDat;
    CODECCMD * pCodeCmd ;
    unsigned char err= 0 ;
    unsigned int len = 0 ;
   // unsigned int counter = 0 ;
    
    hstbuf[0] = 0 ; //ack cmd
    hstbuflen = 1 ;  
    
    switch(pPcCmdData->CmdType)
    {
        case RAW_WRITE_CMD :      
        case MCM_CMD_HEAD :  
            if(pPcCmdData->CmdType == RAW_WRITE_CMD)  {
                pCodeCmd = (CODECCMD *)pPcCmdData->CmdData.RawWriteCmd.Data ;
                
            } else {
                pCodeCmd = (CODECCMD *)pPcCmdData->CmdData.buf ;
                
            }
            
            switch(pCodeCmd->CmdId)  {
              
                case MCM_RESET_CMD :                   
                    printf("\r\n Got command to reset MCU...MCM_RESET_CMD");
                    printf("\r\n That's all folks ! Bye !\r\n");             
                    Reset_CMD_Flag = 1 ;                                      
                    while(1) {
                        AT91C_BASE_RSTC->RSTC_RCR = 0xa5000005 ; //reset MCU     
                    }
                break ;
                
                case MCM_SET_SSC_CMD :      
                    CmdDn_Hook_Flag = 1;
                    CmdDn_Flag =  pCodeCmd->CmdDat.SetSsc;
                break ;
                
                case MCM_INIT_SSC_CMD :                  
                    CmdDn_Hook_Flag = 2;
                    CmdDn_Flag =  pCodeCmd->CmdDat.SetSsc;
                break ;      
                
                case MCM_RST_TIME :
                    delay_time_us = pCodeCmd->CmdDat.SetSsc.MODE ;
                break ;
                case MCM_SET_RST :                 
                break ;
                case MCM_CLEAR_RST :                 
                break ;
         
                case MCM_READ_PLAY_SUM :              // 
                    RawReadBuf.datbuf[0] = TxdCrcSumP&0xff ;
                    RawReadBuf.datbuf[1] = (TxdCrcSumP>>8)&0xff ;
                    RawReadBuf.datbuf[2] = (TxdCrcSumP>>16)&0xff ;
                    RawReadBuf.datbuf[3] = (TxdCrcSumP>>24)&0xff ;
                    if(TxCrcLen >0)   {
                        len = (TxCrcLen-1) * (AUDDSpeakerDriver_BYTESPERFRAME/4) ;
                        
                    }  else  {
                        len = 0 ;
                        
                    }
                    RawReadBuf.datbuf[4] = len&0xff ;
                    RawReadBuf.datbuf[5] = (len>>8)&0xff ;
                    RawReadBuf.datbuf[6] = (len>>16)&0xff ;
                    RawReadBuf.datbuf[7] = (len>>24)&0xff ;
                break ;
                
                case MCM_READ_RCD_SUM :               // 
                    RawReadBuf.datbuf[0] = RxdCrcSumP&0xff ;
                    RawReadBuf.datbuf[1] = (RxdCrcSumP>>8)&0xff ;
                    RawReadBuf.datbuf[2] = (RxdCrcSumP>>16)&0xff ;
                    RawReadBuf.datbuf[3] = (RxdCrcSumP>>24)&0xff ;
                    if(RxCrcLen >0)   {
                        len = (RxCrcLen-1) * (AUDDSpeakerDriver_BYTESPERFRAME/4) ;
                        
                    }  else  {
                        len = 0 ;
                        
                    }
                    RawReadBuf.datbuf[4] = len&0xff ;
                    RawReadBuf.datbuf[5] = (len>>8)&0xff ;
                    RawReadBuf.datbuf[6] = (len>>16)&0xff ;
                    RawReadBuf.datbuf[7] = (len>>24)&0xff ;
                break ;
                
                case MCM_READ_ERR_CODE :
                break ;
                
                case MCM_Clear_PLAY :
                    TxdCrcSum = 0 ;
                    TxdCrcSumP= 0 ;
                    TxCrcLen  = 0 ;
                    
                break ;
                
                case MCM_Clear_RCD :
                    RxdCrcSumP = 0 ;
                    RxdCrcSum = 0 ;
                    RxCrcLen  = 0 ;                     
                break ;
                
                case MCM_SET_AUDIO_NAME :
                break ;
                
                default :
                break ;
            }
        break ;
        
        case RAW_READ_CMD :
            err = 0x40 ;
            RawReadBuf.datLen = 8 ;
            memcpy(hstbuf,RawReadBuf.datbuf,RawReadBuf.datLen) ;
            hstbuflen = RawReadBuf.datLen ;
        break ;
        
        default :
            err = CMD_NOT_SURRPORT ;
        break ;
        
    }
    
    return(err) ;
    
    
}




void CmdDn_Hook( void )
{    
    unsigned int counter = 0 ;
    
    if( CmdDn_Hook_Flag == 0 ) {        
        return;        
    } 
    
    if( CmdDn_Hook_Flag == 1 ) {  
        CmdDn_Hook_Flag = 0 ;
        printf("\r\n Got command to re-enumerate USB...MCM_SET_SSC_CMD");  
        I2s_Init(CmdDn_Flag.MODE,
                 CmdDn_Flag.Brt,
                 CmdDn_Flag.Host
                 ) ;
        FramRat = CmdDn_Flag.Brt ;
        if( FramRat == 44100 )  {                      
            flag_sr_fraction = FLAG_SR_441K;   
            
        } else if(FramRat == 22050) {
            flag_sr_fraction = FLAG_SR_2205K;  
            
        } else  {                      
            flag_sr_fraction = 0;    
            
        }      

        Usb_Init() ;                
        while ( USBD_GetState() < USBD_STATE_CONFIGURED ) {                        
            if( counter++ > INSTALL_TIME_OUT ) {
                printf("\r\n USB install timeout...%ds",(unsigned short)counter/1000);              
                return ;//0xF0; //return err                        
            }
            delay_ms(1);
        };                  
        I2S_RxOn         = 1 ; //Set_I2S_RX() ;
        I2S_TxOn         = 1 ; //Set_I2S_TX() ;
        I2S_On           = 1 ; //Set_I2S() ;

    } else {
        
        CmdDn_Hook_Flag = 0 ;
        printf("\r\n Got command to reset SSC...MCM_INIT_SSC_CMD");
        I2s_Init(   CmdDn_Flag.MODE,
                    CmdDn_Flag.Brt,
                    CmdDn_Flag.Host
                 ) ;

        I2S_RxOn         = 1 ; //Set_I2S_RX() ;
        I2S_TxOn         = 1 ; //Set_I2S_TX() ;
        
    }
    
}        



void InitNoahBuf(void)
{
  
    unsigned int i=0;
    RESENDCMD *pSendCmd ;
    for(i=0;i<PCSENDBUFNUM;i++)
    {
         pSendCmd = (RESENDCMD *)pcCmdSendBuf[i] ;
         pSendCmd->repeatnum = 0 ;
         pSendCmd->time = 0 ;
         pSendCmd->ptime= 0 ;
    }
    pccmdbufin = 0;
    pccmdbufout = 0 ;
    
    //systemctr.cmdnum = 0;
    
}

