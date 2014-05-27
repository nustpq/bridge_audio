#if !defined PCCMD_H
#define PCCMD_H

#define BUFNUM          3
#define MAXBUFLEN       30
#define PCTXDBUFLEN     500
#define PCCMDBUFMAXNUM  5  
#define PCSENDBUFNUM    50 
#define CMD_STAT_SYNC1    0x00
#define CMD_STAT_SYNC2    0x01
#define CMD_STAT_FLAG     0x02
#define CMD_STAT_LENTH    0x04
#define CMD_STAT_DATA     0x08
#define CMD_STAT_FRAM     0x10
#define CMD_STAT_CHECKSUM 0x20

#define CMD_DATA_SYNC1    0xeb
#define CMD_DATA_SYNC2    0x90
#define CMD_DATA_SYNC2_1  0x90
#define FRAM_TYPE_ACK     0x00
#define FRAM_TYPE_NAK     0x01
#define FRAM_TYPE_DATA    0x02
#define FRAM_TYPE_EST     0x3e
#define FRAM_TYPE_ESTA    0x3f

#define DATA_TYPE_GPIO    0x03
#define DATA_TYPE_ADC     0x31
#define DATA_TYPE_DM      0x11
#define DATA_TYPE_PM      0x16
#define DATA_TYPE_EM      0x22
#define DATA_TYPE_RAW     0x26
#define DATA_TYPE_MONIT   0x82
#define DATA_TYPE_REGTIME 0x84
//PC to MCU
#define GPIO_WRITE_CMD        0x00   // set GPIO H or L
#define GPIO_HIGHTZ_CMD       0x01   // set GPIO Hight Impendance
#define GPIO_READ_CMD         0x02   // read GPIO
#define DM_SINGLE_WRITE_CMD   0x10   // DM single write
#define DM_BURST_WRITE_CMD    0x12   // DM burst write
#define DM_SINGLE_READ_CMD    0x13   // DM single read
#define PM_BURST_WRITE_CMD    0x14   // PM burst write
#define PM_SINGLE_READ_CMD    0x15   // PM single read
#define INTERFACE_CTR_CMD     0x20   // Interface ctr
#define AD_READ_CMD           0x30   // AD read
#define RELAY_CTR_CMD         0x40   // relay ctr
#define SESSION_CTR_CMD       0x50   // session ctr
#define REPEAT_CTR_CMD        0x51   // repeat ctr
#define DELAY_CTR_CMD         0x52   // delay ctr
#define SPECIAL_CTR_CMD       0x80   // special ctr
#define POWER_ADJ_CMD         0x60   // adjust voltage
#define POWER_CTR_CMD         0x61   // power turn on-off
#define FRQ_ADJ_CMD           0x70   // adjust frq
#define FRQ_CTR_CMD           0x71   // FRQ turn on-off
#define EM_WRITE_CMD          0x21   // eeprom write
#define EM_READ_CMD           0x23   // eeprom read
#define RAW_WRITE_CMD         0x24   // RAW write
#define RAW_READ_ACK          0x25   // RAW read ack
#define RAW_READ_CMD          0x26   // RAW read
#define GPIO_RECORDE_CMD      0x81   // gpio recorde      
#define REG_TIME_CMD          0x83    
#define MCM_CMD_HEAD          0xa0
// codec cmd
#define MCM_LOAD_FILE         0x00
#define MCM_VOL_CMD           0x01
#define MCM_START_CMD         0x02
#define MCM_STOP_CMD          0x03
#define MCM_RESET_CMD         0x05
#define MCM_SET_SSC_CMD       0x06
#define MCM_READ_PLAY_SUM     0x07
#define MCM_READ_RCD_SUM      0x08
#define MCM_READ_ERR_CODE     0x09
#define MCM_Clear_PLAY        0x0a
#define MCM_Clear_RCD         0x0b
#define MCM_Clear_ERR_CODE    0x0c
#define MCM_SET_AUDIO_NAME    0x0d
#define MCM_INIT_SSC_CMD      0x25
#define MCM_SET_SSC_TEST_ST   0x26
#define MCM_RST_TIME          0x31
#define MCM_SET_RST           0x32
#define MCM_CLEAR_RST         0x33

#define RESET_CMD             0x01
#define PLAY_CMD              0x02
#define SET_VOL_CMD           0x03
#define WRITE_FILE_MSG_CMD    0x04
#define STOP_CMD              0x05
#define SET_FRAM_CMD          0x06
#define DOWN_LOAD_CMD         0x07
#define READ_FLIE_MSG_CMD     0x08
#define START_CMD             0x09
#define LOAD_BUF              0x0a
// MCU to PC
#define GPIO_RPT 0x03       // GPIO report
#define DM_SRPT  0x11       // DM single report
#define PM_SRPT  0x16       // PM single report
#define ADC_RPT  0x31       // ADC report
#define REPORT_FINISH 0x53  // report finish
#define EM_RPT   0x22       // eeprom report
#define RAW_RPT  0x26       // RAW RPT
#define CMD_ERR_RPT    0xff
#define CMDDN_ERR_RPT  0xfe
#define UNKOW_ERR_RPT  0x66

#pragma pack(1)

typedef struct __RESENDCMD
{
    unsigned int  repeatnum ; // 重发次数
    unsigned int  time ;      // 重发时间间隔
    unsigned int  ptime ;
    unsigned int  datlen ;
    unsigned char buf[1] ;    // 数据
}RESENDCMD ;
typedef struct __GPIOCMD
{
    unsigned long PinDefine ;
    unsigned long PinData ;
}GPIOCMD ;

typedef struct __DMCMD
{
    unsigned char DmNum ;
    unsigned int  Data ;
}DMCMD ;

typedef struct __DMBUSTCMD
{
    unsigned char DmNum ;
    unsigned char DatBuf[1] ;
}DMBUSTCMD ;

typedef struct __ADCMD
{
    unsigned char addefine ;
}ADCMD ;

typedef struct __RELAYCMD
{
    unsigned char RelayDefine ;
    unsigned char RelayState ;
}RELAYCMD ;
typedef struct __DELAYCMD
{
    unsigned int Time : 14;
    unsigned int Unit : 2;
}DELAYCMD ;

typedef struct __POWERAJDCMD
{
    unsigned char Ch ;
    unsigned int Voltage ;
}POWERADJCMD ;

typedef struct __POWERCTRCMD
{
    unsigned char define ;
    unsigned char Ctr ;
}POWERCTRCMD ;

typedef struct __FRQADJCMD
{
    unsigned char Ch ;
    unsigned char FrqType ;
    unsigned long Frq ;
    unsigned int  Volt ;
}FRQADJCMD ;

typedef struct __FRQCTRCMD
{
    unsigned char Ch ;
    unsigned char Ctr ;
}FRQCTRCMD ;

typedef struct __REPEATCMD
{
    unsigned int Times : 15;
    unsigned int Flage : 1 ;
}REPEATCMD ;

typedef struct __INTERFACECMD
{
    unsigned char DeviceId ;
    unsigned char Speed ;
    unsigned char Mode ;
}INTERFACECMD ;

typedef struct __ADCCMD
{
    unsigned char AdcCh ;
    unsigned char mult ;
}ADCCMD ;

typedef struct __RAWWRITECMD
{
    unsigned char DataLen ;
    unsigned char Data[10] ;
}RAWWRITECMD ;

typedef struct __EPWRITECMD
{
    unsigned char DataLen ;
    unsigned int DataAddr ;
    unsigned char Data[1] ;
}EPWRITECMD ;

typedef struct __GPIORECORDE
{
    unsigned long PinDefine ;
    unsigned char TimeCyc ;
    unsigned char TimeDlay ;
}GPIORECORDECMD ;

typedef struct __REGTIMECMD
{
    unsigned int DmAddr ;
    unsigned int Mask ;
             int DmGate ;
    unsigned char OpCode ;
}REGTIMECMD ;
typedef union __PCCMDDAT
{
    unsigned char buf[50] ;
    GPIOCMD   GpioCmd ;
    DMCMD     DmCmd ;
    DMBUSTCMD DmBustCmd ;
    ADCCMD    AdcDefine ;
    RELAYCMD  RelayCmd ;
    unsigned char SessionType ;
    DELAYCMD  Delay;
    POWERADJCMD PowerAdjCmd ;
    POWERCTRCMD PowerCtrCmd ;
    FRQADJCMD   FrqAdjCmd ;
    FRQCTRCMD   FrqCtrCmd  ;
    REPEATCMD   RepeatCmd ;
    INTERFACECMD InterfaceCmd ;
    EPWRITECMD  EpWriteCmd ;  
    RAWWRITECMD RawWriteCmd ;
    GPIORECORDECMD GpioRecordeCmd ;
    REGTIMECMD   RegTimeCmd ;
}PCCMDDAT ;

typedef struct __PCCMDDATA
{
    unsigned char CmdType ;
    PCCMDDAT CmdData ;
}PCCMDDATA ;

// pc cmd
typedef union __FRAM
{
    unsigned char FramType ;
    unsigned char FramId   ;
}FRAM ;
typedef struct __PCCMD
{
    FRAM Fram;
    unsigned char DataLen ;
    unsigned char Data[256] ;
}PCCMD ;

typedef struct __PLAYFILE
{
    unsigned char FileIndxL ;
    unsigned char FileIndxR ;
}PLAYFILE ;
typedef struct __LOADBUF
{
    unsigned int LfAddr ;
    unsigned int LfLen  ;
    unsigned int RfAddr ;
    unsigned int RfLen  ;
}LOADBUF ;
typedef struct __SETVOL 
{
    unsigned char vol_l ;
    unsigned char vol_r ;
}SETVOL ;

typedef struct __SETFILEMSG 
{
    unsigned char FileIndx ;
    unsigned int  FileLen  ;
    unsigned int  FileAddr ;
    unsigned int  FramRat  ;
    unsigned char FileName[1] ;
}SETFILEMSG ;

typedef struct __LOADFILE
{
    unsigned char FileIndx ;
}LOADFILE ;

typedef struct __READFILEMSG
{
    unsigned char FileIndx ;   
}READFILEMSG ;

typedef struct __SETSSC
{
    unsigned int    MODE ;  // 接口模式
    unsigned short  Brt  ;  // 采样率
    unsigned char   Host ;  // 主 or 从
}SETSSC ;

typedef struct __RAW_READ_BUF
{
    unsigned char datLen ;
    unsigned char datbuf[30] ;
}RAW_READ_BUF;

typedef union __CMDDAT 
{
    unsigned char dat[50] ;
    PLAYFILE      PlayFile ;
    LOADBUF       LoadBuf ;
    SETVOL        SetVol ;
    SETFILEMSG    SetFileMsg ;
    LOADFILE      LoadFile ;
    READFILEMSG   ReadFileMsg ;
    SETSSC        SetSsc ;
}CMDDAT ;

typedef struct __CODECCMD
{
    unsigned char DivAddr ;  // 器件地址
    unsigned char CmdId   ;  // 命令码
    CMDDAT     CmdDat ;
}CODECCMD ;

#pragma pack()

extern unsigned char PCuartbuf[] ;   

extern void pcInt(unsigned char ch) ;
extern void pcSendTask(void) ;
extern void pcCmdRcv_Task(void) ;
extern void pcSendDateToBuf(unsigned int len,unsigned char *pdat) ;
extern void pcSendAck(unsigned char framtype) ;

extern void CmdDn_Hook( void ) ;


#endif

