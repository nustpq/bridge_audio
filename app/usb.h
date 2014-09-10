#ifndef _USB_INC_
#define _USB_INC_


extern unsigned long long total_received ;
extern unsigned long long total_transmit ;
extern unsigned int error_bulkout_full  ;
extern unsigned int error_bulkout_empt  ;
extern unsigned int error_bulkin_full   ;
extern unsigned int error_bulkin_empt   ;



extern void UsbDataReceived(  unsigned int unused,
                              unsigned char status,
                              unsigned int received,
                              unsigned int remaining );

extern void UsbDataTransmit(  unsigned int unused,
                              unsigned char status,
                              unsigned int transmit,
                              unsigned int remaining );


void Usb_Init(void);
void Init_Bulk_FIFO( void );
void Init_Bulk_In_FIFO( void );
void Init_Bulk_Out_FIFO( void );
                            
                              
#endif
