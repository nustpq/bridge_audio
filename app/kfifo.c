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
*                                          APP PACKAGE
*
*                                         Atmel  SAM3U4C
*                                             on the
*                                      iSAM Audio Bridge Board
*
* Filename      : kfifo.c
* Version       : V1.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/
#include <stdlib.h>
#include <string.h>
#include <irq/irq.h>
#include "kfifo.h"



// Note: 
// The fifo size must be 2^n length.

#define min(a,b) ((a) < (b)? (a) : (b) )

void kfifo_init(kfifo_t *fifo, int size) {
	fifo->buffer = (unsigned char *) malloc(size);
	fifo->size = size;
	fifo->in = fifo->out = 0;
}

void kfifo_free(kfifo_t *fifo) {
	free(fifo->buffer);
}

void kfifo_init_static(kfifo_t *fifo, unsigned char *pBuf, int size) {
	fifo->buffer = pBuf;
	fifo->size = size;
	fifo->in = fifo->out = 0;
}

void kfifo_reset(kfifo_t *fifo) {
	fifo->in = fifo->out = 0;
}



unsigned int kfifo_put(kfifo_t *fifo, unsigned char *buffer, unsigned int len) {
	unsigned int l;
        //__disable_interrupt(); //PQ
	len = min(len, fifo->size - fifo->in + fifo->out);
	/* first put the data starting from fifo->in to buffer end */
	l = min(len, fifo->size - (fifo->in & (fifo->size - 1)));
	//l<=len
	memcpy(fifo->buffer + (fifo->in & (fifo->size - 1)), buffer, l);
	/* then put the rest (if any) at the beginning of the buffer */
	memcpy(fifo->buffer, buffer + l, len - l);
	fifo->in += len;
        //__enable_interrupt(); //PQ
	return len;
}

unsigned int kfifo_get(kfifo_t *fifo, unsigned char *buffer, unsigned int len) {
	unsigned int l;       
        //__disable_interrupt(); //PQ
	len = min(len, fifo->in - fifo->out);
	/* first get the data from fifo->out until the end of the buffer */
	l = min(len, fifo->size - (fifo->out & (fifo->size - 1)));
	memcpy(buffer, fifo->buffer + (fifo->out & (fifo->size - 1)), l);
	/* then get the rest (if any) from the beginning of the buffer */
	memcpy(buffer + l, fifo->buffer, len - l);
	fifo->out += len;
        //__enable_interrupt(); //PQ
	return len;
}

unsigned int kfifo_release(kfifo_t *fifo, unsigned int len) {
        //__disable_interrupt(); //PQ
	len = min(len, fifo->in - fifo->out);	
	fifo->out += len;
        //__enable_interrupt(); //PQ
	return len;
}


//no interruption for debug printf use
unsigned int kfifo_get_free_space(kfifo_t *fifo) {
    
    return( fifo->size - fifo->in + fifo->out );     

}

unsigned int kfifo_get_data_size(kfifo_t *fifo) {
           
    return( fifo->in - fifo->out );     
	
}

