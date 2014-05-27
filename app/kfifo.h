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

#ifndef _KFIFO_INC_
#define _KFIFO_INC_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Note that with only one concurrent reader and one concurrent
 * writer, you don't need extra locking to use these functions.
 */

typedef struct _kfifo {
	unsigned char *buffer;	// the buffer holding the data
	unsigned int size;		// the size of the allocated buffer
	unsigned int in;		// data is added at offset (in % size)
	unsigned int out;		// data is extracted from off. (out % size)
} kfifo_t;


void kfifo_init(kfifo_t *fifo, int size);
void kfifo_init_static(kfifo_t *fifo, unsigned char *pBuf, int size);
void kfifo_free(kfifo_t *fifo);

// put data into buffer
unsigned int kfifo_put(kfifo_t *fifo, unsigned char *buffer, unsigned int len);

// take data from buffer
unsigned int kfifo_get(kfifo_t *fifo, unsigned char *buffer, unsigned int len);

//release data memory
unsigned int kfifo_release(kfifo_t *fifo, unsigned int len) ;

// calculate free space
unsigned int kfifo_get_free_space(kfifo_t *fifo);

unsigned int kfifo_get_data_size(kfifo_t *fifo);


#ifdef __cplusplus
}
#endif

#endif
