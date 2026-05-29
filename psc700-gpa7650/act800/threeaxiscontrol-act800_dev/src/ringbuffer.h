/**
 * @file ringbuffer.h
 * @brief Ring buffer implementation for UART byte stream handling.
 *
 * @details This file provides the declarations for managing a circular buffer (ring buffer)
 *          used to store UART-transmitted or received bytes.
 */

#ifndef _RINGBUFFER    /* Guard against multiple inclusion */
#define _RINGBUFFER


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include <stdint.h>
#include <stdlib.h>

#define	max(a,b)	(((a) > (b)) ? (a) : (b))
#define	min(a,b)	(((a) < (b)) ? (a) : (b))

/**
 * @struct RingBufferU8
 * @brief Structure representing an 8-bit ring buffer.
 */
typedef struct {
  volatile uint8_t* storage;      /**< Pointer to the start of the buffer storage array */
  volatile uint8_t* end;          /**< Pointer to the end of the buffer storage */
  volatile uint16_t size;         /**< Total size of the buffer */
  volatile uint8_t* read;         /**< Current read pointer */
  volatile uint8_t* write;        /**< Current write pointer */
  volatile uint16_t available;    /**< Number of bytes currently stored in the buffer */
  volatile uint16_t line_counts;  /**< Number of newline characters in the buffer (for line-based reading) */
} RingBufferU8;

void RingBufferU8_init(RingBufferU8* ring, uint8_t* storage, uint16_t size);
uint16_t RingBufferU8_available(RingBufferU8* ring);
uint16_t RingBufferU8_free(RingBufferU8* ring);
void RingBufferU8_clear(RingBufferU8* ring);
void RingBufferU8_read(RingBufferU8* ring, uint8_t* buffer, uint16_t size);
uint8_t RingBufferU8_readByte(RingBufferU8* ring);
void RingBufferU8_write(RingBufferU8* ring, uint8_t* buffer, uint16_t size);
void RingBufferU8_writeByte(RingBufferU8* ring, uint8_t b);
uint16_t RingBufferU8_readUntil(RingBufferU8* ring, uint8_t* buffer, uint16_t size, uint8_t stopByte);
uint16_t RingBufferU8_readLine(RingBufferU8* ring, uint8_t* buffer, uint16_t size);
uint8_t RingBufferU8_peek(RingBufferU8* ring);
uint8_t RingBufferU8_peekn(RingBufferU8* ring, uint16_t i);
uint8_t RingBufferU8_deleteByte(RingBufferU8* ring);

#endif //_RINGBUFFER

/* *****************************************************************************
 End of File
 */
