/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

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

typedef struct {
  volatile uint8_t* storage;
  volatile uint8_t* end;
  volatile uint16_t size;
  volatile uint8_t* read;
  volatile uint8_t* write;
  volatile uint16_t available;
  volatile uint16_t line_counts;
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

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
