/**
 * @file ringbuffer.c
 * @brief Ring buffer implementation for UART byte stream handling.
 *
 * @details This file provides the declarations for managing a circular buffer (ring buffer)
 *          used to store UART-transmitted or received bytes.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "math.h"
#include "ringbuffer.h"


/**
 * @brief Initializes a RingBufferU8 instance.
 *
 * @details Sets up the internal pointers and counters for a given ring buffer structure.
 *          This function must be called before using the ring buffer.
 *
 * @param ring Pointer to the RingBufferU8 structure to initialize.
 * @param storage Pointer to the buffer array where data will be stored.
 * @param size Size of the buffer array in bytes.
 */
void RingBufferU8_init(RingBufferU8* ring, uint8_t* storage, uint16_t size) {
  ring->storage = storage;
  ring->size = size;
  ring->end = ring->storage + ring->size;
  ring->read = ring->storage;
  ring->write = ring->storage;
  ring->available = 0;
  ring->line_counts = 0;
}

/**
 * @brief Gets the number of bytes currently stored in the ring buffer.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @return Number of bytes available to read.
 */
uint16_t RingBufferU8_available(RingBufferU8* ring) {
  return ring->available;
}

/**
 * @brief Gets the number of bytes free for writing in the ring buffer.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @return Number of free bytes.
 */
uint16_t RingBufferU8_free(RingBufferU8* ring) {
  return ring->size - ring->available;
}

/**
 * @brief Clears the ring buffer.
 *
 * @details Resets read/write pointers, available count, and line count.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 */
void RingBufferU8_clear(RingBufferU8* ring) {
  ring->read = ring->storage;
  ring->write = ring->storage;
  ring->available = 0;
  ring->line_counts = 0;
}

/**
 * @brief Reads one byte from the ring buffer.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @return The byte read, or 0 if buffer is empty.
 */
uint8_t RingBufferU8_readByte(RingBufferU8* ring) {
  if (ring->available == 0) {
    return 0;
  }
  uint8_t ret = *ring->read++;
  ring->available--;
  if (ring->read >= ring->end) {
    ring->read = ring->storage;
  }
  if (ret=='\n')
    ring->line_counts--;
  return ret;
}

/**
 * @brief Reads multiple bytes from the ring buffer.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @param buffer Pointer to the destination buffer.
 * @param size Number of bytes to read.
 */
void RingBufferU8_read(RingBufferU8* ring, uint8_t* buffer, uint16_t size) {
  uint16_t i;

  // TODO can be optimized
  for (i = 0; i < size; i++) {
    buffer[i] = RingBufferU8_readByte(ring);
  }
}

/**
 * @brief Writes a single byte to the ring buffer.
 *
 * @details If the buffer is full, the oldest byte will be discarded.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @param b The byte to write.
 */
void RingBufferU8_writeByte(RingBufferU8* ring, uint8_t b) {
  if (ring->available >= ring->size) {
    RingBufferU8_readByte(ring);
  }

  *ring->write = b;
  ring->write++;
  ring->available++;
  if (ring->write >= ring->end) {
    ring->write = ring->storage;
  }
  if (b == '\n')
    ring->line_counts++;
}

/**
 * @brief Writes multiple bytes to the ring buffer.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @param buffer Pointer to the source buffer.
 * @param size Number of bytes to write.
 */
void RingBufferU8_write(RingBufferU8* ring, uint8_t* buffer, uint16_t size) {
  uint16_t i;

  // TODO can be optimized
  for (i = 0; i < size; i++) {
    RingBufferU8_writeByte(ring, buffer[i]);
  }
}

/**
 * @brief Reads a line (ending with '\\n') from the ring buffer.
 *
 * @details Wrapper for readUntil with `\n` as the stop byte.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @param buffer Pointer to the destination buffer.
 * @param size Size of the destination buffer.
 * @return Number of bytes read, including the '\\n'.
 */
uint16_t RingBufferU8_readLine(RingBufferU8* ring, uint8_t* buffer, uint16_t size) {
  return RingBufferU8_readUntil(ring, buffer, size, '\n');
}

/**
 * @brief Reads bytes until a specified stop byte is encountered.
 *
 * @details Stops when `stopByte` is found or buffer is full. Output is null-terminated.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @param buffer Pointer to the destination buffer.
 * @param size Size of the destination buffer.
 * @param stopByte The byte to stop at.
 * @return Number of bytes read, or 0 if stopByte not found.
 */
uint16_t RingBufferU8_readUntil(RingBufferU8* ring, uint8_t* buffer, uint16_t size, uint8_t stopByte) {
  uint8_t b;
  uint16_t i;
  for (i = 0; i < min(ring->available, size - 1); i++) {
    b = RingBufferU8_peekn(ring, i);
    if (b == stopByte) {
      i++;
      RingBufferU8_read(ring, (uint8_t*) buffer, i);
      buffer[i] = '\0';
      return i;
    }
  }
  buffer[0] = '\0';
  return 0;
}

/**
 * @brief Peeks the first byte in the ring buffer without removing it.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @return First byte in the buffer.
 */
uint8_t RingBufferU8_peek(RingBufferU8* ring) {
  return RingBufferU8_peekn(ring, 0);
}

/**
 * @brief Peeks the nth byte in the ring buffer without removing it.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @param i Index of the byte to peek (0 = first byte).
 * @return The nth byte, or 0 if out of range.
 */
uint8_t RingBufferU8_peekn(RingBufferU8* ring, uint16_t i) {
  if (i >= ring->available) {
    return 0;
  }

  uint8_t* read = (uint8_t*)ring->read;
  uint8_t* p = read + i;
  if (p >= ring->end) {
    p -= ring->size;
  }
  return *p;
}

/**
 * @brief Deletes the most recently written byte in the ring buffer.
 *
 * @details Adjusts write pointer and available count. If the deleted byte was '\\n',
 *          line count is decreased.
 *
 * @param ring Pointer to the RingBufferU8 structure.
 * @return 1 if a byte was deleted, 0 if buffer was empty.
 */
uint8_t RingBufferU8_deleteByte(RingBufferU8* ring) {
    if (ring->available > 0) {
        if (*ring->write == '\n')
            ring->line_counts--;
        ring->write--;
        ring->available--;
        
        if (ring->write < ring->storage) {
            ring->write = ring->end - 1;
        }
        return 1;
    }
    return 0;
}

/* *****************************************************************************
 End of File
 */
