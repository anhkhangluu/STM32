/* Includes ----------------------------------------------------------------- */
#include "cbuffer.h"

/* Private defines ---------------------------------------------------------- */
#define FAST_THRESHOLD (5)

/* Public function prototypes ----------------------------------------------- */

void cbuffer_init(cbuffer_t *cb, void *buf, uint16_t size)
{
  cb->data     = (uint8_t *)buf;
  cb->size     = size;
  cb->reader   = 0;
  cb->writer   = 0;
  cb->overflow = 0;
}

void cbuffer_clear(cbuffer_t *cb)
{
  cb->reader   = 0;
  cb->writer   = 0;
  cb->overflow = 0;
}

uint16_t cbuffer_read(cbuffer_t *cb, uint8_t *buf, uint16_t n_bytes)
{
  uint16_t i;

  for (i = 0; i < n_bytes; i++)
  {
    // See if any data is available
    if (cb->reader != cb->writer)
    {
      // Grab a byte from the internal buffer
      *buf = cb->data[cb->reader];
      buf++;

      // Check for wrap-around
      if (cb->reader + 1 == cb->size)
        cb->reader = 0;
      else
        cb->reader = cb->reader + 1;
    }
    else
    {
      break;
    }
  }

  return i; // Number of bytes read
}

uint16_t cbuffer_write(cbuffer_t *cb, uint8_t *buf, uint16_t n_bytes)
{
  uint16_t i;

  for (i = 0; i < n_bytes; i++)
  {
    // First check to see if there is space in the buffer
    if ((cb->writer + 1 == cb->reader) || 
        ((cb->writer + 1 == cb->size) && 
         (cb->reader == 0))
       )
    {
      cb->overflow += (n_bytes - i);
      break;
    }
    else
    {
      // Write a byte to the internal buffer
      cb->data[cb->writer] = *buf;
      buf++;

      // Check for wrap-around
      if (cb->writer + 1 == cb->size)
        cb->writer = 0;
      else
        cb->writer = cb->writer + 1;
    }
  }

  return i; // Number of bytes write
}

uint16_t cbuffer_data_count(cbuffer_t *cb)
{
  uint16_t tmp_reader, tmp_writer;
  tmp_reader = cb->reader;
  tmp_writer = cb->writer;

  if (tmp_reader <= tmp_writer)
    return (tmp_writer - tmp_reader);
  else
    return (tmp_writer + cb->size - tmp_reader);
}

uint16_t cbuffer_space_count(cbuffer_t *cb)
{
  return (cb->size - 1 - cbuffer_data_count(cb));
}
/* End of file -------------------------------------------------------------- */
