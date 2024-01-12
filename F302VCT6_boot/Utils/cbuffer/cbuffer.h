#ifndef __CBUFFER_H
#define __CBUFFER_H
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* Includes ----------------------------------------------------------------- */
#include "stdint.h"
#include "stdbool.h"
#include "string.h"

/* Public enumerate/structure ----------------------------------------------- */
/**
 * \brief Cbuffer manage handler
 * 
 */
typedef struct
{
  uint8_t *data;      /**< Poiter which data log to */
  uint16_t size;      /**< size of buffer */
  uint16_t writer;    /**< Writing position */
  uint16_t reader;    /**< Reading position */
  uint16_t overflow;  /**< Number data overflow */
}
cbuffer_t;

/* Public function prototypes ----------------------------------------------- */
/**
 * \brief   Initializes the CB structure with the given buffer and size \n
 *
 * \param[in]    cb         CBuffer Pointer
 * \param[in]    buf        CBuffer Buffer
 * \param[in]    size       size of CBuffer Buffer
 *
 * \return  None
 */
void cbuffer_init(cbuffer_t *cb, void *buf, uint16_t size);

/**
 * \brief   Reset the CB structure
 *
 * \param[in]    cb         CBuffer Pointer
 *
 * \return  None
 */
void cbuffer_clear(cbuffer_t *cb);

/**
 * \brief   Read upto nBytes from the CB. \n
 *          Actual number of read bytes is returned
 *
 * \param[in]    cb         CBuffer Pointer
 * \param[in]    buf        Pointer of read buffer
 * \param[in]    nBytes     Number of bytes want to read
 *
 * \return  Number of read bytes
 */
uint16_t cbuffer_read(cbuffer_t *cb, uint8_t *buf, uint16_t bytes);

/**
 *
 * \brief   Write upto nBytes to the CB. \n
 *          Actual number of written byte is returned.
 *
 * \param[in]    cb         CBuffer Pointer
 * \param[in]    buf        Pointer of write buffer
 * \param[in]    nBytes     Number of bytes want to write
 *
 * \return  Number of write bytes
 */
uint16_t cbuffer_write(cbuffer_t *cb, uint8_t *buf, uint16_t bytes);

/**
 * \brief   Return number of unread bytes in the CB.
 *
 * \param[in]    cb         CBuffer Pointer
 *
 * \return  unread bytes in the CB.
 */
uint16_t cbuffer_data_count(cbuffer_t *cb);

/**
 * \brief   Return number of free bytes in the CB.
 *
 * \param[in]    cb         CBuffer Pointer
 *
 * \return  free bytes in the CB.
 */
uint16_t cbuffer_space_count(cbuffer_t *cb);

#ifdef __cplusplus
  } // extern "C" {
#endif
#endif // __CBUFFER_H
/* End of file -------------------------------------------------------------- */
