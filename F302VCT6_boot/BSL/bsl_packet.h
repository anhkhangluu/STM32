/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __BSL_PACKET_H
#define __BSL_PACKET_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "bsl_def.h"

/* Public defines ----------------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------------- */
typedef struct
{  
  uint8_t   *buf;    /**<  */
  uint16_t   cnt;    /**<  */
  uint16_t   len;    /**<  */
}
bsl_builder_hdl_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
typedef void (*bsl_cb_t) (void);

/* Public APIs -------------------------------------------------------------- */

/* Builder ------------------------------------------------------------------ */
/**
 * @brief       Initialize the builder with the given information
 *
 * @param[in]   <h>       Pointer to builder structure
 * @param[in]   <buf>     Buffer to store the output packet
 * @param[in]   <len>     Length of <buf>, must be at least 4 bytes long
 *
 * @attention   None
 *
 * @return      None
 */
int bsl_builder_init(bsl_builder_hdl_t *h, uint8_t *buf, uint16_t len);

/**
 * @brief       Reset buffer and start building packet
 *
 * @param[in]   <h>        Pointer to builder structure
 * @param[in]   <pkt_type> Packet type
 *
 * @attention   None
 *
 * @return      None
 */
int bsl_builder_begin(bsl_builder_hdl_t *h);

/**
 * @brief       Write ACK
 *
 * @param[in]   <h>       Pointer to builder structure
 * @param[in]   <ack>     ACK
 *
 * @attention   None
 *
 * @return
 *  - BSL_OK:   Always succeed
 *  - BSL_ERR:  Not enough space
 */
int bsl_builder_write_ack(bsl_builder_hdl_t *h, uint8_t ack);

/**
 * @brief       Stop building packet and calculate packet CRC
 *
 * @param[in]   <h>           Pointer to builder structure
 * @param[in]   <crc16_func>  Pointer to crc16_ccitt algorithm
 *
 * @return
 *  - BSL_OK:   Succeed
 *  - BSL_ERR:  Not enough space to append CRC field
 */
int bsl_builder_end(bsl_builder_hdl_t *h, bsl_crc16_func_t crc16_func);


/* Parser ------------------------------------------------------------------- */
/*!
 * @brief  This API initialize the parser
 *
 * @param[in]  data: pointer to data buffer which is to be stored payload
 * @param[in]  len : length of data
 * @param[in]  crc16_func : Pointer to crc16_ccitt algorithm
 *
 * @return  
 *  - BSL_OK: Success
 *  - BSL_ERR: Error
 *  - BSL_RSP_ERR_CHECKSUM_INCORRECT: Checksum failed
 */
int bsl_parser_handle(uint8_t *data, uint16_t len, bsl_crc16_func_t crc16_func);

int bsl_verify_image();
/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

#endif /* __BSL_PACKET_H */

/* End of file -------------------------------------------------------------- */
