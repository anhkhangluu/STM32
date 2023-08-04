/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BSL_TRANSPORT_H
#define __BSL_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "bsl_def.h"

/* Public defines ----------------------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public enumerate/structure ----------------------------------------- */
/* Exported constants ------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/*!
 * @brief  This API initialize transport layer
 */
void bsl_transport_init();

/*!
 * @brief  This API writes data from the BSL transport layer to the linker layer.
 *
 * @param[in]  data: Pointer of write buffer.
 * @param[in]  len: Length want to write
 *
 * @return  Length written
 */
uint16_t bsl_write(uint8_t *buf, uint16_t len);

/*!
 * @brief  This API read data from the linker layer to the BSL transport layer.
 *
 * @param[out]  data: Pointer of read buffer.
 * @param[in]  len: Length want to write
 *
 * @return  Length written
 */
uint16_t bsl_read(uint8_t *data, uint16_t len);

/*!
 * @brief  This API get number of bytes unread in ring buffer.
 *
 * @return  Number of bytes unread
 */
uint16_t bsl_get_unread_byte(void);

/*!
 * @brief  This API calculates checksum by using the crc16-ccitt algorithm.
 *
 * @param[out]  data: Pointer to buffer to calculate CRC.
 * @param[in]  len: Length want to calculate.
 *
 * @return  2 bytes CRC
 */
uint16_t bsl_crc16_ccitt(const uint8_t *data, uint16_t len);
/* -------------------------------------------------------------------- */
#ifdef __cplusplus
}
#endif /* __cplusplus */ 

#endif // __BSL_TRANSPORT_H
/* End of file -------------------------------------------------------- */
