/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "stm32f3xx_hal.h"
#include "flash_desc.h"

/* Public defines ----------------------------------------------------- */
#define FLASH_APP_START_ADDRESS ((uint32_t)(APP_START))
#define FLASH_APP_END_ADDRESS   ((uint32_t)(APP_START + APP_SIZE)) /**< Leave a little extra space at the end. */
#define FLASH_APP_PAGE_MAX      (((APP_SIZE / 1024) / 2)) // Page max - (page's amount for bootloader & configuration)

/* Public macros ------------------------------------------------------ */
/* Public enumerate/structure ----------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

/* Status report for the functions. */
typedef enum 
{
  FLASH_OK              = 0x00u, /**< The action was successful. */
  FLASH_ERROR_SIZE      = 0x01u, /**< The binary is too big. */
  FLASH_ERROR_WRITE     = 0x02u, /**< Writing failed. */
  FLASH_ERROR_READBACK  = 0x04u, /**< Writing was successful, but the content of the memory is wrong. */
  FLASH_ERROR           = 0xFFu  /**< Generic error. */
} 
flash_status_t;

/**
 * @brief   This function erases in the flash memory.
 * 
 * @param[in] page: The page's position start is erase
 * @param[in] num_page: The number of page want to be erased
 * 
 * @return  Report about the success of the erasing.
 */
flash_status_t flash_erase(uint32_t page, uint32_t num_page);

/**
 * @brief   This function is write flashes the memory.
 * 
 * @param[in] address: First address to be written to.
 * @param[in] buf:   Pointer of write buffer.
 * @param[in] len: len want to write
 * 
 * @return  Report about the success of the writing.
 */
flash_status_t flash_write(uint32_t address, uint64_t *buf, uint32_t len);

/**
 * @brief   This function read data in the memory.
 * @param[in]  address: First address to be read to.
 * @param[out] data:   Pointer of read buffer
 * @param[in]  len: len want to read
 * @return  Report about the success of the read.
 */
flash_status_t flash_read(uint32_t address, uint8_t *data, uint32_t len);

/*!
 * @brief  Gets the page of a given address
 *
 * @param[in] address: Address of the FLASH Memory
 *
 * @return  The page of a given address
 */
uint32_t flash_get_page(uint32_t addr);

/*!
 * @brief  This functon jump to applicaion
 */
void flash_jump_to_app(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */ 

#endif // __FLASH_H
/* End of file -------------------------------------------------------- */
