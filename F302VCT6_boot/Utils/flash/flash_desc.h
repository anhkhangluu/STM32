/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __FLASH_DESC_H
#define __FLASH_DESC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
/* Public defines ----------------------------------------------------- */
//        KTC FLASH
//  +---------------------+ 0x0800 0000
//  |                     |
//  |  BOOLOADER (28 KB)  |
//  |                     |
//  +-------------+-------+ 0x0800 7000
//  |  CONFIG (2 KB)      |
//  +-------------+-------+ 0x0800 7800
//  |             | HEADER|
//  |             +-------+ 0x0800 7900
//  | APP (98 KB) |       |
//  |             | IMAGE |
//  |             |       |
//  +-------------+-------+ 0x0803 ffff

// FLASH
#define FLASH_START ((uint32_t)0x08000000u)
#undef FLASH_SIZE
#define FLASH_SIZE  (0x40000u) // 256 * 1024 bytes

// BOOTLOADER
#define BOOTLOADER_START (FLASH_START) // 0x0800 0000
#define BOOTLOADER_SIZE  (0x7000u)     // 28 * 1024 bytes

// CONFIG
#define CONFIG_START     ((uint32_t)0x08007000u)
#define CONFIG_SIZE      (0x800) // 2 * 1024 bytes

// APP
#define APP_START (BOOTLOADER_START + BOOTLOADER_SIZE + CONFIG_SIZE)
#define APP_SIZE  (FLASH_SIZE - BOOTLOADER_SIZE - CONFIG_SIZE)

// APP HEADER
#define APP_HEADER_START (APP_START)
#define APP_HEADER_SIZE  (0x100u)    // 256 bytes

// APP IMAGE
#define APP_IMAGE_START (APP_START + APP_HEADER_SIZE)
#define APP_IMAGE_SIZE  (APP_SIZE  - APP_HEADER_SIZE)

/* Public enumerate/structure ----------------------------------------- */
typedef struct{
  char     version[128]; /*!< Image version */
  uint32_t len;          /*!< Image length */
  uint8_t  digest[32];   /*!< Append SHA-256 disgest */
  uint8_t  reserv[92];   /*!< 256 - 128 - 4 -32 */
} 
app_header_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */ 

#endif // __FLASH_DESC_H
/* End of file -------------------------------------------------------- */
