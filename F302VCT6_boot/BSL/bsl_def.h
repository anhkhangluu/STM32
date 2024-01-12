#ifndef __BSL_DEF_H
#define __BSL_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#ifdef CONFIG_BSL_DEBUG
#include <stdio.h>
#else
#undef xlog_i
#define xlog_i(...)
#endif

/* Global configurations ---------------------------------------------- */
#define CONFIG_BSL_USE_USB    1
#define CONFIG_BSL_USE_UART   2

// Setting default transport layer to USB
#ifndef CONFIG_BSL_TRANSPORT
#define CONFIG_BSL_TRANSPORT    CONFIG_BSL_USE_USB
#endif

/* Public macros ------------------------------------------------------ */
#ifdef BSL_CHECK
#undef BSL_CHECK
#endif

#define BSL_CHECK(expr, rc)  \
  do {                                                \
    if (!(expr)) {                                    \
      printf("[error] %s:%d\r\n", __FILE_NAME__, __LINE__);  \
      return rc;                                      \
    }                                                 \
  } while (0)

/* Public defines ----------------------------------------------------- */
#define BSL_START_FLAG               0x80 /*! From API doc */
#define BSL_MAX_PL_SIZE              1005

/*! BSL */
#define BSL_OK                       0x00
#define BSL_ERR                      0x01
#define BSL_ERR_CHECK_CRC            0x03

/*! BSL command */
#define BSL_CMD_EARSE               0x10
#define BSL_CMD_WRITE_DATA          0x11
#define BSL_CMD_VERIFY_IMAGE        0x12
#define BSL_CMD_LOAD_PC             0x13

/*! BSL ERROR CODE */
#define BSL_RSP_ACK                    0x50
#define BSL_RSP_ERR_HEADER_INCORRECT   0x51
#define BSL_RSP_ERR_CHECKSUM_INCORRECT 0x52
#define BSL_RSP_ERR_PKT_SIZE_ERROR     0x53
#define BSL_RSP_ERR_UNKNOWN_CMD        0x54
#define BSL_RSP_ERR_UNKNOWN            0x55
#define BSL_RSP_ERR_FLASH              0x56
#define BSL_RSP_ERR_VERIFY_IMAGE       0x57

/*! BSL BOOT MODE */
#define BSL_BOOT_FIRST_TIME 0xffffffffffffffff
#define BSL_BOOT_NORMAL     0xffffffffffffffa0
#define BSL_BOOT_DFU        0xffffffffffffffb0

/*! BSL LEN*/
#define BSL_HEADER_LEN 1
#define BSL_PL_LEN     2
#define BSL_CRC_LEN    2

/*!
 * @brief  BSL Command index value
 */
enum
{
   BSL_CMD_IDX_HEADER = 0
  ,BSL_CMD_IDX_PL_SIZE_NL /*! Low byte */
  ,BSL_CMD_IDX_PL_SIZE_NH /*! High byte */
  ,BSL_CMD_IDX_CORE_CMD
  ,BSL_CMD_IDX_ADDR_1
  ,BSL_CMD_IDX_ADDR_2
  ,BSL_CMD_IDX_ADDR_3
  ,BSL_CMD_IDX_ADDR_4
  ,BSL_CMD_IDX_DATA = 8 
};

/*!
 * @brief  BSL Response index value
 */
enum
{
   BSL_RSP_IDX_HEADER = 0
  ,BSL_RSP_IDX_PL_SIZE_NL /*! Low byte */
  ,BSL_RSP_IDX_PL_SIZE_NH /*! High byte */
  ,BSL_RSP_IDX_ACK
  ,BSL_RSP_IDX_CKL
  ,BSL_RSP_IDX_CKH
};

/* Public enumerate/structure ----------------------------------------- */
typedef enum 
{
   RX_IDLE
  ,RX_RECEIVING
  ,RX_DISPATCH
  ,RX_MAX
}
bsl_rx_state_t;

typedef enum {
   RX_RECEIVING_HEADER
  ,RX_RECEIVING_PAYLOAD_LEN
  ,RX_RECEIVING_PAYLOAD
  ,RX_RECEIVING_CHECKSUM
} 
bsl_receiving_state_t;

typedef uint16_t (*bsl_crc16_func_t)(const uint8_t *data, uint16_t len);

/*!
 * @brief 
 */
typedef struct
{
  uint8_t *buf;
  uint16_t buf_cnt;
  uint16_t remaining_byte;
  uint32_t last_receive_tick;
  bsl_rx_state_t state;
  bsl_receiving_state_t receiveing_state;
  uint8_t ack;
}
bsl_hdl_t;

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */ 

#endif // __BSL_DEF_H
