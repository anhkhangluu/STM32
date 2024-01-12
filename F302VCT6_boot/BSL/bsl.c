/* Includes ----------------------------------------------------------- */
#include "bsl.h"
#include "bsl_transport.h"
#include "bsl_packet.h"
#include "main.h"
#include "flash.h"
#include "flash_desc.h"
#include "stdio.h"

/* Private defines ---------------------------------------------------- */
#define BSL_RECV_TIMEOUT 1000
#define BSL_BUF_MAX 1024
#define BSL_BUIDER_LEN 6

/* Private macros ----------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static uint8_t m_builder_buf[BSL_BUIDER_LEN] = { 0 };
static bsl_builder_hdl_t m_builder;

static uint8_t m_bsl_buf[BSL_BUF_MAX] = { 0 };
static bsl_hdl_t m_bsl_hdl;

/* Private function prototypes ---------------------------------------- */
static void m_bsl_fsm(bsl_hdl_t *self);

/* Function definitions ----------------------------------------------- */
int bsl_init()
{
  bsl_transport_init();
  bsl_builder_init(&m_builder, m_builder_buf, sizeof(m_builder_buf));

  m_bsl_hdl.buf   = m_bsl_buf;
  m_bsl_hdl.state = RX_IDLE;
  m_bsl_hdl.ack   = BSL_RSP_ACK;
  
  printf("bsl init\r\n");

  return BSL_OK;
}

void bsl_loop()
{
  m_bsl_fsm(&m_bsl_hdl);
}

/* Private function definitions --------------------------------------- */
static void m_bsl_fsm(bsl_hdl_t *self)
{
  uint16_t  btr       = 0;
  bsl_hdl_t * const h = (bsl_hdl_t *)self;

  switch (h->state)
  {
    case RX_IDLE:
    {
      btr = bsl_get_unread_byte();

      if (btr == 0)
        return; // continue
      
      memset(h->buf, 0, BSL_BUF_MAX);
      h->buf_cnt           = 0;
      h->last_receive_tick = HAL_GetTick();
      h->state             = RX_RECEIVING;
      h->receiveing_state  = RX_RECEIVING_HEADER;
      h->remaining_byte    = BSL_HEADER_LEN;
      h->ack               = BSL_RSP_ACK;
      // Fallthrough
    }

    case RX_RECEIVING:
    {
      LBL_CONTINUE_RECEIVE:
      /// 
      /// @brief Detect inter-byte timeout
      /// @{
      btr = bsl_get_unread_byte();

      if (btr == 0)
      {
        if (HAL_GetTick() - h->last_receive_tick >= BSL_RECV_TIMEOUT)
          goto LBL_RX_IDLE;
          
        return; // continue
      }

      h->last_receive_tick = HAL_GetTick();
      /// @}

      /// 
      /// @brief Actual byte to read
      /// @{
      if (btr >= h->remaining_byte)
        btr = h->remaining_byte;

      // Read data from cbuf
      bsl_read(&h->buf[h->buf_cnt], btr);
      h->buf_cnt        += btr;
      h->remaining_byte -= btr;

      if (h->remaining_byte > 0)
        return; // continue
      /// @}

      switch (h->receiveing_state)
      {
        case RX_RECEIVING_HEADER:
        {
          h->receiveing_state  = RX_RECEIVING_PAYLOAD_LEN;
          h->remaining_byte    = BSL_PL_LEN;
          goto LBL_CONTINUE_RECEIVE;
        }

        case RX_RECEIVING_PAYLOAD_LEN:
        {
          h->receiveing_state = RX_RECEIVING_PAYLOAD;
          h->remaining_byte   = ((h->buf[BSL_CMD_IDX_PL_SIZE_NH] << 8) | h->buf[BSL_CMD_IDX_PL_SIZE_NL]);
          printf("RX_RECEIVING_PAYLOAD_LEN: %d - %d\r\n", btr, h->remaining_byte);
          goto LBL_CONTINUE_RECEIVE;
        }

        case RX_RECEIVING_PAYLOAD:
        {
          h->receiveing_state  = RX_RECEIVING_CHECKSUM;
          h->remaining_byte    = BSL_CRC_LEN;
          printf("RX_RECEIVING_PAYLOAD: %d\r\n", btr);
          goto LBL_CONTINUE_RECEIVE;
        }

        case RX_RECEIVING_CHECKSUM:
        {
          printf("RX_RECEIVING_CHECKSUM: %d\r\n", btr);
          h->state = RX_DISPATCH;
        }
      }
    }

    case RX_DISPATCH:
    {
      // Parse package cmd and handle
      h->ack = bsl_parser_handle(h->buf, h->buf_cnt, bsl_crc16_ccitt);

      // Build ACK packet 
      bsl_builder_begin(&m_builder);
      bsl_builder_write_ack(&m_builder, h->ack);
      bsl_builder_end(&m_builder, bsl_crc16_ccitt);

      // Send ACK packet to host
      bsl_write(m_builder_buf, BSL_BUIDER_LEN);
    }

    LBL_RX_IDLE:
      h->state = RX_IDLE;

    default:
      break;
  }
}
/* End of file -------------------------------------------------------- */
