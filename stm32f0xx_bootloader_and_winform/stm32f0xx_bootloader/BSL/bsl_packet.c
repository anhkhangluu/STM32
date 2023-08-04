/* Public includes ---------------------------------------------------------- */
#include "bsl_packet.h"
#include "flash.h"
#include "tinycrypt/sha256.h"
#include "stdio.h"

/* Private includes --------------------------------------------------------- */
/* Private defines ---------------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------------- */
/* Private macros ----------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
/* Private prototypes ------------------------------------------------------- */

/*!
 * @brief  This API configures the boot mode in flash memory.
 *
 * @param[in]  : Pointer of cfg structure.
 *
 * @return  
 * - BSL_OK: Writing cfg success.
 * - BSL_ERR: Writing cfg fail.
 */
static uint8_t m_write_cfg_flash(uint64_t *cfg);

/* Public implementations --------------------------------------------------- */
int bsl_builder_init(bsl_builder_hdl_t *h, uint8_t *buf, uint16_t len)
{
	BSL_CHECK(h != NULL && buf != NULL && len > 0, BSL_ERR);

	h->buf = buf;
	h->len = len;
	h->cnt = 0;

	return BSL_OK;
}

int bsl_builder_begin(bsl_builder_hdl_t *h)
{
	BSL_CHECK(h != NULL && h->buf != NULL, BSL_ERR);

	h->buf[BSL_RSP_IDX_HEADER]     = BSL_START_FLAG;
	h->buf[BSL_RSP_IDX_PL_SIZE_NL] = 0;
	h->buf[BSL_RSP_IDX_PL_SIZE_NH] = 0;
    h->cnt = 3;
	return BSL_OK;
}

int bsl_builder_write_ack(bsl_builder_hdl_t *h, uint8_t ack)
{
	BSL_CHECK(h != NULL && h->buf != NULL, BSL_ERR);

	uint16_t payload_size = 1;
    h->cnt++;
	// Write ACK
	h->buf[BSL_RSP_IDX_ACK] = ack;

	// Update new payload size
	h->buf[BSL_RSP_IDX_PL_SIZE_NL] = (uint8_t) (payload_size & 0xFF);
	h->buf[BSL_RSP_IDX_PL_SIZE_NH] = (uint8_t) (payload_size >> 8);

	return BSL_OK;
}

int bsl_builder_end(bsl_builder_hdl_t *h, bsl_crc16_func_t crc16_func)
{
	BSL_CHECK(h != NULL && crc16_func != NULL, BSL_ERR);

	// Calculate and write CRC
	uint16_t pkt_crc = crc16_func(h->buf, h->cnt);
	h->buf[BSL_RSP_IDX_CKL]  = (uint8_t) (pkt_crc & 0xFF);
	h->buf[BSL_RSP_IDX_CKH]  = (uint8_t) (pkt_crc >> 8);

	return BSL_OK;
}

int bsl_parser_handle(uint8_t *data, uint16_t len, bsl_crc16_func_t crc16_func)
{
	BSL_CHECK((data != NULL && len > 0), BSL_RSP_ERR_UNKNOWN);

	uint16_t header   = 0;
	uint16_t pl_len   = 0;
	uint16_t crc_recv = 0;
	uint16_t crc_calc = 0;
	uint8_t  cmd      = 0;

	header   = data[BSL_CMD_IDX_HEADER];
	pl_len   = (data[BSL_CMD_IDX_PL_SIZE_NH] << 8) | data[BSL_CMD_IDX_PL_SIZE_NL];
	cmd      = data[BSL_CMD_IDX_CORE_CMD];
	crc_recv = (data[len - 1] << 8) | data[len - 2];
	crc_calc = crc16_func(data, len - 2);

	/*< Check header */
	BSL_CHECK(BSL_START_FLAG == header, BSL_RSP_ERR_HEADER_INCORRECT);

	/*< Check length */
	BSL_CHECK(BSL_MAX_PL_SIZE >= pl_len, BSL_RSP_ERR_PKT_SIZE_ERROR);

	printf("crc_calc: %2x - crc_recv: %2x\r\n", crc_calc, crc_recv);

	/*< Check CRC payload */
	BSL_CHECK(crc_calc == crc_recv, BSL_RSP_ERR_CHECKSUM_INCORRECT);

	/*< Execute command  */
	switch (cmd)
	{
	case BSL_CMD_EARSE:
	{
		/* NOTE: While writing data, only ever erase the flash once. */
		printf("BSL_CMD_EARSE\r\n");

		uint32_t page = flash_get_page(FLASH_APP_START_ADDRESS);

		/* Erase from address start to application */
		BSL_CHECK(FLASH_OK == flash_erase(page, FLASH_APP_PAGE_MAX), BSL_RSP_ERR_FLASH);

		printf("flash_erase done\r\n");

		break;
	}
	case BSL_CMD_WRITE_DATA:
	{
		printf("BSL_CMD_WRITE_DATA\r\n");
		uint32_t addr = 0;
		uint32_t length = 0;

		/* Get address */
		addr  = ((((uint32_t) data[BSL_CMD_IDX_ADDR_4]) << 24) & 0xff000000);
		addr |= ((((uint32_t) data[BSL_CMD_IDX_ADDR_3]) << 16) & 0x00ff0000);
		addr |= ((((uint32_t) data[BSL_CMD_IDX_ADDR_2]) <<  8) & 0x0000ff00);
		addr |= ((((uint32_t) data[BSL_CMD_IDX_ADDR_1])      ) & 0x000000ff);
		printf("address: %#lx \r\n", addr);

		/* pl_len - 5 <=> pl_len - 4 bytes address - 1 byte cmd */
		length = (pl_len - 5) / 8;
		if ((pl_len - 5) % 8 != 0)
		{
			++length;
		}

		BSL_CHECK(FLASH_OK == flash_write(addr, (uint64_t *)&data[BSL_CMD_IDX_DATA], length), BSL_RSP_ERR_FLASH);

		break;
	}
	case BSL_CMD_VERIFY_IMAGE:
	{
		printf("BSL_CMD_VERIFY_IMAGE\r\n");

		/* Verify image */
		BSL_CHECK(BSL_OK ==  bsl_verify_image(), BSL_RSP_ERR_VERIFY_IMAGE);

		break;
	}
	case BSL_CMD_LOAD_PC:
	{
		printf("BSL_CMD_LOAD_PC\r\n");

		/* Update config value */
		uint64_t cfg_boot = BSL_BOOT_NORMAL;
		BSL_CHECK(BSL_OK == m_write_cfg_flash(&cfg_boot), BSL_RSP_ERR_FLASH);

		/* Jump to application */
		flash_jump_to_app();

		break;
	}

	default:
	{
		printf("Unknown command\r\n");
		return BSL_RSP_ERR_UNKNOWN_CMD;
	}
	}

	return BSL_RSP_ACK;
}

int bsl_verify_image()
{
	uint32_t read_len      = 0;
	uint32_t remain_len    = 0;
	uint32_t offset        = APP_IMAGE_START;
	uint8_t  recv_buf[100] = { 0 };
	uint8_t  digest[32]    = { 0 };
	uint8_t  status        = BSL_OK;
	struct tc_sha256_state_struct sha256;
	static app_header_t app_header;

	// Get app header
	printf("start verify image\r\n");
	if (FLASH_OK != flash_read(APP_HEADER_START, (uint8_t *)&app_header, sizeof(app_header)))
	{
		printf("[err] flash_read\r\n");
		status = BSL_ERR;
		goto __LBL_EXIT__;
	}

	// Init SHA-256
	tc_sha256_init(&sha256);

	if(app_header.len == 0xffffffff)
	{
		printf("[err] len fail\r\n");
		status = BSL_ERR;
		goto __LBL_EXIT__;
	}

	remain_len = app_header.len;

	do
	{
		// Clear receive buffer
		memset(recv_buf, 0x00, sizeof(recv_buf));
		read_len = (remain_len > sizeof(recv_buf)) ? sizeof(recv_buf) : remain_len;

		// Read data
		if (FLASH_OK != flash_read(offset, recv_buf, read_len))
		{
			printf("[err] flash_read\r\n");
			status = BSL_ERR;
			goto __LBL_EXIT__;
		}

		// Calculate sha256
		if (1 != tc_sha256_update(&sha256, (const uint8_t *)recv_buf, read_len))
		{
			printf("[err] tc_sha256_update\r\n");
			status = BSL_ERR;
			goto __LBL_EXIT__;
		}

		// Update address offset
		offset += read_len;

		// Update remain len
		remain_len -= read_len;

	} while (remain_len != 0);

	printf("verify image done\r\n");

	// Get digest
	if (1 != tc_sha256_final(digest, &sha256))
	{
		printf("[err] tc_sha256_final\r\n");
		status = BSL_ERR;
		goto __LBL_EXIT__;
	}

	printf("bin file's len: %ld \r\n", app_header.len);
	printf("version: %s \r\n", app_header.version);

	// Compare digest
	if (memcmp(digest, app_header.digest, sizeof(digest)) != 0)
	{
		status = BSL_ERR;
	}

	__LBL_EXIT__:
	return status;
}
/* Private implementations -------------------------------------------------- */
static uint8_t m_write_cfg_flash(uint64_t *cfg)
{
	uint8_t status = BSL_OK;
	uint32_t page = flash_get_page(CONFIG_START);

	if (FLASH_OK != flash_erase(page, 1))
	{
		status = BSL_ERR;
		printf("flash_erase configure fail!\r\n");
	}

	if (status == BSL_OK)
	{
		if (FLASH_OK != flash_write(CONFIG_START, cfg, 1))
		{
			status = BSL_ERR;
			printf("flash_write configure fail\r\n");
		}
	}

	return status;
}
/* End of file -------------------------------------------------------------- */
