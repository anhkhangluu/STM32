/* Includes ----------------------------------------------------------- */
#include "flash_boot.h"
#include "flash_desc.h"
#include "stdint.h"
/* Private defines ---------------------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
/* Private variables ---------------------------------------------------------*/

/* Function pointer for jumping to user application. */
typedef void (*fnc_ptr)(void);

flash_status_t flash_erase(uint32_t page, uint32_t num_page)
{
  uint32_t pageAddress = FLASH_BASE + page*FLASH_PAGE_SIZE;
  flash_status_t status = FLASH_OK;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
#if 1
  /* Clear pending flags (if any) */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);


  FLASH_EraseInitTypeDef erase_init;
  uint32_t error = 0u;

  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = pageAddress;
  erase_init.NbPages = num_page;

  /* Do the actual erasing */
  if (HAL_OK != HAL_FLASHEx_Erase(&erase_init, &error))
  {
    status = FLASH_ERROR;
  }
#else
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = CONFIG_START;
	EraseInitStruct.NbPages = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
		/*Error occurred while page erase.*/
		return;
	}
#endif
  HAL_FLASH_Lock();

  return status;
}

flash_status_t flash_write(uint32_t address, uint64_t *buf, uint32_t len)
{
  flash_status_t status = FLASH_OK;

  HAL_FLASH_Unlock();

  /* Loop through the array. */
  for (uint32_t i = 0u; (i < len) && (FLASH_OK == status); i++)
  {
    /* If we reached the end of the memory, then report an error and don't do anything else.*/
    if (FLASH_APP_END_ADDRESS <= address)
    {
      status |= FLASH_ERROR_SIZE;
    }
    else
    {
      /* The actual flashing. If there is an error, then report it. */
      if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, buf[i]))
      {
        status |= FLASH_ERROR_WRITE;
      }

      /* Read back the content of the memory. If it is wrong, then report an error. */
      if ((buf[i]) != (*(volatile uint64_t*)address))
      {
        status |= FLASH_ERROR_READBACK;
      }

      /* Shift the address by a double word. */
      address += 8;
    }
  }

  HAL_FLASH_Lock();

  return status;
}

flash_status_t flash_read(uint32_t address, uint8_t *data, uint32_t len)
{
  flash_status_t status = FLASH_OK;

  for (uint32_t i = 0u; (i < len) && (FLASH_OK == status); i++)
  {
     /* If we reached the end of the memory, then report an error and don't do anything else.*/
    if (FLASH_APP_END_ADDRESS <= address)
    {
      status |= FLASH_ERROR_SIZE;
    }
    else
    {
      /* Read back the content of the memory. If it is wrong, then report an error. */
      data[i] = *((volatile uint8_t*)address);

      /* Shift the address by a word. */
      address++;
    }
  }

  return FLASH_OK;
}

uint32_t flash_get_page(uint32_t addr)
{
  return (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

void flash_jump_to_app(void)
{
  /* Function pointer to the address of the user application. */
  fnc_ptr jump_to_app;

  HAL_RCC_DeInit();
  HAL_DeInit();

  /* Turn off fault harder*/
  //Haven't found the code for this duty on stm32f0xx yet

  /* Change the main stack pointer. */
  __set_MSP(*(volatile uint32_t*)APP_IMAGE_START);
  
  jump_to_app = (fnc_ptr)(*(volatile uint32_t*) (APP_IMAGE_START + 4U));
  
  jump_to_app();
}

/* Private function definitions --------------------------------------- */
/* End of file -------------------------------------------------------- */
