/**
  ******************************************************************************
  * @file    STM32F4xx_IAP/src/flash_if.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    10-October-2011
  * @brief   This file provides all the memory related operation functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/** @addtogroup STM32F4xx_IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_if2.h"
#include "flash_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static uint32_t GetSector(uint32_t Address);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{ 
#ifdef USE_HAL_DRIVER
  HAL_FLASH_Unlock();

  /* Clear pending flags (if any) */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
#else
  FLASH_Unlock(); 

  /* Clear pending flags (if any) */
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
#endif
}

/**
  * @brief  This function does an erase of all user flash area
  * @param  StartSector: start of user flash area
  * @retval 0: user flash area successfully erased
  *         1: error occurred
  */
uint32_t FLASH_If_Erase(uint32_t StartSector)
{
#ifdef USE_HAL_DRIVER
  __IO uint32_t UserStartSector = FLASH_SECTOR_1;
#else
  __IO uint32_t UserStartSector = FLASH_Sector_1;
#endif
  
  uint32_t i = 0;

  /* Get the sector where start the user flash area */
  UserStartSector = GetSector(APPLICATION_ADDRESS);

#ifdef USE_HAL_DRIVER
  // stm32f411xe �� FLASH_SECTOR_xx�� 0~7 ����
  //FLASH_EraseInitTypeDef eraseInitStruct;
  uint32_t err;

  HAL_FLASH_Unlock();
  
  for(i = UserStartSector; i <= LAST_FLASH_SECTOR_NN; i += 1)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */ 
    FLASH_Erase_Sector(i, FLASH_VOLTAGE_RANGE_3);
    FLASH_WaitForLastOperation(1000);

    err = HAL_FLASH_GetError();
    if (err != 0) {
      HAL_FLASH_Lock();

      return (err);
    }
  }
  
  HAL_FLASH_Lock();
#else
  for(i = UserStartSector; i <= FLASH_Sector_11; i += 8)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */ 
    if (FLASH_EraseSector(i, VoltageRange_3) != FLASH_COMPLETE)
    {
      /* Error occurred while page erase */
      return (1);
    }
  }
#endif
  
  return (0);
}

uint32_t FLASH_If_BACKUP_Erase(uint32_t StartSector){
#ifdef USE_HAL_DRIVER
  __IO uint32_t UserStartSector = FLASH_SECTOR_1;
#else
  __IO uint32_t UserStartSector = FLASH_Sector_1;
#endif

  uint32_t i = 0;

  /* Get the sector where start the user flash area */
  UserStartSector = GetSector(BACKUP_APPLICATION_ADDRESS);

#ifdef USE_HAL_DRIVER
  // stm32f411xe �� FLASH_SECTOR_xx�� 0~7 ����
  //FLASH_EraseInitTypeDef eraseInitStruct;
  uint32_t err;

  HAL_FLASH_Unlock();

  for(i = UserStartSector; i <= LAST_BACKUP_APPLICATION_ADDRESS_FLASH_SECTOR_NN; i += 1)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */
    FLASH_Erase_Sector(i, FLASH_VOLTAGE_RANGE_3);
    FLASH_WaitForLastOperation(1000);

    err = HAL_FLASH_GetError();
    if (err != 0) {
      HAL_FLASH_Lock();

      return (err);
    }
  }

  HAL_FLASH_Lock();
#else
  for(i = UserStartSector; i <= FLASH_Sector_11; i += 8)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */
    if (FLASH_EraseSector(i, VoltageRange_3) != FLASH_COMPLETE)
    {
      /* Error occurred while page erase */
      return (1);
    }
  }
#endif

  return (0);
}

uint32_t FLASH_If_MAIN_Erase(uint32_t StartSector){
#ifdef USE_HAL_DRIVER
  __IO uint32_t UserStartSector = FLASH_SECTOR_1;
#else
  __IO uint32_t UserStartSector = FLASH_Sector_1;
#endif

  uint32_t i = 0;

  /* Get the sector where start the user flash area */
  UserStartSector = GetSector(MAIN_APPLICATION_ADDRESS);

#ifdef USE_HAL_DRIVER
  // stm32f411xe �� FLASH_SECTOR_xx�� 0~7 ����
  //FLASH_EraseInitTypeDef eraseInitStruct;
  uint32_t err;

  HAL_FLASH_Unlock();

  for(i = UserStartSector; i <= LAST_MAIN_APPLICATION_ADDRESS_FLASH_SECTOR_NN; i += 1)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */
    FLASH_Erase_Sector(i, FLASH_VOLTAGE_RANGE_3);
    FLASH_WaitForLastOperation(1000);

    err = HAL_FLASH_GetError();
    if (err != 0) {
      HAL_FLASH_Lock();

      return (err);
    }
  }

  HAL_FLASH_Lock();
#else
  for(i = UserStartSector; i <= FLASH_Sector_11; i += 8)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */
    if (FLASH_EraseSector(i, VoltageRange_3) != FLASH_COMPLETE)
    {
      /* Error occurred while page erase */
      return (1);
    }
  }
#endif

  return (0);
}

/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  FlashAddress: start address for writing data buffer
  * @param  Data: pointer on data buffer
  * @param  DataLength: length of data buffer (unit is 32-bit word)   
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  */

uint32_t FLASH_If_Write(__IO uint32_t* FlashAddress, uint32_t* Data, uint32_t DataLength)
{
  uint32_t i = 0;

#ifdef USE_HAL_DRIVER
  HAL_FLASH_Unlock();
#endif
  
  for (i = 0; (i < DataLength) && (*FlashAddress <= (USER_FLASH_END_ADDRESS-4)); i++)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */ 
#ifdef USE_HAL_DRIVER
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, *FlashAddress, *(uint32_t*)(Data+i)) == HAL_OK)
#else
    if (FLASH_ProgramWord(*FlashAddress, *(uint32_t*)(Data+i)) == FLASH_COMPLETE)
#endif
    {
     /* Check the written value */
      if (*(uint32_t*)*FlashAddress != *(uint32_t*)(Data+i))
      {
        /* Flash content doesn't match SRAM content */
#ifdef USE_HAL_DRIVER
        HAL_FLASH_Lock();
#endif
        return(2);
      }
      /* Increment FLASH destination address */
      *FlashAddress += 4;
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
#ifdef USE_HAL_DRIVER
      HAL_FLASH_Lock();
#endif
      return (1);
    }
  }

#ifdef USE_HAL_DRIVER
  HAL_FLASH_Lock();
#endif
  
  return (0);
}

/**
  * @brief  Returns the write protection status of user flash area.
  * @param  None
  * @retval 0: No write protected sectors inside the user flash area
  *         1: Some sectors inside the user flash area are write protected
  */
uint16_t FLASH_If_GetWriteProtectionStatus(void)
{
#ifdef USE_HAL_DRIVER
  uint32_t UserStartSector = FLASH_SECTOR_1;
#else
  uint32_t UserStartSector = FLASH_Sector_1;
#endif

  /* Get the sector where start the user flash area */
  UserStartSector = GetSector(APPLICATION_ADDRESS);

  /* Check if there are write protected sectors inside the user flash area */
  if ((FLASH_OB_GetWRP() >> (UserStartSector/8)) == (0xFFF >> (UserStartSector/8)))
  { /* No write protected sectors inside the user flash area */
    return 1;
  }
  else
  { /* Some sectors inside the user flash area are write protected */
    return 0;
  }
}

/**
  * @brief  Disables the write protection of user flash area.
  * @param  None
  * @retval 1: Write Protection successfully disabled
  *         2: Error: Flash write unprotection failed
  */
uint32_t FLASH_If_DisableWriteProtection(void)
{
#ifdef USE_HAL_DRIVER
  uint32_t UserStartSector = FLASH_SECTOR_1, UserWrpSectors = OB_WRP_SECTOR_1;
#else
  uint32_t UserStartSector = FLASH_Sector_1, UserWrpSectors = OB_WRP_Sector_1;
#endif

  /* Get the sector where start the user flash area */
  UserStartSector = GetSector(APPLICATION_ADDRESS);

  /* Mark all sectors inside the user flash area as non protected */  
  UserWrpSectors = 0xFFF-((1 << (UserStartSector/8))-1);
   
  /* Unlock the Option Bytes */
#ifdef USE_HAL_DRIVER
  HAL_FLASH_OB_Unlock();
  /* Disable the write protection for all sectors inside the user flash area */
#else
  FLASH_OB_Unlock();
#endif

  /* Disable the write protection for all sectors inside the user flash area */
  FLASH_OB_WRPConfig(UserWrpSectors, DISABLE);

  /* Start the Option Bytes programming process. */  
#ifdef USE_HAL_DRIVER
  if (HAL_FLASH_OB_Launch() != HAL_OK)
#else
  if (FLASH_OB_Launch() != FLASH_COMPLETE)
#endif
  {
    /* Error: Flash write unprotection failed */
    return (2);
  }

  /* Write Protection successfully disabled */
  return (1);
}

/**
  * @brief  Gets the sector of a given address
  * @param  Address: Flash address
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
#ifdef USE_HAL_DRIVER
    sector = FLASH_SECTOR_0;
#else
    sector = FLASH_Sector_0;
#endif
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
#ifdef USE_HAL_DRIVER
    sector = FLASH_SECTOR_1;
#else
    sector = FLASH_Sector_1;
#endif
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
#ifdef USE_HAL_DRIVER
    sector = FLASH_SECTOR_2;
#else
    sector = FLASH_Sector_2;  
#endif
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
#ifdef USE_HAL_DRIVER
    sector = FLASH_SECTOR_3;  
#else
    sector = FLASH_Sector_3;  
#endif
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
#ifdef USE_HAL_DRIVER
    sector = FLASH_SECTOR_4;
#else
    sector = FLASH_Sector_4;
#endif
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
#ifdef USE_HAL_DRIVER
    sector = FLASH_SECTOR_5;
#else
    sector = FLASH_Sector_5;
#endif
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
#ifdef USE_HAL_DRIVER
    sector = FLASH_SECTOR_6;
#else
    sector = FLASH_Sector_6;
#endif
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
#ifdef USE_HAL_DRIVER
    sector = FLASH_SECTOR_7;
#else
    sector = FLASH_Sector_7;
#endif
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
#ifdef USE_HAL_DRIVER
#if defined(FLASH_SECTOR_8)
    sector = FLASH_SECTOR_8;
#else
    sector = LAST_FLASH_SECTOR_NN;
#endif
#else
    sector = FLASH_Sector_8;
#endif
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
#ifdef USE_HAL_DRIVER
#if defined(FLASH_SECTOR_9)
    sector = FLASH_SECTOR_9;
#else
    sector = LAST_FLASH_SECTOR_NN;
#endif
#else
    sector = FLASH_Sector_9;
#endif
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
#ifdef USE_HAL_DRIVER
#if defined(FLASH_SECTOR_10)
    sector = FLASH_SECTOR_10;
#else
    sector = LAST_FLASH_SECTOR_NN;
#endif
#else
    sector = FLASH_Sector_10;
#endif
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11))*/
  {
#ifdef USE_HAL_DRIVER
#if defined(FLASH_SECTOR_11)
    sector = FLASH_SECTOR_11;
#else
    sector = LAST_FLASH_SECTOR_NN;
#endif
#else
    sector = FLASH_Sector_11;
#endif
  }
    return sector;
}

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
