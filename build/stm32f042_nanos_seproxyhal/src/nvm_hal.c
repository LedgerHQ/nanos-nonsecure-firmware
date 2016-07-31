#include "stm32f0xx.h"

#include "nvm.h"

#ifdef NVM_PAGE_SIZE_B

extern const unsigned char * nvm_write_page_address;

void nvm_write_flush(void) {
  unsigned int page = nvm_write_page_address;

  // small basic check
  if (!nvm_write_page_address) {
    return;
  }

  // BEGIN HAL

  //__asm volatile ("cpsid i");
  HAL_FLASH_Unlock();

  FLASH_PageErase(page);
  FLASH_WaitForLastOperation(HAL_MAX_DELAY);
  CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

  const unsigned short * page_ptr = page;
  const unsigned short * buffer_ptr = nvm_page_D;
  while ((unsigned int)buffer_ptr < ((unsigned int)nvm_page_D)+sizeof(nvm_page_D)) {
    FLASH_Program_HalfWord(page_ptr, *buffer_ptr);
    FLASH_WaitForLastOperation(HAL_MAX_DELAY);    
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
    buffer_ptr++;
    page_ptr++;
  }
  HAL_FLASH_Lock();

  //__asm volatile ("cpsie i");

  /* CRC check is meant to detect this 
  for (page=0;page<sizeof(nvm_page_D);page++) {
    if (nvm_write_page_address[page] != nvm_page_D[page]) {
      for(;;);
    }
  }
  */

  // END HAL

  nvm_write_page_address = NULL;
}

#endif // NVM_PAGE_SIZE_B

