/*******************************************************************************
*   Ledger Blue - Non secure firmware
*   (c) 2016 Ledger
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
********************************************************************************/

#include "stdint.h"
#include "string.h"
#include "nvm.h"

#ifdef NVM_PAGE_SIZE_B
unsigned char nvm_page_D [NVM_PAGE_SIZE_B];

// todo later
const unsigned char * nvm_write_page_address;

void nvm_write_init(void) {
  nvm_write_page_address = NULL;  
}

static void nvm_write_page_load(const unsigned char* page, unsigned int load_content) {
  if (page != nvm_write_page_address) {
    nvm_write_flush();
    if(load_content) {
      memcpy(nvm_page_D, page, NVM_PAGE_SIZE_B);
    }
    // page in cache
    nvm_write_page_address = page;
  }  
}

void nvm_write(void * dst_adr, void* src_adr, unsigned int src_len) {
#define DST_ADR ((const unsigned char*) dst_adr)
#define SRC_ADR ((const unsigned char*) src_adr)
  const unsigned char* page;
  unsigned short len;

  if (src_len == 0) {
    return;
  }

  // head, align dst_adr on a page if not
  page = (const unsigned char*)(((unsigned int)DST_ADR) & ~(NVM_PAGE_SIZE_B-1));
  if(page != DST_ADR) {
    unsigned short page_off = (DST_ADR-page);
    nvm_write_page_load(page, 1);
    len = NVM_PAGE_SIZE_B-page_off;
    if (len > src_len) {
      len = src_len;
    }
    if (SRC_ADR) {
      memcpy(nvm_page_D+page_off, SRC_ADR, len);
      src_adr = SRC_ADR + len;
    }
    else {
      memset(nvm_page_D+page_off, 0, len);
    }
    src_len -= len;
    page += NVM_PAGE_SIZE_B;
  }

  while(src_len >= NVM_PAGE_SIZE_B) {
    nvm_write_page_load(page, 0);
    if (SRC_ADR) {
      memcpy(nvm_page_D, SRC_ADR, NVM_PAGE_SIZE_B);
      src_adr = SRC_ADR + NVM_PAGE_SIZE_B;
    }
    else {
      memset(nvm_page_D, 0, NVM_PAGE_SIZE_B);
    }
    src_len -= NVM_PAGE_SIZE_B;
    page += NVM_PAGE_SIZE_B;
  }

  if (src_len) {
    nvm_write_page_load(page, 1);
    if (SRC_ADR) {
      memcpy(nvm_page_D, SRC_ADR, src_len);
    }
    else {
      memset(nvm_page_D, 0, src_len);
    }
  }
}
#endif // NVM_PAGE_SIZE_B
