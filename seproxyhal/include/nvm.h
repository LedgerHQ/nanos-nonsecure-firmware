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

#ifndef NVM_H
#define NVM_H

#ifdef NVM_PAGE_SIZE_B
extern unsigned char nvm_page_D [NVM_PAGE_SIZE_B];
void nvm_write_init(void);
void nvm_write(void * dst_adr, void* src_adr, unsigned int src_len);
void nvm_write_flush(void);
#endif // NVM_PAGE_SIZE_B

#endif
