/*
 * Copyright (c) 2019 Computer Architecture and Paralllel Processing Lab, 
 * Seoul National University, Republic of Korea. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistribution of source code must retain the above copyright 
 *        notice, this list of conditions and the follwoing disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright 
 *        notice, this list conditions and the following disclaimer in the 
 *        documentation and/or other materials provided with the distirubtion.
 *     3. Neither the name of the copyright holders nor the name of its 
 *        contributors may be used to endorse or promote products derived from 
 *        this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Hyokeun Lee (hklee@capp.snu.ac.kr)
 *
 */

#ifndef __PCMCSIM_MEM_INFO_H_
#define __PCMCSIM_MEM_INFO_H_

#include "base/PCMCTypes.h" 
#include "base/DataBlock.h"

namespace PCMCsim
{
    class MemoryControlSystem;
    
    class AddressDecoder
    {
      public:
        AddressDecoder(MemoryControlSystem* memsys, std::string cfg_header="address");
        AddressDecoder( ) = delete;
        ~AddressDecoder( ) { }

        uint64_t HOST_TX_OFFSET; 
        uint64_t PAGE_OFFSET; 
        uint64_t BLOCK_OFFSET;

        uint64_t get_order(addr_field_t field) { return orders[field]; }
        uint64_t get_width(addr_field_t field) { return widths[field]; }
        uint64_t decode_addr(uint64_t addr, addr_field_t field);
        uint64_t encode_addr(std::vector<uint64_t>& bits);
        uint64_t mask_host_offset(uint64_t addr);
        uint64_t mask_page_offset(uint64_t addr);
        uint64_t mask_block_offset(uint64_t addr);

        void print( );

        addr_field_t get_field(uint64_t order);
        void set_width(addr_field_t field, uint64_t width) { widths[field] = width; }

      private:
        std::string mem_name;
        uint64_t widths[NUM_FIELDS];
        uint64_t orders[NUM_FIELDS];
    };

    class MemInfo
    {
      public:
        MemInfo(MemoryControlSystem* memsys, std::string cfg_header, AddressDecoder* decoder);
        MemInfo( ) = delete;
        ~MemInfo( ) { }

        uint64_t HOST_TX_SIZE;
        uint64_t PAGE_SIZE;
        uint64_t BLOCK_SIZE;
        uint64_t META_SIZE; 

        uint64_t get_capacity_bits( ) { return cap_bits; }
        uint64_t get_channels( ) { return num_ch; }
        uint64_t get_ranks( ) { return num_ranks; }
        uint64_t get_bankgroups( ) { return num_bgs; }
        uint64_t get_banks( ) { return num_banks; }
        uint64_t get_partitions( ) { return num_parts; }
        uint64_t get_rowsPerMAT( ) { return WLs_per_MAT; }
        uint64_t get_cols( ) { return num_cols; }
        uint64_t get_devs( ) { return num_devs; }
        uint64_t get_DQs( ) { return num_DQs; }
        uint64_t get_prefetch_length( ) { return prefetch_len; }
        ncycle_t get_ctrl_freq( ) { return ctrl_freq; }
        bool is_half_bank( ) { return lr_bank; }
        void set_ctrl_freq(uint64_t ticks_per_cycle) 
        { 
            ctrl_freq = (uint64_t)1e12/ticks_per_cycle; 
        } 
        
        void print( );

      private:
        std::string mem_name;
        ncycle_t ctrl_freq;
        
        /* IO information */
        uint64_t prefetch_len;
        uint64_t data_window;   // access granularity of a memory core
        uint64_t num_DQs;
        uint64_t num_devs;
        uint64_t max_bw;

        /* Module information */
        bool lr_bank;
        uint64_t num_ch;                // logical channels 
        uint64_t num_ranks;
        uint64_t num_bgs;
        uint64_t num_banks;             // =number of banks/bank-group
        uint64_t num_parts;             // determine power/area of BLSA (not used now)
        uint64_t num_MATs;              // determine power/area of BLSA (not used now)
        uint64_t num_cols;
        uint64_t WLs_per_MAT;
        uint64_t BLs_per_MAT;
        uint64_t BLs_per_row;
        uint64_t cap_bits;
    };
};
#endif
