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
 * Description: This is a class of defining packet
 */

#ifndef __PCMCSIM_PACKET_H_
#define __PCMCSIM_PACKET_H_

#include "base/PCMCTypes.h"
#include "base/DataBlock.h"

namespace PCMCsim
{
    class Component;
    class EntryDB;

    class Packet
    {
      public:
        Packet( );
        Packet(uint64_t num_words);
        ~Packet( );
        
        Packet& operator=(Packet& rhs);
        void reset_dvalid(uint64_t num_words);

        /* Command signals */
        cmd_t cmd;
        id_t req_id;
        id_t src_id;
        id_t dst_id;
        id_t tid;
        
        uint64_t LADDR;     // Logical address, error management request has INVALID_ADDR
        uint64_t PADDR;     // Physical address of LADDR
        uint64_t LADDR_MAP; // Logical address of newly mapped physical address. BLKMV/SWP has this
        uint64_t PADDR_MAP; // Physical address that is newly mapped to. BLKMV/SWP has this

        /* Data path signals */
        int64_t buffer_idx;
        std::vector<bool> dvalid;
        std::list<uint64_t> byte_enable;
        bool mvalid;
        DataBlock buffer_data;
        DataBlock buffer_meta;

        /* Meta status signals */
        bool poison;    //XXX: mod if needed (1-bit or 2-bit?)
        bool urgent;
        bool swap;

        /* Add your flag definition in PCMCTypes.h */
        Component* owner;   // packet generated module
        Component* from;    // last module that issue this packet
        Component* dest;    // final destination for this packet (used for xbar)

        ncycle_t recvTick;  // For calculating latency of each module generally
        ncycle_t recvTick_recvr;
        ncycle_t recvTick_dcache;
        ncycle_t recvTick_ucmde;

        /* Signals used in Prefetcher */
        bool is_prefetch;   // is prefetch request?
        bool pb_hit;        // prefetch buffer hit

        /* Signals used in ReadModifyWrite or XBar */
        int64_t dbe_idx;            // corresponding DBUF index
        bool isDATA;                // used by RMW and XBar
        bool need_redirect;         // redirect data dpu->dcache directly
        ncycle_t merge_end_tick;    // need to merge data 
        ncycle_t wdcache_latency;
    };
};

#endif
