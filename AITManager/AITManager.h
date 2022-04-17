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
 * Authors: Hyungsuk Kim, Hyokeun Lee ({kimhs, hklee}@capp.snu.ac.kr)
 *
 * Description: Address mapper that translate logical address (LA)
 * to physical address using address indirection table (AIT),
 * where each entry of AIT is formatted as [RSVD|PAGE_KEY|PBA]
 * and indexed by logical block address (LBA).
 * For wear leveling req from MCU firmware, followings are determined:
   1. if BLKMV, LADDR(LBA) & PADDR(old AIT-entry) & PADDR_MAP(new AIT-entry)
   2. if BLKSWP, LADDR(LBA) & LADDR_MAP(LBA of PADDR_MAP) & PADDR & PADDR_MAP 
   (LADDR->PADDR_MAP, LADDR_MAP->PADDR)
 *  
 */

#ifndef __PCMCSIM_AIT_MANAGER_H_
#define __PCMCSIM_AIT_MANAGER_H_

#include "base/PCMCTypes.h"
#include "base/Component.h"

namespace PCMCsim
{
    class MemoryControlSystem;
    class Packet;
    class DataCache;
    class ReadModifyWrite;
    class Parser;
    class GPCache;
    class AITPrefetcher;

    class AITManager : public Component
    {
      public:
        AITManager(MemoryControlSystem* memsys_, std::string cfg_header);
        ~AITManager( );

        /* Communication functions */
        bool isReady(Packet* pkt) override;
        void recvRequest(Packet* pkt, ncycle_t delay=0) override;
        void recvResponse(Packet* pkt, ncycle_t delay=0) override;
        void handle_events(ncycle_t curr_tick) override;
        
        void calculate_stats( ) override;
        void print_stats(std::ostream& os) override;
        
        /* Externally connected components */
        Component* mcu;
        Component* rmw;
        Component* tcache;  // Cache of DRAM-AIT
        Component* tdram;   // DRAM-AIT
        AITPrefetcher* prefetcher;

        void init_ait_entry(uint64_t IDX_LINE, DataBlock& data);
        void get_gpc_info(uint64_t& line_offset, uint64_t& byte_offset);

      private:
        /* Internal types */
        enum _ait_qid
        {
            AIT_RDQ = 0,
            AIT_WRQ,

            NUM_AITQ
        };

        enum _st_ait 
        {
            ST_NORMAL = 0,
            ST_FORBID_RD,

            NUM_STATES
        };

        typedef struct _qentry_t
        {
            bool ready;
            Packet* pkt;
        } qentry_t;

        typedef struct _bentry_t
        {
            bool valid;
            bool inflight_ait_rd;
            Packet* pkt;
        } bentry_t;

        typedef struct _pbe_t
        {
            bool valid;
            bool useful;
            uint64_t addr;
            DataBlock data;
        } pbe_t;

        uint64_t gpc_cline_bits;    // $line offset
        uint64_t gpc_byte_offset;   // data block offset in a $line address
        uint64_t num_blocks;
        uint64_t width_block;

        /* Module input buffers */
        uint64_t buffer_bits;
        uint64_t size_buffer;
        uint64_t size_mcuq;
        uint64_t credit_buffer;

        ncycle_t wake_mcuq;
        ncycle_t last_wake_mcuq;

        std::vector<bentry_t> cmd_buffer; // buffer of managing normal cmds & assigned ID
        std::queue<Packet*> mcuq;         // queue of MCU reqs, min size==2 for pipeline op
        
        void cycle_mcuq( );

        /* ID mapping for normal req */
        std::list<uint64_t> avail_ID;
        void ID_remap(Packet* pkt);
        
        /* Block manager that generates address remap/swap req */
        Packet* proc_wlv; // currently working cmd in BLKGNR
        uint64_t num_avail_remap_slots;

        void gen_blk_remap( );
        void check_inflight_ait_rd( );
        void check_mcu_resp( );

        /* AIT$ input MUX between: block manager & ID mapper */
        uint64_t size_ait_rdq;
        uint64_t size_ait_wrq;
        int status_ait;
        
        ncycle_t wake_mux;
        ncycle_t last_wake_mux;
        ncycle_t free_mux;
        
        std::queue<qentry_t> ait_rdq; // addr trans cmds of HOST
        std::queue<qentry_t> ait_wrq; // addr update cmds from BLKMGR, fixed as 3
        
        bool isIssuable(int qid);
        void ait_mux( );
        void ait_issue(int qid);

        /* Response path of AIT cache */
        void handle_await_resps( ) override;
        void ID_unmap(Packet* pkt);
        Packet* address_translate(Packet* pkt);

        /* RMW issue side */
        uint64_t size_remapq; // # of pages in a block * 2
        
        ncycle_t wake_hostq;
        ncycle_t last_wake_hostq;
        ncycle_t wake_remapq;
        ncycle_t last_wake_remapq;
        
        std::queue<Packet*> hostq;  // cmds from host
        std::queue<Packet*> remapq; // cmds generated by BLKMGR

        void cycle_hostq( );
        void cycle_remapq( );

        /* Parameters for address translation */
        uint64_t BLOCK_OFFSET;
        uint64_t PAGE_OFFSET;
        uint64_t PAGE_PER_BLOCK_BIT;
        uint64_t HOST_TX_SIZE;
        uint64_t PAGE_SIZE;

        ncycle_t tCMD;

        /* Stats */
        uint64_t num_update;
        uint64_t num_map;

        ncycle_t max_wait_ait;
        ncycle_t min_wait_ait;
        double avg_wait_ait;

        ncycle_t max_issue_lat;
        ncycle_t min_issue_lat;
        double avg_issue_lat;
        uint64_t num_issue;

        void register_stats( ) override;
    };
};

#endif
