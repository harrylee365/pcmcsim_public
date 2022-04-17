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
 * Description: 
 * This is a general-pupose cache that supports MSHR, read-allocate,
 * and write-llocate, and various eviction policies. 
 * Note RMW or partial-write is supported in low-level memory (LLM)
 * This cache is implemented to be adopted not only in AITManager,
 * but also various. The cache is structured as 3-stage pipeline:
 * tag-read (rtag), tag-match (mtag), hit/miss handle.
 *
 * This cache has following DPSRAMs (dual port SRAM):
 * 1. tmem: tag memory, which has MxN structure (M-set, N-way)
 *    that gets index bit as input and outputs the hit-way or miss
 *    > read port: used by tag-read stage
 *    > write port: used for updating tag during allocation
 * 
 * 2. dmem: data memory, which is N-way set associative cache having
 *    eight x64 devices if the cacheline size is 64B and CPU compute
 *    unit is 64-bit
 *    > read port: MUXed by read-hit (on hit) and eviction-read (on miss), 
 *      but these cases are exclusive; hence, no contention occurs
 *    > write port: MUXed by write-hit and cacheline fill signals. 
 *      Note the latter one may incur contention
 */

#ifndef __PCMCSIM_GPCACHE_H_
#define __PCMCSIM_GPCACHE_H_

#include "base/PCMCTypes.h"
#include "base/Component.h"

namespace PCMCsim
{
    class MemoryControlSystem;
    class PipeBufferv2;
    class Packet;
    class ReplacePolicy;

    class GPCache : public Component
    {
      public:
        GPCache(MemoryControlSystem* memsys_, std::string cfg_header="default.gpc");
        ~GPCache( );

        /* Communication functions */
        bool isReady(Packet* pkt) override;
        void recvRequest(Packet* pkt, ncycle_t delay=0) override;
        void recvResponse(Packet* pkt, ncycle_t delay=0) override;
        void handle_events(ncycle_t curr_tick) override;

        void calculate_stats( ) override;

      private:
        /* Internal types */
        enum _status
        {
            NO_STALL = 0,       // hit/miss handled without stall

            RACE_HITWR_FILL,    // write hit but competes with fillq processing
            HIT_RSVD,           // hit, but waiting for filling / MSHR merge overflow
            WAIT_RDRESP,        // waiting for RD-RESP port being free

            MSHR_ALLOC_NEW,     // temporal status. MSHR allocated, ready $ allocation
            MSHR_ALLOC_FAIL,    // MSHR allocation fails
            TMEM_ALLOC_FAIL,    // MSHR allocated, $-alloc. fails/missq is unavailable
            MSHR_SUBENTRY_FULL, // no more subentry
            WAIT_MISSQ,         // waiting for availability of missq

            NUM_STATUS
        };

        typedef struct _tmem_t
        {
            bool valid;
            bool dirty;
            bool rsvd;
            uint64_t tag;
        } tmem_t;

        typedef struct _mshr_t
        {
            bool cline_filled;      // cache line is filled w/ data
            int64_t cline_way;      // allocated cline way 
            uint64_t addr;          // address w/o line offset
            std::list<Packet*> subentry;  // merge subsequent reqs having same address
        } mshr_t;

        /* Top module */
        bool need_wake;
        
        std::queue<Packet*> cmdq; // min size==2 for cycle-by-cycle pipeline
        
        void execute( );
        void handle_await_resps( );
        
        /* Tag read stage */
        void cmdq_proceed( );

        /* Hit/miss handling stage */
        PipeBufferv2* pipe_access;
        ncycle_t wake_missq;
        ncycle_t last_wake_missq;
        ncycle_t free_req;
        std::list<Packet*> missq;
        
        void access_exec( );
        void handle_hitmiss(bool hit, int64_t hit_way, Packet* pkt);
        void hit_handle(Packet* pkt, int64_t hit_way);
        void hit_wdata(Packet* pkt, int64_t hit_way);
        void miss_handle(Packet* pkt);
        void stall_handle(Packet* pkt, int64_t hit_way);
        void cycle_missq( );
       
        /* Fill queue (response) path */
        ncycle_t wake_fillq;
        ncycle_t last_wake_fillq;
        ncycle_t free_resp;
        std::list<Packet*> fillq;

        void cycle_fillq( );
        void sendParentResp(Packet* pkt, ncycle_t delay=0);

        /* Cache definition */ 
        uint64_t cline_bits;    // # bit of each cache line
        uint64_t byte_offset;   // data block byte offset
        uint64_t bit_sets;      // # bit of index
        uint64_t num_sets;
        uint64_t num_ways;
        uint64_t num_blocks;
        uint64_t width_block;
        uint64_t cline_bytes;
        uint64_t size_cmdq;
        uint64_t size_missq; 
        uint64_t size_mshr;
        uint64_t size_subentry_mshr;
        int status;
        bool write_only;
        bool write_alloc;

        std::vector<std::vector<tmem_t>> tmem;   // [SET][WAY]
        std::vector<std::vector<DataBlock>> dmem;
        std::set<mshr_t*> mshr;

        ReplacePolicy* victim_policy;

        int mshr_check(Packet* pkt);

        int gpc_alloc(Packet* pkt);
        bool isRespPortAvailable( );    // check rd response port is avail.
        bool isWritePortAvailable( );   // check dmem write port is avail.
        void fill_line(uint64_t set, int64_t way, Packet* pkt);

        /* Peripheral functions */
        uint64_t get_set_idx(uint64_t addr);
        uint64_t get_tag(uint64_t addr);
        uint64_t get_block(uint64_t addr);
        void get_cache_position(uint64_t& sidx, int64_t& widx, Packet* pkt);
        
        void print_info( );

        /* Timing parameters */
        ncycle_t tRESP;
        ncycle_t tLLM_RD;
        ncycle_t tLLM_WR;

        /* Energy related params */
        double Esrpa; // search energy per acccess
        double Erdpa; // read energy per access
        double Ewrpa; // write energy per access

        /* Stats */
        uint64_t read_hit;
        uint64_t read_miss;
        uint64_t write_hit;
        uint64_t write_miss;
        uint64_t num_evct;
        uint64_t num_wait_rdresp;   // read-hit pended by read response
        uint64_t num_wait_rsvd;
        uint64_t mshr_hit;
        uint64_t mshr_miss;
        uint64_t mshr_subentry_full;

        double write_miss_rate;
        double read_miss_rate;
        double overall_miss_rate;

        ncycle_t trigger_wait_rdresp;
        ncycle_t max_wait_rdresp;
        ncycle_t min_wait_rdresp;
        double avg_wait_rdresp;

        double Esr;
        double Erd;
        double Ewr;
        double Etot;

        void register_stats( ) override;
    };
};
#endif
