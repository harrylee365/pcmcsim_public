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
 * Authors: Seungyong Lee, Hyokeun Lee ({sylee, hklee}@capp.snu.ac.kr)
 *
 * Description: This is a request receiver from controller parser
 * that can deal with data hazard
 */

#ifndef __PCMCSIM_DATACACHE_H_
#define __PCMCSIM_DATACACHE_H_

#include "base/PCMCTypes.h"
#include "base/Component.h"

namespace PCMCsim
{
    class MemoryControlSystem;
    class PipeBufferv2;
    class ReplacePolicy;

    class DataCache : public Component
    {
      public:
        DataCache( ) = delete;
        DataCache(MemoryControlSystem* memsys_, std::string cfg_header);
        ~DataCache( );

        /* Connected modules */
        Component* recvr;
        Component* aitm;
        Component* rmw;

        /* Communication functions */
        bool isReady(Packet* pkt) override;
        void recvRequest(Packet* pkt, ncycle_t delay) override;
        void recvResponse(Packet* pkt, ncycle_t delay=1) override;
        void handle_events(ncycle_t curr_tick) override;

        void calculate_stats( ) override;

      private:
        /* Internal types */
        enum _status
        {
            NO_STALL = 0,       // hit/miss handled without stall

            HIT_RSVD,           // Re-confirm process shows cache-hit
            RACE_HITWR_FILL,    // write hit but competes with fillq processing

            WAIT_WDATA_RECV,    // wait recvn' wdata in from ReqRecvr
            WAIT_WDATA_SEND,    // wait wdata allow signal from RMW
            WAIT_AITMGR,        // wait AITManager becoming ready
            WAIT_RDRESP,        // wait for RD-RESP port being free

            ALLOC_FAIL,         // cache line alloca failed, no available way

            NUM_STATUS
        };

        enum _handler_sub_status
        {
            SUB_NORMAL = 0,
            SUB_RDMISS_ISSUE,
            SUB_RDMISS_EVICT,

            NUM_SUB_STATUS
        };

        typedef struct _entry_t
        {
            uint64_t tag;
            bool valid;
            bool dirty;
            bool rsvd;
            DataBlock data;
        } entry_t;

        /* Top functions */
        bool need_wakeup;
        uint64_t size_cmdq;
        std::queue<Packet*> cmdq;

        void execute( );
        void handle_await_resps( ) override;

        /* Tag read stage */
        void cmdq_proceed( );

        /* Hit & miss Handling functions */
        PipeBufferv2* pipe_access;
        Packet* evct_pkt = NULL;
        int sub_status = SUB_NORMAL;
        std::list<uint64_t> avail_WID;
        std::set<uint64_t> alloc_WID;
        uint64_t credit_WID;
        
        void access_exec( );
        void handle_hitmiss(bool hit, int64_t hit_way, Packet* pkt);
        void hit_handle(Packet* pkt, int64_t hit_way);
        void miss_handle(Packet* pkt);
        void stall_handle(Packet* pkt, int64_t hit_way);
        int issue_cmd(Packet* pkt);
        int dcache_alloc(Packet* pkt);
        
        /* Write request path */
        void gen_evct(uint64_t addr, DataBlock& evct_data);
        void WID_map(Packet* pkt); 
        void WID_unmap(Packet* pkt);
        void recv_wdata(Packet* pkt, bool wr_bypass=true, int64_t wr_hit_way=-1);
        int req_wdata(Packet* pkt);

        /* Read response path */
        ncycle_t wake_resp_mux;         // when resp MUX will work
        ncycle_t last_wake_resp_mux;    // that last moment when resp MUX worked
        ncycle_t free_resp;             // when resp port becomes free
        
        std::queue<Packet*> fillq;

        bool isRespAvailable( );
        bool isWrPtAvailable( );
        void rdhit_resp(Packet* pkt);
        void cycle_fillq( );
        void fill_line(uint64_t cidx, uint64_t set, int64_t way, Packet* pkt);

        /* Cache definition */
        uint64_t num_cache;
        uint64_t bit_sets;
        uint64_t num_sets;
        uint64_t num_ways;
        int status;
        bool write_only;
        bool write_alloc;

        std::vector<std::vector<std::vector<entry_t>>> cache; // [NUM][SET][WAY]
        std::vector<ReplacePolicy*> victim_policy;

        uint64_t get_cache_idx(uint64_t addr); // 0: even cache, 1: odd cache
        uint64_t get_set_idx(uint64_t addr);
        uint64_t get_tag(uint64_t addr);
        void get_cache_position(uint64_t& cidx, uint64_t& sidx, int64_t& widx, Packet* pkt);

        /* Timing signal */
        uint64_t tCMD;
        uint64_t tBURST_HOST;
        uint64_t tBURST_RMW;

        /* For address translation */
        uint64_t HOST_TX_OFFSET;
        uint64_t PAGE_OFFSET;
        uint64_t HOST_TX_SIZE;
        uint64_t PAGE_SIZE;

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
        uint64_t num_persist;
        uint64_t num_wait_rsvd;

        double write_miss_rate;
        double read_miss_rate;
        double overall_miss_rate;

        ncycle_t trigger_wait_rdresp;
        ncycle_t max_wait_rdresp;
        ncycle_t min_wait_rdresp;
        double avg_wait_rdresp;

        ncycle_t max_persist_lat;
        ncycle_t min_persist_lat;
        double avg_persist_lat;

        uint64_t num_rdhit_resp;
        ncycle_t max_rdhit_lat;
        ncycle_t min_rdhit_lat;
        double avg_rdhit_lat;
        
        uint64_t num_rdmiss_issue;
        ncycle_t max_rdmiss_lat;
        ncycle_t min_rdmiss_lat;
        double avg_rdmiss_lat;

        uint64_t num_wrget;
        ncycle_t max_wrget_lat;
        ncycle_t min_wrget_lat;
        double avg_wrget_lat;

        ncycle_t max_issue_lat;
        ncycle_t min_issue_lat;
        double avg_issue_lat;
        uint64_t num_issue;

        double* Esr;
        double* Erd;
        double* Ewr;
        double* Etot;

        void register_stats( ) override;
    };
};

#endif
