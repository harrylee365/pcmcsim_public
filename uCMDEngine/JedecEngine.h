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
 * Decription: uCMD engine that follows traditional JEDEC DDR4 protocol
 * which is modified from NVMain (https://github.com/SEAL-UCSB/NVmain.git)
 */

#ifndef __PCMCSIM_JEDEC_ENGINE_H_
#define __PCMCSIM_JEDEC_ENGINE_H_

#include "uCMDEngine/uCMDEngine.h"

namespace PCMCsim
{
    class AddressDecoder;
    class MemInfo;
    class StateMachine;

    class JedecEngine : public uCMDEngine
    {
      public:
        JedecEngine( ) = delete;
        JedecEngine(MemoryControlSystem* memsys_, AddressDecoder* adec_, 
                    MemInfo* info_, std::string cfg_header, 
                    ncycle_t ticks_per_cycle_, uint64_t id_=0);
        ~JedecEngine( );

        Component* master=NULL;

        bool isReady(Packet* pkt) override;
        void recvRequest(Packet* pkt, ncycle_t delay=1) override;
        void recvResponse(Packet* pkt, ncycle_t delay=1) override;

        void handle_events(ncycle_t curr_tick) override;
        
        /* Tools */
        AddressDecoder* adec;
        MemInfo* info;

        double getParamFLOAT(std::string var, double def=0.0);
        uint64_t getParamUINT64(std::string var, uint64_t def=0);
        bool getParamBOOL(std::string var, bool def=false);
        std::string getParamSTR(std::string var, std::string def="");

      protected:
        /* Event processing functions */
        void handle_await_reqs( ) override;
        void handle_await_resps( ) override;

        /* Inner type definitions */
        typedef std::vector<std::vector<bool>> vec2b_t;
        typedef std::vector<std::vector<std::vector<bool>>> vec3b_t;
        typedef std::vector<std::vector<uint64_t>> vec2u64_t;
        typedef std::vector<std::vector<std::vector<uint64_t>>> vec3u64_t;

        enum _ucmdq_arbitrate_schemes
        {
            RANK_FIRST_RR = 0,  // qs are in rank ascending order
            BANK_FIRST_RR,      // qs are in bank acending order
            NUM_SCHEMES
        };

        enum _page_policies
        {
            CLOSED_PAGE = 0,
            OPEN_PAGE,
            NUM_POLICIES
        };

        enum _powerdown_modes
        {
            FAST_EXIT = 0,
            SLOW_EXIT,
            NUM_PDS
        };

        /* Modes */        
        int arbit_scheme;
        int page_policy;
        int pd_mode;
        bool use_pd;
        bool use_refresh;
        bool use_sref;
        bool standalone_mode; // a separate DPU is connected or write request ID is considered

        /* Input-side of the engine: reqlist->ucmdq */
        uint64_t size_reqlist;
        std::list<Packet*> reqlist;         // input request list
        ncycle_t wake_reqlist;
        ncycle_t last_wake_reqlist;

        virtual void cycle_reqlist( );      // just for event scheduling

        /* Scheduler */
        vec2b_t open_banks;
        vec3b_t open_parts; 
        vec3u64_t open_rows; 
        vec3u64_t cntr_starv;
        uint64_t th_starv;

        bool is_req_schedulable(uint64_t ucmdq_idx);
        bool is_just_arrived(Packet* pkt);
        bool is_ucmdq_empty(uint64_t qidx);
        bool is_ucmdq_empty(uint64_t rank, uint64_t bank);
        bool is_rank_ucmdqs_empty(uint64_t rank);
        bool is_refresh_needed(uint64_t rank, uint64_t bank);
        bool is_refresh_ucmdq_empty(uint64_t rank, uint64_t bank);
        bool is_row_hit(uint64_t rank, uint64_t bank, 
                        uint64_t partition, uint64_t row); 
        void insert_ucmdq(Packet* pkt);  // inserted generated ucmds to ucmdq
        void schedule_postupdate(Packet* pkt);
        void postupdate_states_cb(Packet* pkt);
        
        std::pair<bool, ncycle_t> timely_issuable( );

        /* High-level gadgets for child-scheduler classes */
        bool find_starved(Packet*& found_req);
        bool find_row_hit(Packet*& found_req);
        bool find_open_bank(Packet*& found_req);
        bool find_closed_bank(Packet*& found_req);

        /* Output-side of the engine: ucmdq->media */
        StateMachine* sm;
        std::vector<std::queue<Packet*>> ucmdq; // per-bank queue

        ncycle_t wake_ucmdq;
        ncycle_t last_wake_ucmdq;
        uint64_t ucmdq_ptr;

        void cycle_ucmdq( );
        void update_ucmdq_ptr(uint64_t curr_idx);

        /* Refresh-related part */
        vec2b_t refresh_needed;         // need refresh but ucmd is not generated
        vec2b_t refresh_ucmdq_standby;  // refresh standby in ucmdq flag 
        vec2u64_t refresh_postponed;    // postponed refreshes
        uint64_t slices_per_rank;       // refresh slices within tREFI (1-rank)
        uint64_t refresh_rank_ptr;
        uint64_t refresh_bank_ptr;
        uint64_t th_postpone;
        ncycle_t handle_refresh_tick;
        std::vector<void*> refresh_pulses;

        void refresh_cb(void* pulse);
        void mark_refresh(void* pulse);
        void gen_refresh_ucmds( );
        void prepare_refresh( );
        void rst_refresh_needed(uint64_t rank, uint64_t bank);
        void set_refresh_needed(uint64_t rank, uint64_t bank);
        void rst_refresh_ucmdq_standby(uint64_t rank, uint64_t bank);
        void set_refresh_ucmdq_standby(uint64_t rank, uint64_t bank);
        void incr_refresh_postponed(uint64_t rank, uint64_t bank);
        void decr_refresh_postponed(uint64_t rank, uint64_t bank);
        bool reach_postpone_threshold(uint64_t rank, uint64_t bank);

        /* Low-power operations */
        std::vector<uint64_t> pd_ranks;
        std::vector<uint64_t> sref_ranks;
        ncycle_t wake_lp;
        ncycle_t last_wake_lp;

        void lp_handle( );
        ncycle_t lp_handle_pd( );
        ncycle_t lp_handle_sref( );

        /* Functions for standalone mode */
        std::queue<Packet*> respq;
        ncycle_t last_wake_respq;
        ncycle_t wake_respq;
        ncycle_t free_respq;

        void cycle_respq( );

        /* Commonly used tool functions */
        void close_bank(uint64_t rank, uint64_t bank);
        void close_bank(uint64_t rank, uint64_t bank, uint64_t part);
        uint64_t get_ucmdq_idx(uint64_t rank, uint64_t bank);
        uint64_t get_bank_idx(Packet* pkt);
        Packet* gen_ucmd(Packet* ref_pkt, cmd_t type);
        Packet* gen_ucmd(uint64_t rank, uint64_t bank, cmd_t type);

        ncycle_t tBURST_RESP;

        /* Frequently used memory information */
        uint64_t num_ranks;
        uint64_t num_bgs;
        uint64_t num_banks;
        uint64_t num_all_banks;
        uint64_t num_ucmdq;

        uint64_t deadlock_timer;

        /* Stats */
        ncycle_t last_updated_tick;
        uint64_t row_hits=0;
        uint64_t row_miss=0;
        uint64_t req_reads=0;
        uint64_t req_writes=0;

        ncycle_t max_issue_lat;
        ncycle_t min_issue_lat;
        double avg_issue_lat;
        uint64_t num_issue;

        void register_stats( ) override;
        void calculate_stats( ) override;
        void print_stats(std::ostream& os) override;
    };
};

#endif
