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
 * Description: This is a class of read modify write,
 * which is a hub module of all masters. 
 */

#ifndef __PCMCSIM_READMODIFYWRITE_H_
#define __PCMCSIM_READMODIFYWRITE_H_

#include "base/PCMCTypes.h"
#include "base/Component.h"

namespace PCMCsim
{
    class Component;
    class Packet;
    class PipeBufferv2;
    class MemoryControlSystem;
    class DataCache;
    class AITManager;
    class uCMDEngine;
    class ErrorManager;
    class DataPathUnit;

    class ReadModifyWrite : public Component
    {
      public:
        ReadModifyWrite(MemoryControlSystem* memsys_, std::string cfg_header);
        ~ReadModifyWrite( );

        /* Communication functions */
        bool isReady(Packet* pkt) override;
        void recvRequest(Packet* pkt, ncycle_t delay=1) override; 
        void recvResponse(Packet* pkt, ncycle_t delay=1) override; 
        void handle_events(ncycle_t curr_tick) override;

        void calculate_stats( ) override;

        /* Connected modules */
        DataCache* dcache;
        AITManager* aitm;
        Component* errMgr;
        Component* ucmde;
        Component* dpu;
        Component* mcu;

      private:
        enum _status {
            NO_STALL = 0,
            FULL_HOSTRDQ,       // register for host read queue is occupied now
            UNALLOCABLE,        // RMWQ & DBUF is unallocable 
            WAIT_DCACHE_WDATA,  // waiting for dcache write data
            HZD_UNMERGEABLE,    // cmd is hazard but unmergeable, pend anyway
            NUM_STATUS
        };

        enum _rdqs {
            RDQ_DPU = 0,
            RDQ_DCACHE,
            /* Expansion possible */
            NUM_RDQ
        };

        enum _out_cmd_sels {
            OUT_NONHZD_HOSTRD = 0,
            OUT_RMW_RD,
            OUT_RMW_WR,
            NUM_OUT_CMD
        };

        enum _st_inArbtr {
            ST_NORMAL = 0,
            ST_FORBID_HOST,
            ST_FORBID_BLKMGR,
            NUM_ST
        };

        /* RMWQ entry definition */
        typedef std::pair<int64_t, int64_t> WLVPair_t;
        class rmwqe_t
        {
          public:
            rmwqe_t( );
            ~rmwqe_t( ) { }
            
            /* All fields are assumed as registers */
            Packet* pkt;            // urgent, swap, LA, PA, PA_map, c/d/eFlags
            
            bool issuable;
            bool RD_issued;         // RMW read is issued
            bool RD_only;           // for BLKMV source request, =1

            WLVPair_t remap_link;   // <src ID & req ID>

            void link(std::list<rmwqe_t>::iterator entry);
        };
        typedef std::list<rmwqe_t>::iterator rmwqeIter_t;
        
        /* DBUF entry definition */
        class dbe_t
        {
          public:
            dbe_t( ) = delete;
            dbe_t(uint64_t num_words);
            ~dbe_t( );

            /* Flag fields: reg */
            bool alloc;
            uint64_t dcache_read;
            bool meta_issued;       // check linked one is issued
            std::vector<bool> dvalid;
            bool mvalid;
            
            Packet* pkt;            // original packet
            Packet* HostHzdPkt_RD;

            /* SRAM field (initially, 0-byte) */
            DataBlock data;
            DataBlock meta;
        };

        /* Issued info storage */
        typedef struct _wcmd_t
        {
            Packet* pkt;
            WLVPair_t link;
            bool wdata_issued;
        } wcmd_t;
        
        /* Top functions */
        bool need_wakeup;
        int64_t num_blkmgr_ctrl;
        std::vector<uint64_t> size_cmdq;
        std::vector<std::queue<Packet*>> cmdq;

        bool is_cmdq_avail( );        
        bool is_cmdq_empty( );
        void execute( );

        /* Input arbiter (IA) */
        uint64_t numReqPorts;
        uint64_t src_id_IA;
        int st_inArbtr;
        int issued_blks = 0;
        std::vector<uint64_t> servCntrs;
        std::vector<uint64_t> servWeights;
        
        void cmdq_proceed( );
        void inArbtr_exec(bool isIssued);

        /* Hazard checker (hzdCheck) */
        uint64_t size_nonHzdHostRD_cmdq;
        int st_hzdCheck;

        PipeBufferv2* pipe_hzdCheck;
        std::queue<Packet*> nonHzdHostRD_cmdq;

        Packet* host_wdata_wait_pkt = NULL; // packet that waiting for host wdata from dcache

        void hzdCheck_exec( );
        void non_hzd_handle(Packet* proc_pkt);
        void hzd_handle(int64_t hit_idx, Packet* proc_pkt);
        void unmergeable_handle(int64_t match_dbe, Packet* proc_pkt);
        void stall_handle(Packet* pkt);
        int req_wdata(Packet* pkt, int64_t dbe_idx);

        /* Request issue path */
        bool flushRMW_RD;
        bool flushRMW_WR;
        uint64_t maxLevelRMWQ_RD;
        uint64_t maxLevelRMWQ_WR;
        uint64_t maxFlushRMW_RD;
        uint64_t maxFlushRMW_WR;
        uint64_t maxStarvRMW_RD;
        uint64_t maxStarvRMW_WR;
        uint64_t flushCntRMW_RD;
        uint64_t flushCntRMW_WR;
        uint64_t starvCntRMW_RD;
        uint64_t starvCntRMW_WR;
        int64_t wbuffer_idx_dpu;
        
        uint64_t intr_th_WLV;
        uint64_t max_pwcnt;

        ncycle_t wake_outArbtr;
        ncycle_t last_wake_outArbtr;
        ncycle_t free_outArbtr;

        wcmd_t wcmd_issued;

        void issue_cmd( );
        ncycle_t issue_wdata(Packet* pkt, int64_t dbe_idx);
        bool resp_HostHzd_RD(Packet* pkt, int64_t dbe_idx); 
        void check_meta(Packet* pkt);
        int outArbtr_exec(uint64_t NumRMWQ_RD, uint64_t NumRMWQ_WR);

        /* Response path with DCACHE WR-MUX */
        ncycle_t wake_wdcache;
        ncycle_t last_wake_wdcache;
        ncycle_t free_wdcache;

        std::vector<uint64_t> size_rdq;
        std::vector<std::queue<Packet*>> rdq;
        
        void handle_await_resps( ) override;
        void handle_dpu_resps(Packet* pkt);
        void handle_dcache_resps(Packet* pkt);
        void dcache_wmux(dbe_t* hzd_dbe=NULL);
        void rdq_push(Packet* pkt, int rdq_id);
        void rdq_pop(int rdq_id);
        void check_n_set_merge(int rdq_id);

        /* RMW CMD queue controller */
        uint64_t size_rmwq;
        std::list<rmwqe_t> rmwq;

        rmwqeIter_t find_rmwq(Packet* pkt);
        rmwqeIter_t insert_rmwq(Packet* pkt);
        rmwqeIter_t find_n_link(rmwqeIter_t ref_rmwqe);

        std::pair<uint64_t, rmwqeIter_t> cnt_rmwq_RD( );
        std::pair<uint64_t, rmwqeIter_t> cnt_rmwq_WR( );

        /* Data buffer (DBUF) definition */
        uint64_t size_all_dbuf;             // all size
        std::vector<uint64_t> size_dbuf;    // size reg of each region
        std::vector<uint64_t> base_dbuf;    // base idx reg of each region
        std::vector<dbe_t*> dbuf;

        /* DBUF RD-MUX (rdq[DCACHE] & rdq[DPU] & wcmd_issued) */
        ncycle_t wake_rdbuf;
        ncycle_t last_wake_rdbuf;
        ncycle_t free_rdbuf;
        bool wait_redir = false; 
        
        void dbuf_rmux( );

        /* DBUF WR-MUX (rdq[DCACHE] & rdq[DPU]) */
        ncycle_t wake_wdbuf;
        ncycle_t last_wake_wdbuf;
        ncycle_t free_wdbuf;
        
        void dbuf_wmux( );
        void check_issuable(int64_t dbe_idx);
        Packet* isStorableExist( );
        Packet* isMergeRequired( );

        /* DBUF management */
        int64_t alloc_dbuf(Packet* pkt);
        int64_t get_allocable_dbuf(id_t src_id);
        int64_t find_hzd(Packet* pkt, int64_t& match_dbe);
        int64_t find_dbuf(id_t src_id, id_t req_id);
        int64_t find_dbuf(Packet* pkt);
        int64_t find_hostWR(Packet* pkt);
        uint64_t cnt_dbuf(id_t src_id);
        void init_dbuf_data(int64_t dbe_idx);
        void retire_dbuf( );
        void wack(Packet* pkt, cmd_t type);

        /* For address translation */
        uint64_t PAGE_OFFSET;
        uint64_t HOST_TX_SIZE;
        uint64_t PAGE_SIZE;
        uint64_t META_SIZE;

        /* Timing parameters */
        ncycle_t tDBUF;
        ncycle_t tBURST;
        ncycle_t tCMD;
        
        /* Stats */
        uint64_t num_HOST;
        uint64_t num_BLKMGR;

        uint64_t num_WAW;
        uint64_t num_mg_WAW;
        uint64_t num_RAW;
        uint64_t num_mg_RAW;

        uint64_t num_issue_RMW_RD;
        uint64_t num_issue_RMW_WR;
        uint64_t num_issue_nonHzdHostRD;
        uint64_t num_flush_RMW_RD; // <num_RMW_RD
        uint64_t num_flush_RMW_WR; // <num_RMW_WR
        uint64_t num_starv_RMW_RD; // <num_RMW_RD
        uint64_t num_starv_RMW_WR; // <num_RMW_WR

        uint64_t num_WLV;
        ncycle_t trigger_WLV;
        ncycle_t max_interval_WLV;
        ncycle_t min_interval_WLV;
        double avg_interval_WLV;

        uint64_t num_issue;
        ncycle_t max_issue_lat;
        ncycle_t min_issue_lat;
        double avg_issue_lat;

        ncycle_t max_issue_nonhzdrd_lat;
        ncycle_t min_issue_nonhzdrd_lat;
        double avg_issue_nonhzdrd_lat;

        ncycle_t max_issue_rmwrd_lat;
        ncycle_t min_issue_rmwrd_lat;
        double avg_issue_rmwrd_lat;

        ncycle_t max_issue_wr_lat;
        ncycle_t min_issue_wr_lat;
        double avg_issue_wr_lat;

        void register_stats( ) override;
    };
};
#endif

