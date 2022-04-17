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
 * that can deal with data hazard. The module has 2-stage pipeline:
 * hazard checker, request issue MUX
 */

#ifndef __PCMCSIM_REQRECEIVER_H_
#define __PCMCSIM_REQRECEIVER_H_

#include "base/PCMCTypes.h"
#include "base/Component.h" 

namespace PCMCsim
{
    class MemoryControlSystem;
    class DataCache;
    class Parser;

    class RequestReceiver : public Component
    {
     public:
        RequestReceiver(MemoryControlSystem* memsys_, std::string cfg_header);
        ~RequestReceiver( );

        Component* parser;
        Component* dcache;

        void recvRequest(Packet* pkt, ncycle_t delay) override;
        void recvResponse(Packet* pkt, ncycle_t delay=1) override; 
        bool isReady(Packet* pkt) override;
        void handle_events(ncycle_t curr_tick) override;

        void calculate_stats( ) override;

     private:
        /* Internally used types */
        enum _qtype
        {
            RDQ=0,
            WRQ,

            NUM_QTYPE
        };

        enum _mux_prior
        {
            PRIORITY_READ=0,
            PRIORITY_WRITE,

            NUM_PRIORITY
        };

        typedef enum _hzd_t
        {
            WAR = 0,
            WAW,
            RAW
        } hzd_t;

        typedef struct _entry_t 
        {
            union 
            {
                uint64_t id;    // user: WAR linkage buffer
                uint64_t addr;  // uesr: w/rbuffer
            };
            bool valid;
            DataBlock wdata;    // user: wbuffer
        } entry_t;

        /* Interface logics */
        std::vector<uint64_t> credits;

        bool drain_mode;
        uint64_t credit_RD;
        uint64_t credit_WR;

        void insert_buffer(Packet* pkt);
        void resp_handle(Packet* pkt);
        void credit_calc( cmd_t cmd, bool in );

        /* ID remap management (Stage-1) */
        std::list<uint64_t> avail_RID;
        std::list<uint64_t> avail_WID;
        std::map<uint64_t, uint64_t> alloc_RID; // <ID, host ID>
        std::map<uint64_t, uint64_t> alloc_WID;

        void ID_remap( Packet* pkt );
        uint64_t ID_unmap( uint64_t ID, cmd_t type, Packet* pkt ); // return host ID
        uint64_t get_bid(Packet* pkt);
        uint64_t get_bid(uint64_t ID);

        /* Hazard checker (Stage-1) */
        void CAM_check( Packet* pkt );
        void hazard_handle( Packet* pkt, hzd_t hzd, uint64_t hzd_bid );
        void cycle_CAM_stall( );

        /* Request MUX (Stage-2) */
        uint64_t maxIssue_RD;
        uint64_t maxIssue_WR;
        uint64_t mux_st;
        ncycle_t wake_req_mux;
        ncycle_t last_wake_req_mux;

        bool isIssuable(cmd_t type=CMD_READ);
        void req_mux( );
        void req_issue(cmd_t type=CMD_READ);

        /* Read & write buffers storing addresses */
        uint64_t buffer_size_bits;
        uint64_t buffer_size;
        std::vector<entry_t> rbuffer;
        std::vector<entry_t> wbuffer;

        /* WAR management buffer for linking BID of read & write */
        std::vector<entry_t> rbuffer_WAR_wid;
        std::vector<entry_t> wbuffer_WAR_rid;

        void link_WAR(uint64_t rbid, uint64_t wbid);
        void unlink_WAR(uint64_t rbid, uint64_t LADDR);

        /* FIFOs that stack requests */
        std::vector<std::list<Packet*>> FIFO;

        /* READ response path */
        ncycle_t wake_resp_mux;         // when resp mux will work 
        ncycle_t last_wake_resp_mux;    // the last moment when resp mux worked
        ncycle_t free_resp;             // when response port becomes free
        
        std::queue<Packet*> respq;      // Read response from next module
        std::queue<Packet*> CAM_stalls; // 2-stage register when CAM is stalled by respq

        void resp_mux( );

        /* Latencies */
        ncycle_t tCMD;
        ncycle_t tDATA;
        ncycle_t tRESP;
        ncycle_t tCAM;

        /* Stats */
        uint64_t num_WAR;
        uint64_t num_WAW;
        uint64_t num_RAW;
        uint64_t num_xHZD;
        uint64_t num_reads;
        uint64_t num_writes;

        ncycle_t max_rd_lat;
        ncycle_t min_rd_lat;
        double avg_rd_lat;

        ncycle_t max_wr_lat;
        ncycle_t min_wr_lat;
        double avg_wr_lat; 

        ncycle_t max_issue_lat;
        ncycle_t min_issue_lat;
        double avg_issue_lat;
        uint64_t num_issue;

        ncycle_t max_rd_issue_lat;
        ncycle_t min_rd_issue_lat;
        double avg_rd_issue_lat;
        uint64_t num_rd_issue;

        ncycle_t max_wr_issue_lat;
        ncycle_t min_wr_issue_lat;
        double avg_wr_issue_lat;
        uint64_t num_wr_issue;
        
        void register_stats( ) override;
   };
};

#endif
