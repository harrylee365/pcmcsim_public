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
 * Authors: Seokbo Shim (sbshim@capp.snu.ac.kr)
 *
 * Description: DPU Unit header for MemoryControlSystem
 *
 * Basically, Bypass Module
 * Supports Write with Latency delay handshaked through write buffer id
 * Supports Read Bypassing with Latency
 * Additive latency control with Error type would be supported for further experiment
 *
 * Write cmd from uCMDEngine & Write data from RMW
 * Read cmd from uCMDEngine
 */

#ifndef __PCMCSIM_DPU_H_
#define __PCMCSIM_DPU_H_

#include "base/PCMCTypes.h"
#include "base/Component.h" 

namespace PCMCsim
{
    class Component;
    class Packet;
    class MemoryControlSystem;

    class DataPathUnit : public Component
    {
      public:
        DataPathUnit(MemoryControlSystem* memsys_, std::string cfg_header, 
                ncycle_t ticks_per_cycle_=0, uint64_t id_=0);
        ~DataPathUnit( );

        /* Communication functions */
        bool isReady(Packet* pkt) override;
        void recvRequest(Packet* pkt, ncycle_t delay=1) override;
        void recvResponse(Packet* pkt, ncycle_t delay=1) override;

        /* Connected Modules */
        Component*  rmw;
        Component* ucmde;
        //Component* emu;
        Component* media;

     private:
        /* Latencies */
        ncycle_t tECC_WR;
        ncycle_t tECC_RD;
        ncycle_t tBURST;

        /* Buffer variables */
        std::map<uint64_t, Packet*> dpu_wbuf;
        std::set<uint64_t> dpu_wbuf_index;
        std::queue<Packet*> dpu_rbuf;

        uint64_t dpu_wbuf_size;
        uint64_t dpu_rbuf_size;
        uint64_t dpu_wbid_to_RMW; // Uptated Write buffer index after Write from RMW
        bool     wbid_stalled = false;
        void     push_wdata(Packet* pkt);
        void     confer_wbid2rmw( );
        int64_t  find_empty_wbid( );
        Packet*  media_wr_pkt;
        Packet*  dpu_resp_pkt;

        /* Event handling function */
        void handle_events(ncycle_t curr_tick) override;
        void handle_await_resps( ) override;

        ncycle_t wake_rbuf = 0;
        ncycle_t last_wake_rbuf = 0;
        void cycle_rbuf( );

        /* fool ECC model */
        bool ecc_enable;
        uint64_t ecc_cap;
        void ecc_check(Packet* pkt);

        /* Stats */
        uint64_t ber_errors;    // occurred errors
        uint64_t ber_correct;   // correted errors
        uint64_t ber_due;       // detected uncorrectable errors
        uint64_t ber_sdc;       // silent data corruption (not awared by ECC)
        uint64_t ber_reads;     // # of read bits
        
        void register_stats( ) override;
   };
};
#endif

