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
 * Description: Block level wear leveling (WLV) algorithm
 * Referenced from the studies proposed by Y.-M Chang et al.,
 * "Age-based PCM Wear Leveling with Nearly Zero Search Cost" and
 * "Improving PCM Endurance with a Constant-Cost Wear Leveling Design",
 * which are published in DAC 2012 and ACM TODAES 2016, respectively
 */

#ifndef __PCMCSIM_BUCKET_WLV_H_
#define __PCMCSIM_BUCKET_WLV_H_

#include "Subsystems/MCU/MicroControlUnit.h"

namespace PCMCsim
{
    class AddressDecoder;
    class MemInfo;
    class AITDecoder;

    class BlockAddress
    {
      public:
        BlockAddress( )
        {
            LBA = INVALID_ADDR;
            PBA = INVALID_ADDR;
        }

        BlockAddress(uint64_t LBA_, uint64_t PBA_)
        {
            LBA = LBA_;
            PBA = PBA_;
        }

        BlockAddress(uint64_t PBA_) 
        {
            LBA = INVALID_ADDR;
            PBA = PBA_; 
        }

        uint64_t LBA;
        uint64_t PBA;
    };

    class BucketWLV : public MicroControlUnit
    {
      public:
        BucketWLV( ) = delete;
        BucketWLV(MemoryControlSystem* memsys_, std::string cfg_header, 
                 ncycle_t ticks_per_cycle_=0);
        ~BucketWLV( );

        bool isReady(Packet* pkt) override;
        void recvRequest(Packet* pkt, ncycle_t delay=1) override;
        void recvResponse(Packet* pkt, ncycle_t delay=1) override;

        void handle_events(ncycle_t curr_tick) override;

      protected:
        enum _list_ops
        {
            IS_HEAD = 0,
            IS_TAIL,

            NUM_OPS
        };

        enum _wlv_states
        {
            ST_IDLE = 0,
            ST_BUSY,

            NUM_STATES
        };
        
        /* Request and response processing */
        void handle_await_reqs( ) override;
        void handle_await_resps( ) override;

        /* ISRs */
        void register_isr(Packet* pkt) override;
        void run_wlv(void* args);
        void handle_aitm_irq(void* args);
        
        /* AIT information used for request generation */
        // TODO: used for dummy reqs for getting wr cntr or ait entry
        AITDecoder* tdec; 

        /* Basic data structures for algorithm */
        int state;
        ncycle_t tWLV;
        uint64_t* wr_cntr; // TODO: actual req expected for more accurate simulation
        std::vector<std::list<BlockAddress>> alloc_list;
        std::vector<std::list<BlockAddress>> free_list;

        uint64_t BLOCK_OFFSET;
        uint64_t NUM_BLOCKS;
        uint64_t num_buckets;
        uint64_t max_bucket_cnt;
        uint64_t th_wlv;
        uint64_t base_bucket;
        uint64_t oldest_free_bucket;
        uint64_t gpc_cline_offset;
        uint64_t gpc_byte_offset;

        void issue_wlv_req(uint64_t LBA, uint64_t PBA, uint64_t LBA_MAP, uint64_t PBA_MAP);

        /* List operations */
        std::pair<BlockAddress, bool> get_free_block( );
        void push_block(std::vector<std::list<BlockAddress>>& dest_list, 
                        uint64_t dest_bidx, BlockAddress block, int op_pos);
        BlockAddress pop_block(std::vector<std::list<BlockAddress>>& src_list,
                       uint64_t src_bidx, int op_pos);
        uint64_t get_bucket_idx(uint64_t PBA);

        /* Stats */
        uint64_t num_swap;
        uint64_t num_move;
    };
};
#endif
