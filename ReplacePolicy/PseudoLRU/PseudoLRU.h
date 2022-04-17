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
 * Description: This is pseudo LRU policy 
 * that requires O(N) area for each set in the cache
 * Data structure is managed with heap
 */

#ifndef __PCMCSIM_PSEUDO_LRU_H_
#define __PCMCSIM_PSEUDO_LRU_H_

#include "base/PCMCTypes.h"
#include "ReplacePolicy/ReplacePolicy.h"

namespace PCMCsim
{
    class PseudoLRU : public ReplacePolicy
    {
      public:
        PseudoLRU(uint64_t num_sets, uint64_t num_ways);
        PseudoLRU( ) = delete;
        ~PseudoLRU( );

        void init_policy( ) override;
        void update_victim(uint64_t set, int64_t way) override;
        int64_t get_victim(uint64_t set) override;

      private:
        uint64_t max_level;
        std::vector<std::vector<int64_t>> pLRU;
        std::vector<uint64_t> filled_ways;

        int64_t recurs_init(uint64_t set, int64_t idx);
        int64_t recurs_update(uint64_t set, int64_t dst_way, int64_t idx, int64_t lmw);
        int64_t recurs_trav(uint64_t set, int64_t idx);
    };
};

#endif 
