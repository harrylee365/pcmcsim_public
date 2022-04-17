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
 * Decription: Policy generator that insantiates a policy inherited from JedecEngine
 */

#ifndef __PCMCSIM_JEDEC_ENGINE_POLICY_FACTORY_H_
#define __PCMCSIM_JEDEC_ENGINE_POLICY_FACTORY_H_

#include "base/PCMCTypes.h"

namespace PCMCsim
{
    class uCMDEngine;
    class MemoryControlSystem;
    class AddressDecoder;
    class MemInfo;

    class JedecPolicyFactory
    {
      public:
        JedecPolicyFactory( ) { }
        ~JedecPolicyFactory( ) { }

        static uCMDEngine* create_engine_policy(
            MemoryControlSystem* memsys_, AddressDecoder* adec_, MemInfo* info_,
            std::string cfg_header, ncycle_t ticks_per_cycle_=0, uint64_t id_=0);

        static uCMDEngine* create_engine_policy(std::string policy, 
            MemoryControlSystem* memsys_, AddressDecoder* adec_, MemInfo* info_,
            std::string cfg_header, ncycle_t ticks_per_cycle_=0, uint64_t id_=0);
    };
};

#endif
