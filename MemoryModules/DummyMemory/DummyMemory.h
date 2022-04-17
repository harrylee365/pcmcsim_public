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
 * Description: This is a parent class to roughly simulate memories
 */

#ifndef __PCMCSIM_DUMMY_MEMORY_H_
#define __PCMCSIM_DUMMY_MEMORY_H_

#include "base/PCMCTypes.h"
#include "base/MemoryControlSystem.h"
#include "base/Component.h"

namespace PCMCsim
{
    class MemoryControlSystem;
    class Packet;

    class DummyMemory : public Component
    {
      public:
        DummyMemory( ) = delete;
        DummyMemory(MemoryControlSystem* memsys_): Component(memsys_) { ready = true; }
        ~DummyMemory( );

        void handle_events(ncycle_t curr_tick) override = 0;

        bool is_data_enable( ) { return data_enable; }

        bool get_stored(Packet* pkt);
        DataBlock get_stored(uint64_t PADDR);
        void set_stored(uint64_t PADDR, bool set_true, DataBlock* data, DataBlock* meta=NULL);
        bool get_true(Packet* pkt);
 
      protected:
        bool data_enable;
        bool meta_enable;
        bool true_enable;
        uint64_t tRD;
        uint64_t tWR;
        
        std::map<uint64_t, MemoryPair> store_data;
        std::map<uint64_t, MemoryPair> true_data;
    };
};

#endif
