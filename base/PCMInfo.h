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
 */

#ifndef __PCMCSIM_PCM_INFO_H_
#define __PCMCSIM_PCM_INFO_H_

#include "base/PCMCTypes.h" 
#include "base/DataBlock.h"

namespace PCMCsim
{
    class MemoryControlSystem;
    class AddressDecoder;
    class MemInfo;

    class MetaDecoder
    {
      public:
        MetaDecoder(MemoryControlSystem* memsys, std::string cfg_header, MemInfo* info);
        MetaDecoder( ) = delete;
        ~MetaDecoder( ) { }

        uint64_t get_width(pcm_meta_field_t field) { return widths[field]; }
        DataBlock get_meta(const DataBlock& meta, pcm_meta_field_t field);
        void set_meta(DataBlock& meta, pcm_meta_field_t field, const DataBlock& new_data);

      private:
        uint64_t META_SIZE; 
        uint64_t widths[NUM_META_FLD];
    };

    class AITDecoder
    {
      public:
        AITDecoder(MemoryControlSystem* memsys, std::string cfg_header, AddressDecoder* adec);
        AITDecoder( ) = delete;
        ~AITDecoder( ) { }

        uint64_t get_width(ait_field_t field) { return widths[field]; }
        uint64_t get_ait(uint64_t e_ait, ait_field_t field);
        void set_ait(uint64_t& e_ait, ait_field_t field, uint64_t new_data);

      private:
        MemInfo* info;
        uint64_t widths[NUM_AIT_FLD];
    };
};
#endif
