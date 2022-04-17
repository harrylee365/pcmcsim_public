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
 * Description: This is a class of defining data container
 * which is required because data might be longer than 64-bit
 */

#ifndef __PCMCSIM_DATABLOCK_H_
#define __PCMCSIM_DATABLOCK_H_

#include "base/PCMCTypes.h"

namespace PCMCsim
{
    class DataBlock
    {
      public:
        DataBlock( );
        ~DataBlock( );
        DataBlock( const DataBlock& copy );

        void printData( );
        void setSize(uint64_t num_bytes);
        uint64_t getSize( ) const;
        void setByte(uint64_t byte, nbyte_t value);
        nbyte_t getByte(uint64_t byte) const;

        DataBlock extract(uint64_t obj_bits, uint64_t prev_bits) const;
        void partial_set(const DataBlock& new_data, uint64_t obj_bits, uint64_t prev_bits);

        void wrap_u64(uint64_t value);
        uint64_t unwrap_u64( );

        DataBlock& operator=(const DataBlock& rhs);
        bool operator==(const DataBlock rhs);

        nbyte_t* rawData;
      private:
        uint64_t num_bytes;
    };
};

#endif
