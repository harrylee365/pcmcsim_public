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
 * Description: This is a class for parsing trace lines
 */

#ifndef __PCMCSIM_TRC_GEN_H_
#define __PCMCSIM_TRC_GEN_H_

#include "base/PCMCTypes.h"

namespace PCMCsim
{
    class TraceGen 
    {
      public:
        enum _trc_type
        {
            NVMV=0,         // NVMain formatted
            VALIDATE_INPUT, // very input
            VALIDATE_CPATH, // control path
            VALIDATE_DPATH, // data path

            NUM_TRC_TYPES
        };

        typedef struct _trc_line_t
        {
            uint64_t cycle;
            cmd_t cmd_type;
            uint64_t LADDR;
            uint64_t PADDR;
            uint8_t* data;
            uint8_t* meta;
            id_t id;
        } trc_line_t;

        TraceGen(std::string trcPath, uint64_t data_byte=64, uint64_t meta_byte=0);
        ~TraceGen( );

        trc_line_t line_info;
        
        int init_trc_gen( );
        int getTrcType( ) { return trc_type; }
        uint32_t getDataSize( ) { return DATA_BYTE; }
        uint32_t getMetaSize( ) { return META_BYTE; }
        bool getNextTrcLine( );
        void printTrcLine( );
        
      private:
        uint64_t DATA_BYTE;
        uint64_t META_BYTE;
        uint32_t trc_type;
        std::ifstream trcFile;

        bool decode_trc(std::string& field_str, uint8_t& field_ptr);
    };
};

#endif
