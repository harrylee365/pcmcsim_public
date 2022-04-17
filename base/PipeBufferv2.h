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
 * Description: This is a class of pipeline buffer,
 * which emulates the stage registers of sub-module 
 */

#ifndef __PCMCSIM_PIPE_BUFFERV2_H_
#define __PCMCSIM_PIPE_BUFFERV2_H_

#include "base/PCMCTypes.h"

namespace PCMCsim
{
    class Component;
    class Packet;

    class PipeBufferv2
    {
      private:
        typedef struct _pbe_t
        {
            void* signals;
            int stage;
        } pbe_t;

      public:
        PipeBufferv2( );
        PipeBufferv2(int latency);
        ~PipeBufferv2( );

        /* Structure of the pipeline buffer */
        bool prgm_ready; // ready to accept pkt (programmable by user)
        bool prgm_stall; // stall to output pkt (programmable by user)
        
        void* input;
        pbe_t* buffer; // circular buffer
        
        PipeBufferv2* nxt_pb;

        /* Functions used in execution part of a module */
        void* proceed( );
        bool isAcceptable( );
        bool isFull( );
        bool isEmpty( );

        int getSize( );
        int getHeadIdx( );
        int getRearIdx( );

        void enqueue(void* pkt);
        void* dequeue_rear( );

      private:
        int size; // size = total stage num
        int front; // first content
        int rear; // last content

        /* Inner functions of pipeline buffer */
        bool canProceed(int idx);
        void* dequeue( );
    };
};

#endif
