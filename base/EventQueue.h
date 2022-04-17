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
 * Description: This is a global event queue class, which has 
 * a master clock frequency of 1THz. Therefore, clock frequencies 
 * of all modules must be SLOWER than the master clock.
 */

#ifndef __PCMCSIM_EVENTQUEUE_H_
#define __PCMCSIM_EVENTQUEUE_H_

#include "base/PCMCTypes.h"
#include "base/Component.h"
#include "base/DataBlock.h"

namespace PCMCsim 
{
    class Packet;

    typedef std::map<uint64_t, std::set<Component*>> event_queue_t;

    class GlobalEventQueue
    {
      public:
        GlobalEventQueue( );
        ~GlobalEventQueue( );

        void insertEvent(ncycle_t wakeupTime, Component* component);
        void handle_events( );
        void handle_events(ncycle_t __curr_tick);
        void set_dbg_msg(bool setup);
        bool event_exist( ) { return (event_q.empty( )==false); }

        ncycle_t getCurrentTick( );

        bool escape;
            
      private:
        event_queue_t event_q;
        ncycle_t curr_tick;

        bool dbg_msg;
    };

    typedef enum _event_type
    {
        TX_EVENT = 0,
        CB_EVENT,
        NUM_EVENT
    } event_type;


    class LocalEvent
    {
      public:
        LocalEvent( );
        ~LocalEvent( );

        event_type type;
        Component* component;

        Packet* pkt;
        
        CallbackPtr cb_method;
        void* cb_arg;
        int cb_priority; // the larger number, the higher priority
    };
};

#endif
