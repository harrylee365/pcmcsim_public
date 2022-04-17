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
 * Description: This is a base class of all modules in PCMC
 * which defines clock frequency, event queue, and module id
 */

#ifndef __PCMCSIM_COMPONENT_H_
#define __PCMCSIM_COMPONENT_H_

#include "base/PCMCTypes.h"
#include "base/DataBlock.h"
#include <functional>

namespace PCMCsim
{
    class MemoryControlSystem;
    class Packet;
    class LocalEvent;
    class GlobalEventQueue;
    class Component;
    class TraceGen;
    class Stats;

    typedef void (Component::*CallbackPtr)(void*);

    class Component 
    {
      public:
        Component( );
        Component(MemoryControlSystem* memsys_);
        virtual ~Component( );

        /* Interface functions */
        virtual bool isReady(Packet* pkt);
        virtual void recvRequest(Packet* pkt, ncycle_t delay=1);
        virtual void recvResponse(Packet* pkt, ncycle_t delay=1);

        virtual void setParent(Component* p);
        virtual void setChild(Component* c);

        void set_ticks_per_cycle(uint64_t ticks) { ticks_per_cycle = ticks; }

        static std::string get_cmd_str(Packet* pkt);
        static std::string get_cmd_str(cmd_t cmd);
        std::string get_name( ) { return cp_name; }

        /* Event scheduling functions (user-defined requried) */
        virtual void handle_events(ncycle_t curr_tick) = 0;
        virtual void handle_await_reqs( ) { }
        virtual void handle_await_resps( ) { }

        /* Inner Event scheduling functions */
        void prepare_events(ncycle_t curr_tick);
        void handle_await_callbacks( );
        void registerCallback(CallbackPtr cb, ncycle_t delay,
                              int priority=0, void* cb_arg=NULL);
        void registerCallbackAt(CallbackPtr cb, ncycle_t wakeup,
                                int priority=0, void* cb_arg=NULL);

        /* Event queue functions */
        void setTicksPerCycle(ncycle_t ticks) { ticks_per_cycle = ticks; } 
        ncycle_t getTicksPerCycle( );
        GlobalEventQueue* getGlobalEventQueue( );
        
        /* Stats */
        virtual void calculate_stats( ) { }
        virtual void print_stats(std::ostream& os);

        bool is_msg( ) { return dbg_msg; }

      protected:
        uint32_t id;                // module id
        bool dbg_msg;
        bool ready;                 // ready signal only accept when it is set
        ncycle_t ticks_per_cycle;   // # of ticks per component clock
        MemoryControlSystem* memsys;        // PCM controller top module
        GlobalEventQueue* geq;      // event queue pointer (gotten from PCMC)
        std::string cp_name;        // component name

        Component* parent;
        Component* child;
        
        /* Awaiting events extracted from {resp, req, cb}_events */ 
        std::list<LocalEvent*> await_resp;
        std::list<LocalEvent*> await_req;
        std::list<LocalEvent*> await_cb;

        /* Stats */
        Stats* stats;
        virtual void register_stats( ) { }
        
//      private:
        /* Events that have just got into the module */
        std::multimap<ncycle_t, LocalEvent*> resp_events;
        std::multimap<ncycle_t, LocalEvent*> cb_events;
        std::multimap<ncycle_t, LocalEvent*> req_events;
    };
    
};

#endif
