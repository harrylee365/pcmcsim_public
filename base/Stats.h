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

 * Description: Statistics recording macros, which is modified
 * from NVMain (https://github.com/SEAL-UCSB/NVmain.git)
 */

#ifndef __PCMCSIM_STATS_H_
#define __PCMCSIM_STATS_H_

#include <typeinfo>
#include "base/PCMCTypes.h"

/* Usage available for users */
#define RESET_STATS(STAT, resetVal)                    \
    do {                                                \
        if (typeid(STAT)!=typeid(resetVal))             \
        {                                               \
            std::cerr << "Error! Cannot reset ["        \
                << #STAT << "] due to unmatched types"  \
                << " compared with the reset value."    \
                << std::endl;                           \
            assert(0);                                  \
            exit(1);                                    \
        }                                               \
        memcpy((void*)&STAT, &resetVal, sizeof(STAT));  \
    } while(0);

#define ADD_STATS(MASTER_NAME, STAT)                   \
    do {                                                \
        _ADD_STATS_CORE(MASTER_NAME, STAT, "")         \
    } while(0);

#define ADD_STATS_N_UNIT(MASTER_NAME, STAT, UNIT)      \
    do {                                                \
        _ADD_STATS_CORE(MASTER_NAME, STAT, UNIT)       \
    } while(0);

#define ADD_STATS_ITER(MASTER_NAME, STATBASE, ITER_IDX)            \
    do {                                                            \
        _ADD_STATS_ITER_CORE(MASTER_NAME, STATBASE, ITER_IDX, "")  \
    } while(0);

#define ADD_STATS_ITER_UNIT(MASTER_NAME, STATBASE, ITER_IDX, UNIT) \
    do {                                                            \
        _ADD_STATS_ITER_CORE(MASTER_NAME, STATBASE, ITER_IDX, UNIT)\
    } while(0);

/* Not for users (core macros) */
#define _ADD_STATS_CORE(MASTER_NAME, STAT, UNIT)        \
    do {                                                \
        std::string tmp_name = MASTER_NAME;             \
        this->stats->add_stat((void*)&STAT,             \
               typeid(STAT).name( ),                    \
               sizeof(STAT),                            \
               tmp_name+"."+#STAT, UNIT);               \
    } while(0);

#define _ADD_STATS_ITER_CORE(MASTER_NAME, STATBASE, ITER_IDX, UNIT)        \
    do {                                                                    \
        std::string tmp_name = MASTER_NAME;                                 \
        this->stats->add_stat((void*)(STATBASE+ITER_IDX),                   \
            typeid(*STATBASE).name( ),                                      \
            sizeof(*STATBASE),                                              \
            tmp_name+"."+#STATBASE+"["+std::to_string(ITER_IDX)+"]", UNIT); \
    } while(0);

namespace PCMCsim
{
    class StatsContainer
    {
      public:
        StatsContainer( ) = delete;
        StatsContainer(void* stat_ptr, std::string type_name,
                      size_t type_size, std::string stat_name, std::string unit);
        ~StatsContainer( ) { }

        void* getStatPtr( ) { return sptr; }
        std::string getStatName( ) { return name; }
        std::string getUnit( ) { return unit_name; }
        std::string getTypeName( ) { return type_name; }
        size_t getTypeSize( ) { return type_size; }
        
      private:
        std::string name;
        std::string unit_name;
        std::string type_name;
        size_t type_size;
        
        void* sptr;
    };

    class Stats
    {
      public:
        Stats( ) { }
        ~Stats( );

        void add_stat(void* stat_ptr, std::string type_name, 
                      size_t type_size, std::string stat_name, std::string unit);
        void remove_stat(void* stat_ptr);
        void print(std::ostream& os);

      private:
        std::list<StatsContainer*> slist;
    };
};

#endif

