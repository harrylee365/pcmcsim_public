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
 * Author: Hyokeun Lee (hklee@capp.snu.ac.kr)
 *
 * Description: This is a class that assembles all modules for PCMC
 */

#ifndef __PCMCSIM_MEMSYS_H_
#define __PCMCSIM_MEMSYS_H_

#include "base/PCMCTypes.h"

namespace PCMCsim
{
    class GlobalEventQueue;
    class Component;
    class DataBlock;
    class Packet;
    class Parser;
    class RequestReceiver;
    class DataCache;
    class MicroControlUnit;
    class AITManager;
    class ReadModifyWrite;
    class GPCache;
    class DummyMemory;
    class uCMDEngine;
    class DataPathUnit;
    class MemInfo;
    class AddressDecoder;
    class MetaDecoder;
    class AITDecoder;
    class XBar;

    typedef std::pair<DataBlock*, DataBlock*> MemoryPair; //<data, meta>

    class MemoryControlSystem
    {
      public:
        MemoryControlSystem(const std::string & cfgfile, 
                            GlobalEventQueue* geq, std::string _path_prefix="");
        ~MemoryControlSystem( );

        void recvRequest(Packet* pkt, ncycle_t delay=1);
        void recvResponse(Packet* pkt, ncycle_t delay=1);
        bool isReady(Packet* pkt);

        void setup_pcmc(Component* host_itf=NULL);  // setup PCM controller
        void setup_dmc(Component* host_itf=NULL);   // setup DRAM controller

        /* Functions for getting parameters from configuration */
        double getParamFLOAT(const std::string& key, double def) const;
        uint64_t getParamUINT64(const std::string& key, uint64_t def) const;
        bool getParamBOOL(const std::string& key, bool def) const; 
        std::string getParamSTR(const std::string& key, std::string def="") const;
        void getParamSRAM(const uint64_t num_sets, const uint64_t num_ways, 
                          double* Esr, double* Erd, double* Ewr);

        GlobalEventQueue* getGlobalEventQueue( );
        
        bool getMemData(Packet* pkt);
        void setMemData(uint64_t PA, DataBlock& data, DataBlock& meta);

        /* Memory information object */
        MemInfo* info;
        AddressDecoder* adec;
        MetaDecoder* mdec;
        AITDecoder* tdec;
        std::string sys_name;
        
        /* Modules in PCMC */
        Component* host_itf;
        Parser* parser;
        RequestReceiver* recvr;
        DataCache* dcache;
        AITManager* aitm;
        ReadModifyWrite* rmw;
        std::vector<RequestReceiver*> recvr_dmc;
        std::vector<uCMDEngine*> ucmde;
        std::vector<DataPathUnit*> dpu;
        std::vector<DummyMemory*> media;

        /* Subsystem in PCMC */
        XBar* xbar;
        MicroControlUnit* mcu;
        Component* ait_mem;

        AddressDecoder* ait_adec;
        MemInfo* ait_info;
        uCMDEngine* ait_dmc;

        uint64_t op_rate;   // overprovision ratio in WLV

        /* Stats */ 
        void print_stats(std::ostream& os);

      private:
        /* Simulation-related variables */
        GlobalEventQueue* geq;
        std::map<std::string, std::string> params;
        std::string path_prefix;

        /* TODO : large size expansion -> file management */
        std::map<uint64_t, MemoryPair> memoryData;
    };

};


#endif
