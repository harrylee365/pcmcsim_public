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
 * Authors: Hyungsuk Kim (kimhs@capp.snu.ac.kr)
 */

#ifndef __PCMCSIM_X_BAR_H_
#define __PCMCSIM_X_BAR_H_

#include "base/PCMCTypes.h"
#include "base/Component.h"

namespace PCMCsim
{
	class XBar : public Component
	{
	public:
		XBar(MemoryControlSystem* memsys_, std::string cfg_header);
		~XBar( );

        /* Communication Functions */
		bool isReady(Packet* pkt) override;
		void recvRequest(Packet* pkt, ncycle_t delay = 1) override;
		void recvResponse(Packet* pkt, ncycle_t delay = 1) override;
		void handle_events(ncycle_t curr_tick) override;

        void expand_port(Component* ip);

	private:
		/* Top functions */
		bool need_wakeup;
        uint64_t size_cmdq;
        uint64_t num_ports;
		std::vector<std::queue<Packet*>> cmdq;
        std::vector<uint64_t> credit_cmdq;
        std::map<Component*, uint64_t> ip_map;

        void handle_await_reqs( ) override;

        /* Arbiter */
        ncycle_t last_wake_arbit;
        ncycle_t wake_arbit;
        ncycle_t free_arbit;
        uint64_t out_port_id;

        bool is_cmdq_avail( );
        void cmdq_proceed( );

        /* Timing parameter */
        ncycle_t tBURST;
	};
};

#endif
