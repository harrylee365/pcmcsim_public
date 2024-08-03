#include "base/MemoryControlSystem.h"
#include "base/MemInfo.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "Parsers/Parser.h"

using namespace PCMCsim;

Parser::Parser(MemoryControlSystem* memsys_, std::string cfg_header)
:Component(memsys_), last_rdresp(0)
{
    cp_name = cfg_header;
    dbg_msg = memsys->getParamBOOL(cp_name+".dbg_msg", false);
    ticks_per_cycle= memsys->getParamUINT64("global.ticks_per_cycle", 1);
}

Parser::~Parser( )
{
}

void Parser::recvRequest(Packet* pkt, ncycle_t delay)
{
    uint64_t ch = 0;
    if (memsys->sys_name=="DRAM")
        ch = memsys->adec->decode_addr(pkt->LADDR, FLD_CH);

    paths[ch]->recvRequest(pkt, delay);
    
    PCMC_DBG(dbg_msg, "[Parser] Dispatch [LA=0x%lx, PA=%lx, ID=%ld]\n",
        pkt->LADDR, pkt->PADDR, pkt->req_id);
}

void Parser::recvResponse(Packet* pkt, ncycle_t delay)
{
    if (pkt->cmd==CMD_READ)
    {
//        assert(last_rdresp!=geq->getCurrentTick( ));
        last_rdresp = geq->getCurrentTick( );
    }

    PCMC_DBG(dbg_msg, "[Parser] %s of [LA=0x%lx, PA=%lx, ID=%ld]\n",
        (pkt->cmd==CMD_READ)? "Make RD-resp":"Virtual resp", 
        pkt->LADDR, pkt->PADDR, pkt->req_id);

    memsys->recvResponse(pkt, delay);
}

bool Parser::isReady(Packet* pkt)
{
    if (pkt->isDATA) // for standalone mode
        return true;
    else
    {
        uint64_t ch = 0;
        if (memsys->sys_name=="DRAM")
            ch = memsys->adec->decode_addr(pkt->LADDR, FLD_CH);
        return paths[ch]->isReady(pkt);
    }
}

void Parser::handle_events(ncycle_t /*curr_tick*/)
{
}

void Parser::expand_path(Component* c)
{
    paths.push_back(c);
}

