#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "uCMDEngine/uCMDEngine.h"

using namespace PCMCsim;

uCMDEngine::uCMDEngine(MemoryControlSystem* memsys_)
:Component(memsys_), dpu(NULL), media(NULL)
{
}

uCMDEngine::~uCMDEngine( )
{
}

void uCMDEngine::wack(Packet* pkt, cmd_t type)
{
    assert(type==CMD_WACK_ID || type==CMD_PKT_DEL);
    
    if (type==CMD_WACK_ID)
    {
        Packet* resp_pkt = new Packet( );
        *resp_pkt = *pkt;
        resp_pkt->from = this;
        resp_pkt->cmd = type;
        PCMC_DBG(dbg_msg,"[uCMDE] Issue WR-FIN from ucmde, "
            "Req[0x%lx, ID=%lx]\n", pkt->LADDR, pkt->req_id);
        parent->recvResponse(resp_pkt);
        delete resp_pkt;
    }
    else
    {
        pkt->from = this;
        pkt->cmd = type;
        pkt->owner->recvResponse(pkt);
    }
}

