#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "Subsystems/XBar/XBar.h"

using namespace PCMCsim;

XBar::XBar(MemoryControlSystem* memsys_, std::string cfg_header):
Component(memsys_), need_wakeup(true), num_ports(0),
last_wake_arbit(0), wake_arbit(0), free_arbit(0), out_port_id(0)
{
    assert(memsys);

    /* Parameter load */
    dbg_msg = memsys->getParamBOOL(cfg_header+".dbg_msg", true);
    size_cmdq = memsys->getParamUINT64(cfg_header+".size_cmdq", 1);
    ticks_per_cycle = memsys->getParamUINT64("global.ticks_per_cycle", 1);
    tBURST = memsys->getParamUINT64("dramWpr.tBURST", 4);
}

XBar::~XBar( )
{
}

bool XBar::isReady(Packet* pkt)
{
    assert(cmdq.size( )>0);
    assert(ip_map.count(pkt->from)>0);

    bool rv = true;
    if (credit_cmdq[ip_map[pkt->from]]==0)
        rv = false;
    return rv;
}

void XBar::expand_port(Component* ip)
{
    ip_map[ip] = num_ports;
    num_ports+=1;
    
    cmdq.resize(num_ports);
    credit_cmdq.resize(num_ports, size_cmdq);
}

void XBar::recvRequest(Packet* pkt, ncycle_t delay)
{
    assert(isReady(pkt));
    
    credit_cmdq[ip_map[pkt->from]]--;
    Component::recvRequest(pkt, delay);
    
    PCMC_DBG(dbg_msg, "[XBar] Recv %s  [0x%lx, ID=%lx, CMD=%c]\n",
        (pkt->cmd==CMD_READ && pkt->isDATA)? "resp" : "req",
        pkt->LADDR, pkt->req_id, (pkt->cmd==CMD_READ)? 'R':'W');
}

void XBar::recvResponse(Packet* pkt, ncycle_t delay)
{
    recvRequest(pkt, delay);
}

void XBar::handle_events(ncycle_t curr_tick)
{
    /* Don't change following order */
    prepare_events(curr_tick);

    if (!await_cb.empty( ))
        handle_await_callbacks( );

    if (!await_req.empty( ))
        handle_await_reqs( );
}

void XBar::handle_await_reqs( )
{
    std::list<LocalEvent*>::iterator e_it = await_req.begin( );
    for ( ; e_it!=await_req.end( ); )
    {
        Packet* pkt = (*e_it)->pkt;
        cmdq[ip_map[pkt->from]].push(pkt);

        /* Free event */
        delete (*e_it);
        e_it = await_req.erase(e_it);
    }

    if (last_wake_arbit==wake_arbit &&
        is_cmdq_avail( )==true)
    {
        wake_arbit = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_arbit = std::max(wake_arbit, free_arbit);
        registerCallbackAt((CallbackPtr)&XBar::cmdq_proceed, wake_arbit);
    }
}

bool XBar::is_cmdq_avail( )
{
    bool rv = false;
    for (uint64_t i=0; i<cmdq.size( ); i++)
    {
        if (cmdq[i].size( )>0)
        {
            rv = true;
            break;
        }
    }
    return rv;
}

void XBar::cmdq_proceed( )
{
    /* Issue command */
    ncycle_t LAT = 1;
    bool isIssued = false;
    if (cmdq[out_port_id].size( )>0)
    {
        Packet* pkt = cmdq[out_port_id].front( );
        if ((pkt->cmd==CMD_READ && pkt->isDATA) || // read response
            pkt->dest->isReady(pkt))
        {
            cmdq[out_port_id].pop( );
            credit_cmdq[out_port_id]++;
            isIssued = true;

            LAT = (pkt->isDATA)? tBURST : LAT;
            if (pkt->cmd==CMD_READ && pkt->isDATA)
                pkt->dest->recvResponse(pkt, LAT);
            else // write request or read request
                pkt->dest->recvRequest(pkt, LAT);
        }
    }

    /* Round robin arbitration */
    for (uint64_t i=1; i<=num_ports; i++)
    {
        uint64_t tmp_port_id = (out_port_id+i)%num_ports;
        if (cmdq[tmp_port_id].size( )>0)
        {
            out_port_id = tmp_port_id;
            break;
        }
    }

    /* Issue to selected port */
    last_wake_arbit = wake_arbit;
    free_arbit = (isIssued==false)? free_arbit:(geq->getCurrentTick( )+LAT*ticks_per_cycle);
    if (is_cmdq_avail( ))
    {
        wake_arbit = geq->getCurrentTick( )+LAT*ticks_per_cycle;
        registerCallback((CallbackPtr)&XBar::cmdq_proceed, LAT);
    }
}

