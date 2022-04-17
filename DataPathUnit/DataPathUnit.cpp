#include "base/MemoryControlSystem.h"
#include "base/EventQueue.h"
#include "base/Packet.h"
#include "base/Stats.h"
#include "DataPathUnit/DataPathUnit.h"
#include "ReadModifyWrite/ReadModifyWrite.h"
#include "uCMDEngine/uCMDEngine.h"
#include "MemoryModules/DummyMemory/DummyMemory.h"

using namespace PCMCsim;

DataPathUnit::DataPathUnit(MemoryControlSystem* memsys_, 
        std::string cfg_header, ncycle_t ticks_per_cycle_, uint64_t id_)
: Component(memsys_), rmw(NULL), ucmde(NULL), ber_errors(0), ber_correct(0),
ber_due(0), ber_sdc(0), ber_reads(0)
{
    id = id_;
    cp_name = cfg_header + "[" + std::to_string(id) + "]";
    
    dbg_msg = memsys->getParamBOOL(cp_name+".dbg_msg", false);
    if (ticks_per_cycle_==0)
        ticks_per_cycle= memsys->getParamUINT64(cp_name+".ticks_per_cycle", 1);
    else 
        ticks_per_cycle = ticks_per_cycle_;

    tECC_WR = memsys->getParamUINT64(cp_name+".tECC_WR", 9);
    tECC_RD = memsys->getParamUINT64(cp_name+".tECC_RD", 15);
    dpu_wbuf_size   = memsys->getParamUINT64(cp_name+".wbuffer_size", 64);
    tBURST          = memsys->getParamUINT64(cp_name+".tBURST", 4);
    dpu_wbid_to_RMW = 0;
    media_wr_pkt = NULL;

    ecc_enable = memsys->getParamBOOL(cp_name+".ecc_enable", false);
    ecc_cap = memsys->getParamUINT64(cp_name+".ecc_capability", 1);

    register_stats( );
}

DataPathUnit::~DataPathUnit( )
{
}

bool DataPathUnit::isReady(Packet* pkt)
{
    bool rv = true;
    if (pkt->from==ucmde)
    {
        assert(pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WRITE_PRE);
        if (dpu_wbuf.find(pkt->buffer_idx)==dpu_wbuf.end( ))
            rv = false;
    }
    else if (dpu_wbuf_index.size( )>dpu_wbuf_size) // from rmw
    {
        rv = false;
        PCMC_DBG(dbg_msg,"[DPU] DPU Write Buffer FULL\n"); 
    }
    return rv;
}

void DataPathUnit::recvRequest(Packet* pkt, ncycle_t delay)
{
    ncycle_t LAT=delay+tECC_WR;

    if (((pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WRITE_PRE) && 
          pkt->isDATA==false) ||
        pkt->cmd==CMD_BWT)                       // cmd from ucmde
    {
        std::map<uint64_t, Packet*>::iterator iter;
        iter=dpu_wbuf.find(pkt->buffer_idx);
        assert(pkt->from==ucmde && iter!=dpu_wbuf.end( ));
        media_wr_pkt = iter->second;

        PCMC_DBG(dbg_msg,"[DPU] Req [0x%lx, ID=%lx, CMD=BWT] "
            "from ucmde of WB_ID=%lx has Wrote to Media\n", 
            media_wr_pkt->LADDR, pkt->req_id, media_wr_pkt->buffer_idx);

        /* Issue wdata pkt to media, and confer new wbuffer id to RMW */
        media->recvRequest(media_wr_pkt, tBURST);

        dpu_wbuf_index.erase(pkt->buffer_idx);   // Erase wbid from index set
        dpu_wbuf.erase(iter);                    // Erase wbuf contents from Wbuf_Map
        if (wbid_stalled)
            confer_wbid2rmw( );
    }
    else if (pkt->cmd==CMD_WRITE && pkt->isDATA)  //Write from RMW
    {
        PCMC_DBG(dbg_msg,"[DPU] Get Req-Write [0x%lx, ID=%lx, CMD=W, "
            "wb_id=%lx] from RMW\n", pkt->LADDR, pkt->req_id, pkt->buffer_idx); 

        /* Store incoming RMW Write packet to WBuffer */
        registerCallback((CallbackPtr)&DataPathUnit::push_wdata,
            LAT, 1, reinterpret_cast<void*>(pkt));
    }
    else 
        assert(0);
}

void DataPathUnit::recvResponse(Packet* pkt, ncycle_t delay)
{
    assert(pkt->cmd==CMD_READ);
    Component::recvResponse(pkt, delay);
}

void DataPathUnit::handle_await_resps( )
{
    /* Push read response data to read buffer */
    std::list<LocalEvent*>::iterator e_it = await_resp.begin( );
    for ( ; e_it!=await_resp.end( ); )
    {
        Packet* pkt = (*e_it)->pkt;
        dpu_rbuf.push(pkt);
        
        PCMC_DBG(dbg_msg,"[DPU] Push RD-RESP of "
            "Req [0x%lx, ID=%lx, CMD=R] to rbuf\n",
            pkt->LADDR, pkt->req_id); 

        /* Free event */
        delete (*e_it);
        e_it = await_resp.erase(e_it);
    }

    if (last_wake_rbuf==wake_rbuf &&
        dpu_rbuf.empty( )==false)
    {
        wake_rbuf = geq->getCurrentTick( )+1*ticks_per_cycle;
        registerCallback((CallbackPtr)&DataPathUnit::cycle_rbuf, 1);
    }
}

void DataPathUnit::cycle_rbuf( )
{
    assert(dpu_rbuf.empty( )==false);
    Packet* pkt = dpu_rbuf.front( );
    ncycle_t LAT = tECC_RD+tBURST;

    /* Perform ECC */
    ecc_check(pkt);
   
    pkt->from = this; 
    rmw->recvResponse(pkt, LAT);
    dpu_rbuf.pop( );

    PCMC_DBG(dbg_msg,"[DPU] READ-resp to rmw of "
        "Req [0x%lx, ID=%lx, CMD=R] after %lu cyc\n",
        pkt->LADDR, pkt->req_id, LAT); 

    last_wake_rbuf = wake_rbuf;
    if (dpu_rbuf.empty( )==false)
    {
        wake_rbuf = geq->getCurrentTick( )+1*ticks_per_cycle;
        registerCallback((CallbackPtr)&DataPathUnit::cycle_rbuf, 1);
    }
}

void DataPathUnit::ecc_check(Packet* pkt)
{
    ber_reads += pkt->buffer_data.getSize( );
    
    if (ecc_enable==false)
        return;

    Packet* true_pkt = new Packet( );
    *true_pkt = *pkt;
    if (dynamic_cast<DummyMemory*>(media)->get_true(true_pkt))
    {
        assert(true_pkt->buffer_data.getSize( )==
            pkt->buffer_data.getSize( ));

        uint64_t tmp_errors = 0;
        for (uint64_t b=0; b<pkt->buffer_data.getSize( ); b++)
        {
            uint64_t true_byte = true_pkt->buffer_data.getByte(b);
            uint64_t store_byte = pkt->buffer_data.getByte(b);
            if (true_byte==store_byte) continue;

            uint8_t xor_byte = true_byte ^ store_byte;
            while (xor_byte>0)
            {
                tmp_errors += xor_byte&0x1;
                xor_byte = xor_byte>>1;
            }
        }

        ber_errors += tmp_errors;
        if (tmp_errors>0 && tmp_errors<=ecc_cap)
            ber_correct += tmp_errors;
        else if (tmp_errors>ecc_cap)
        {
            if (ecc_cap+1==tmp_errors)
                ber_due += tmp_errors;
            else if (ecc_cap+1<tmp_errors)
                ber_sdc += tmp_errors;
        }
    }

    delete true_pkt;
}

int64_t DataPathUnit::find_empty_wbid( )
{ 
    /* find empty write buffer index */
    int64_t empty_index=-1;
    std::set<uint64_t>::iterator iter;
    for (uint64_t i=0; i<dpu_wbuf_size; i++)
    {
        iter = dpu_wbuf_index.find(i);
        if (iter==dpu_wbuf_index.end())
        {
            empty_index = (int64_t)i;
            break;
        }
    }
    return empty_index;
}

void DataPathUnit::handle_events(ncycle_t curr_tick)
{
    prepare_events(curr_tick);

    if (!await_resp.empty( ))
        handle_await_resps( );

    if (!await_cb.empty( ))
        handle_await_callbacks( );
}

void DataPathUnit::confer_wbid2rmw( )
{
    /* Update wbuffer ID instantly */
    wbid_stalled = false;
    int64_t new_index = find_empty_wbid( );
    if (new_index>=0)
    {
        Packet* dpu_resp_pkt = new Packet( );
        dpu_resp_pkt->cmd = CMD_WRITE;
        dpu_resp_pkt->buffer_idx = new_index;
        dpu_resp_pkt->owner = this;
        dpu_resp_pkt->from = this;
        rmw->recvResponse(dpu_resp_pkt, 1);
        
        PCMC_DBG(dbg_msg,"[DPU] Updated Wbuffer id to rmw of "
            "id=%lx a cycle (size=%ld)\n", dpu_resp_pkt->buffer_idx, dpu_wbuf.size( ));
    }
    else
        wbid_stalled = true;
}

void DataPathUnit::push_wdata(Packet* pkt)
{
    PCMC_DBG(dbg_msg,"[DPU] PUT data to WTBUFFER MAP of "
        "[0x%lx, ID=%lx, wb_id=%lx]\n", pkt->LADDR, pkt->req_id, pkt->buffer_idx); 

    dpu_wbuf.insert(std::pair<uint64_t,Packet*>(pkt->buffer_idx, pkt));
    dpu_wbuf_index.insert(pkt->buffer_idx);

    confer_wbid2rmw( );
}

void DataPathUnit::register_stats( )
{
    ADD_STATS(cp_name, ber_errors);
    ADD_STATS(cp_name, ber_correct);
    ADD_STATS(cp_name, ber_due);
    ADD_STATS(cp_name, ber_sdc);
    ADD_STATS(cp_name, ber_reads);
}
