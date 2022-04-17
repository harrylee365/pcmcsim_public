#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/PipeBufferv2.h"
#include "base/EventQueue.h"
#include "base/Stats.h"
#include "MemoryModules/GPCache/GPCache.h"
#include "ReplacePolicy/TrueLRU/TrueLRU.h"
#include "ReplacePolicy/PseudoLRU/PseudoLRU.h"
#include "ReplacePolicy/RoundRobin/RoundRobin.h"

using namespace PCMCsim;

typedef struct _signals_t 
{
    Packet* pkt;
    bool hit;
    int64_t tmp_way;
} signals_t;

GPCache::GPCache(MemoryControlSystem* memsys_, std::string cfg_header)
:Component(memsys_), need_wake(true), wake_missq(0), 
last_wake_missq(0), free_req(0), wake_fillq(0), last_wake_fillq(0), 
free_resp(0), status(NO_STALL)
{
    cp_name = cfg_header;

    /* Construct the cache */
    dbg_msg = memsys->getParamBOOL(cfg_header+".dbg_msg", false);
    ticks_per_cycle= memsys->getParamUINT64("global.ticks_per_cycle", 1);
    cline_bits = memsys->getParamUINT64(cfg_header+".cline_bits", 6);
    cline_bytes = 1 << cline_bits;

    byte_offset = memsys->getParamUINT64(cfg_header+".byte_offset", 3);
    assert(byte_offset>0 && byte_offset<=cline_bits);
    num_blocks = 1 << (cline_bits - byte_offset);
    width_block = 1 << byte_offset; 

    bool bypass = memsys->getParamBOOL(cfg_header+".bypass", false);
    bit_sets = memsys->getParamUINT64(cfg_header+".bit_sets", 4);
    num_sets = (bypass)? 0:(1<<bit_sets);
    num_ways = memsys->getParamUINT64(cfg_header+".num_ways", 4);
    size_cmdq = memsys->getParamUINT64(cfg_header+".size_cmdq", 2);
    size_missq = memsys->getParamUINT64(cfg_header+".size_missq", 16);
    tmem.resize(num_sets);
    dmem.resize(num_sets);
    for (uint64_t s = 0; s < num_sets; s++)
    {
        tmem_t e_tag {false, false, false, 0};
        DataBlock e_data;
        tmem[s].resize(num_ways, e_tag);
        dmem[s].resize(num_ways, e_data);
    }
    assert(bypass || (num_sets>0 && num_ways>=1 && size_cmdq>=2 && size_missq>=1));

    if (num_sets==1 && num_ways==1)
    {
        std::cerr << "[GPC] Error! Invalid cache size, 1-set & 1-way" << std::endl;
        assert(0);
        exit(1);
    }

    std::string got_policy = memsys->getParamSTR(cfg_header+".policy");
    victim_policy = NULL;
    if (got_policy=="TRUE_LRU")
        victim_policy = new TrueLRU(num_sets, num_ways);
    else if (got_policy=="PSEUDO_LRU")
        victim_policy = new PseudoLRU(num_sets, num_ways);
    else if (got_policy == "ROUND_ROBIN")
        victim_policy = new RoundRobin(num_sets, num_ways);
    else
        assert(0);

    std::string got_walloc = memsys->getParamSTR(cfg_header+".allocate_method", "WRITE_ALLOC");
    if (got_walloc == "WRITE_ALLOC" || got_walloc=="WRITE-ALLOC")
        write_alloc = true;
    else if (got_walloc == "NO_WRITE_ALLOC" || got_walloc=="NO-WRITE-ALLOC")
        write_alloc = false;
    else
        assert(0);

    write_only = memsys->getParamBOOL(cfg_header+".write_only", false);
    if (write_only && write_alloc==false)
    {
        std::cerr << "[GPC] GPCache of " << cfg_header << "'s allocation method is "
            << "NO WRITE ALLOC, whereas WRITE ONLY cache is configured" << std::endl;
        assert(0);
        exit(1);
    }

    /* MSHR setup */
    size_mshr = memsys->getParamUINT64(cfg_header+".size_mshr", 1);
    size_subentry_mshr = memsys->getParamUINT64(cfg_header+".size_subentry_mshr", 1);
    assert(size_subentry_mshr>0);

    /* Timing parameter setup */
    tRESP = memsys->getParamUINT64(cfg_header+".tRESP", 1);
    tLLM_RD = memsys->getParamUINT64(cfg_header+".tLLM_RD", 1);
    tLLM_WR = memsys->getParamUINT64(cfg_header+".tLLM_WR", 1);

    /* Setup pipeline buffers (static latency) */
    pipe_access = new PipeBufferv2( );

    /* Get energy parameters */
    memsys->getParamSRAM(num_sets, num_ways, &Esrpa, &Erdpa, &Ewrpa);

    /* Stats registration */
    register_stats( );
    
    print_info( );
}

GPCache::~GPCache( )
{
    delete pipe_access;
    delete victim_policy;
}

bool GPCache::isReady(Packet* /*pkt*/)
{
    bool rv = true;
    if (cmdq.size( )>=size_cmdq)
        rv = false;

    return rv;
}

void GPCache::recvRequest(Packet* pkt, ncycle_t delay)
{
    cmdq.push(pkt);
    if (need_wake)
    {
        need_wake = false;
        registerCallback((CallbackPtr)&GPCache::execute, delay);
    }

    PCMC_DBG(dbg_msg, "[GPC] Recvd request [0x%lx, CMD=%c, ID=%lx]\n", 
        pkt->LADDR, (pkt->cmd==CMD_READ)? 'R':'W', pkt->req_id); 
}

void GPCache::recvResponse(Packet* pkt, ncycle_t delay)
{
    if (pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WACK_ID)
    {
        PCMC_DBG(dbg_msg, "[GPC] %s\n", (pkt->owner==this)? 
            "Directly delete generated eviction req":
            "Virtual response for WRITE");
        
        if (pkt->owner==this) // eviction
            delete pkt;
        else
            sendParentResp(pkt);
    }
    else if (pkt->cmd==CMD_READ)
    {
        assert(pkt->buffer_data.getSize( )==cline_bytes);
        PCMC_DBG(dbg_msg, "[GPC] Recvd response [0x%lx, CMD=%c, ID=%lx]\n", 
            pkt->LADDR, (pkt->cmd==CMD_READ)? 'R':'W', pkt->req_id); 
        Component::recvResponse(pkt, delay);
    }
    else
        assert(0);
}

void GPCache::handle_events(ncycle_t curr_tick)
{
    /* Get events from queue */
    prepare_events(curr_tick);

    /* Don't change following order */
    if (!await_resp.empty( ))
        handle_await_resps( );

    if (!await_cb.empty( ))
        handle_await_callbacks( );
}

void GPCache::execute( )
{
    /* Execute main path sub-modules' functions */
    access_exec( );

    /* Advance pipeline stages */
    cmdq_proceed( );
    pipe_access->proceed( );

    if (pipe_access->isEmpty( )==false ||
        cmdq.empty( )==false)
    {
        registerCallback((CallbackPtr)&GPCache::execute, 1);
    }
    else
        need_wake = true;
}

void GPCache::handle_await_resps( )
{
    assert(await_resp.size( )==1);
    std::list<LocalEvent*>::iterator e_it = await_resp.begin( );
    for ( ; e_it!=await_resp.end( ); )
    {
        /* Response of read-allocate */
        Packet* pkt = (*e_it)->pkt;
        assert(pkt->cmd==CMD_READ && pkt->buffer_data.getSize( )==cline_bytes);

        fillq.push_back(pkt);

        /* Free event */
        delete (*e_it);
        e_it = await_resp.erase(e_it);
    }

    /* Schedule fillq execution if a pkt firstly gets in */
    if (last_wake_fillq!=wake_fillq)
        return;
    else if (fillq.empty( )==false)
    {
        wake_fillq = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_fillq = std::max(wake_fillq, free_resp);
        registerCallbackAt((CallbackPtr)&GPCache::cycle_fillq, wake_fillq);
    }
}

void GPCache::cmdq_proceed( )
{
    if (cmdq.size( )>0 && 
        pipe_access->isAcceptable( ))
    {
        Packet* pkt = cmdq.front( );
        cmdq.pop( );
        
        signals_t* new_pb_signals = new signals_t;
        new_pb_signals->pkt = pkt;
        new_pb_signals->hit = false;
        new_pb_signals->tmp_way = -1; 
        pipe_access->input = new_pb_signals;

        PCMC_DBG(dbg_msg, "[GPC] Read tag for [0x%lx, ID=%lx, %s]\n",
            pkt->LADDR, pkt->req_id, (pkt->cmd==CMD_READ)? "RD":"WR");
    }
}

void GPCache::access_exec( )
{
    int rear = pipe_access->getRearIdx( );
    if (pipe_access->isEmpty( )==false && pipe_access->buffer[rear].stage==0)
    {
        /* Determine cache hit/miss */
        signals_t* pb_signals = (signals_t*)(pipe_access->buffer[rear].signals);
        Packet* proc_pkt = pb_signals->pkt;
        if (num_sets>0)
        {
            uint64_t sidx = 0;
            get_cache_position(sidx, pb_signals->tmp_way, proc_pkt);
            if (pb_signals->tmp_way>=0 &&
                victim_policy->get_rsvd(sidx, pb_signals->tmp_way)==false)
            {
                pb_signals->hit = true;
            }
        }

        PCMC_DBG(dbg_msg, "[GPC] %s-%s on request [0x%lx, ID=%lx]\n",
            (proc_pkt->cmd==CMD_READ)? "RD" : "WR", 
            (pb_signals->tmp_way)? "HIT" : "MISS", 
            proc_pkt->LADDR, proc_pkt->req_id);

        handle_hitmiss(pb_signals->hit, pb_signals->tmp_way, proc_pkt);

        if (status==NO_STALL)
        {
            pipe_access->dequeue_rear( );
            pipe_access->prgm_stall = false;
            delete pb_signals;
        }
    }
    else if (pipe_access->isEmpty( )==false)
    {
        /* Stalled req handling */
        signals_t* pb_signals = (signals_t*)(pipe_access->buffer[rear].signals);
        Packet* proc_pkt = pb_signals->pkt;
        stall_handle(proc_pkt, pb_signals->tmp_way);

        if (status==NO_STALL)
        {
            pipe_access->dequeue_rear( );
            pipe_access->prgm_stall = false;
            delete pb_signals;
        }
    }

    /* Schedule fillq execution if direct response is performed */
    if (last_wake_fillq==wake_fillq && fillq.empty( )==false)
    {
        wake_fillq = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_fillq = std::max(wake_fillq, free_resp);
        registerCallbackAt((CallbackPtr)&GPCache::cycle_fillq, wake_fillq);
    }

    /* Schedule missq execution if a pkt firstly gets in */
    if (last_wake_missq==wake_missq && missq.empty( )==false)
    {
        wake_missq = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_missq = std::max(wake_missq, free_req);
        registerCallbackAt((CallbackPtr)&GPCache::cycle_missq, wake_missq);
    }
}

void GPCache::get_cache_position(uint64_t& sidx, int64_t& widx, Packet* proc_pkt)
{
    sidx = get_set_idx(proc_pkt->LADDR);
    uint64_t tag = get_tag(proc_pkt->LADDR);
    for (uint64_t w = 0; w < num_ways; w++)
    {
        if (tmem[sidx][w].valid==false) continue;

        if (tmem[sidx][w].tag==tag)
        {
            widx = (int64_t)w;
            break;
        }
    }

    Esr+=Esrpa;
}

void GPCache::handle_hitmiss(bool hit, int64_t hit_way, Packet* pkt)
{
    assert(status==NO_STALL);

    /* Hit/miss handling */
    if (hit==false)
        miss_handle(pkt);
    else
        hit_handle(pkt, hit_way);
    
    /* Stats update */
    if (hit || status==HIT_RSVD)
    {
        if (pkt->cmd==CMD_READ)
            read_hit+=1;
        else
            write_hit+=1;
        if (hit==false)
            num_wait_rsvd+=1;
    }
    else if (pkt->cmd==CMD_READ)
        read_miss+=1;
    else
        write_miss+=1;
}

void GPCache::hit_handle(Packet* pkt, int64_t hit_way)
{
    /* Access cache and respond directly if possible  */
    assert(hit_way>=0);
    uint64_t blk = get_block(pkt->LADDR);
    uint64_t s = get_set_idx(pkt->LADDR);
    int64_t w = hit_way;
    int nxt_status = NO_STALL;
    assert(dmem[s][w].getSize( )==cline_bytes);

    PCMC_DBG(dbg_msg, "[GPC] %s is %s to HIT req [0x%lx, ID=%lx]\n",
        (pkt->cmd==CMD_READ)? "RD-resp port":"DMEM write port",
        (pkt->cmd==CMD_READ)? ((isRespPortAvailable( ))? "free":"not free"):
        ((isWritePortAvailable( ))? "free":"not free"),
        pkt->LADDR, pkt->req_id);

    if (pkt->cmd==CMD_READ)
    {
        /* Get partial data for read response */
        pkt->buffer_data.setSize(width_block);
        for (uint64_t i = 0; i < width_block; i++)
            pkt->buffer_data.setByte(i, dmem[s][w].getByte(blk*width_block+i));
    
        /* Check if read-resp port is available (HIGHER priority to fillq) */
        if (isRespPortAvailable( ))
        {
            sendParentResp(pkt, tRESP);
            free_resp = geq->getCurrentTick( )+tRESP*ticks_per_cycle;
        }
        else
        {
            pipe_access->prgm_stall = true;
            nxt_status = WAIT_RDRESP;
            
            trigger_wait_rdresp = geq->getCurrentTick( );
        }
        
        Erd+=Erdpa;
    }
    else if (pkt->cmd==CMD_WRITE)
    {
        if (isWritePortAvailable( ))
            hit_wdata(pkt, hit_way);
        else
        {
            /* Stall if cacheline is being filled */
            pipe_access->prgm_stall = true;
            nxt_status = RACE_HITWR_FILL;
        }
    }
    else
        assert(0);

    victim_policy->update_victim(s, w);
    status = nxt_status;
}

void GPCache::miss_handle(Packet* pkt)
{
    signals_t* pb_signals = 
        (signals_t*)(pipe_access->buffer[pipe_access->getRearIdx( )].signals);

    status = mshr_check(pkt);
    if (status==MSHR_ALLOC_NEW)         // MSHR entry alloc NEWLY
        status = gpc_alloc(pkt);
    else if (status==MSHR_ALLOC_FAIL)   // MSHR entry alloc FAILS
    {
        if (num_sets==0 ||                              // bypass mode
            (!write_alloc && pkt->cmd==CMD_WRITE) ||    // no-wr-alloc & wr
            (write_only && pkt->cmd==CMD_READ))         // read write-only $
        {
            /* Bypass request */
            if (missq.size( )<size_missq)
            {
                /* Mask partial data */
                if (pkt->cmd==CMD_WRITE &&
                    num_blocks!=1 && 
                    pkt->buffer_data.getSize( )==width_block)
                {
                    uint64_t blk = get_block(pkt->LADDR);
                    DataBlock tmp_data;
                    tmp_data.setSize(cline_bytes);
                    for (uint64_t i=0; i<width_block; i++)
                    {
                        tmp_data.setByte(blk*width_block+i, pkt->buffer_data.getByte(i));
                        pkt->byte_enable.push_back(blk*width_block+i);
                    }
                    pkt->buffer_data = tmp_data;
                }

                missq.push_back(pkt);
                status = NO_STALL;

                PCMC_DBG(dbg_msg, "[GPC] Req [0x%lx, CMD=%c, ID=%lx] bypassed\n",
                    pkt->LADDR, (pkt->cmd==CMD_READ)? 'R':'W', pkt->req_id);
            }
            else
            {
                status = WAIT_MISSQ;
                pipe_access->prgm_stall = true;
            }
        }
        else if (size_mshr==0)          // MSHR is OFF
        {
            if (pb_signals->tmp_way>=0) // hit on RSVD $line
            {
                pipe_access->prgm_stall = true;
                status = HIT_RSVD;
            }
            else                        // totally miss!
                status = gpc_alloc(pkt);
        }
        else
        {
            /* Wait for availability in MSHR */
            pipe_access->prgm_stall = true;
        }
    }
    else if (status==MSHR_SUBENTRY_FULL) // MSHR hit but sub-entry FULL
        pipe_access->prgm_stall = true;
}

int GPCache::gpc_alloc(Packet* pkt)
{
    /* Check if miss is able to be handled in this cycle */
    assert(num_sets>0); 
    int nxt_status = NO_STALL;
    uint64_t s = (num_sets==0)? 0 : get_set_idx(pkt->LADDR);
    int64_t w = (num_sets==0)? -1 : victim_policy->get_victim(s); 

    if (w>=0 &&
        ((write_alloc && missq.size( )<size_missq-3) ||         // ev+wr+rd
         (write_alloc==false && missq.size( )<size_missq-2)))   // ev+rd
    {
        /* Check dirtiness */
        if (tmem[s][w].dirty)
        {
            Packet* ev_pkt = new Packet( );
            ev_pkt->cmd = CMD_WRITE;
            ev_pkt->owner = this;
            ev_pkt->isDATA = true;
            ev_pkt->LADDR = (tmem[s][w].tag<<(cline_bits+bit_sets)) | (s<<cline_bits);
            ev_pkt->req_id = pkt->req_id;
            ev_pkt->buffer_data.setSize(cline_bytes);
            for (uint64_t i = 0; i < cline_bytes; i++)
                ev_pkt->buffer_data.setByte(i, dmem[s][w].getByte(i));

            missq.push_back(ev_pkt);
            tmem[s][w].dirty = false;
            
            /* Stats update */
            num_evct+=1;
            Erd+=Erdpa;
        }

        /* Allocate cache block */
        tmem[s][w].tag = get_tag(pkt->LADDR);
        tmem[s][w].valid = true;
        victim_policy->set_rsvd(s, w);
        victim_policy->update_victim(s, w);

        /* Link way # and MSHR entry */
        uint64_t pkt_addr = pkt->LADDR >> cline_bits;
        std::set<mshr_t*>::iterator it = mshr.begin( );
        for ( ; it!=mshr.end( ); it++)
        {
            uint64_t ref_addr = (*it)->addr;
            if (ref_addr==pkt_addr)
            {
                if (num_blocks==1 && pkt->cmd==CMD_WRITE)
                {
                    /* Delete allocated MSHR as if nothing happens */
                    assert((*it)->subentry.size( )==0);
                    delete (*it);
                    mshr.erase(it);
                }
                else
                    (*it)->cline_way = w;
                break;
            }
        }

        /* Push CMD to missq */
        if (pkt->cmd==CMD_READ)
            missq.push_back(pkt);
        else if (num_blocks==1)
        {
            /* Case of $line-size==wdata-size (perf. gain>30%) */
            fill_line(s, w, pkt);
            sendParentResp(pkt);
            
            victim_policy->unset_rsvd(s, w);
        }
        else
        {
            assert(write_alloc);
            Packet* rd_pkt = new Packet( );
            *rd_pkt = *pkt;
            rd_pkt->cmd = CMD_READ;
            rd_pkt->owner = this;

            /* Mask partial data */
            uint64_t blk = get_block(pkt->LADDR);
            DataBlock tmp_data;
            tmp_data.setSize(cline_bytes);
            for (uint64_t i=0; i<width_block; i++)
            {
                tmp_data.setByte(blk*width_block+i, pkt->buffer_data.getByte(i));
                pkt->byte_enable.push_back(blk*width_block+i);
            }
            pkt->buffer_data = tmp_data;

            /* Partial write-after-read in LLM */
            missq.push_back(pkt);       // write new data
            missq.push_back(rd_pkt);    // get updated data
        }
    }
    else
    {
        pipe_access->prgm_stall = true;
        nxt_status = TMEM_ALLOC_FAIL;
    }
    
    PCMC_DBG(dbg_msg, "[GPC] Req [0x%lx, CMD=%c, ID=%lx] ",
        pkt->LADDR, (pkt->cmd==CMD_READ)? 'R':'W', pkt->req_id);
    if (nxt_status==NO_STALL)
        PCMC_DBG(dbg_msg, "gets into missq which replaced "
            "(set=%lu, way=%ld)\n", s, w);
    else 
        PCMC_DBG(dbg_msg, "reserv fails on set=%lu, reason: %s\n",
            s, (w>=0)? "MISSQ-overflow":"NO-WAY");

    return nxt_status;
}

bool GPCache::isRespPortAvailable( )
{
    bool rv = true;
    if ((geq->getCurrentTick( )<free_resp ||
         fillq.empty( )==false)) // may incur cycle in-accuracy if this func exec first, but okay
        rv = false;
    return rv;
}

bool GPCache::isWritePortAvailable( )
{
    bool rv = true;
    if (fillq.empty( )==false)
        rv = false;
    return rv;
}
void GPCache::hit_wdata(Packet* pkt, int64_t hit_way)
{
    /* write dmem on cache hit */
    assert(isWritePortAvailable( ) && hit_way>=0);

    uint64_t s = get_set_idx(pkt->LADDR);
    fill_line(s, hit_way, pkt);
    sendParentResp(pkt); 
}

void GPCache::stall_handle(Packet* pkt, int64_t hit_way)
{
    assert(pkt);

    if (status==MSHR_ALLOC_FAIL)
    {
        PCMC_DBG(dbg_msg, "[GPC-stall] re-try MSHR & cache alloc for "
            "[0x%lx, ID=%lx, CMD=%c]\n", pkt->LADDR, pkt->req_id,
            (pkt->cmd==CMD_READ)? 'R':'W');
        
        miss_handle(pkt);
    }
    else if (status==WAIT_MISSQ)
    {
        if (missq.size( )<size_missq)
        {
            /* Mask partial data */
            if (pkt->cmd==CMD_WRITE &&
                num_blocks!=1 && 
                pkt->buffer_data.getSize( )==width_block)
            {
                uint64_t blk = get_block(pkt->LADDR);
                DataBlock tmp_data;
                tmp_data.setSize(cline_bytes);
                for (uint64_t i=0; i<width_block; i++)
                {
                    tmp_data.setByte(blk*width_block+i, pkt->buffer_data.getByte(i));
                    pkt->byte_enable.push_back(blk*width_block+i);
                }
                pkt->buffer_data = tmp_data;
            }

            missq.push_back(pkt);
            status = NO_STALL;

            PCMC_DBG(dbg_msg, "[GPC] Req [0x%lx, CMD=%c, ID=%lx] bypassed\n",
                pkt->LADDR, (pkt->cmd==CMD_READ)? 'R':'W', pkt->req_id);
        }
    }
    else if (status==TMEM_ALLOC_FAIL)
    {
        PCMC_DBG(dbg_msg, "[GPC-stall] re-try cache alloc for "
            "[0x%lx, ID=%lx, CMD=%c]\n", pkt->LADDR, pkt->req_id,
            (pkt->cmd==CMD_READ)? 'R':'W');

        status = gpc_alloc(pkt);
    }
    else if (status==HIT_RSVD || status==MSHR_SUBENTRY_FULL)
    {
        uint64_t s = get_set_idx(pkt->LADDR);
        int64_t w = hit_way;
        assert(w>=0);
        if (victim_policy->get_rsvd(s, w)==false)
        {
            PCMC_DBG(dbg_msg, "[GPC-stall] check rsvd entry!\n");
            hit_handle(pkt, hit_way);
        }
    }
    else if (status==WAIT_RDRESP)
    {
        if (isRespPortAvailable( ))
        {
            PCMC_DBG(dbg_msg, "[GPC-stall] RD-resp port is free for "
                "RD-HIT [0x%lx, ID=%lx, CMD=R]\n", pkt->LADDR, pkt->req_id);
            
            sendParentResp(pkt, tRESP);
            free_resp = geq->getCurrentTick( )+tRESP*ticks_per_cycle;

            status = NO_STALL;

            /* Stats update */
            ncycle_t tmp_wait_rdresp = geq->getCurrentTick( )-trigger_wait_rdresp;
            if (tmp_wait_rdresp>max_wait_rdresp)
                max_wait_rdresp = tmp_wait_rdresp;
            if (tmp_wait_rdresp<min_wait_rdresp)
                min_wait_rdresp = tmp_wait_rdresp;

            avg_wait_rdresp = (avg_wait_rdresp*num_wait_rdresp+tmp_wait_rdresp)
                                /(double)(num_wait_rdresp+1);
            num_wait_rdresp += 1;
        }
    }
    else if (status==RACE_HITWR_FILL)
    {
        if (isWritePortAvailable( ))
        {
            PCMC_DBG(dbg_msg, "[GPC-stall] DMEM write port is free for "
                "WR-HIT [0x%lx, ID=%lx, CMD=W]\n", pkt->LADDR, pkt->req_id);
            
            hit_wdata(pkt, hit_way);
            status = NO_STALL;
        }
    }
    else
        assert(0);
}

void GPCache::cycle_missq( )
{
    assert(missq.empty( )==false);
    
    /* Dispatch if low-level memory is available */
    bool isIssued = false;
    ncycle_t LAT = 1;
    Packet* pkt = missq.front( );
    pkt->from = this;
    if (child->isReady(pkt))
    {
        LAT = (pkt->cmd==CMD_READ)? tLLM_RD : 
            ((pkt->buffer_data.getSize( )!=cline_bytes)? tLLM_RD:tLLM_WR); //XXX:partial write?

        child->recvRequest(pkt, LAT);
        missq.pop_front( );
        isIssued = true;
    }

    last_wake_missq = wake_missq;
    free_req = (!isIssued)? free_req : (wake_missq+LAT*ticks_per_cycle);
    if (missq.empty( )==false)
    {
        wake_missq = geq->getCurrentTick( )+LAT*ticks_per_cycle;
        registerCallback((CallbackPtr)&GPCache::cycle_missq, LAT);
    }

    if (isIssued)
    {
        PCMC_DBG(dbg_msg, "[GPC] Issue miss req [0x%lx, ID=%lx, CMD=%c] (missq=%lu), ",
            pkt->LADDR, pkt->req_id, (pkt->cmd==CMD_READ)? 'R':'W', missq.size( ));
        if (pkt->owner==this)
            PCMC_DBG(dbg_msg, "%s to LLM\n", (pkt->cmd==CMD_READ)?
                "RD-ALLOC by WR-MISS":"eviction-WR");
        else
            PCMC_DBG(dbg_msg, "%s to LLM\n", (pkt->cmd==CMD_READ)?
                "RD-ALLOC":"partial-WR");
    }
}

void GPCache::cycle_fillq( )
{
    assert(fillq.empty( )==false);

    /* Bypass mode case */
    if (num_sets==0 || fillq.front( )->is_prefetch)
    {
        Packet* proc_pkt = fillq.front( );
        assert(proc_pkt->owner!=this && proc_pkt->cmd==CMD_READ);
        uint64_t blk = get_block(proc_pkt->LADDR);
        
        DataBlock tmp_data = proc_pkt->buffer_data;
        proc_pkt->buffer_data.setSize(width_block);
        for (uint64_t i = 0; i < width_block; i++)
            proc_pkt->buffer_data.setByte(i, tmp_data.getByte(blk*width_block+i));

        sendParentResp(proc_pkt, tRESP);
        fillq.pop_front( );

        /* Self schedule if fillq is not empty */
        last_wake_fillq = wake_fillq;
        ncycle_t LAT = tRESP;
        free_resp = wake_fillq+tRESP*ticks_per_cycle;
        if (fillq.empty( )==false)
        {
            wake_fillq = geq->getCurrentTick( )+LAT*ticks_per_cycle;
            registerCallback((CallbackPtr)&GPCache::cycle_fillq, LAT);
        }
        return;
    }

    /* 
     * Respond data cycle-by-cycle 
     * For write-only $: 
       > w=-1 is possible (MSHR: read & no matched entry, Other: simple read)
       > MSHR can be served by the cmd that doesn't originally generate entry
         For example, same address cmds in fillq, earlier one is pure READ 
         later one is READ gen by WRITE that originally gens MSHR entry
     */
    Packet* proc_pkt = fillq.front( );
    uint64_t s = 0; int64_t w = -1;
    get_cache_position(s, w, proc_pkt);

    assert(write_only || (w>=0 && tmem[s][w].tag==get_tag(proc_pkt->LADDR)));

    /* Find MSHR for merged request & determine processing packet */
    uint64_t pkt_addr = proc_pkt->LADDR >> cline_bits;
    std::set<mshr_t*>::iterator it = mshr.begin( );
    for ( ; it!=mshr.end( ); it++)
    {
        uint64_t ref_addr = (*it)->addr;
        if (ref_addr==pkt_addr)
        {
            if ((*it)->cline_filled)
            {
                /* After cacheline fill, respond reqs in subentries */
                proc_pkt = (*it)->subentry.front( );
                (*it)->subentry.pop_front( );
            }
            else
            {
                /* 
                 * Merged MSHR requests requires reference on fillq, 
                 * request on fillq will be responded firstly and deleted,
                 * thus duplicate a dummy one on the fillq for further 
                 * processing of merged requests
                 */
                Packet* dummy = new Packet( );
                *dummy = *proc_pkt;
                dummy->owner = this;
                fillq.pop_front( );
                fillq.push_front(dummy);
                if (w<0)
                    w = (*it)->cline_way;
            }
            break;
        }
    }
    assert(size_mshr==0 || write_only || it!=mshr.end( ));

    /* Fill cache line */
    if (w>=0 && 
        (size_mshr==0 ||                                    // MSHR OFF or...
        (it!=mshr.end( ) && (*it)->cline_filled==false)))   // 1st req in MSHR
    {
        assert(proc_pkt->cmd==CMD_READ);

        PCMC_DBG(dbg_msg, "[GPC] Fill data [0x%lx, ID=%lx] in (set=%lu, way=%ld), "
            "caused by %s-MISS(MSHR=%s)\n", proc_pkt->LADDR, proc_pkt->req_id, s, w,
            (proc_pkt->owner==this)? "WR":"RD", 
            (it!=mshr.end( ) && (*it)->subentry.size( )==0)? "empty":"remained");

        fill_line(s, w, proc_pkt);
        if (size_mshr>0)
            (*it)->cline_filled = true;
    }
    else
        PCMC_DBG(dbg_msg, "[GPC] %s [0x%lx, ID=%lx] in (set=%lu, way=%ld), "
            "MSHR=%s\n", (proc_pkt->cmd==CMD_WRITE)? "Write data":"Respond data",
            proc_pkt->LADDR, proc_pkt->req_id, s, w,
            (it!=mshr.end( ) && (*it)->subentry.size( )==0)? "empty":"remained");

    /* Response process of requests */
    bool isHostRd = (proc_pkt->owner!=this && 
                     proc_pkt->cmd==CMD_READ)? true : false;
    if (proc_pkt->owner==this) // RD of WR-MISS
        delete proc_pkt;
    else if (proc_pkt->cmd==CMD_READ)
    {
        /* 
         * This read process can be implemented with registers 
         * So read energy caculation is skipped here
         */
        uint64_t blk = get_block(proc_pkt->LADDR);
        if (w>=0) 
        {
            /* General read data response (except for write-only) */
            proc_pkt->buffer_data.setSize(width_block);
            for (uint64_t i = 0; i < width_block; i++)
                proc_pkt->buffer_data.setByte(i, dmem[s][w].getByte(blk*width_block+i));
        }
        else 
        {
            /* Bypassed RD in write-only mode->direct response */
            assert(write_only);
            DataBlock got_data = proc_pkt->buffer_data;
            proc_pkt->buffer_data.setSize(width_block);
            for (uint64_t i = 0; i < width_block; i++)
                proc_pkt->buffer_data.setByte(i, got_data.getByte(blk*width_block+i));
        }
        sendParentResp(proc_pkt, tRESP);
    }
    else if (proc_pkt->cmd==CMD_WRITE)
    {
        /* Merged request in MSHR */
        assert(size_mshr>0 && proc_pkt->buffer_data.getSize( )==width_block);
        fill_line(s, w, proc_pkt);
        sendParentResp(proc_pkt);
    }
    else
        assert(0);
   
    /* No more merged req in MSHR in subentries */
    if (it==mshr.end( ) ||
        (*it)->subentry.size( )==0)
    {
        /* Validate reserved cache line if cmd is cached */
        if (w>=0)
        {
            victim_policy->unset_rsvd(s, w);
            victim_policy->update_victim(s, w);
        }

        /* If all merged reqs are processed, delete whole MSHR entry */
        if (it!=mshr.end( ))
        {
            delete (*it);
            mshr.erase(it);
        }

        /* If cmd is found in MSHR, fillq front will be dummy */
        if (it!=mshr.end( ) && fillq.front( )->owner==this)
            delete fillq.front( );
        fillq.pop_front( );
    }

    /* Self schedule if fillq is not empty */
    last_wake_fillq = wake_fillq;
    ncycle_t LAT = (isHostRd)? tRESP : 1;
    free_resp = (isHostRd)? (wake_fillq+tRESP*ticks_per_cycle):free_resp;
    if (fillq.empty( )==false)
    {
        wake_fillq = geq->getCurrentTick( )+LAT*ticks_per_cycle;
        registerCallback((CallbackPtr)&GPCache::cycle_fillq, LAT);
    }
}

void GPCache::sendParentResp(Packet* pkt, ncycle_t delay)
{
    pkt->from = this;
    parent->recvResponse(pkt, delay);
}

int GPCache::mshr_check(Packet* pkt)
{
    if (num_sets==0)
        return MSHR_ALLOC_FAIL;

    int nxt_status = NO_STALL;
    
    /* MSHR search */
    std::set<mshr_t*>::iterator it = mshr.begin( );
    uint64_t pkt_addr = pkt->LADDR >> cline_bits;
    uint64_t ref_addr = std::numeric_limits<uint64_t>::max( );
    for ( ; it!=mshr.end( ); it++)
    {
        ref_addr = (*it)->addr;
        if (ref_addr==pkt_addr)
            break;
    }

    if (it!=mshr.end( ))
    {
        /* Found, check if mergeable to MSHR */
        if ((*it)->subentry.size( )<size_subentry_mshr)
            (*it)->subentry.push_back(pkt);
        else // no more subentry - stall.
        {
            nxt_status = MSHR_SUBENTRY_FULL;
            mshr_subentry_full++;
        }

        mshr_hit++;        
        PCMC_DBG(dbg_msg, "[GPC] MSHR HIT by req [0x%lx, CMD=%c, ID=%lx], "
            "%s (ref=0x%lx vs. probe=0x%lx, mergeNum=%lu)\n", pkt->LADDR, 
            (pkt->cmd==CMD_READ)? 'R':'W', pkt->req_id,
            (nxt_status==HIT_RSVD)? "but unable to merge!":"and merged!",
            ref_addr, pkt_addr, (*it)->subentry.size( ));
    }
    else if ((write_only && pkt->cmd==CMD_WRITE) || // write-only & write cmd or
             (write_only==false &&                  // if write-only disabled and...
             (write_alloc || pkt->cmd==CMD_READ)))  // check whether write-alloc enabled
    {
        /* Allocate if not found in MSHR & allocable */
        if (mshr.size( )<size_mshr)
        {
            mshr_t* new_entry = new mshr_t;
            new_entry->cline_filled = false;
            new_entry->cline_way = -1;
            new_entry->addr = pkt->LADDR >> cline_bits;
            mshr.insert(new_entry);
            nxt_status = MSHR_ALLOC_NEW;

            mshr_miss++;
            PCMC_DBG(dbg_msg, "[GPC] MSHR MISS and newly allocate for req "
                "[0x%lx, CMD=%c, ID=%lx] (probe=0x%lx)\n", 
                pkt->LADDR, (pkt->cmd==CMD_READ)? 'R':'W', pkt->req_id, 
                pkt_addr);
        }
        else // MSHR is full
            nxt_status = MSHR_ALLOC_FAIL;
    }
    else
        nxt_status = MSHR_ALLOC_FAIL;

   return nxt_status;
}

void GPCache::fill_line(uint64_t s, int64_t w, Packet* pkt)
{
    assert(w>=0);
    if (pkt->cmd==CMD_WRITE)
        tmem[s][w].dirty = true;

    if (num_blocks!=1 && pkt->buffer_data.getSize( )==width_block)
    {
        /* Partial write */
        uint64_t blk = get_block(pkt->LADDR);
        for (uint64_t i = 0; i < width_block; i++)
            dmem[s][w].setByte(blk*width_block+i, pkt->buffer_data.getByte(i));
    
        Erd+=Erdpa;
    }
    else
        dmem[s][w] = pkt->buffer_data;

    Ewr+=Ewrpa;
}

uint64_t GPCache::get_set_idx(uint64_t addr)
{
    return ((addr >> cline_bits) % num_sets);
}

uint64_t GPCache::get_tag(uint64_t addr)
{
    return (addr >> (cline_bits+bit_sets));
}

uint64_t GPCache::get_block(uint64_t addr)
{
    return ((addr >> (cline_bits-byte_offset)) % num_blocks);
}

void GPCache::print_info( )
{
    if (num_sets==0)
    {
        std::cout << "[GPC] General-purpose cache, named as "
            << "\"" << cp_name << "\", is set as bypass mode" << std::endl;
    }
    else
    {
        std::cout << "[GPC] General-purpose cache, named as \"" << cp_name 
                  << "\", is architected as: \n"
                  << "Cache line size=" << cline_bytes << "B\n" 
                  << "Data Block size=" << width_block << "B\n"
                  << "Number of sets=" << num_sets << "\n"
                  << "Number of ways/set=" << num_ways << "/set\n"
                  << "Cache mode=" << ((write_only)? "WRITE-ONLY(enforce WRITE-ALLOC)":
                        ((write_alloc)? "READ-WRITE":"NO-WRITE-ALLOC(READ-ONLY)")) << "\n"
                  << "Miss queue size=" << size_missq << "\n"
                  << "MSHR size=" << size_mshr 
                  << "[SIZE-SUBENTRY=" << size_subentry_mshr << "]\n" << std::endl;
    }
}

/*========== Below is stats setting ==========*/
void GPCache::register_stats( )
{
    uint64_t u64_zero = 0;
    uint64_t u64_max = std::numeric_limits<uint64_t>::max( );
    double df_init = 0.0;
    RESET_STATS(read_hit, u64_zero); 
    RESET_STATS(read_miss, u64_zero); 
    RESET_STATS(write_hit, u64_zero); 
    RESET_STATS(write_miss, u64_zero);
    RESET_STATS(num_evct, u64_zero);
    RESET_STATS(num_wait_rdresp, u64_zero);
    RESET_STATS(num_wait_rsvd, u64_zero);
    RESET_STATS(mshr_hit, u64_zero);
    RESET_STATS(mshr_miss, u64_zero);
    RESET_STATS(mshr_subentry_full, u64_zero);

    trigger_wait_rdresp = u64_zero;
    RESET_STATS(max_wait_rdresp, u64_zero);
    RESET_STATS(min_wait_rdresp, u64_max);
    RESET_STATS(avg_wait_rdresp, df_init);

    RESET_STATS(Esr, df_init);
    RESET_STATS(Erd, df_init);
    RESET_STATS(Ewr, df_init);
    RESET_STATS(Etot, df_init);

    ADD_STATS(cp_name, read_hit);
    ADD_STATS(cp_name, read_miss);
    ADD_STATS(cp_name, write_hit);
    ADD_STATS(cp_name, write_miss);
    ADD_STATS(cp_name, read_miss_rate);
    ADD_STATS(cp_name, write_miss_rate);
    ADD_STATS(cp_name, overall_miss_rate);
    ADD_STATS(cp_name, mshr_hit);
    ADD_STATS(cp_name, mshr_miss);
    ADD_STATS(cp_name, mshr_subentry_full);
    ADD_STATS(cp_name, num_evct);
    ADD_STATS_N_UNIT(cp_name, num_wait_rsvd, "cycles");
    ADD_STATS_N_UNIT(cp_name, num_wait_rdresp, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_wait_rdresp, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_wait_rdresp, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_wait_rdresp, "cycles");
    ADD_STATS_N_UNIT(cp_name, Esr, "nJ");
    ADD_STATS_N_UNIT(cp_name, Erd, "nJ");
    ADD_STATS_N_UNIT(cp_name, Ewr, "nJ");
    ADD_STATS_N_UNIT(cp_name, Etot, "nJ");
}

void GPCache::calculate_stats( )
{
    if (num_wait_rdresp>0)
    {
        max_wait_rdresp /= ticks_per_cycle;
        min_wait_rdresp /= ticks_per_cycle;
        avg_wait_rdresp /= ticks_per_cycle;
    }

    if (read_hit+read_miss==0)
        read_miss_rate = 0.0;
    else
        read_miss_rate = (double)read_miss/(read_hit+read_miss);

    if (write_hit+write_miss==0)
        write_miss_rate = 0.0;
    else
        write_miss_rate = (double)write_miss/(write_hit+write_miss);

    if (write_hit+write_miss+read_hit+read_miss==0)
        overall_miss_rate = 0.0;
    else
        overall_miss_rate = (double)(read_miss+write_miss)/
                            (read_hit+read_miss+write_hit+write_miss);

    Etot = Esr+Erd+Ewr;
}

