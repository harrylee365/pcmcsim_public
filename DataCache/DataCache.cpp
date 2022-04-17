#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "base/PipeBufferv2.h"
#include "base/Stats.h"
#include "base/MemInfo.h"
#include "DataCache/DataCache.h"
#include "ReplacePolicy/RoundRobin/RoundRobin.h"
#include "ReplacePolicy/TrueLRU/TrueLRU.h"
#include "ReplacePolicy/PseudoLRU/PseudoLRU.h"
#include "TraceGen/TraceGen.h"

using namespace PCMCsim;

typedef struct _signals_t
{
    Packet* pkt;
    bool hit;
    int64_t tmp_way;
} signals_t;

DataCache::DataCache(MemoryControlSystem* memsys_, std::string cfg_header)
:Component(memsys_), recvr(NULL), need_wakeup(true), size_cmdq(32),
wake_resp_mux(0), last_wake_resp_mux(0), free_resp(0), status(NO_STALL)
{
    cp_name = cfg_header;
    dbg_msg = memsys->getParamBOOL(cp_name+".dbg_msg", false);
    ticks_per_cycle= memsys->getParamUINT64("global.ticks_per_cycle", 1);
    tCMD = memsys->getParamUINT64(cp_name+".tCMD", 1);

    HOST_TX_OFFSET = memsys->adec->HOST_TX_OFFSET;
    PAGE_OFFSET = memsys->adec->PAGE_OFFSET;
    HOST_TX_SIZE = memsys->info->HOST_TX_SIZE;
    PAGE_SIZE = memsys->info->PAGE_SIZE;

    /* Construct cache */
    num_cache = PAGE_SIZE / HOST_TX_SIZE;
    assert(num_cache > 0);
    size_cmdq = memsys->getParamUINT64(cp_name+".size_cmdq", 32);

    bool bypass = memsys->getParamBOOL(cp_name+".bypass", false);
    bit_sets = memsys->getParamUINT64(cp_name+".bit_sets", 8);
    num_sets = (bypass)? 0:(1<<bit_sets);
    num_ways = memsys->getParamUINT64(cp_name+".num_ways", 32);

    if (num_sets==1 && num_ways==1)
    {
        std::cerr << "[DC] Error! Invalid cache size, 1-set & 1-way" << std::endl;
        assert(0);
        exit(1);
    }

    cache.resize(num_cache);
    for (uint64_t c = 0; c < num_cache; c++)
    {
        cache[c].resize(num_sets);
        for (uint64_t s = 0; s < num_sets; s++)
        {
            DataBlock e_data;
            entry_t e_init {0, 0, 0, 0, e_data};
            cache[c][s].resize(num_ways, e_init);
        }
    }

    std::string got_policy = memsys->getParamSTR(cp_name+".policy");
    victim_policy.resize(num_cache);
    for (uint64_t c=0; c<num_cache; c++)
    {
        if (got_policy == "ROUND_ROBIN")
            victim_policy[c] = new RoundRobin(num_sets, num_ways);
        else if (got_policy == "TRUE_LRU")
            victim_policy[c] = new TrueLRU(num_sets, num_ways);
        else if (got_policy == "PSEUDO_LRU")
            victim_policy[c] = new PseudoLRU(num_sets, num_ways);
        else
        {
            std::cerr << "[DC] Error! Invalid replacement policy!" << std::endl;
            assert(0);
            exit(1);
        }
    }

    std::string got_walloc = memsys->getParamSTR(cp_name+".allocate_method", "WRITE_ALLOC");
    if (got_walloc == "WRITE_ALLOC" || got_walloc=="WRITE-ALLOC")
        write_alloc = true;
    else if (got_walloc == "NO_WRITE_ALLOC" || got_walloc=="NO-WRITE-ALLOC")
        write_alloc = false;
    else
        assert(0);

    write_only = memsys->getParamBOOL(cp_name+".write_only", true);
    if (write_only && write_alloc==false && num_sets>0)
    {
        std::cerr << "[DC] Allocation method is NO-WRITE-ALLOC, "
            << "whereas WRITE-ONLY cache is configured" << std::endl;
        assert(0);
        exit(1);
    }

    /* 
     * Note DCACHE, BLKMGR, RMW, uCMDE, DPU has common ID space 
     * Write ID should be conferred again in DCACHE, 
     * since ReqRecv's WID is retired after data comes to DCACHE
     */
    uint64_t buffer_size_bits = memsys->getParamUINT64("reqRecv.buffer_size_bits", 7);
    uint64_t num_wid = 1<<buffer_size_bits;
    for (uint64_t id=0; id<num_wid; id++)
        avail_WID.push_back((id|(1<<buffer_size_bits)));
    credit_WID = num_wid;

    /* Timing parameter setup */
    tBURST_HOST = memsys->getParamUINT64(cp_name+".tDATA_HOST", 2); // partial data
    tBURST_RMW = memsys->getParamUINT64(cp_name+"tDATA_MEM", 4);    // full data

    /* Setup pipeline buffers (static latency) */
    pipe_access = new PipeBufferv2( );

    /* Get energy parameters */
    memsys->getParamSRAM(num_sets, num_ways, &Esrpa, &Erdpa, &Ewrpa);
    Esr = new double[num_cache];
    Erd = new double[num_cache];
    Ewr = new double[num_cache];
    Etot = new double[num_cache];

    /* Stats registration */
    register_stats( );
}

DataCache::~DataCache( )
{
    delete pipe_access;

    delete [] Esr;
    delete [] Erd;
    delete [] Ewr;
    delete [] Etot;

    for (uint64_t c=0; c<num_cache; c++)
        delete victim_policy[c];
}

bool DataCache::isReady(Packet* /*pkt*/)
{
    bool rv = true;
    if (cmdq.size( )>=size_cmdq)
        rv = false;

    return rv;
}

void DataCache::recvRequest(Packet* pkt, ncycle_t delay)
{
    assert(pkt->src_id==SRC_HOST);
    cmdq.push(pkt);
    pkt->recvTick = geq->getCurrentTick( )+delay*ticks_per_cycle;

    if (need_wakeup)
    {
        need_wakeup = false;
        registerCallback((CallbackPtr)&DataCache::execute, delay);
    }
}

void DataCache::recvResponse(Packet* pkt, ncycle_t delay)
{
    if (pkt->cmd==CMD_WACK_ID)
    {
        assert(pkt->owner==this);
        WID_unmap(pkt);
    }
    else if (pkt->cmd==CMD_PKT_DEL)
    {
        assert(pkt->owner==this);
        delete pkt;
    }
    else if (pkt->cmd==CMD_WRITE && pkt->from==dynamic_cast<Component*>(rmw))
    {
        /* Ready to send eviction data to RMW */
        assert(status==WAIT_WDATA_SEND && evct_pkt==pkt);
        PCMC_DBG(dbg_msg, "[DC] Push evct-data of [0x%lx, ID=%lx] to RMW\n",
            evct_pkt->LADDR, evct_pkt->req_id);
        signals_t* pb_signals = 
            (signals_t*)(pipe_access->buffer[pipe_access->getRearIdx( )].signals);

        /* Clear dirty bit */
        if (num_sets>0 && 
            (write_alloc ||                 // write-alloc
             sub_status==SUB_RDMISS_EVICT)) // no-wr-alloc but RD miss
        {
            uint64_t cidx = get_cache_idx(evct_pkt->LADDR); 
            uint64_t sidx = get_set_idx(evct_pkt->LADDR); 
            cache[cidx][sidx][pb_signals->tmp_way].dirty = false;
        }

        /* Evct write data */
        evct_pkt->from = this;
        rmw->recvResponse(evct_pkt, tBURST_RMW);
        evct_pkt = NULL;

        if (sub_status==SUB_RDMISS_EVICT)
        {
            /* Data is just evicted */
            sub_status = SUB_NORMAL;
            pipe_access->dequeue_rear( );
            pipe_access->prgm_stall = false;
            status = NO_STALL;
            delete pb_signals;
        }
        else if (num_sets>0 && write_alloc)
        {
            /* Request data from ReqRecvr */
            status = req_wdata(pb_signals->pkt);
            pipe_access->prgm_stall = true;
        }
        else
        {
            pipe_access->dequeue_rear( );
            pipe_access->prgm_stall = false;
            status = NO_STALL;
            delete pb_signals;
        }
    }
    else
        Component::recvResponse(pkt, delay);
}

void DataCache::handle_events(ncycle_t curr_tick)
{
    /* Get events from queue */
    prepare_events(curr_tick);

    /* Don't change following order */
    if (!await_resp.empty( ))
        handle_await_resps( );

    if (!await_cb.empty( ))
        handle_await_callbacks( );
}

void DataCache::execute( )
{
    /* Execute main path sub-modules' functions */
    access_exec( );

    /* Advance pipeline buffers */
    cmdq_proceed( );
    pipe_access->proceed( );

    if (pipe_access->isEmpty( )==false ||
        cmdq.empty( )==false)
    {
        registerCallback((CallbackPtr)&DataCache::execute, 1);
    }
    else
        need_wakeup = true;
}

void DataCache::handle_await_resps( )
{
    /* Process packets from RMW and reqRecvr */
    std::list<LocalEvent*>::iterator e_it = await_resp.begin( );
    for ( ; e_it!=await_resp.end( ); )
    {
        Packet* pkt = (*e_it)->pkt;
        if (pkt->cmd==CMD_READ)
        {
            /* Respond read request from RMW */
            assert(pkt->from==dynamic_cast<Component*>(rmw));
            assert(pkt->buffer_data.getSize( )==PAGE_SIZE);
            fillq.push(pkt);

            PCMC_DBG(dbg_msg, "[DC] Ready to respond RD [0x%lx, ID=%lx] "
                "(qsize=%ld)\n", pkt->LADDR, pkt->req_id, fillq.size( ));
        }
        else if (pkt->cmd==CMD_WRITE)
        {
            assert(status==WAIT_WDATA_RECV);
            signals_t* pb_signals = 
                (signals_t*)(pipe_access->buffer[pipe_access->getRearIdx( )].signals);

            if (num_sets==0 ||              // bypass mode
                (write_alloc==false && 
                 pb_signals->tmp_way<0))    // no-write alloc & wr miss
            {
                /* Getting data & Bypass write miss request */
                recv_wdata(pkt); 
                if (aitm->isReady(evct_pkt))
                    status = issue_cmd(evct_pkt);
                else
                    status = WAIT_AITMGR;
            }
            else if (isWrPtAvailable( ))
            {
                /* Fill write hit data */
                recv_wdata(pkt, false, pb_signals->tmp_way);
                pipe_access->dequeue_rear( );
                pipe_access->prgm_stall = false;
                status = NO_STALL;
                delete pb_signals;
            }
            else
                status = RACE_HITWR_FILL;
        }
        else
            assert(0);

        /* Free event */
        delete (*e_it);
        e_it = await_resp.erase(e_it);
    }

    /* Schedule response MUX event */
    if (last_wake_resp_mux!=wake_resp_mux)
        return;
    else if (fillq.empty( )==false)
    {
        wake_resp_mux = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_resp_mux = std::max(wake_resp_mux, free_resp);
        registerCallbackAt((CallbackPtr)&DataCache::cycle_fillq, wake_resp_mux);
    }
}

void DataCache::cmdq_proceed( )
{
    /* Check whether cmdq is ready to pop */
    bool pop = false;
    static uint64_t WID_LOW_BOUND = 2;

    if (cmdq.size( )>0)
    {
        if (write_alloc)
        {
            if (write_only)
            {
                if (cmdq.front( )->cmd==CMD_READ || 
                    credit_WID>WID_LOW_BOUND)
                    pop = true;
            }
            else if (credit_WID>WID_LOW_BOUND)
                pop = true;
        }
        else if (credit_WID>WID_LOW_BOUND)
            pop = true;
    }

    /* Get a command from the cmd queue */
    if (pipe_access->isAcceptable( ) && pop) 
    {
        Packet* pkt = cmdq.front( );
        cmdq.pop( );

        signals_t* new_pb_signals = new signals_t;
        new_pb_signals->pkt = pkt;
        new_pb_signals->hit = false;
        new_pb_signals->tmp_way = -1; 
        pipe_access->input = new_pb_signals;
        
        PCMC_DBG(dbg_msg, "[DC] Read tag for [0x%lx, ID=%lx, %s] (credit=%ld)\n",
            pkt->LADDR, pkt->req_id, (pkt->cmd==CMD_READ)? "RD":"WR", credit_WID);
    }
}

void DataCache::access_exec( )
{
    int rear = pipe_access->getRearIdx( );
    if (pipe_access->isEmpty( )==false && pipe_access->buffer[rear].stage==0)
    {
        /* Check cache hit / miss */
        signals_t* pb_signals = (signals_t*)(pipe_access->buffer[rear].signals);
        Packet* proc_pkt = pb_signals->pkt;
        if (num_sets>0)
        {
            uint64_t cidx = 0; uint64_t sidx = 0;
            get_cache_position(cidx, sidx, pb_signals->tmp_way, proc_pkt);
            if (pb_signals->tmp_way>=0 &&
                victim_policy[cidx]->get_rsvd(sidx, pb_signals->tmp_way)==false)
            {
                pb_signals->hit = true;
            }
            
            PCMC_DBG(dbg_msg, "[DC] CAM check %s-%s on [0x%lx, ID=%lx] rsvd=%s\n", 
                (proc_pkt->cmd==CMD_READ)? "RD":"WR", 
                (pb_signals->tmp_way>=0)? "HIT":"MISS", 
                proc_pkt->LADDR, proc_pkt->req_id, 
                (pb_signals->tmp_way>=0 && pb_signals->hit==false)? "true":"false");
        }

        handle_hitmiss(pb_signals->hit, pb_signals->tmp_way, proc_pkt);

        if (status==NO_STALL)
        {
            pipe_access->dequeue_rear( );
            pipe_access->prgm_stall = false;
            status = NO_STALL;
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
            status = NO_STALL;
            delete pb_signals;
        }
    }

    /* Schedule response MUX event */
    if (last_wake_resp_mux==wake_resp_mux && fillq.empty( )==false)
    {
        wake_resp_mux = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_resp_mux = std::max(wake_resp_mux, free_resp);
        registerCallbackAt((CallbackPtr)&DataCache::cycle_fillq, wake_resp_mux);
    }
}

void DataCache::get_cache_position(uint64_t& cidx, uint64_t& sidx, int64_t& widx, Packet* pkt)
{
    cidx = get_cache_idx(pkt->LADDR);
    sidx = get_set_idx(pkt->LADDR);
    uint64_t tag = get_tag(pkt->LADDR);
    for (uint64_t i = 0; i < num_ways; i++)
    {
        if (cache[cidx][sidx][i].valid==false) continue;

        if (cache[cidx][sidx][i].tag==tag)
        {
            widx = (int64_t)i;
            break;
        }
    }
    
    Esr[cidx]+=Esrpa;
}

void DataCache::handle_hitmiss(bool hit, int64_t hit_way, Packet* pkt)
{
    assert(status==NO_STALL);

    /* Hit/miss handling */
    if (num_sets==0)    // bypass mode
        miss_handle(pkt);
    else if (hit==false)
    {
        if (hit_way<0)  // miss
            miss_handle(pkt);
        else            // hit but data not arrive
        {
            pipe_access->prgm_stall = true;
            status = HIT_RSVD;
        }
    }
    else                // hit
        hit_handle(pkt, hit_way);

    /* Stats update */
    if (hit)
    {
        if (pkt->cmd==CMD_READ)
            read_hit+=1;
        else
            write_hit+=1;
        if (status==HIT_RSVD)
            num_wait_rsvd+=1;
    }
    else if (pkt->cmd==CMD_READ)
        read_miss+=1;
    else
        write_miss+=1;
}

void DataCache::hit_handle(Packet* pkt, int64_t hit_way)
{
    assert(hit_way>=0);
    uint64_t cidx = get_cache_idx(pkt->LADDR);
    uint64_t s = get_set_idx(pkt->LADDR);
    int64_t w = hit_way;
    int nxt_status = NO_STALL;

    if (pkt->cmd==CMD_READ)
    {
        /* Direct service with data */
        pkt->buffer_data = cache[cidx][s][w].data;
        assert(pkt->buffer_data.getSize( )>0);
        if (isRespAvailable( ))
        {
            rdhit_resp(pkt);
            free_resp = geq->getCurrentTick( )+tBURST_HOST*ticks_per_cycle;
        }
        else
        {
            pipe_access->prgm_stall = true;
            nxt_status = WAIT_RDRESP;

            trigger_wait_rdresp = geq->getCurrentTick( );
        }

        Erd[cidx]+=Erdpa;

        PCMC_DBG(dbg_msg, "[DC] RD-resp port is %s to RD-HIT [0x%lx, ID=%lx]\n", 
            (nxt_status==WAIT_RDRESP)? "not free":"free", pkt->LADDR, pkt->req_id);
    }
    else if (pkt->cmd==CMD_WRITE)
    {
        /* Request write data from ReqRecvr and stall the pipe */
        nxt_status = req_wdata(pkt);
        pipe_access->prgm_stall = true;
    }
    else
        assert(0);
    
    victim_policy[cidx]->update_victim(s, w);
    status = nxt_status;
}

void DataCache::rdhit_resp(Packet* pkt)
{
    pkt->from = this;
    recvr->recvResponse(pkt, tBURST_HOST);

    /* Stats update */
    ncycle_t tmp_lat = geq->getCurrentTick( )-pkt->recvTick;
    if (max_rdhit_lat<tmp_lat)
        max_rdhit_lat = tmp_lat;
    if (min_rdhit_lat>tmp_lat)
        min_rdhit_lat = tmp_lat;

    avg_rdhit_lat = (avg_rdhit_lat*num_rdhit_resp+tmp_lat)/(double)(num_rdhit_resp+1);
    num_rdhit_resp += 1;
}

void DataCache::miss_handle(Packet* pkt)
{
    /* Miss handling */
    status = dcache_alloc(pkt);
}

void DataCache::stall_handle(Packet* pkt, int64_t hit_way)
{
    assert(pkt);
    if (status==HIT_RSVD)
    {
        assert(hit_way>=0);
        uint64_t c = get_cache_idx(pkt->LADDR);
        uint64_t s = get_set_idx(pkt->LADDR);
        int64_t w = hit_way;
        if (victim_policy[c]->get_rsvd(s, w)==false)
            hit_handle(pkt, hit_way);
    }
    else if (status==WAIT_AITMGR)
    {
        /* Issue command to address mapper if it's ready */
        Packet* issue_pkt = NULL;
        if (evct_pkt==NULL || sub_status==SUB_RDMISS_ISSUE)
            issue_pkt = pkt;
        else if (sub_status==SUB_NORMAL || sub_status==SUB_RDMISS_EVICT)
            issue_pkt = evct_pkt;
        else
            assert(0);

        if (aitm->isReady(issue_pkt))
        {
            status = issue_cmd(issue_pkt);

            PCMC_DBG(dbg_msg, "[DC] AITManager is ready now to "
                "%s [0x%lx, ID=%lx, CMD=%c]\n", (evct_pkt)? "evct":"issue MISS",
                issue_pkt->LADDR, issue_pkt->req_id, 
                (issue_pkt->cmd==CMD_READ)? 'R':'W');
        }
    }
    else if (status==WAIT_RDRESP)
    {
        /* Try to respond the directly served-cmd to ReqRecvr */
        if (isRespAvailable( ))
        {
            rdhit_resp(pkt);
            free_resp = geq->getCurrentTick( )+tBURST_HOST*ticks_per_cycle;
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

            PCMC_DBG(dbg_msg, "[DC] RD-resp port is free now for [0x"
                "%lx, ID=%lx]\n", pkt->LADDR, pkt->req_id);
        }
    }
    else if (status==RACE_HITWR_FILL)
    {
        if (isWrPtAvailable( ))
        {
            /* Fill write hit data */
            recv_wdata(pkt, false, hit_way);
            status = NO_STALL;
        }
    }
    else if (status==ALLOC_FAIL)
    {
        PCMC_DBG(dbg_msg, "[DC] re-try cache alloc for "
            "[0x%lx, ID=%lx, CMD=%c]\n", pkt->LADDR, pkt->req_id,
            (pkt->cmd==CMD_READ)? 'R':'W');

        status = dcache_alloc(pkt);
    }
    else if (status==WAIT_WDATA_RECV || status==WAIT_WDATA_SEND)
    {
        /* Just wait for wdata coming from ReqRecvr or ack from RMW */
    }
    else
        assert(0);
}

void DataCache::gen_evct(uint64_t addr, DataBlock& evct_data)
{
    assert(evct_pkt==NULL && addr!=INVALID_ADDR);
    uint64_t cidx = get_cache_idx(addr);

    /* Generate eviction request */
    evct_pkt = new Packet(PAGE_SIZE/HOST_TX_SIZE);
    evct_pkt->cmd = CMD_WRITE;
    evct_pkt->owner = this;
    evct_pkt->LADDR = addr;
    evct_pkt->dvalid[cidx] = true;
    evct_pkt->src_id = SRC_HOST;
    evct_pkt->recvTick_dcache = geq->getCurrentTick( );
    
    /* Prepare data for eviction right now for convenience */
    uint64_t base_byte = HOST_TX_SIZE*cidx;
    evct_pkt->buffer_data.setSize(PAGE_SIZE);
    for (uint64_t i=0; i<HOST_TX_SIZE; i++)
        evct_pkt->buffer_data.setByte(base_byte+i, evct_data.getByte(i));
    
    WID_map(evct_pkt); // Newly map WID
}

void DataCache::WID_map(Packet* pkt)
{
    assert(pkt->cmd==CMD_WRITE);
    uint64_t newID = 0;
    assert(!avail_WID.empty( ));
    newID = avail_WID.front( );
    avail_WID.pop_front( );
    alloc_WID.insert(newID);
    pkt->req_id = newID;

    credit_WID-=1;

    PCMC_DBG(dbg_msg, "[DC] Confer WID=%lx to evct-req [0x%lx, CMD=W]\n", 
        newID, pkt->LADDR);
}

void DataCache::WID_unmap(Packet* pkt)
{
    assert(pkt->cmd==CMD_WACK_ID && pkt->req_id>=0);
    uint64_t retire_WID = (uint64_t)pkt->req_id;

    avail_WID.push_back(retire_WID);
    alloc_WID.erase(retire_WID);

    credit_WID+=1;

    /* Stats update */
    ncycle_t tmp_lat = geq->getCurrentTick( )-pkt->recvTick_dcache;
    if (tmp_lat>max_persist_lat)
        max_persist_lat = tmp_lat;
    if (tmp_lat<min_persist_lat)
        min_persist_lat = tmp_lat;

    avg_persist_lat = (avg_persist_lat*num_persist+tmp_lat)/(double)(num_persist+1);
    num_persist+=1;

    PCMC_DBG(dbg_msg, "[DC] Retire WID=%lx of evct-req [0x%lx, CMD=W](credit=%ld)\n", 
        retire_WID, pkt->LADDR, avail_WID.size( ));
}

void DataCache::recv_wdata(Packet* pkt, bool wr_bypass, int64_t wr_hit_way)
{
    assert(pkt->from==dynamic_cast<Component*>(recvr));
    if (wr_bypass)
    {
        /* Generate evct_pkt for bypassing write miss request */
        gen_evct(pkt->LADDR, pkt->buffer_data);
        evct_pkt->recvTick = pkt->recvTick; // inherits tick for NO-WRITE-ALLOC 
    }
    else
    {
        uint64_t cidx = get_cache_idx(pkt->LADDR);
        uint64_t sidx = get_set_idx(pkt->LADDR);
        fill_line(cidx, sidx, wr_hit_way, pkt);
    }

    /* Stats update */
    ncycle_t tmp_lat = geq->getCurrentTick( )-pkt->recvTick;
    if (max_wrget_lat<tmp_lat)
        max_wrget_lat = tmp_lat;
    if (min_wrget_lat>tmp_lat)
        min_wrget_lat = tmp_lat;

    avg_wrget_lat = (avg_wrget_lat*num_wrget+tmp_lat)/(double)(num_wrget+1);
    num_wrget += 1;
    
    /* Virtual response */
    pkt->cmd = CMD_PKT_DEL;
    pkt->owner->recvResponse(pkt);
}

int DataCache::req_wdata(Packet* pkt)
{
    /* Request WDATA from ReqRecvr before issue */
    assert(pkt->cmd==CMD_WRITE);
    PCMC_DBG(dbg_msg, "[DC] Request wdata of [0x%lx, ID=%lx] "
        "from ReqRecv and wait\n", pkt->LADDR, pkt->req_id);
    
    pkt->buffer_idx = pkt->req_id;
    pkt->from = this;
    recvr->recvResponse(pkt, tCMD);

    return WAIT_WDATA_RECV;
}

int DataCache::issue_cmd(Packet* pkt)
{
    /* Issue miss command to AITManager */
    assert(aitm->isReady(pkt));
    int ret = NO_STALL;
    if (pkt->cmd==CMD_WRITE) // evction req
    {
        assert(pkt->owner==this || write_alloc==false || num_sets==0);
        ret = WAIT_WDATA_SEND;
    }
    else if (pkt->cmd==CMD_READ)
    {
        /* Issue evct command in next cycle */
        if (sub_status==SUB_RDMISS_ISSUE)
        {
            sub_status = SUB_RDMISS_EVICT;
            ret = WAIT_AITMGR;
        }

        /* Stats update */
        ncycle_t tmp_lat = geq->getCurrentTick( )-pkt->recvTick;
        if (max_rdmiss_lat<tmp_lat)
            max_rdmiss_lat = tmp_lat;
        if (min_rdmiss_lat>tmp_lat)
            min_rdmiss_lat = tmp_lat;

        avg_rdmiss_lat = (avg_rdmiss_lat*num_rdmiss_issue+tmp_lat)/
                    (double)(num_rdmiss_issue+1);
        num_rdmiss_issue += 1;
    }
    else
        assert(0);

    /* Stats update */
    ncycle_t tmp_lat = geq->getCurrentTick( )-pkt->recvTick;
    if (tmp_lat>max_issue_lat)
        max_issue_lat = tmp_lat;
    if (tmp_lat<min_issue_lat)
        min_issue_lat = tmp_lat;
    avg_issue_lat = 
        (avg_issue_lat*num_issue+tmp_lat)/(double)(num_issue+1);
    num_issue += 1;

    /* Mask packet address & issue */
    pkt->from = this;
    aitm->recvRequest(pkt, tCMD);

    return ret;
}

int DataCache::dcache_alloc(Packet* pkt)
{
    /* Ready to allocate an way by replacing victim */
    signals_t* pb_signals = 
        (signals_t*)(pipe_access->buffer[pipe_access->getRearIdx( )].signals);
    uint64_t cidx = (num_sets==0)? 0 : get_cache_idx(pkt->LADDR);
    uint64_t s = (num_sets==0)? 0 : get_set_idx(pkt->LADDR);
    int nxt_status = NO_STALL;
    
    if (num_sets==0 ||                                  // bypass mode
        (write_alloc==false && pkt->cmd==CMD_WRITE) ||  //!write-alloc & write
        (write_only && pkt->cmd==CMD_READ))             // write only & read
    {
        /* Bypass request to address mapper */
        if (pkt->cmd==CMD_WRITE)            // write
            nxt_status = req_wdata(pkt);
        else if (aitm->isReady(pkt))      // read
            nxt_status = issue_cmd(pkt);
        else                                // wait 
            nxt_status = WAIT_AITMGR;

        PCMC_DBG(dbg_msg, "[DC] Ready to bypass %s-MISS [0x%lx, ID=%lx] "
            "to PMEM-side %s\n", get_cmd_str(pkt).c_str( ), 
            pkt->LADDR, pkt->req_id, 
            (nxt_status==WAIT_AITMGR||nxt_status==WAIT_WDATA_RECV)? 
            ", but it's not ready":"");
    }
    else
    {
        /* Allocate if request is not bypassable */
        assert(write_alloc || pkt->cmd==CMD_READ);
        int64_t w = victim_policy[cidx]->get_victim(s);
        if (w<0)
        {
            pipe_access->prgm_stall = true;
            nxt_status = ALLOC_FAIL;
        }
        else 
        {
            PCMC_DBG(dbg_msg, "[DC] Reserve (%lu, %lu, %ld, dirty=%d) "
                "for Req [0x%lx, CMD=%s, ID=%lx]\n", cidx, s, w, 
                cache[cidx][s][w].dirty, pkt->LADDR, 
                get_cmd_str(pkt).c_str( ), pkt->req_id);

            if (cache[cidx][s][w].dirty)
            {
                /* Dirty line needs to be evicted to address mapper */
                uint64_t evct_LADDR = (cache[cidx][s][w].tag<<(PAGE_OFFSET+bit_sets)) |
                                    (s<<PAGE_OFFSET) | (cidx << HOST_TX_OFFSET);
                gen_evct(evct_LADDR, cache[cidx][s][w].data);

                /* RD miss is issued prior to evict & stall the pipe */
                sub_status = (pkt->cmd==CMD_READ)? SUB_RDMISS_ISSUE : SUB_NORMAL;
                Packet* issue_pkt = (pkt->cmd==CMD_READ)? pkt : evct_pkt;

                if (aitm->isReady(issue_pkt))
                    nxt_status = issue_cmd(issue_pkt);
                else
                    nxt_status = WAIT_AITMGR;
                assert(nxt_status!=NO_STALL);

                /* Stats update */
                num_evct+=1;
                Erd[cidx] = Erd[cidx]+Erdpa;

                PCMC_DBG(dbg_msg, "[DC] Ready to evct [0x%lx, ID=%lx] cause (c=%lu, "
                    "s=%lu, w=%ld) is dirty %s\n", evct_pkt->LADDR, evct_pkt->req_id,
                    cidx, s, w, (nxt_status==WAIT_AITMGR)? ", but it's not ready":"");
            }
            else
            {
                /* For clean cache line, W: req wdata; R: issue */
                if (pkt->cmd==CMD_WRITE)
                    nxt_status = req_wdata(pkt);
                else if (aitm->isReady(pkt))
                {
                    assert(pkt->cmd==CMD_READ);
                    nxt_status = issue_cmd(pkt);
                }
                else
                    nxt_status = WAIT_AITMGR;
            }

            cache[cidx][s][w].valid = true;
            cache[cidx][s][w].tag = get_tag(pkt->LADDR);
            victim_policy[cidx]->set_rsvd(s, w);
            pb_signals->tmp_way = w; // Note tag of evct_pkt is occupied by new address
        }
    }

    if (nxt_status!=NO_STALL)
        pipe_access->prgm_stall = true;

    return nxt_status;
}

bool DataCache::isRespAvailable( )
{
    bool rv = true;
    if (geq->getCurrentTick( )<free_resp || 
        fillq.empty( )==false)
        rv = false;
    return rv;
}

bool DataCache::isWrPtAvailable( )
{
    bool rv = true;
    if (write_only==false && fillq.empty( )==false)
        rv = false;
    return rv;
}

void DataCache::cycle_fillq( )
{
    /* MUX between fillq & stalled cmd that served by $ */
    assert(geq->getCurrentTick( )>=free_resp);
    assert(fillq.empty( )==false);

    Packet* pkt = fillq.front( );
    assert(pkt->buffer_data.getSize( )==PAGE_SIZE);
    
    uint64_t cidx = 0; uint64_t sidx = 0; int64_t widx = -1;
    if (num_sets>0)
        get_cache_position(cidx, sidx, widx, pkt);

    /* Process responded read data */
    bool responded = false;
    if (widx<0 ||                               // no $line is allocated
        cache[cidx][sidx][widx].dirty==false)   // dirty line has evcted
    {
        /* Truncate data as host desired size */
        DataBlock resp_data;
        resp_data.setSize(HOST_TX_SIZE);
        for (uint64_t i=0; i<HOST_TX_SIZE; i++)
            resp_data.setByte(i, pkt->buffer_data.getByte(cidx*HOST_TX_SIZE+i));
        pkt->buffer_data = resp_data;

        /* Fill line in $ that is non-bypass & non-write-only */
        if (write_only==false && widx>=0)
        {
            assert(victim_policy[cidx]->get_rsvd(sidx, widx));
            fill_line(cidx, sidx, widx, pkt);
        }
    
        /* Respond data to ReqRecvr */
        responded = true;
        pkt->from = this;
        recvr->recvResponse(pkt, tBURST_HOST);
        fillq.pop( );
    }

    last_wake_resp_mux = wake_resp_mux;
    ncycle_t LAT = (responded)? tBURST_HOST : 1;
    free_resp = (responded)? (wake_resp_mux+tBURST_HOST*ticks_per_cycle) : free_resp;
    if (fillq.empty( )==false)
    {
        wake_resp_mux = geq->getCurrentTick( )+LAT*ticks_per_cycle;
        registerCallback((CallbackPtr)&DataCache::cycle_fillq, LAT);
    }
}

void DataCache::fill_line(uint64_t cidx, uint64_t s, int64_t w, Packet* pkt)
{
    assert(w>=0);
    /* Fill the data */
    uint64_t tag = get_tag(pkt->LADDR);
    cache[cidx][s][w].data = pkt->buffer_data;
    cache[cidx][s][w].tag = tag;
    cache[cidx][s][w].valid = true;
    victim_policy[cidx]->unset_rsvd(s, w);
    victim_policy[cidx]->update_victim(s, w);

    /* Set dirty */    
    if (pkt->cmd==CMD_WRITE)
        cache[cidx][s][w].dirty = true; // direct write w/o updating==dirty
    
    PCMC_DBG(dbg_msg, "[DC] Fill data of [0x%lx, ID=%lx, CMD=%s] to "
        "(c=%lu, s=%lu, w=%ld, dirty=%d)\n", pkt->LADDR, pkt->req_id, 
        get_cmd_str(pkt->cmd).c_str( ), cidx, s, w, cache[cidx][s][w].dirty);
    
    Ewr[cidx]+=Ewrpa;
}

uint64_t DataCache::get_cache_idx(uint64_t addr)
{
    assert(num_cache > 0);
    uint64_t cidx = (addr >> HOST_TX_OFFSET) % num_cache;
    return cidx; 
}

uint64_t DataCache::get_set_idx(uint64_t addr)
{
    assert(num_sets > 0);
    uint64_t sidx = (addr >> PAGE_OFFSET) % num_sets;
    return sidx;
}

uint64_t DataCache::get_tag(uint64_t addr)
{
    uint64_t tag = addr >> (PAGE_OFFSET+bit_sets);
    return tag;
}

/*========== Below is stats setting ==========*/
void DataCache::register_stats( )
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
    RESET_STATS(num_persist, u64_zero);
    RESET_STATS(num_wait_rsvd, u64_zero);

    trigger_wait_rdresp = u64_zero;
    RESET_STATS(max_wait_rdresp, u64_zero);
    RESET_STATS(min_wait_rdresp, u64_max);
    RESET_STATS(avg_wait_rdresp, df_init);
    
    RESET_STATS(max_persist_lat, u64_zero);
    RESET_STATS(min_persist_lat, u64_max);
    RESET_STATS(avg_persist_lat, df_init);
    
    num_rdhit_resp = u64_zero;
    RESET_STATS(max_rdhit_lat, u64_zero);
    RESET_STATS(min_rdhit_lat, u64_max);
    RESET_STATS(avg_rdhit_lat, df_init);
    
    num_rdmiss_issue = u64_zero;
    RESET_STATS(max_rdmiss_lat, u64_zero);
    RESET_STATS(min_rdmiss_lat, u64_max);
    RESET_STATS(avg_rdmiss_lat, df_init);

    num_wrget = u64_zero;
    RESET_STATS(max_wrget_lat, u64_zero);
    RESET_STATS(min_wrget_lat, u64_max);
    RESET_STATS(avg_wrget_lat, df_init);
    
    num_issue = u64_zero;
    RESET_STATS(max_issue_lat, u64_zero);
    RESET_STATS(min_issue_lat, u64_max);
    RESET_STATS(avg_issue_lat, df_init);

    ADD_STATS(cp_name, read_hit);
    ADD_STATS(cp_name, read_miss);
    ADD_STATS(cp_name, write_hit);
    ADD_STATS(cp_name, write_miss);
    ADD_STATS(cp_name, read_miss_rate);
    ADD_STATS(cp_name, write_miss_rate);
    ADD_STATS(cp_name, overall_miss_rate);
    ADD_STATS(cp_name, num_evct);
    ADD_STATS(cp_name, num_persist);
    ADD_STATS(cp_name, num_wait_rsvd);
    ADD_STATS(cp_name, num_wait_rdresp);
    ADD_STATS_N_UNIT(cp_name, max_wait_rdresp, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_wait_rdresp, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_wait_rdresp, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_persist_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_persist_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_persist_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_rdhit_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_rdhit_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_rdhit_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_rdmiss_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_rdmiss_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_rdmiss_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_wrget_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_wrget_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_wrget_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_issue_lat, "cycles");

    memset(Esr, 0, num_cache*sizeof(double));
    memset(Erd, 0, num_cache*sizeof(double));
    memset(Ewr, 0, num_cache*sizeof(double));
    memset(Etot, 0, num_cache*sizeof(double));
    for (uint64_t i=0; i<num_cache; i++)
    {
        ADD_STATS_ITER_UNIT(cp_name, Esr, i, "nJ");
        ADD_STATS_ITER_UNIT(cp_name, Erd, i, "nJ");
        ADD_STATS_ITER_UNIT(cp_name, Ewr, i, "nJ");
        ADD_STATS_ITER_UNIT(cp_name, Etot, i, "nJ");
    }
}

void DataCache::calculate_stats( )
{
    if (num_wait_rdresp>0)
    {
        max_wait_rdresp /= ticks_per_cycle;
        min_wait_rdresp /= ticks_per_cycle;
        avg_wait_rdresp /= ticks_per_cycle;
    }

    if (num_persist>0)
    {
        max_persist_lat /= ticks_per_cycle;
        min_persist_lat /= ticks_per_cycle;
        avg_persist_lat /= ticks_per_cycle;
    }
    
    if (num_rdhit_resp>0)
    {
        max_rdhit_lat /= ticks_per_cycle;
        min_rdhit_lat /= ticks_per_cycle;
        avg_rdhit_lat /= ticks_per_cycle;
    }
    
    if (num_rdmiss_issue>0)
    {
        max_rdmiss_lat /= ticks_per_cycle;
        min_rdmiss_lat /= ticks_per_cycle;
        avg_rdmiss_lat /= ticks_per_cycle;
    }
    
    if (num_wrget>0)
    {
        max_wrget_lat /= ticks_per_cycle;
        min_wrget_lat /= ticks_per_cycle;
        avg_wrget_lat /= ticks_per_cycle;
    }

    if (num_issue)
    {
        max_issue_lat /= ticks_per_cycle;
        min_issue_lat /= ticks_per_cycle;
        avg_issue_lat /= ticks_per_cycle;
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

    for (uint64_t cidx=0; cidx<num_cache; cidx++)
        Etot[cidx] = Esr[cidx]+Erd[cidx]+Ewr[cidx];
}

