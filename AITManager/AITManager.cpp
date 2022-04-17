#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "base/Stats.h"
#include "base/MemInfo.h"
#include "base/PCMInfo.h"
#include "AITManager/AITManager.h"
#include "MemoryModules/GPCache/GPCache.h"
#include "TraceGen/TraceGen.h"

using namespace PCMCsim;

AITManager::AITManager(MemoryControlSystem* memsys_, std::string cfg_header)
:Component(memsys_), mcu(NULL), rmw(NULL), tcache(NULL), tdram(NULL),
wake_mcuq(0), last_wake_mcuq(0), proc_wlv(NULL), status_ait(ST_NORMAL), 
wake_mux(0), last_wake_mux(0), free_mux(0), wake_hostq(0), 
last_wake_hostq(0), wake_remapq(0), last_wake_remapq(0)
{
    cp_name = cfg_header;

    /* Parameter load */
    dbg_msg = memsys->getParamBOOL(cp_name+".dbg_msg", false);
    ticks_per_cycle= memsys->getParamUINT64("global.ticks_per_cycle", 1);
    tCMD = memsys->getParamUINT64(cp_name+".tCMD", 1);
    buffer_bits = memsys->getParamUINT64(cp_name+".buffer_bits", 6);
    size_buffer = 1 << buffer_bits;
    size_mcuq = memsys->getParamUINT64(cp_name+".size_mcuq", 2);
    size_ait_rdq = memsys->getParamUINT64(cp_name+".size_ait_rdq", 2);
    assert(size_ait_rdq>=2 && size_mcuq>=2 && size_buffer>0);

    /* Hard parameter */
    BLOCK_OFFSET = memsys->adec->BLOCK_OFFSET;
    PAGE_OFFSET = memsys->adec->PAGE_OFFSET;
    HOST_TX_SIZE = memsys->info->HOST_TX_SIZE;
    PAGE_SIZE = memsys->info->PAGE_SIZE;
    PAGE_PER_BLOCK_BIT = BLOCK_OFFSET - PAGE_OFFSET;
    size_ait_wrq = 3;
    size_remapq = 2 * (1<<PAGE_PER_BLOCK_BIT);
    num_avail_remap_slots = size_remapq;

    /* Data structure init */
    credit_buffer = size_buffer;
    bentry_t b_init{false, false, NULL};
    cmd_buffer.resize(size_buffer, b_init);
    for(uint64_t id=0 ; id<size_buffer; id++)
        avail_ID.push_back(id);

    /* Construct cache */
    tcache = new GPCache(memsys_, cp_name+".gpc");
    
    /* Followings are hardly fixed */
    gpc_byte_offset = memsys->getParamUINT64(cp_name+".gpc.byte_offset", 3);
    gpc_cline_bits = memsys->getParamUINT64(cp_name+".gpc.cline_bits", 6);
    width_block = 1 << gpc_byte_offset;
    num_blocks = 1 << (gpc_cline_bits-gpc_byte_offset);

    /* Stats registration */
    register_stats( );
}

AITManager::~AITManager()
{
    if (tcache)
        delete tcache;
}

bool AITManager::isReady(Packet* pkt)
{
    bool rv = true;

    if ((pkt->src_id==SRC_BLKMGR && mcuq.size( )>=size_mcuq) ||
        (pkt->src_id==SRC_HOST && 
         (credit_buffer==0 || ait_rdq.size( )>=size_ait_rdq)))
    {
        rv = false;
    }

    return rv;
}

void AITManager::recvRequest(Packet* pkt, ncycle_t delay)
{
    pkt->recvTick = geq->getCurrentTick( )+delay*ticks_per_cycle;
    if (pkt->src_id==SRC_HOST)
    {
        credit_buffer--;
        assert(credit_buffer<=size_buffer);

        registerCallback((CallbackPtr)&AITManager::ID_remap, delay,
                         0, reinterpret_cast<void*>(pkt));
    }
    else if (pkt->src_id==SRC_MCU)
    {
        assert(pkt->cmd==CMD_WLV);
        mcuq.push(pkt);

        if (last_wake_mcuq==wake_mcuq &&
            status_ait==ST_NORMAL)
        {
            assert(proc_wlv==NULL);
            wake_mcuq = geq->getCurrentTick( )+delay*ticks_per_cycle;
            registerCallback((CallbackPtr)&AITManager::cycle_mcuq, delay);
        }
    }
    else
        assert(0);
}

void AITManager::recvResponse(Packet* pkt, ncycle_t delay)
{
    if (pkt->cmd==CMD_PKT_DEL)
    {
        assert(pkt->owner==this);
        delete pkt;
    }
    else if (pkt->from==rmw)
    {
        assert(pkt->src_id==SRC_BLKMGR && pkt->cmd==CMD_WACK_ID);
        num_avail_remap_slots += 1;

        if (num_avail_remap_slots==size_remapq)
        {
            Packet* irq_ready = new Packet( ); 
            irq_ready->owner = this;
            irq_ready->from = this;
            irq_ready->cmd = CMD_IRQ;
            mcu->recvRequest(irq_ready, tCMD);
        }
    }
    else if (pkt->from==tcache)
    {
        /* Response of address get cmd */
        assert(pkt->cmd==CMD_READ);
        assert(pkt->buffer_data.getSize( )==width_block);
        Component::recvResponse(pkt, delay);
    }
    else
        assert(0);
}

void AITManager::handle_events(ncycle_t curr_tick)
{
    /* Don't change following order */
    prepare_events(curr_tick);
    
    if (!await_resp.empty( ))
        handle_await_resps( );

    if (!await_cb.empty( ))
        handle_await_callbacks( );
}

void AITManager::handle_await_resps( )
{
    std::list<LocalEvent*>::iterator e_it = await_resp.begin( );
    for ( ; e_it!=await_resp.end( ); )
    {
        Packet* pkt = (*e_it)->pkt;
        assert(pkt->src_id==SRC_HOST && pkt->from!=rmw);
        assert(hostq.size( )<size_buffer);

        /* Translate address & get original in cmd_buffer */
        Packet* origin = address_translate(pkt);
        hostq.push(origin);

        /* Calculate waited cycles in referencin AIT */
        ncycle_t tmp_wait_ait = geq->getCurrentTick( )-pkt->recvTick;
        if (tmp_wait_ait>max_wait_ait)
            max_wait_ait = tmp_wait_ait;
        if (tmp_wait_ait<min_wait_ait)
            min_wait_ait = tmp_wait_ait;

        avg_wait_ait = (avg_wait_ait*num_map+tmp_wait_ait)
                        /(double)(num_map+1);
        num_map+=1;

        PCMC_DBG(dbg_msg, "[AM] AIT reference cycles of "
            "[0x%lx, ID=%lx]: %lu-%lu=%lu ticks=>%lu cycles\n", 
            pkt->LADDR, pkt->req_id, geq->getCurrentTick( ), 
            pkt->recvTick, tmp_wait_ait, tmp_wait_ait/ticks_per_cycle);

        /* Free event */
        delete pkt;
        delete (*e_it);
        e_it = await_resp.erase(e_it);
    }

    if (last_wake_hostq==wake_hostq &&
        hostq.empty( )==false)
    {
        wake_hostq = geq->getCurrentTick( )+1*ticks_per_cycle;
        registerCallback((CallbackPtr)&AITManager::cycle_hostq, 1);
    }
}

void AITManager::cycle_mcuq( )
{
    assert(proc_wlv==NULL);
    proc_wlv = mcuq.front( );

    mcuq.pop( );
    status_ait = ST_FORBID_RD;
    last_wake_mcuq = wake_mcuq;

    check_inflight_ait_rd( );

    PCMC_DBG(dbg_msg, "[AM] A new BLKMGR CMD [0x%lx->0x%lx, %s] starts to process, "
        "stall AIT$ RD by HOST-REQ\n", proc_wlv->PADDR, proc_wlv->PADDR_MAP,
        (proc_wlv->swap)? "SWP":"MV");
}

void AITManager::ID_remap(Packet* pkt)
{
    assert(!avail_ID.empty( ));

    /* Allocate an inner ID */
    uint64_t newID = avail_ID.front( );
    assert(cmd_buffer[newID].valid==false);
    avail_ID.pop_front( );
    cmd_buffer[newID].valid = true;
    cmd_buffer[newID].pkt = pkt;

    /* 
     * Generate req to get AIT-entry from AIT$ 
     * We want to consider 8 consecutive data blocks as a cline
     * Thus, only left shift 3 bits
     */
    Packet* ait_pkt = new Packet( );
    ait_pkt->req_id = newID;
    ait_pkt->src_id = SRC_HOST;
    ait_pkt->cmd = CMD_READ;
    ait_pkt->owner = this;
    ait_pkt->from = this;
    ait_pkt->dest = tdram;
    ait_pkt->LADDR = (pkt->LADDR>>BLOCK_OFFSET) << gpc_byte_offset;
    ait_pkt->PADDR = ait_pkt->LADDR;
    init_ait_entry(ait_pkt->PADDR, ait_pkt->buffer_data);

    ait_pkt->recvTick = geq->getCurrentTick( );


    qentry_t new_entry{true, ait_pkt};
    ait_rdq.push(new_entry);

    PCMC_DBG(dbg_msg, "[AM] Confer tmp-ID=%lx to [0x%lx, ID=%lx, CMD=%c]"
        " to gen AIT$ RD [0x%lx]\n", newID, pkt->LADDR, pkt->req_id,
        (pkt->cmd==CMD_READ)? 'R':'W', ait_pkt->LADDR);
    
    /* Schedule to issue AIT$ cmd */
    if (last_wake_mux==wake_mux)
    {
        wake_mux = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_mux = std::max(wake_mux, free_mux);
        registerCallbackAt((CallbackPtr)&AITManager::ait_mux, wake_mux);
    }
}

void AITManager::init_ait_entry(uint64_t IDX_LINE, DataBlock& data)
{
    // IDX_LINE: AIT index in $-line address 
    // -- each address gets 8 entries
    uint64_t mem_addr = IDX_LINE>>gpc_cline_bits;
    data.setSize(1<<gpc_cline_bits);
    for (uint64_t b=0; b<num_blocks; b++)
    {
        /* Set initial AIT entry value */
        uint64_t dataFull = 0;
        for (uint64_t e=0; e<NUM_AIT_FLD; e++)
        {
            if (e==AIT_PBA)
            {
                uint64_t pba = (mem_addr<<gpc_byte_offset)|b;
                memsys->tdec->set_ait(dataFull, (ait_field_t)e, pba);
            }
            else
                memsys->tdec->set_ait(dataFull, (ait_field_t)e, 0);
        }

        /* Fill up data block */
        for (uint64_t i=0; i<width_block; i++)
        {
            uint8_t dataByte = (dataFull>>8*i) & 0xFF;
            data.setByte(b*width_block+i, dataByte);
        }
    }
}

void AITManager::get_gpc_info(uint64_t& line_offset, uint64_t& byte_offset)
{
    line_offset = gpc_cline_bits;
    byte_offset = gpc_byte_offset;
}

void AITManager::gen_blk_remap( )
{
    assert(ait_wrq.empty( ) && remapq.empty( ) && status_ait==ST_FORBID_RD);
    assert(proc_wlv->cmd==CMD_WLV);
    assert(num_avail_remap_slots==size_remapq);

    PCMC_DBG(dbg_msg, "[AM] Gen BLK and AIT$ WR reqs for [0x%lx->0x%lx, %s]\n",
        proc_wlv->PADDR, proc_wlv->PADDR_MAP, (proc_wlv->swap)? "SWP":"MV");

    /* Generate req to update AIT$ */
    for (uint64_t i=0; i<2; i++)
    {
        Packet* ref_pkt = new Packet( );
        uint64_t LBA = (i==0)? proc_wlv->LADDR : proc_wlv->LADDR_MAP;
        uint64_t e_ait = (i==0)? proc_wlv->PADDR_MAP : proc_wlv->PADDR; //TODO: BWCNT update
        ref_pkt->cmd = CMD_WRITE;
        ref_pkt->owner = this;
        ref_pkt->from = this;
        ref_pkt->dest = tdram;
        ref_pkt->isDATA = true;
        ref_pkt->src_id = SRC_BLKMGR;
        ref_pkt->req_id = i | (1<<buffer_bits);
        ref_pkt->LADDR = LBA << gpc_byte_offset;
        ref_pkt->PADDR = ref_pkt->LADDR;
        ref_pkt->buffer_data.setSize(width_block);
        for (uint64_t b=0; b<width_block; b++)
        {
            uint8_t newByte = (e_ait>>8*b)%8;
            ref_pkt->buffer_data.setByte(b, newByte);
        }

        qentry_t new_entry{true, ref_pkt};
        ait_wrq.push(new_entry);
        
        if (proc_wlv->swap==false)
            break;
    }
   
    /* Generate remap requests: (src0, dst0)...(src31, dst31) */
    uint64_t LBA = 0;
    uint64_t PBA = 0;
    uint64_t key = 0;
    uint64_t LPN = 0;
    uint64_t PPN = 0;
    uint64_t PBA_MAP = 0;
    uint64_t key_MAP = 0;
    uint64_t PPN_MAP = 0;
    for (uint64_t i=0; i<size_remapq; i++)
    {
        Packet* gen_pkt = new Packet(PAGE_SIZE/HOST_TX_SIZE);
        gen_pkt->cmd = CMD_WRITE;
        gen_pkt->owner = this;
        gen_pkt->from = this;
        gen_pkt->src_id = SRC_BLKMGR;
        gen_pkt->req_id = i;
        gen_pkt->swap = proc_wlv->swap;

        /* Assume KEY and BWCNT info is given by MCU */
        if (i%2==0 || proc_wlv->swap==false)
        {
            LBA = proc_wlv->LADDR;
            PBA = memsys->tdec->get_ait(proc_wlv->PADDR, AIT_PBA);
            PBA_MAP = memsys->tdec->get_ait(proc_wlv->PADDR_MAP, AIT_PBA);
            key = memsys->tdec->get_ait(proc_wlv->PADDR, AIT_KEY);
            key_MAP = memsys->tdec->get_ait(proc_wlv->PADDR_MAP, AIT_KEY);
        }
        else
        {
            LBA = proc_wlv->LADDR_MAP;
            PBA = memsys->tdec->get_ait(proc_wlv->PADDR_MAP, AIT_PBA);
            PBA_MAP = memsys->tdec->get_ait(proc_wlv->PADDR, AIT_PBA);
            key = memsys->tdec->get_ait(proc_wlv->PADDR_MAP, AIT_KEY);
            key_MAP = memsys->tdec->get_ait(proc_wlv->PADDR, AIT_KEY);
        }

        LPN = i/2;
        PPN = LPN^key;
        PPN_MAP = LPN^key_MAP;
        gen_pkt->LADDR = (LBA<<BLOCK_OFFSET) | (LPN<<PAGE_OFFSET);
        gen_pkt->PADDR = (PBA<<BLOCK_OFFSET) | (PPN<<PAGE_OFFSET);
        gen_pkt->PADDR_MAP = (PBA_MAP<<BLOCK_OFFSET) | (PPN_MAP<<PAGE_OFFSET);

        remapq.push(gen_pkt);
        num_avail_remap_slots -= 1;
    }
    
    /* Schedule to issue AIT$ cmd */
    if (last_wake_mux==wake_mux)
    {
        wake_mux = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_mux = std::max(wake_mux, free_mux);
        registerCallbackAt((CallbackPtr)&AITManager::ait_mux, wake_mux);
    }

    /* Schedule to issue remap req to RMW */
    if (last_wake_remapq==wake_remapq)
    {
        wake_remapq = geq->getCurrentTick( )+1*ticks_per_cycle;
        registerCallback((CallbackPtr)&AITManager::cycle_remapq, 1);
    }
}

void AITManager::check_inflight_ait_rd( )
{
    /* Check if no more HOST req is in AIT$ to generate remap req */
    assert(status_ait==ST_FORBID_RD);
    bool inflight_rd = false;
    for (uint64_t i=0; i<size_buffer; i++)
    {
        if (cmd_buffer[i].pkt && 
            cmd_buffer[i].inflight_ait_rd)
        {
            inflight_rd = true;
            break;
        }
    }

    if (inflight_rd==false)
    {
        registerCallback((CallbackPtr)&AITManager::gen_blk_remap, 1);
        PCMC_DBG(dbg_msg, "[AM] No AIT$ RD to be flushed, able to start BLK req gen\n");
    }
}

void AITManager::check_mcu_resp( )
{
    /* Check whether next BLK-REMAP is able to process or not */
    if (proc_wlv && 
        rmw->isReady(proc_wlv) &&
        ait_wrq.empty( ) && remapq.empty( ))
    {
        PCMC_DBG(dbg_msg, "[AM] All AIT$ WR and BLK reqs of [0x%lx->0x%lx, %s]"
            " are flushed, preapre to restart AIT$ RD\n", proc_wlv->PADDR,
            proc_wlv->PADDR_MAP, (proc_wlv->swap)? "SWP":"MV");

        proc_wlv->cmd = CMD_PKT_DEL;
        proc_wlv->owner->recvResponse(proc_wlv);
        
        proc_wlv = NULL;
        status_ait = ST_NORMAL;

        /* Schedule to issue AIT$ cmd */
        if (last_wake_mux==wake_mux &&
            ait_rdq.empty( )==false)
        {
            wake_mux = geq->getCurrentTick( )+1*ticks_per_cycle;
            wake_mux = std::max(wake_mux, free_mux);
            registerCallbackAt((CallbackPtr)&AITManager::ait_mux, wake_mux);
        }
    }

    if (last_wake_mcuq==wake_mcuq && 
        status_ait==ST_NORMAL && 
        mcuq.empty( )==false)
    {
        wake_mcuq = geq->getCurrentTick( )+1*ticks_per_cycle;
        registerCallback((CallbackPtr)&AITManager::cycle_mcuq, 1);
    }

    if (proc_wlv)
        registerCallback((CallbackPtr)&AITManager::check_mcu_resp, 1);
}

bool AITManager::isIssuable(int qid)
{
    std::queue<qentry_t>* ait_q = (qid==AIT_RDQ)? &ait_rdq : &ait_wrq;
    bool rv = true;
    
    if (ait_q->empty( ) || 
        ait_q->front( ).ready==false ||
        tcache->isReady(ait_q->front( ).pkt)==false)
        return false;

    return rv;
}

void AITManager::ait_mux( )
{
    bool rd_issuable = isIssuable(AIT_RDQ);
    bool wr_issuable = isIssuable(AIT_WRQ);
    bool isIssued = false;
    if (wr_issuable)
    {
        assert(status_ait==ST_FORBID_RD);
        ait_issue(AIT_WRQ);
        check_mcu_resp( );
        isIssued = true;
    }
    else if (rd_issuable && status_ait==ST_NORMAL)
    {
        ait_issue(AIT_RDQ);
        isIssued = true;
    }

    last_wake_mux = wake_mux;
    free_mux = (isIssued==false)? free_mux : (wake_mux+1*ticks_per_cycle);
    if (ait_rdq.empty( )==false || ait_wrq.empty( )==false)
    {
        wake_mux = geq->getCurrentTick( )+1*ticks_per_cycle;
        registerCallback((CallbackPtr)&AITManager::ait_mux, 1);
    }
}

void AITManager::ait_issue(int qid)
{
    /* Issue cmd to AIT$ according qid */
    Packet* pkt = NULL;
    if (qid==AIT_RDQ)
    {
        assert(ait_rdq.empty( )==false);
        pkt = ait_rdq.front( ).pkt;
        cmd_buffer[pkt->req_id].inflight_ait_rd = true;
        ait_rdq.pop( );
    }
    else if (qid==AIT_WRQ)
    {
        assert(ait_wrq.empty( )==false);
        assert(ait_wrq.front( ).ready);
        pkt = ait_wrq.front( ).pkt;
        ait_wrq.pop( );
    }
    else
        assert(0);

    tcache->recvRequest(pkt, tCMD);

    PCMC_DBG(dbg_msg, "[AM] Issue %s to AIT$, where [0x%lx, ID=%lx]\n", 
        (qid==AIT_RDQ)? "RD":"WR", pkt->LADDR, pkt->req_id);
}

void AITManager::ID_unmap(Packet* pkt)
{
    /* Retire assigned ID when request is issued */
    uint64_t i = 0;
    for ( ; i<size_buffer; i++)
    {
        if (cmd_buffer[i].pkt==pkt)
        {
            cmd_buffer[i].valid = false;
            cmd_buffer[i].pkt = NULL;
            cmd_buffer[i].inflight_ait_rd = false;
            avail_ID.push_back(i);
            break;
        }
    }
    assert(i<size_buffer);
    credit_buffer++;

    /* Check whether all ait read is retired */
    if (status_ait==ST_FORBID_RD)
        check_inflight_ait_rd( );

    PCMC_DBG(dbg_msg, "[AM] Retire tmp-ID=%lx and issue [0x%lx, ID=%lx, "
        "CMD=%c] to RMW\n", i, pkt->LADDR, pkt->req_id,
        (pkt->cmd==CMD_READ)? 'R':'W');
}

Packet* AITManager::address_translate(Packet* pkt)
{
    assert(pkt);

    /* Find corresponding original cmd in cmd_buffer*/
    assert(cmd_buffer[pkt->req_id].valid);
    Packet* origin = cmd_buffer[pkt->req_id].pkt;

    /* Get AIT$ read data */
    uint64_t e_ait = 0;
    for (uint64_t b=0; b<width_block; b++)
        e_ait |= (pkt->buffer_data.getByte(b) << (8*b)); // 8 is 8-bit/byte

    /* Decode it to PBA & PPN and concatenate */
    uint64_t LPN = (origin->LADDR>>PAGE_OFFSET) % (1<<PAGE_PER_BLOCK_BIT);
    uint64_t PBA = memsys->tdec->get_ait(e_ait, AIT_PBA);
    uint64_t key = memsys->tdec->get_ait(e_ait, AIT_KEY);
    uint64_t PPN = LPN ^ key;
    origin->PADDR = (PBA<<BLOCK_OFFSET) | (PPN<<PAGE_OFFSET);

    assert(origin);
    PCMC_DBG(dbg_msg, "[AM] Translate [LA=0x%lx, PA=0x%lx, tmp-ID=%lx"
        ", ID=%lx, CMD=%c] and push into hostq (qsize=%ld)\n", origin->LADDR, 
        origin->PADDR, pkt->req_id, origin->req_id, 
        (origin->cmd==CMD_READ)? 'R':'W', hostq.size( )+1);

    return origin;
}

void AITManager::cycle_hostq( )
{
    assert(hostq.empty( )==false);

    /* Issue packet */
    Packet* pkt = hostq.front( );
    if (rmw->isReady(pkt))
    {
        /* Release corresponding ID */
        ID_unmap(pkt);

        /* Stats update */
        ncycle_t tmp_lat = geq->getCurrentTick( )-pkt->recvTick;
        if (tmp_lat>max_issue_lat)
            max_issue_lat = tmp_lat;
        if (tmp_lat<min_issue_lat)
            min_issue_lat = tmp_lat;
        avg_issue_lat = (avg_issue_lat*num_issue+tmp_lat)/(double)(num_issue+1);
        num_issue += 1;

        pkt->from = this;
        rmw->recvRequest(pkt, tCMD);
        hostq.pop( );
    }

    /* Schedule event */
    last_wake_hostq = wake_hostq;
    if (hostq.empty( )==false)
    {
        wake_hostq = geq->getCurrentTick( )+1*ticks_per_cycle;
        registerCallback((CallbackPtr)&AITManager::cycle_hostq, 1);
    }
}

void AITManager::cycle_remapq( )
{
    assert(remapq.empty( )==false);

    /* Issue packet */
    Packet* pkt = remapq.front( );
    if (rmw->isReady(pkt))
    {
        pkt->from = this;
        rmw->recvRequest(pkt, tCMD);
        remapq.pop( );
        check_mcu_resp( );
    }

    /* Schedule event */
    last_wake_remapq = wake_remapq;
    if (remapq.empty( )==false)
    {
        wake_remapq = geq->getCurrentTick( )+1*ticks_per_cycle;
        registerCallback((CallbackPtr)&AITManager::cycle_remapq, 1);
    }
}

/*========== Below is stats setting ==========*/
void AITManager::register_stats( )
{
    double df_init = 0.0;
    uint64_t u64_zero = 0;
    uint64_t u64_max = std::numeric_limits<uint64_t>::max( );

    RESET_STATS(num_update, u64_zero);
    RESET_STATS(num_map, u64_zero);
    RESET_STATS(max_wait_ait, u64_zero);
    RESET_STATS(min_wait_ait, u64_max);
    RESET_STATS(avg_wait_ait, df_init);
    RESET_STATS(max_issue_lat, u64_zero);
    RESET_STATS(min_issue_lat, u64_max);
    RESET_STATS(avg_issue_lat, df_init);
    num_issue = 0;

    ADD_STATS(cp_name, num_update);
    ADD_STATS(cp_name, num_map);
    ADD_STATS_N_UNIT(cp_name, max_wait_ait, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_wait_ait, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_wait_ait, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_issue_lat, "cycles");
}

void AITManager::calculate_stats( )
{
    if (num_map>0)
    {
        max_wait_ait /= ticks_per_cycle;
        min_wait_ait /= ticks_per_cycle;
        avg_wait_ait /= ticks_per_cycle;
    }

    if (num_issue>0)
    {
        max_issue_lat /= ticks_per_cycle;
        min_issue_lat /= ticks_per_cycle;
        avg_issue_lat /= ticks_per_cycle;
    }
}

void AITManager::print_stats(std::ostream& os)
{
    stats->print(os);
    tcache->calculate_stats( );
    tcache->print_stats(os);
}
