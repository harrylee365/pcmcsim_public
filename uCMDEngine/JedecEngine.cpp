#include "base/MemoryControlSystem.h"
#include "base/EventQueue.h"
#include "base/Packet.h"
#include "base/Stats.h"
#include "base/MemInfo.h"
#include "uCMDEngine/JedecEngine.h"
#include "uCMDEngine/StateMachines.h"

using namespace PCMCsim;

#define PRIORITY_STATEMACHINE   50
#define PRIORITY_CYCLE_REQL     40
#define PRIORITY_CYCLE_UCMDQ    30
#define PRIORITY_REFRESH        20
#define PRIORITY_LP             10

#define MAX std::max
#define MIN std::min

typedef struct _refresh_pulse_t
{
    uint64_t rank;
    uint64_t bank_idx;
} refresh_pulse_t;

JedecEngine::JedecEngine(MemoryControlSystem* memsys_, AddressDecoder* adec_, 
    MemInfo* info_, std::string cfg_header, ncycle_t ticks_per_cycle_, uint64_t id_)
:uCMDEngine(memsys_), adec(adec_), info(info_), wake_reqlist(0), 
last_wake_reqlist(0), wake_ucmdq(0), last_wake_ucmdq(0), ucmdq_ptr(0),
wake_lp(0), last_wake_lp(0), last_wake_respq(0), wake_respq(0), 
free_respq(0), last_updated_tick(0)
{
    id = id_;
    cp_name = cfg_header + "[" + std::to_string(id) + "]";
    dbg_msg = memsys->getParamBOOL(cp_name+".dbg_msg", true);
    if (ticks_per_cycle_==0)
        ticks_per_cycle= memsys->getParamUINT64(cp_name+".ticks_per_cycle", 1);
    else 
        ticks_per_cycle = ticks_per_cycle_;

    deadlock_timer = getParamUINT64(cp_name+".deadlock_timer", 1000000000);

    /* Setup parameters */
    num_ranks = info->get_ranks( );
    num_bgs = info->get_bankgroups( );
    num_banks = info->get_banks( );
    num_all_banks = num_banks*num_bgs;
    uint64_t num_parts = info->get_partitions( ) * ((info->is_half_bank( ))? 2:1);
    uint64_t num_rows = info->get_rowsPerMAT( );

    num_ucmdq = num_ranks*num_bgs*num_banks;
    std::string read_str = memsys->getParamSTR(cp_name+".arbitrate_scheme");
    if (read_str=="RANK_FIRST_RR")
        arbit_scheme = RANK_FIRST_RR;
    else if (read_str=="BANK_FIRST_RR")
        arbit_scheme = BANK_FIRST_RR;
    else
        assert(0);

    read_str = memsys->getParamSTR(cp_name+".page_policy");
    if (read_str=="CLOSED_PAGE")
        page_policy = CLOSED_PAGE;
    else if (read_str=="OPEN_PAGE")
        page_policy = OPEN_PAGE;
    else
        assert(0);

    size_reqlist = memsys->getParamUINT64(cp_name+".size_reqlist", 2);
    use_pd = memsys->getParamBOOL(cp_name+".powerdown", false);
    use_refresh = memsys->getParamBOOL(cp_name+".AR", false);
    use_sref = memsys->getParamBOOL(cp_name+".SR", false);
    th_starv = memsys->getParamUINT64(cp_name+".threshold_starvation", 4);

    tBURST_RESP = memsys->getParamUINT64(cp_name+".tBURST_RESP", 4);

    if (use_pd)
    {
        read_str = memsys->getParamSTR(cp_name+".powerdown_mode");
        if (read_str=="FAST_EXIT")
            pd_mode = FAST_EXIT;
        else if (read_str=="SLOW_EXIT")
            pd_mode = SLOW_EXIT;
        else
            assert(0);
    
        if (use_refresh==false)
            std::cerr << "Warning! Be aware of data retention without"
                << " auto-refresh in power-down mode!" << std::endl;
    }

    if (use_sref && use_refresh==false)
        std::cerr << "Warning! Be aware of data retention without"
            << " auto-refresh in self-refresh mode!" << std::endl;

    if (use_pd && use_sref)
    {
        std::cerr << "Error! Powerdown mode and self-refresh cannot be"
            << " enabled simultaneously currently!" << std::endl;
        assert(0);
        exit(1);
    }

    standalone_mode = memsys->getParamBOOL(cp_name+".standalone_mode", true);

    /* Setup state machine, including timing & current */
    sm = new StateMachine(this);

    /* Setup state data according to parameters */
    open_banks.resize(num_ranks);
    open_parts.resize(num_ranks);
    open_rows.resize(num_ranks);
    cntr_starv.resize(num_ranks);
    for (uint64_t r=0; r<num_ranks; r++)
    {
        open_banks[r].resize(num_banks*num_bgs, false);
        open_parts[r].resize(num_banks*num_bgs);
        open_rows[r].resize(num_banks*num_bgs);
        cntr_starv[r].resize(num_banks*num_bgs);
        for (uint64_t b=0; b<num_banks*num_bgs; b++)
        {
            open_parts[r][b].resize(num_parts, false);
            open_rows[r][b].resize(num_parts, num_rows); 
            cntr_starv[r][b].resize(num_parts, 0);
        }
    }

    if (use_refresh)
    {
        /* 
         * Note that for SB refresh, only the number of banks 
         * in one bank group is visible as the total number of banks 
         * because SB refresh processes same banks across all bank groups
         */
        uint64_t num_visible_banks = (sm->sb_refresh)? num_banks : num_banks*num_bgs;
        slices_per_rank = num_visible_banks/sm->banks_per_refresh;
        ncycle_t slice_interval = sm->tREFI/(num_ranks*slices_per_rank);
        refresh_needed.resize(num_ranks);
        refresh_ucmdq_standby.resize(num_ranks);
        refresh_postponed.resize(num_ranks);
        for (uint64_t r=0; r<num_ranks; r++)
        {
            refresh_needed[r].resize(num_visible_banks, false);
            refresh_ucmdq_standby[r].resize(num_banks*num_visible_banks, false);
            refresh_postponed[r].resize(slices_per_rank, 0);
            for (uint64_t s=0; s<slices_per_rank; s++)
            {
                uint64_t bk_head = s*sm->banks_per_refresh;
                refresh_pulse_t* tmp_pulse = new refresh_pulse_t;
                tmp_pulse->rank = r;
                tmp_pulse->bank_idx = bk_head;
                refresh_pulses.push_back(tmp_pulse);

                /* Register refresh callback functions */
                ncycle_t slice_cycle_offset = (r*slices_per_rank+s)*slice_interval;
                registerCallback((CallbackPtr)&JedecEngine::refresh_cb,
                    sm->tREFI+slice_cycle_offset, PRIORITY_REFRESH,
                    reinterpret_cast<void*>(tmp_pulse));
            }
        }

        th_postpone = memsys->getParamUINT64(cp_name+".AR.threshold_postpone", 0);
        refresh_rank_ptr = 0;
        refresh_bank_ptr = 0;
    }

    if (use_pd)
        pd_ranks.resize(num_ranks, false);

    if (use_sref)
        sref_ranks.resize(num_ranks, false);

    ucmdq.resize(num_ranks*num_bgs*num_banks);

    register_stats( );
}

JedecEngine::~JedecEngine( )
{
    delete sm;

    for (uint64_t i=0; i<refresh_pulses.size( ); i++)
        delete (refresh_pulse_t*)(refresh_pulses[i]);
    refresh_pulses.clear( );
}

bool JedecEngine::isReady(Packet* /*pkt*/) 
{
    bool rv = true;
    uint64_t num_reqs = reqlist.size( )+req_events.size( )+await_req.size( );
    if (num_reqs>=size_reqlist)
        rv = false;
    return rv;
}

void JedecEngine::recvRequest(Packet* pkt, ncycle_t delay)
{
    assert(isReady(pkt));
    
    pkt->recvTick = geq->getCurrentTick( )+delay*ticks_per_cycle;
    pkt->recvTick_ucmde = geq->getCurrentTick( )+delay*ticks_per_cycle;
    Component::recvRequest(pkt, delay);

    /* Update Stats */
    if (pkt->cmd==CMD_READ)
        req_reads += 1;
    else 
        req_writes += 1;
}

void JedecEngine::recvResponse(Packet* pkt, ncycle_t delay)
{
    if (pkt->cmd==CMD_PKT_DEL)
    {
        assert(pkt->owner==this);
        delete pkt;
    }
    else
    {
        assert(pkt->cmd==CMD_READ && master);
        pkt->isDATA = true;
        pkt->from = this;
        Component::recvResponse(pkt, delay);
    }
}

void JedecEngine::handle_events(ncycle_t curr_tick)
{
    /* Get events from queue */
    prepare_events(curr_tick);

    /* Don't change following order */
    if (!await_req.empty( ))
        handle_await_reqs( );

    if (!await_cb.empty( ))
        handle_await_callbacks( );

    if (!await_resp.empty( ))
        handle_await_resps( );
}

void JedecEngine::handle_await_reqs( )
{
    std::list<LocalEvent*>::iterator e_it = await_req.begin( );
    for ( ; e_it!=await_req.end( ); )
    {
        Packet* pkt = (*e_it)->pkt;
        if (standalone_mode) 
        {
            /* Assign dest for resp. later */
            pkt->dest = pkt->from;
            
            /* Directly respond write command */
            if (pkt->cmd==CMD_WRITE)
            {
                Packet* cpy_pkt = new Packet( );
                *cpy_pkt = *pkt;
                cpy_pkt->owner = this;
                pkt->owner->recvResponse(pkt);
                pkt = cpy_pkt;
            }
        }

        reqlist.push_back(pkt);

        PCMC_DBG(dbg_msg, "[JE] Push req [0x%lx, ID=%lx, CMD=%s](%p)"
            " in request list (size=%lu/%lu)\n", pkt->LADDR, pkt->req_id, 
            get_cmd_str(pkt).c_str( ), pkt, reqlist.size( ), size_reqlist); 

        /* Free event */
        delete (*e_it);
        e_it = await_req.erase(e_it);
    }

    /* Schedule event of request list */
    if (last_wake_reqlist==wake_reqlist && reqlist.empty( )==false)
    {
        std::list<Packet*>::iterator l_it = reqlist.begin( );
        for ( ; l_it!=reqlist.end( ); l_it++)
        {
            uint64_t rank = adec->decode_addr((*l_it)->PADDR, FLD_RANK);
            uint64_t bank = get_bank_idx(*l_it);
            if (is_ucmdq_empty(rank, bank))
            {
                wake_reqlist = geq->getCurrentTick( )+1*ticks_per_cycle; 
                registerCallback((CallbackPtr)&JedecEngine::cycle_reqlist,
                    1, PRIORITY_CYCLE_REQL);
                break;
            }
        }
    }
}

void JedecEngine::handle_await_resps( )
{
    assert(standalone_mode);

    std::list<LocalEvent*>::iterator e_it = await_resp.begin( );
    for ( ; e_it!=await_resp.end( ); )
    {
        Packet* pkt = (*e_it)->pkt;
        respq.push(pkt);

        /* Free event */
        delete (*e_it);
        e_it = await_resp.erase(e_it);
    }

    if (last_wake_respq==wake_respq &&
        respq.empty( )==false)
    {
        wake_respq = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_respq = std::max(wake_respq, free_respq);
        registerCallbackAt((CallbackPtr)&JedecEngine::cycle_respq, wake_respq);
    }
}

void JedecEngine::cycle_respq( )
{
    assert(standalone_mode);
    assert(respq.empty( )==false);

    /* Respond packet */
    bool responded = false;
    ncycle_t LAT = 1;
    Packet* pkt = respq.front( );

    PCMC_DBG(dbg_msg, "[JE] RD data of req [0x%lx, ID=%lx, CMD=%s](%p)"
        " is responded to XBAR (size=%lu/%lu)\n", pkt->LADDR, pkt->req_id, 
        get_cmd_str(pkt).c_str( ), pkt, reqlist.size( ), size_reqlist); 

    if (parent->isReady(pkt))
    {
        parent->recvResponse(pkt);
        respq.pop( );

        responded = true;
        LAT = tBURST_RESP;
    }

    /* Schedule event */
    last_wake_respq = wake_respq;
    free_respq = (responded==false)? free_respq:(geq->getCurrentTick( )+LAT*ticks_per_cycle);
    if (respq.empty( )==false)
    {
        wake_respq = geq->getCurrentTick( )+LAT*ticks_per_cycle;
        registerCallback((CallbackPtr)&JedecEngine::cycle_respq, LAT);
    }
}

void JedecEngine::cycle_reqlist( )
{
    /* Find empty ucmdq to avoid unnecessary scheduling */
    bool can_schedule = false;
    for (uint64_t i=0; i<num_ucmdq; i++)
    {
        if (is_ucmdq_empty(i) &&
            is_req_schedulable(i))
        {
            can_schedule = true;
            break;
        }
    }

    last_wake_reqlist = wake_reqlist;
    if (can_schedule)
    {
        wake_reqlist = geq->getCurrentTick( )+1*ticks_per_cycle; 
        registerCallback((CallbackPtr)&JedecEngine::cycle_reqlist,
            1, PRIORITY_CYCLE_REQL);
    }
}

bool JedecEngine::is_req_schedulable(uint64_t ucmdq_idx)
{
    bool rv = false;
    std::list<Packet*>::iterator l_it = reqlist.begin( );
    for ( ; l_it!=reqlist.end( ); l_it++)
    {
        uint64_t rank = adec->decode_addr((*l_it)->PADDR, FLD_RANK);
        uint64_t bank = get_bank_idx(*l_it);
        if (get_ucmdq_idx(rank, bank)==ucmdq_idx)
        {
            rv = true;
            break;
        }
    }

    return rv;
}

bool JedecEngine::is_just_arrived(Packet* pkt)
{
    bool rv = false;
    if (geq->getCurrentTick( )==pkt->recvTick_ucmde)
        rv = true;
    return rv;
}

bool JedecEngine::is_ucmdq_empty(uint64_t qidx)
{
    bool rv = (ucmdq[qidx].empty( ))? true : false;
    return rv;
}

bool JedecEngine::is_ucmdq_empty(uint64_t rank, uint64_t bank)
{
    uint64_t qidx = get_ucmdq_idx(rank, bank);
    return is_ucmdq_empty(qidx);
}

bool JedecEngine::is_rank_ucmdqs_empty(uint64_t rank)
{
    bool rv = true;
    for (uint64_t i=0; i<num_all_banks; i++)
    {
        uint64_t qidx = get_ucmdq_idx(rank, i);
        if (is_ucmdq_empty(qidx)==false)
        {
            rv = false;
            break;
        }
    }

    return rv;
}

bool JedecEngine::is_refresh_needed(uint64_t rank, uint64_t bank)
{
    if (use_refresh==false)
        return false;

    /* Note input parameter, bank, is index across all bankgroups */
    bool rv = false;
    bank = (sm->sb_refresh)? bank%num_banks : bank;
    if (refresh_needed[rank][bank] || refresh_ucmdq_standby[rank][bank])
        rv = true;
    return rv;
}

bool JedecEngine::is_refresh_ucmdq_empty(uint64_t rank, uint64_t bank)
{
    bool rv = true;
    uint64_t bk_head = (bank/sm->banks_per_refresh) * sm->banks_per_refresh;
    if (sm->sb_refresh)
    {
        /* For SB, check availability of same banks across all bankgroups */
        uint64_t num_banks = info->get_banks( );
        uint64_t num_bgs = info->get_bankgroups( );
        for (uint64_t bg=0; bg<num_bgs; bg++)
        {
            uint64_t bk_offset = bg*num_banks; // 1st bank in bg
            for (uint64_t bidx=0; bidx<sm->banks_per_refresh; bidx++)
            {
                if (is_ucmdq_empty(rank, bk_offset+bk_head+bidx)==false)
                {
                    rv = false;
                    break;
                }
            }
        }
    }
    else
    {
        for (uint64_t bidx=0; bidx<sm->banks_per_refresh; bidx++)
        {
            if (is_ucmdq_empty(rank, bk_head+bidx)==false)
            {
                rv = false;
                break;
            }
        }
    }
    return rv;
}

bool JedecEngine::is_row_hit(uint64_t rank, uint64_t bank, uint64_t part, uint64_t row)
{
    bool rv = false;
    if (//open_banks[rank][bank] &&
        open_parts[rank][bank][part] &&
        open_rows[rank][bank][part]==row)
        rv = true;
    return rv;
}

void JedecEngine::insert_ucmdq(Packet* pkt)
{
    uint64_t PADDR = pkt->PADDR;
    uint64_t rank = adec->decode_addr(PADDR, FLD_RANK);
    uint64_t bank = get_bank_idx(pkt);
    uint64_t row = adec->decode_addr(PADDR, FLD_ROW);
    uint64_t part = adec->decode_addr(PADDR, FLD_HALF) * 
                    info->get_partitions( ) + 
                    adec->decode_addr(PADDR, FLD_PART);
    uint64_t qidx = get_ucmdq_idx(rank, bank);

    assert(is_ucmdq_empty(qidx));
    if (open_banks[rank][bank]==false)
    {
        /* Access closed bank - open it */
        cntr_starv[rank][bank][part] = 0;
        open_banks[rank][bank] = true;
        open_parts[rank][bank][part] = true;
        open_rows[rank][bank][part] = row;
        
        /* Insert ACT */
        Packet* act_pkt = gen_ucmd(pkt, CMD_ACT);
        ucmdq[qidx].push(act_pkt);

        /* Insert implicit cmd (performs auto-PRE) or original cmd */
        if (page_policy==CLOSED_PAGE)
        {
            Packet* converted_pkt = gen_ucmd(pkt, CMD_IMPLICIT);
            ucmdq[qidx].push(converted_pkt);
            close_bank(rank, bank, part);
        }
        else
            ucmdq[qidx].push(pkt);

        row_miss += 1;
    }
    else if (open_banks[rank][bank] && 
        is_row_hit(rank, bank, part, row)==false)
    {
        /* Access open bank & row miss */
        cntr_starv[rank][bank][part] = 0;

        /* Insert PRE */
        Packet* pre_pkt = gen_ucmd(pkt, CMD_PRE);
        ucmdq[qidx].push(pre_pkt);
        close_bank(rank, bank);

        /* Insert ACT & original - open bank */
        Packet* act_pkt = gen_ucmd(pkt, CMD_ACT);
        ucmdq[qidx].push(act_pkt);
        ucmdq[qidx].push(pkt);

        open_banks[rank][bank] = true;
        open_parts[rank][bank][part] = true;
        open_rows[rank][bank][part] = row;
        
        row_miss += 1;
    }
    else if (open_banks[rank][bank] &&
        is_row_hit(rank, bank, part, row))
    {
        /* Row hit */
        assert(page_policy==OPEN_PAGE);
        cntr_starv[rank][bank][part] += 1;
        ucmdq[qidx].push(pkt);
        
        row_hits += 1;
    }
    else
        assert(0);

    PCMC_DBG(dbg_msg, "[JE] Generate uCMDs for accessing "
        " %s bank & row %s [rk=%lu, bk=%lu, part=%lu, row=%lu],"
        " from req [0x%lx, ID=%lx, CMD=%s], qidx=%lu, qsize=%lu\n",
        (open_banks[rank][bank])? "open":"closed", 
        (is_row_hit(rank, bank, part, row))? "hit":"miss",
        rank, bank, part, row, pkt->LADDR, pkt->req_id, 
        get_cmd_str(pkt).c_str( ), qidx, ucmdq[qidx].size( )); 
    
    /* Schedule ucmdq event in a timely manner */
    std::pair<bool, ncycle_t> found_issuable = timely_issuable( );
    if (last_wake_ucmdq==wake_ucmdq && 
        found_issuable.first)
    {
        wake_ucmdq = found_issuable.second;
        registerCallbackAt((CallbackPtr)&JedecEngine::cycle_ucmdq,
            wake_ucmdq, PRIORITY_CYCLE_UCMDQ);
    }
}

void JedecEngine::schedule_postupdate(Packet* pkt)
{
    ncycle_t latency = sm->get_postupdate_latency(pkt);
    if (latency==0)
        return;

    Packet* cb_pkt = new Packet( );
    *cb_pkt = *pkt;
    cb_pkt->owner = this;

    registerCallback((CallbackPtr)&JedecEngine::postupdate_states_cb,
        latency, PRIORITY_STATEMACHINE,
        reinterpret_cast<void*>(cb_pkt));
}

void JedecEngine::postupdate_states_cb(Packet* pkt)
{
    assert(pkt->owner==this);
    sm->postupdate_states(pkt);
    delete pkt;

    /* Lower power handling after state update */
    lp_handle( );
}

bool JedecEngine::find_starved(Packet*& found_req)
{
    bool rv = false;
    found_req = NULL;

    std::list<Packet*>::iterator l_it = reqlist.begin( );
    for ( ; l_it!=reqlist.end( ); l_it++)
    {
        uint64_t PADDR = (*l_it)->PADDR;
        uint64_t rank = adec->decode_addr(PADDR, FLD_RANK);
        uint64_t bank = get_bank_idx(*l_it);
        uint64_t row = adec->decode_addr(PADDR, FLD_ROW);
        uint64_t part = adec->decode_addr(PADDR, FLD_HALF) * 
                        info->get_partitions( ) + 
                        adec->decode_addr(PADDR, FLD_PART);

        if (is_ucmdq_empty(rank, bank)==false) continue;

        if (open_banks[rank][bank] &&
            is_row_hit(rank, bank, part, row)==false &&
            is_refresh_needed(rank, bank)==false &&
            cntr_starv[rank][bank][part]>=th_starv &&
            is_just_arrived(*l_it)==false)
        {
            found_req = (*l_it);
            reqlist.erase(l_it);
            rv = true;
            break;
        }
    }

    return rv;
}

bool JedecEngine::find_row_hit(Packet*& found_req)
{
    bool rv = false;
    found_req = NULL;

    std::list<Packet*>::iterator l_it = reqlist.begin( );
    for ( ; l_it!=reqlist.end( ); l_it++)
    {
        uint64_t PADDR = (*l_it)->PADDR;
        uint64_t rank = adec->decode_addr(PADDR, FLD_RANK);
        uint64_t bank = get_bank_idx(*l_it);
        uint64_t row = adec->decode_addr(PADDR, FLD_ROW);
        uint64_t part = adec->decode_addr(PADDR, FLD_HALF) * 
                        info->get_partitions( ) + 
                        adec->decode_addr(PADDR, FLD_PART);

        if (is_ucmdq_empty(rank, bank)==false) continue;

        if (open_banks[rank][bank] &&
            is_row_hit(rank, bank, part, row) &&
            is_refresh_needed(rank, bank)==false &&
            is_just_arrived(*l_it)==false)
        {
            found_req = (*l_it);
            reqlist.erase(l_it);
            rv = true;
            break;
        }
    }

    return rv;
}

bool JedecEngine::find_open_bank(Packet*& found_req)
{
    bool rv = false;
    found_req = NULL;

    std::list<Packet*>::iterator l_it = reqlist.begin( );
    for ( ; l_it!=reqlist.end( ); l_it++)
    {
        uint64_t PADDR = (*l_it)->PADDR;
        uint64_t rank = adec->decode_addr(PADDR, FLD_RANK);
        uint64_t bank = get_bank_idx(*l_it);

        if (is_ucmdq_empty(rank, bank)==false) continue;

        if (open_banks[rank][bank] &&
            is_refresh_needed(rank, bank)==false &&
            is_just_arrived(*l_it)==false)
        {
            found_req = (*l_it);
            reqlist.erase(l_it);
            rv = true;
            break;
        }
    }

    return rv;
}

bool JedecEngine::find_closed_bank(Packet*& found_req)
{
    bool rv = false;
    found_req = NULL;

    std::list<Packet*>::iterator l_it = reqlist.begin( );
    for ( ; l_it!=reqlist.end( ); l_it++)
    {
        uint64_t PADDR = (*l_it)->PADDR;
        uint64_t rank = adec->decode_addr(PADDR, FLD_RANK);
        uint64_t bank = get_bank_idx(*l_it);

        if (is_ucmdq_empty(rank, bank)==false) continue;

        if (open_banks[rank][bank]==false &&
            is_refresh_needed(rank, bank)==false &&
            is_just_arrived(*l_it)==false)
        {
            found_req = (*l_it);
            reqlist.erase(l_it);
            rv = true;
            break;
        }
    }

    return rv;
}

std::pair<bool, ncycle_t> JedecEngine::timely_issuable( )
{
    bool issuable = false;
    ncycle_t free_tick = std::numeric_limits<uint64_t>::max( );
    for (uint64_t r=0; r<num_ranks; r++)
    {
        for (uint64_t b=0; b<num_all_banks; b++)
        {
            uint64_t qidx = get_ucmdq_idx(r, b);
            if (is_ucmdq_empty(r, b)) continue;

            Packet* head_pkt = ucmdq[qidx].front( );
            free_tick = MIN(free_tick, sm->timely_issuable(head_pkt));
        }
    }

    if (free_tick!=std::numeric_limits<uint64_t>::max( ))
        issuable = true;

    if (free_tick<=geq->getCurrentTick( ))
        free_tick = geq->getCurrentTick( ) + 1*ticks_per_cycle;

    std::pair<bool, uint64_t> rv(issuable, free_tick);
    return rv;
}

void JedecEngine::cycle_ucmdq( )
{
    /* Function for issuing ucmds to media */
    bool issued = false;

    /* Update stats before changing to the next state */
    ncycle_t sync_cycles = 
        (geq->getCurrentTick( )-last_updated_tick) / ticks_per_cycle;
    last_updated_tick = geq->getCurrentTick( );
    sm->update_stats(sync_cycles);

    /* Prepare refresh ucmds */
    if (use_refresh)
        prepare_refresh( );

    /* Ready to issue ucmds to media */
    for (uint64_t i=0; i<num_ucmdq; i++)
    {
        uint64_t qidx = (ucmdq_ptr+i) % num_ucmdq;
        if (ucmdq[qidx].empty( )==false)
        {
            Packet* pkt = ucmdq[qidx].front( );
            if (pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WRITE_PRE)
            {
                pkt->from = this; // for identification in DPU
                if (dpu->isReady(pkt)==false || 
                    sm->is_issuable(pkt)==false)
                    continue;
            }
            else if (sm->is_issuable(pkt)==false)
                continue;
            
            /* ucmds are issuable in this cycle */
            ncycle_t latency = sm->update_states(pkt);
            schedule_postupdate(pkt);

            PCMC_DBG(dbg_msg, "[JE] Issue ucmd of req [0x%lx, ID=%lx,"
                " CMD=%s] from ucmd[%lu] update ucmdq-ptr=%lu\n", 
                pkt->LADDR, pkt->req_id, get_cmd_str(pkt).c_str( ), 
                qidx, ucmdq_ptr); 

            /* Stats update */
            ncycle_t tmp_lat = geq->getCurrentTick( )-pkt->recvTick;
            if (pkt->cmd==CMD_READ || pkt->cmd==CMD_READ_PRE || 
                pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WRITE_PRE)
            {
                if (tmp_lat>max_issue_lat)
                    max_issue_lat = tmp_lat;
                if (tmp_lat<min_issue_lat)
                    min_issue_lat = tmp_lat;
                avg_issue_lat = 
                    (avg_issue_lat*num_issue+tmp_lat)/(double)(num_issue+1);
                num_issue += 1;
            }

            pkt->from = this;
            media->recvRequest(pkt, latency); // send command
            if (pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WRITE_PRE)
            {
                if (standalone_mode) 
                {
                    /* Send data to media (=dpu) */
                    Packet* wdata_pkt = new Packet( );
                    *wdata_pkt = *pkt;
                    wdata_pkt->cmd = CMD_WRITE;
                    wdata_pkt->owner = this;
                    wdata_pkt->isDATA = true;
                    dpu->recvRequest(wdata_pkt, latency);
                }
                else
                {
                    /* Send write command signal to DPU */
                    Packet* wsignal_pkt = pkt;
                    dpu->recvRequest(wsignal_pkt);
                    wack(pkt, CMD_WACK_ID);
                }
            }
            else if (pkt->cmd==CMD_REFRESH)
            {
                uint64_t rank = adec->decode_addr(pkt->PADDR, FLD_RANK);
                uint64_t bank = get_bank_idx(pkt);
                rst_refresh_ucmdq_standby(rank, bank);
            }

            ucmdq[qidx].pop( );
            update_ucmdq_ptr(qidx);

            issued = true;

            /* Schedule event of req-list once ucmdq becomes emtpy */
            bool can_schedule = false;
            if (pkt->cmd==CMD_REFRESH)
            {
                uint64_t rank = adec->decode_addr(pkt->PADDR, FLD_RANK);
                uint64_t bk_head = (get_bank_idx(pkt)/
                        sm->banks_per_refresh) * sm->banks_per_refresh;

                if (sm->sb_refresh)
                {
                    /* For SB, check scheduable banks across all bankgroups */
                    for (uint64_t bg=0; bg<num_bgs; bg++)
                    {
                        uint64_t bk_offset = bg*num_banks;
                        for (uint64_t b=0; b<sm->banks_per_refresh; b++)
                        {
                            uint64_t tmp_qidx = 
                                get_ucmdq_idx(rank, bk_offset+bk_head+b);
                            
                            if(is_ucmdq_empty(tmp_qidx) &&
                               is_req_schedulable(tmp_qidx))
                            {
                                can_schedule = true;
                                break;
                            }
                        }
                    }
                }
                else
                {
                    for (uint64_t i=0; i <sm->banks_per_refresh; i++)
                    {
                        uint64_t tmp_qidx = get_ucmdq_idx(rank, bk_head+i);
                        if(is_ucmdq_empty(tmp_qidx) &&
                           is_req_schedulable(tmp_qidx))
                        {
                            can_schedule = true;
                            break;
                        }
                    }

                }
            }
            else if (is_ucmdq_empty(qidx)
                && is_req_schedulable(qidx))
                can_schedule = true;

            if (can_schedule &&
                last_wake_reqlist==wake_reqlist)
            {
                wake_reqlist = geq->getCurrentTick( )+sm->tCMD*ticks_per_cycle; 
                registerCallback((CallbackPtr)&JedecEngine::cycle_reqlist,
                    sm->tCMD, PRIORITY_CYCLE_REQL);
            }
            break;
        }

        if (ucmdq[qidx].empty( )==false && issued==false)
        {
            /* ucmdqs are not issuable in this cycle */
            Packet* pkt = ucmdq[qidx].front( );
            ncycle_t stalled_duration = geq->getCurrentTick( ) - 
                pkt->recvTick_ucmde;
            if (stalled_duration>=deadlock_timer)
            {
                PCMC_DBG(dbg_msg, "[JE] Deadlock occurs! "
                    "in-tick=%lu, stall-duration=%lu, Req[0x%lx, "
                    "ID=%lx, CMD=%s], qidx=%lu\n", pkt->recvTick_ucmde,
                    stalled_duration, pkt->LADDR, pkt->req_id,
                    get_cmd_str(pkt).c_str( ), qidx); 
                assert(0);
                exit(1);
            }
        }
    }

    /* Lower power handling after state update */
    lp_handle( );

    /* Self schedule */
    last_wake_ucmdq = wake_ucmdq;
    std::pair<bool, ncycle_t> found_issuable = timely_issuable( );
    if (found_issuable.first)
    {
        wake_ucmdq = (issued)? 
            MAX(sm->tCMD, found_issuable.second) : found_issuable.second;
        registerCallbackAt((CallbackPtr)&JedecEngine::cycle_ucmdq,
            wake_ucmdq, PRIORITY_CYCLE_UCMDQ);
    }
}

void JedecEngine::update_ucmdq_ptr(uint64_t curr_idx)
{
    ucmdq_ptr = (curr_idx+1) % num_ucmdq;
}

void JedecEngine::refresh_cb(void* pulse)
{
    mark_refresh(pulse);
    gen_refresh_ucmds( );
}

void JedecEngine::mark_refresh(void* pulse)
{
    /* 
     * Mark that the set of banks related with this pulse requires refresh 
     * This fucntion prevents host request gets into ucmdq
     */
    uint64_t rank = ((refresh_pulse_t*)pulse)->rank;
    uint64_t bank_head = ((refresh_pulse_t*)pulse)->bank_idx;

    if (use_sref && sref_ranks[rank])
        return;

    incr_refresh_postponed(rank, bank_head);

    if (reach_postpone_threshold(rank, bank_head))
        set_refresh_needed(rank, bank_head);

    /* Schedule refresh callback */
    registerCallback((CallbackPtr)&JedecEngine::refresh_cb,
        sm->tREFI, PRIORITY_REFRESH, reinterpret_cast<void*>(pulse));
}

void JedecEngine::gen_refresh_ucmds( )
{
    /* Refresh ready banks in a static round robin manner */
    bool gen = false;
    for (uint64_t r=0; r<num_ranks; r++)
    {
        uint64_t ridx = (refresh_rank_ptr+r) % num_ranks;

        if (use_sref && sref_ranks[ridx])
            continue;

        for (uint64_t s=0; s<slices_per_rank; s++)
        {
            uint64_t num_visible_banks = (sm->sb_refresh)? num_banks : num_all_banks;
            uint64_t bk_head = (refresh_bank_ptr +
                s*sm->banks_per_refresh) % num_visible_banks;

            if (reach_postpone_threshold(ridx, bk_head)==false
                || is_refresh_ucmdq_empty(ridx, bk_head)==false)
                continue;

            /* Precharge if corresponding banks are active */
            // TODO: maybe all-bank requires all-bank precharge
            // per-bank requires per-bank precharge
            // same-bank requires same-bank precharge...
            if (sm->sb_refresh)
            {
                /* For SB, precharge across all bankgroups */
                for (uint64_t bg=0; bg<num_bgs; bg++)
                {
                    uint64_t bk_offset = bg*num_banks;
                    for (uint64_t b=0; b<sm->banks_per_refresh; b++)
                    {
                        uint64_t bidx = bk_offset+bk_head+b;
                        uint64_t qidx = get_ucmdq_idx(ridx, bidx);
                        assert(bidx<num_all_banks);

                        if (open_banks[ridx][bidx]==false)
                            continue;

                        close_bank(ridx, bidx);
                        Packet* pre_ucmd = gen_ucmd(ridx, bidx, CMD_PRE);
                        ucmdq[qidx].push(pre_ucmd);
                    }
                }
            }
            else
            {
                for (uint64_t b=0; b<sm->banks_per_refresh; b++)
                {
                    uint64_t bidx = bk_head+b; 
                    uint64_t qidx = get_ucmdq_idx(ridx, bidx);
                    assert(bidx<num_all_banks);

                    if (open_banks[ridx][bidx]==false)
                        continue;

                    close_bank(ridx, bidx);
                    Packet* pre_ucmd = gen_ucmd(ridx, bidx, CMD_PRE);
                    ucmdq[qidx].push(pre_ucmd);
                }
            }

            /* Generate refresh ucmd for dispatch */
            Packet* refresh_ucmd = gen_ucmd(ridx, bk_head, CMD_REFRESH);
            uint64_t qidx = get_ucmdq_idx(ridx, bk_head);
            ucmdq[qidx].push(refresh_ucmd);

            /* Update refresh states */
            set_refresh_ucmdq_standby(ridx, bk_head);
            decr_refresh_postponed(ridx, bk_head);

            if (reach_postpone_threshold(ridx, bk_head)==false)
                rst_refresh_needed(ridx, bk_head);

            /* Round robin refresh */
            refresh_bank_ptr += sm->banks_per_refresh;
            if (refresh_bank_ptr>=num_visible_banks)
            {
                refresh_bank_ptr = 0;
                refresh_rank_ptr += 1;
                if (refresh_rank_ptr==num_ranks)
                    refresh_rank_ptr = 0;
            }

            /* Schedule ucmdq event in a timely manner */
            std::pair<bool, ncycle_t> found_issuable = timely_issuable( );
            if (last_wake_ucmdq==wake_ucmdq && 
                found_issuable.first)
            {
                wake_ucmdq = found_issuable.second;
                registerCallbackAt((CallbackPtr)&JedecEngine::cycle_ucmdq,
                    wake_ucmdq, PRIORITY_CYCLE_UCMDQ);
            }

            PCMC_DBG(dbg_msg, "[JE] Generate refresh(same-bank=%s) for the bank"
                " [rk=%lu, bk=%lu], where request is [0x%lx, CMD=%s],"
                " qidx=%lu, qsize=%lu, listsize=%lu\n", 
                (sm->sb_refresh)? "true":"false", ridx, bk_head, 
                refresh_ucmd->LADDR, get_cmd_str(refresh_ucmd).c_str( ), 
                qidx, ucmdq[qidx].size( ), reqlist.size( )); 
            
            gen = true;
            break;
        }

        if (gen)
            break;
    }
}

void JedecEngine::prepare_refresh( )
{
    for (uint64_t r=0; r<info->get_ranks( ); r++)
    {
        for (uint64_t s=0; s<slices_per_rank; s++)
        {
            uint64_t bk_head = s * sm->banks_per_refresh;
            if (reach_postpone_threshold(r, bk_head)
                && is_refresh_ucmdq_empty(r, bk_head))
                gen_refresh_ucmds( );
        }
    }
}

void JedecEngine::rst_refresh_needed(uint64_t rank, uint64_t bank)
{
    uint64_t bk_head = (bank/sm->banks_per_refresh) * sm->banks_per_refresh;
    for (uint64_t i=0; i<sm->banks_per_refresh; i++)
        refresh_needed[rank][bk_head+i] = false;
}

void JedecEngine::set_refresh_needed(uint64_t rank, uint64_t bank)
{
    uint64_t bk_head = (bank/sm->banks_per_refresh) * sm->banks_per_refresh;
    for (uint64_t i=0; i<sm->banks_per_refresh; i++)
        refresh_needed[rank][bk_head+i] = true;
}

void JedecEngine::rst_refresh_ucmdq_standby(uint64_t rank, uint64_t bank)
{
    uint64_t bk_head = (bank/sm->banks_per_refresh) * sm->banks_per_refresh;
    for (uint64_t i=0; i <sm->banks_per_refresh; i++)
        refresh_ucmdq_standby[rank][bk_head+i] = false;
}

void JedecEngine::set_refresh_ucmdq_standby(uint64_t rank, uint64_t bank)
{
    uint64_t bk_head = (bank/sm->banks_per_refresh) * sm->banks_per_refresh;
    for (uint64_t i=0; i <sm->banks_per_refresh; i++)
        refresh_ucmdq_standby[rank][bk_head+i] = true;
}

void JedecEngine::incr_refresh_postponed(uint64_t rank, uint64_t bank)
{
    refresh_postponed[rank][bank/sm->banks_per_refresh] += 1;
    if (refresh_postponed[rank][bank/sm->banks_per_refresh]>th_postpone)
        refresh_postponed[rank][bank/sm->banks_per_refresh] = th_postpone+1;
}

void JedecEngine::decr_refresh_postponed(uint64_t rank, uint64_t bank)
{
    refresh_postponed[rank][bank/sm->banks_per_refresh] -= 1;
}

bool JedecEngine::reach_postpone_threshold(uint64_t rank, uint64_t bank)
{
    bool rv = false;
    if (refresh_postponed[rank][bank/sm->banks_per_refresh]>th_postpone)
        rv = true;
    return rv;
}

void JedecEngine::lp_handle( )
{
    /* Update stats before changing to the next state */
    ncycle_t sync_cycles = 
        (geq->getCurrentTick( )-last_updated_tick) / ticks_per_cycle;
    last_updated_tick = geq->getCurrentTick( );
    sm->update_stats(sync_cycles);

    /* 
     * Low-power operation handling 
     * A logic that checks idleness of a rank or ucmdqs
     * and triggers LP operations
     */
    ncycle_t lp_trig_tick = std::numeric_limits<uint64_t>::max( );
    if (use_sref)
        lp_trig_tick = lp_handle_sref( );
    else if (use_pd)
        lp_trig_tick = lp_handle_pd( );
    else
        return;

    if (lp_trig_tick<=geq->getCurrentTick( ))
        lp_trig_tick = geq->getCurrentTick( ) + 1*ticks_per_cycle;

    last_wake_lp = wake_lp;
    if (lp_trig_tick!=std::numeric_limits<uint64_t>::max( ))
    {
        wake_lp = lp_trig_tick;
        registerCallbackAt((CallbackPtr)&JedecEngine::lp_handle,
            wake_lp, PRIORITY_LP);
    }
}

ncycle_t JedecEngine::lp_handle_pd( )
{
    /* 
     * Powerdown handling 
     * PDE & PDX are driven by clock, and addr signals are don't care (DDR4)
     * Therefore, PDE & PDX can be executed simultaneously in multi-rank
     */
    ncycle_t pd_trig_tick = std::numeric_limits<uint64_t>::max( );
    for (uint64_t r=0; r<info->get_ranks( ); r++)
    {
        bool need_refresh = false;
        if (use_refresh)
        {
            for (uint64_t s=0; s<slices_per_rank; s++)
            {
                uint64_t bk_head = s * sm->banks_per_refresh;
                if (reach_postpone_threshold(r, bk_head))
                {
                    need_refresh = true;
                    break;
                }
            }
        }

        if (pd_ranks[r]) // in PD mode now, may need POWERUP
        {
            Packet* powerup_pkt = gen_ucmd(r, 0, CMD_PDX);
            powerup_pkt->from = this;
            if ((need_refresh || is_rank_ucmdqs_empty(r)==false) &&
                sm->is_issuable(powerup_pkt))
            {
                ncycle_t latency = sm->update_states(powerup_pkt);
                schedule_postupdate(powerup_pkt);
                media->recvRequest(powerup_pkt, latency);
                pd_ranks[r] = false;
            }
            else 
            {
                if ((need_refresh || is_rank_ucmdqs_empty(r)==false) 
                    && sm->is_issuable(powerup_pkt)==false)
                    pd_trig_tick = MIN(pd_trig_tick, sm->timely_issuable(powerup_pkt));
                delete powerup_pkt;
            }
        }
        else            // PD can be required if idle
        {
            cmd_t pd_type = CMD_UNDEF;
            if (pd_mode==FAST_EXIT)
                pd_type = CMD_FPPDE;
            else if (pd_mode==SLOW_EXIT)
                pd_type = CMD_SPPDE;
            else
                assert(0);

            if (sm->is_idle(r)==false)
                pd_type = CMD_APDE;

            Packet* pd_pkt = gen_ucmd(r, 0, pd_type);
            pd_pkt->from = this;
            if (is_rank_ucmdqs_empty(r) && sm->is_issuable(pd_pkt))
            {
                ncycle_t latency = sm->update_states(pd_pkt);
                schedule_postupdate(pd_pkt);
                media->recvRequest(pd_pkt, latency);
                pd_ranks[r] = true;
            }
            else
            {
                if (is_rank_ucmdqs_empty(r) 
                    && sm->is_issuable(pd_pkt)==false)
                    pd_trig_tick = MIN(pd_trig_tick, sm->timely_issuable(pd_pkt));
                delete pd_pkt;
            }
        }
    }
    return pd_trig_tick;
}

ncycle_t JedecEngine::lp_handle_sref( )
{
    /*
     * Self-refresh handling
     * SRE & SRX is driven by clock, but addr signals are not don't care
     * Therefore, SRE & SRX only can be issued to one rank at a time
     */
    ncycle_t sref_trig_tick = std::numeric_limits<uint64_t>::max( );
    for (uint64_t r=0; r<info->get_ranks( ); r++)
    {
        if (sref_ranks[r])       // in SR mode now
        {
            Packet* srx_pkt = gen_ucmd(r, 0, CMD_SRX);
            srx_pkt->from = this;
            if (is_rank_ucmdqs_empty(r)==false && sm->is_issuable(srx_pkt))
            {
                ncycle_t latency = sm->update_states(srx_pkt);
                schedule_postupdate(srx_pkt);
                media->recvRequest(srx_pkt, latency);
                sref_ranks[r] = false;
                break;
            }
            else
            {
                if (is_rank_ucmdqs_empty(r)==false
                    && sm->is_issuable(srx_pkt)==false)
                    sref_trig_tick = MIN(sref_trig_tick, sm->timely_issuable(srx_pkt));
                delete srx_pkt;
            }
        }
        else if (sm->is_idle(r)) // enter SR if all-bank idle
        {
            Packet* sre_pkt = gen_ucmd(r, 0, CMD_SRE);
            sre_pkt->from = this;
            if (is_rank_ucmdqs_empty(r) && sm->is_issuable(sre_pkt))
            {
                ncycle_t latency = sm->update_states(sre_pkt);
                schedule_postupdate(sre_pkt);
                media->recvRequest(sre_pkt, latency);
                sref_ranks[r] = true;

                /* Clear AR flags before entering SR mode */
                for (uint64_t s=0; s<slices_per_rank; s++)
                {
                    uint64_t bk_head = s * sm->banks_per_refresh;
                    rst_refresh_needed(r, bk_head);
                    rst_refresh_ucmdq_standby(r, bk_head);
                }
                break;
            }
            else
            {
                if (is_rank_ucmdqs_empty(r)
                    && sm->is_issuable(sre_pkt)==false)
                    sref_trig_tick = MIN(sref_trig_tick, sm->timely_issuable(sre_pkt));
                delete sre_pkt;
            }
        }
    }
    return sref_trig_tick;
}

double JedecEngine::getParamFLOAT(std::string var, double def)
{
    assert(var!="");
    return memsys->getParamFLOAT(cp_name+var, def);
}

uint64_t JedecEngine::getParamUINT64(std::string var, uint64_t def)
{
    assert(var!="");
    return memsys->getParamUINT64(cp_name+var, def);
}

bool JedecEngine::getParamBOOL(std::string var, bool def)
{
    assert(var!="");
    return memsys->getParamBOOL(cp_name+var, def);
}

std::string JedecEngine::getParamSTR(std::string var, std::string def)
{
    assert(var!="");
    return memsys->getParamSTR(cp_name+var, def);
}

void JedecEngine::close_bank(uint64_t rank, uint64_t bank)
{
    uint64_t num_parts = info->get_partitions( ) * 
        ((info->is_half_bank( ))? 2:1);
    uint64_t num_opened = 0;

    open_banks[rank][bank] = false;
    for (uint64_t p=0; p<num_parts; p++)
    {
        if (open_parts[rank][bank][p])
        {
            open_parts[rank][bank][p] = false;
            open_rows[rank][bank][p] = info->get_rowsPerMAT( );
            num_opened++;
        }
    }
    assert(num_opened==1);
}

void JedecEngine::close_bank(uint64_t rank, uint64_t bank, uint64_t part)
{
    open_banks[rank][bank] = false;
    open_parts[rank][bank][part] = false;
    open_rows[rank][bank][part] = info->get_rowsPerMAT( );
}

uint64_t JedecEngine::get_ucmdq_idx(uint64_t rank, uint64_t bank)
{
    uint64_t rv = 0;
    if (arbit_scheme==RANK_FIRST_RR)
        rv = bank*num_ranks + rank;
    else if (arbit_scheme==BANK_FIRST_RR)
        rv = rank*num_all_banks + bank;
    else
        assert(0);

    assert(rv<num_all_banks*num_ranks);
    return rv;
}

uint64_t JedecEngine::get_bank_idx(Packet* pkt)
{
    uint64_t bg = adec->decode_addr(pkt->PADDR, FLD_BANKGRP);
    uint64_t bk = adec->decode_addr(pkt->PADDR, FLD_BANK);
    return (bg * num_banks + bk);
}

Packet* JedecEngine::gen_ucmd(Packet* ref_pkt, cmd_t type)
{
    Packet* pkt = NULL;
    if (type==CMD_IMPLICIT)
    {
        assert(ref_pkt->cmd==CMD_READ || ref_pkt->cmd==CMD_WRITE);
        pkt = ref_pkt;
        pkt->cmd = (ref_pkt->cmd==CMD_READ)? CMD_READ_PRE:CMD_WRITE_PRE; 
    }
    else
    {
        pkt = new Packet( );
        *pkt = *ref_pkt;
        pkt->cmd = type;
        pkt->owner = this;
    }

    return pkt;
}

Packet* JedecEngine::gen_ucmd(uint64_t rank, uint64_t bank_idx, cmd_t type)
{
    Packet* pkt = new Packet( );
    uint64_t bg = bank_idx / info->get_banks( );
    uint64_t bk = bank_idx % info->get_banks( );
    
    /* Setup address */
    std::vector<uint64_t> partial_bits;
    for (uint64_t i=0; i<NUM_FIELDS; i++)
    {
        if (i==FLD_BANK) 
            partial_bits.push_back(bk);
        else if (i==FLD_BANKGRP)
            partial_bits.push_back(bg);
        else if (i==FLD_RANK)
            partial_bits.push_back(rank);
        else
            partial_bits.push_back(0); 
    }
    uint64_t PADDR = adec->encode_addr(partial_bits);

    pkt->LADDR = PADDR;
    pkt->PADDR = PADDR;
    pkt->owner = this;
    pkt->from = this;
    pkt->cmd = type;

    return pkt;
}

void JedecEngine::register_stats( )
{
    uint64_t u64_zero = 0;
    uint64_t u64_max = std::numeric_limits<uint64_t>::max( );
    double df_init = 0.0;

    RESET_STATS(max_issue_lat, u64_zero);
    RESET_STATS(min_issue_lat, u64_max);
    RESET_STATS(avg_issue_lat, df_init);
    num_issue = 0;

    ADD_STATS(cp_name, row_hits);
    ADD_STATS(cp_name, row_miss);
    ADD_STATS(cp_name, req_reads);
    ADD_STATS(cp_name, req_writes);
    ADD_STATS_N_UNIT(cp_name, max_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_issue_lat, "cycles");

    sm->register_stats( );
}

void JedecEngine::calculate_stats( )
{
    /* Update stats before changing to the next state */
    ncycle_t sync_cycles = 
        (geq->getCurrentTick( )-last_updated_tick) / ticks_per_cycle;
    last_updated_tick = geq->getCurrentTick( );
    sm->update_stats(sync_cycles);
    
    sm->calculate_stats( );

    if (num_issue>0)
    {
        max_issue_lat /= ticks_per_cycle;
        min_issue_lat /= ticks_per_cycle;
        avg_issue_lat /= ticks_per_cycle;
    }
}

void JedecEngine::print_stats(std::ostream& os)
{
    sm->print_stats(os);
    stats->print(os);
}

