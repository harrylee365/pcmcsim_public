#include "base/EventQueue.h"
#include "base/Packet.h"
#include "base/Stats.h"
#include "base/MemInfo.h"
#include "uCMDEngine/StateMachines.h"
#include "uCMDEngine/JedecEngine.h"

#define MAX std::max

namespace PCMCsim
{
    enum _machine_states
    {
        ST_CLOSED = 0,
        ST_OPEN,
        ST_REFRESH,
        ST_APD,
        ST_FPPD,
        ST_SPPD,
        ST_SREF,
        NUM_STATES
    };
};

using namespace PCMCsim;

/* Definition of StateMachine */
StateMachine::StateMachine(JedecEngine* je_): je(je_)
{
    num_ranks = je->info->get_ranks( );
    uint64_t num_all_banks = je->info->get_banks( ) *
        je->info->get_bankgroups( );
    setup_timings(num_all_banks);
    setup_currents( );

    for (uint64_t i=0; i<num_ranks; i++)
        rm.push_back(RankMachine(this, i));
}

bool StateMachine::is_idle(uint64_t rank)
{
    return rm[rank].is_idle( );
}

bool StateMachine::is_issuable(Packet* pkt)
{
    uint64_t rank = je->adec->decode_addr(pkt->PADDR, FLD_RANK);
    return rm[rank].is_issuable(pkt);
}

ncycle_t StateMachine::timely_issuable(Packet* pkt)
{
    uint64_t rank = je->adec->decode_addr(pkt->PADDR, FLD_RANK);
    return rm[rank].timely_issuable(pkt);
}

ncycle_t StateMachine::update_states(Packet* pkt)
{
    /* Update states of ACCESSED rank */
    uint64_t rank = je->adec->decode_addr(pkt->PADDR, FLD_RANK);
    ncycle_t rv = rm[rank].update_states(pkt);

    /* Notify unaccessed ranks */
    for (uint64_t r=0; r<num_ranks; r++)
    {
        if (r==rank) continue;
        rm[r].notify(pkt);
    }
    return rv;
}

ncycle_t StateMachine::get_postupdate_latency(Packet* pkt)
{
    /* Scheduling latency of postupdate */
    ncycle_t rv = 0;
    switch (pkt->cmd)
    {
        case CMD_READ_PRE:  // auto-precharge after rv
            rv = tAL+tRTP;  // refer to Micron datasheet
            break;
        case CMD_WRITE_PRE: // auto-precharge after rv
            rv = tAL+tCWL+tBURST+tWR; // refer to Micron
            break;
        case CMD_REFRESH:   // close bank after rv
            rv = tRFC;
            break;
        default:
            rv = 0;
            break;
    }
    return rv;
}

void StateMachine::postupdate_states(Packet* pkt)
{
    /* 
     * Postupdate states. E.g., CLOSED if no bank 
     * is active after PRECAHRGE-ended operations 
     */
    uint64_t rank = je->adec->decode_addr(pkt->PADDR, FLD_RANK);
    rm[rank].postupdate_states(pkt);
}

void StateMachine::update_stats(ncycle_t cycles)
{
    if (cycles==0) return;
    for (uint64_t r=0; r<num_ranks; r++)
        rm[r].update_stats(cycles);
}

void StateMachine::calculate_stats( )
{
    for (uint64_t r=0; r<num_ranks; r++)
        rm[r].calculate_stats( );
}

std::string StateMachine::print_state(int state)
{
    std::string rv = "UNDEF";
    if (state==ST_CLOSED)
        rv = "CLOSED";
    else if (state==ST_OPEN)
        rv = "OPEN";
    else if (state==ST_REFRESH)
        rv = "REFRESHING";
    else if (state==ST_APD || state==ST_FPPD || state==ST_SPPD)
        rv = "POWERDOWN";
    else if (state==ST_SREF)
        rv = "SELF-REFRESHING";
    else
        assert(0);

    return rv;
}

ncycle_t StateMachine::get_tick_after(ncycle_t LAT)
{
    assert((int64_t)LAT>=0);
    return (je->getGlobalEventQueue( )->getCurrentTick( ) 
        + LAT*je->getTicksPerCycle( ));
}

void StateMachine::setup_timings(const uint64_t num_all_banks)
{
    /* Initialize timing-relatd parameters */
    uint64_t num_all_rows = je->info->get_rowsPerMAT( ) *
        je->info->get_partitions( ) * ((je->info->is_half_bank( ))? 2:1);

    XAW = je->getParamUINT64(".XAW", 4);
    order_fgr = je->getParamUINT64(".AR.FGR.order", 1);
    num_bundles = 8192*order_fgr; // 8192-bundle is defined by JEDEC spec.
    rows_per_bundle = num_all_rows/num_bundles;
    
    tCL = je->getParamUINT64(".tCL", 19);
    tRCD = je->getParamUINT64(".tRCD", 19);
    tRP = je->getParamUINT64(".tRP", 19);
    tRAS = je->getParamUINT64(".tRAS", 43);
    tCMD = je->getParamUINT64(".tCMD", 1);
    tBURST = je->getParamUINT64("global.tBURST", 4);
    tCWL = je->getParamUINT64(".tCWL", 18);
    tPPD = je->getParamUINT64(".tPPD", 0);
    tRTP = je->getParamUINT64(".tRTP", 10);
    tWR = je->getParamUINT64(".tWR", 20);
    tAL = je->getParamUINT64(".tAL", 0);
    tRRD_L = je->getParamUINT64(".tRRD_L", 9);
    tRRD_S = je->getParamUINT64(".tRRD_S", 8);
    tCCD_L = je->getParamUINT64(".tCCD_L", 7);
    tCCD_S = je->getParamUINT64(".tCCD_S", 4);
    tWTR_L = je->getParamUINT64(".tWTR_L", 10);
    tWTR_S = je->getParamUINT64(".tWTR_S", 4);
    tXAW = je->getParamUINT64(".tXAW", 40);
    tRTRS = je->getParamUINT64(".tRTRS", 1);

    tRDPDEN = je->getParamUINT64(".PD.tRDPDEN", 24);
    tWRPDEN = je->getParamUINT64(".PD.tWRPDEN", 42);
    tWRAPDEN = je->getParamUINT64(".PD.tWRAPDEN", 43);
    tPD = je->getParamUINT64(".PD.tPD", 7);
    tXP = je->getParamUINT64(".PD.tXP", 8);
    tXPDLL = je->getParamUINT64(".PD.tXPDLL", 32);

    /*
     * Set refresh parameters 
     * Please note that DDR5 does not have CKE, 
     * SREF is exited by a defined sequence, CS_n low to high
     * In PCMCsim, we regard this sequence as SRX,
     * which satisifies the minimum delay requirement DDRR5;
     * that is, tCARSX+tCSH_SRexit+tCSL_SRexit
     */
    tCKESR = je->getParamUINT64(".SR.tCKESR", 8);
    tXS = je->getParamUINT64(".SR.tXS", 270);
    tXSDLL = je->getParamUINT64(".SR.tXSDLL", 854);

    tREFW = je->getParamUINT64(".AR.tREFW", 85333334);
    tRFC = je->getParamUINT64(".AR.tRFC", 467);
    tREFI = tREFW/num_bundles;

    bool use_refresh = je->getParamBOOL(".AR", false);
    std::string refresh_type = je->getParamSTR(".AR.type", "ALL_BANK");
    sb_refresh = false;
    if (refresh_type=="ALL_BANK")
        banks_per_refresh = num_all_banks;
    else if (refresh_type=="PER_BANK")
        banks_per_refresh = 1;
    else if (refresh_type=="SAME_BANK")
    {
        sb_refresh = true;
        banks_per_refresh = 1;
    }
    else if (use_refresh)
    {
        std::cout << "Error! Refresh type is unavailable!" << std::endl;
        assert(0);
        exit(1);
    }

    /* Check on-die ECC option (D5 feature) */
    data_window = je->info->get_DQs( ) * je->info->get_prefetch_length( );
    ode = je->getParamBOOL(".ODE", false);
    if (ode)
    {
        ode_databits = je->getParamUINT64(".ODE.databits", 128);
        tCCD_L_WR = je->getParamUINT64(".tCCD_L_WR", 32);
        tCCD_L_WR2 = je->getParamUINT64(".tCCD_L_WR2", 16);
    }
    else
    {
        ode_databits = data_window;
        tCCD_L_WR = tCCD_L;
        tCCD_L_WR2 = tCCD_L;
    }

    /* Sanity check */
    if (tRAS<tRCD)
    {
        std::cout << "Error! tRAS can't be shorter than tRCD!" << std::endl;
        assert(0);
        exit(1);
    }
    else if ((int64_t)(tRCD-tAL)<=0)
    {
        std::cerr << "Error! tRCD-tAL should be positive!" << std::endl;
        assert(0);
        exit(1);
    }
    else if ((int64_t)(tCL+tBURST+tRTRS-tCWL)<=0)
    {
        std::cerr << "Error! tCL+tBURST+tRTRS-tCWL should be positive!" << std::endl;
        assert(0);
        exit(1);
    }
    else if ((int64_t)(tCWL+tBURST+tRTRS-tCL)<=0)
    {
        std::cerr << "Error! tCWL+tBUSRT+tRTRS-tCL should be positive!" << std::endl;
        assert(0);
        exit(1);
    }
    else if (use_refresh)
    {
        uint64_t refresh_in_tREFI = tRFC*num_ranks;
        if (sb_refresh)
            refresh_in_tREFI *= je->info->get_banks( );
        else
            refresh_in_tREFI *= num_all_banks;
        refresh_in_tREFI /= banks_per_refresh;

        if (refresh_in_tREFI>=tREFI)
        {
            std::cout << "Error! tRFC x # refreshes must "
                << "finish within tREFI!" << std::endl;
            assert(0);
            exit(1);
        }
    }

    timing_set = true;
}

void StateMachine::setup_currents( )
{
    /* Initialize energy-related parameters */
    if (timing_set==false)
    {
        std::cerr << "Please setup timing first by calling setup_timings()" << std::endl;
        assert(0);
        exit(1);
    }

    VDD = je->getParamFLOAT(".VDD", 1.2);
    IDD0 = je->getParamFLOAT(".IDD0", 55);
    IDD2P0 = je->getParamFLOAT(".IDD2P0", 30);
    IDD2P1 = je->getParamFLOAT(".IDD2P1", 30);
    IDD2N = je->getParamFLOAT(".IDD2N", 36);
    IDD3P = je->getParamFLOAT(".IDD3P", 32);
    IDD3N = je->getParamFLOAT(".IDD3N", 40);
    IDD4R = je->getParamFLOAT(".IDD4R", 151);
    IDD4W = je->getParamFLOAT(".IDD4W", 119);
    IDD5 = je->getParamFLOAT(".IDD5", 45);
    IDD6 = je->getParamFLOAT(".IDD6", 32);

    double freq = (double)(je->info->get_ctrl_freq( )) / 1e6; // MHz
    ncycle_t tRC = tRP + tRAS;
    
    E_act_inc = (IDD0*tRC - (IDD3N*tRAS + IDD2N*tRP)) / freq;
    E_rd_inc = (IDD4R-IDD3N) * tBURST / freq;
    E_wr_inc = (IDD4W-IDD3N) * tBURST / freq;
    E_refresh_inc = (IDD5-IDD3N) * tRFC / freq;
    E_sref_inc = IDD6 * VDD / freq;
    E_apd_inc = IDD3P * VDD / freq;
    E_sppd_inc = IDD2P0 * VDD / freq;
    E_fppd_inc = IDD2P1 * VDD / freq;
    E_stnby_act_inc = IDD3N * VDD / freq;
    E_stnby_pre_inc = IDD2N * VDD / freq;
}

uint64_t StateMachine::get_tCCD_L(cmd_t type)
{
    uint64_t rv = tCCD_L;
    if (ode && (type==CMD_WRITE || type==CMD_WRITE_PRE))
    {
        if (data_window<ode_databits)
            rv = tCCD_L_WR;
        else
            rv = tCCD_L_WR2;
    }

    return rv;
}

/* Definition of RankMachine */
RankMachine::RankMachine(StateMachine* sm_, uint64_t id_)
: id(id_), sm(sm_), state(ST_CLOSED), issuable_READ(0),
issuable_WRITE(0), XAW_ptr(0)
{
    num_bgs = sm->je->info->get_bankgroups( );
    num_banks = sm->je->info->get_banks( );
    num_all_banks = num_bgs*num_banks;
    name = sm->je->get_name( ) + ".rank[" + std::to_string(id) + "]";

    last_ACTs.resize(sm->XAW, 0);

    for (uint64_t i=0; i<num_all_banks; i++)
        bm.push_back(BankMachine(sm, i, id));

    /* Stats init */
    stats = new Stats( );

    cycles_apd = 0;
    cycles_fppd = 0;
    cycles_sppd = 0;
    cycles_sref = 0;
    cycles_stnby_act = 0;
    cycles_stnby_pre = 0;
    
    E_total = 0.0;
    E_bckgnd = 0.0;
    E_stnby = 0.0;
    E_refresh = 0.0;
    E_act = 0.0;
    E_rd = 0.0;
    E_wr = 0.0;

    avg_bandwidth = 0.0;
    util_bw = 0.0;
    
    num_ACT = 0;
    num_PRE = 0;
    num_REFRESH = 0;
    num_READ = 0;
    num_WRITE = 0;
    num_PD = 0;
    num_SREF = 0;
}

RankMachine::RankMachine(const RankMachine& obj)
{
    (*this) = obj;
    stats = new Stats( ); // deep-copy for instantiation
}

bool RankMachine::is_idle( )
{
    /* Check rank idleness */
    bool rank_idle = true;
    
    for (uint64_t i=0; i<num_all_banks; i++)
    {
        if (bm[i].is_idle( )==false)
        {
            rank_idle = false;
            break;
        }
    }
    return rank_idle;
}

bool RankMachine::is_issuable(Packet* pkt)
{
    bool rv = true;
    ncycle_t cur_tick = sm->je->getGlobalEventQueue( )->getCurrentTick( );
    uint64_t check_cnt = 0;
    uint64_t bm_idx = get_bm_idx(
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANKGRP),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANK));

    switch (pkt->cmd)
    {
        case CMD_ACT:
        case CMD_REFRESH:
            if ((last_ACTs[(XAW_ptr+1)%sm->XAW]+sm->tXAW)>cur_tick)
            {
                rv = false;
                PCMC_DBG(sm->je->is_msg( ), "[RM] uCMD"
                    " [0x%lx, ID=%lx, CMD=%s] violates XAW\n",
                    pkt->LADDR, pkt->req_id,
                    sm->je->get_cmd_str(pkt).c_str( ));
            }
            else
                rv = bm[bm_idx].is_issuable(pkt);

            if (pkt->cmd==CMD_REFRESH)
            {
                assert(bm_idx % sm->banks_per_refresh==0);

                if (sm->sb_refresh)
                {
                    assert(bm_idx<num_banks);
                    for (uint64_t bg=0; bg<num_bgs; bg++)
                    {
                        uint64_t offset = bg*num_banks+bm_idx;
                        for (uint64_t b=0; b<sm->banks_per_refresh; b++)
                        {
                            if (bm[offset+b].is_issuable(pkt)==false)
                            {
                                rv = false;
                                break;
                            }
                        }
                    }
                }
                else
                {
                    for (uint64_t i=0; i<sm->banks_per_refresh; i++)
                    {
                        if (bm[bm_idx+i].is_issuable(pkt)==false)
                        {
                            rv = false;
                            break;
                        }
                    }
                }
            }
            break;

        case CMD_READ:
        case CMD_READ_PRE:
            if (issuable_READ>cur_tick)
                rv = false;
            else 
                rv = bm[bm_idx].is_issuable(pkt);
            break;

        case CMD_WRITE:
        case CMD_WRITE_PRE:
            if (issuable_WRITE>cur_tick)
                rv = false;
            else
                rv = bm[bm_idx].is_issuable(pkt);
            break;

        case CMD_PRE:
            rv = bm[bm_idx].is_issuable(pkt);
            break;

        case CMD_APDE:
        case CMD_FPPDE:
        case CMD_SPPDE:
            if (state==ST_REFRESH)
                rv = false;
            else
            {
                for (uint64_t b=0; b<num_all_banks; b++)
                {
                    if (bm[b].is_issuable(pkt)==false)
                    {
                        rv = false;
                        break;
                    }
                }
            }
            break;

        case CMD_PDX:
            for (uint64_t b=0; b<num_all_banks; b++)
            {
                if (bm[b].is_issuable(pkt)==false)
                    rv = false;
                else
                    check_cnt += 1; // ensure all banks in PD
            }
            assert(check_cnt==0 || check_cnt==num_all_banks);
            break;

        case CMD_SRE:
            if (state!=ST_CLOSED)
                rv = false;
            else
            {
                for (uint64_t b=0; b<num_all_banks; b++)
                {
                    if (bm[b].is_issuable(pkt)==false)
                    {
                        rv = false;
                        break;
                    }
                }
            }
            break;

        case CMD_SRX:
            for (uint64_t b=0; b<num_all_banks; b++)
            {
                if (bm[b].is_issuable(pkt)==false)
                    rv = false;
                else
                    check_cnt += 1; // ensure all banks in SREF
            }
            assert(check_cnt==0 || check_cnt==num_all_banks);
            break;

        default:
            assert(0);
            break;
    }
    return rv;
}

ncycle_t RankMachine::timely_issuable(Packet* pkt)
{
    ncycle_t issuable = 0;
    uint64_t bm_idx = get_bm_idx(
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANKGRP),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANK));

    if (pkt->cmd==CMD_ACT || pkt->cmd==CMD_REFRESH)
        issuable = last_ACTs[(XAW_ptr+1)%sm->XAW] + sm->tXAW;
    else if (pkt->cmd==CMD_READ || pkt->cmd==CMD_READ_PRE)
        issuable = issuable_READ;
    else if (pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WRITE_PRE)
        issuable = issuable_WRITE;
    
    return MAX(issuable, bm[bm_idx].timely_issuable(pkt));
}

ncycle_t RankMachine::update_states(Packet* pkt)
{
    assert(is_issuable(pkt));
    PCMC_DBG(sm->je->is_msg( ), "[RM] uCMD [0x%lx, ID=%lx, CMD=%s]"
        " will access rank=%lu\n", pkt->LADDR, pkt->req_id,
        sm->je->get_cmd_str(pkt).c_str( ), id);

    /* Update states of ACCESSED rank & bank */
    ncycle_t rv = 0;
    switch (pkt->cmd)
    {
        case CMD_ACT: 
            this->activate(pkt);
            break;

        case CMD_READ:
        case CMD_READ_PRE:
        case CMD_WRITE:
        case CMD_WRITE_PRE:
            rv = this->rdwr(pkt);
            break;

        case CMD_PRE:
            this->precharge(pkt);
            break;

        case CMD_APDE:
        case CMD_FPPDE:
        case CMD_SPPDE:
            this->powerdown(pkt);
            break;

        case CMD_PDX:
            this->powerup(pkt);
            break;

        case CMD_REFRESH:
            this->refresh(pkt);
            break;

        case CMD_SRE:
            this->sre(pkt);
            break;

        case CMD_SRX:
            this->srx(pkt);
            break;

        default:
            assert(0);
            break;
    }

    /* Notify unaccessed banks to update timings */
    uint64_t pkt_bg = sm->je->adec->decode_addr(pkt->PADDR, FLD_BANKGRP);
    uint64_t bm_idx = get_bm_idx(
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANKGRP),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANK));
    for (uint64_t bg=0; bg<num_bgs; bg++)
    {
        for (uint64_t b=0; b<num_banks; b++)
        {
            uint64_t tmp_idx = get_bm_idx(bg, b);
            if (tmp_idx==bm_idx) continue;
            bm[tmp_idx].notify(pkt, bg==pkt_bg);
        }
    }

    return rv;
}

void RankMachine::postupdate_states(Packet* pkt)
{
    uint64_t bm_idx = get_bm_idx(
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANKGRP),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANK));

    if (pkt->cmd==CMD_READ_PRE ||
        pkt->cmd==CMD_WRITE_PRE)
    {
        bm[bm_idx].postupdate_states(pkt);

        if (is_idle( ))
        {
            state = ST_CLOSED;
            PCMC_DBG(sm->je->is_msg( ), "[RM] Rank=%lu is idle now\n", id);
        }
    }
    else if (pkt->cmd==CMD_REFRESH)
    {
        /* Notify refreshed banks for postupdating states */
        assert(bm_idx % sm->banks_per_refresh==0);

        if (sm->sb_refresh)
        {
            assert(bm_idx<num_banks);
            for (uint64_t bg=0; bg<num_bgs; bg++)
            {
                uint64_t offset = bg*num_banks+bm_idx;
                for (uint64_t b=0; b<sm->banks_per_refresh; b++)
                    bm[offset+b].postupdate_states(pkt);
            }
        }
        else
        {
            for (uint64_t i=0; i<sm->banks_per_refresh; i++)
                bm[bm_idx+i].postupdate_states(pkt);
        }

        if (is_idle( ))
        {
            state = ST_CLOSED;
            PCMC_DBG(sm->je->is_msg( ), "[RM] Rank=%lu is idle now\n", id);
        }
    }
    else
        assert(0); // not allow other types at this moment
}

void RankMachine::notify(Packet* pkt)
{
    /* Notify ranks except the accessed one */
    if (pkt->cmd==CMD_READ || pkt->cmd==CMD_READ_PRE)
    {
        issuable_READ = MAX(issuable_READ, sm->get_tick_after(
            sm->tBURST + sm->tRTRS));

        issuable_WRITE = MAX(issuable_WRITE, sm->get_tick_after(
            sm->tCL + sm->tBURST + sm->tRTRS - sm->tCWL));
    }
    else if (pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WRITE_PRE)
    {
        issuable_WRITE = MAX(issuable_WRITE, sm->get_tick_after(
            sm->tBURST)); // DDR3: latency is tBURST+MAX(tRTRS, tOST)

        issuable_READ = MAX(issuable_READ, sm->get_tick_after(
            sm->tBURST + sm->tCWL + sm->tRTRS - sm->tCL));
    }

    if (pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WRITE_PRE ||
        pkt->cmd==CMD_READ || pkt->cmd==CMD_READ_PRE)
    {
        PCMC_DBG(sm->je->is_msg( ), "[RM] Notify to update timing of rank=%lu" 
            " from rank=%lu by CMD=%s, issue-READ @ %lu, issue-WRITE @ %lu\n",
            id, sm->je->adec->decode_addr(pkt->PADDR, FLD_RANK),
            sm->je->get_cmd_str(pkt).c_str( ), issuable_READ, issuable_WRITE);
    }
}

void RankMachine::update_stats(ncycle_t cycles)
{
    /* Update BMs first */
    for (uint64_t b=0; b<num_all_banks; b++)
        bm[b].update_stats(cycles);

    /* Update rank-related stats */
    switch (state)
    {
        case ST_APD:
            cycles_apd += cycles;
            E_bckgnd += (sm->E_apd_inc*cycles);
            break;
        case ST_FPPD:
            cycles_fppd += cycles;
            E_bckgnd += (sm->E_fppd_inc*cycles);
            break;
        case ST_SPPD:
            cycles_sppd += cycles;
            E_bckgnd += (sm->E_sppd_inc*cycles);
            break;
        case ST_REFRESH:
        case ST_OPEN:
            cycles_stnby_act += cycles;
            E_bckgnd += (sm->E_stnby_act_inc*cycles);
            E_stnby += (sm->E_stnby_act_inc*cycles);
            break;
        case ST_CLOSED:
            cycles_stnby_pre += cycles;
            E_bckgnd += (sm->E_stnby_pre_inc*cycles);
            E_stnby += (sm->E_stnby_pre_inc*cycles);
            break;
        case ST_SREF:
            cycles_sref += cycles;
            E_bckgnd += (sm->E_sref_inc*cycles);
            break;
        default:
            assert(0);
            break;
    }
}

void RankMachine::calculate_stats( )
{
    double E_dynamic = 0.0; //ACT+RD+WR
    uint64_t bus_width = sm->je->info->get_DQs( ) * sm->je->info->get_devs( ) / 8;
    double max_bw = (double)(sm->je->info->get_ctrl_freq( ))*bus_width*2.0; //2 for DDR
    
    for (uint64_t b=0; b<num_all_banks; b++)
    {
        bm[b].calculate_stats( );
        E_dynamic += bm[b].get_dynamic_energy( );
        E_refresh += bm[b].E_refresh;
        E_act += bm[b].E_act;
        E_rd += bm[b].E_rd;
        E_wr += bm[b].E_wr;

        avg_bandwidth += bm[b].bandwidth;

        num_ACT += bm[b].num_ACT;
        num_PRE += bm[b].num_PRE;
        num_REFRESH += bm[b].num_REFRESH;
        num_READ += bm[b].num_READ;
        num_WRITE += bm[b].num_WRITE;
        num_PD += bm[b].num_PD;
        num_SREF += bm[b].num_SREF;
    }

    E_bckgnd += E_refresh;
    E_total = E_bckgnd + E_dynamic;

    avg_bandwidth = avg_bandwidth / num_all_banks;
    util_bw = avg_bandwidth / (max_bw/(double)(1ull<<20));
}

ncycle_t RankMachine::activate(Packet* pkt)
{
    uint64_t bm_idx = get_bm_idx(
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANKGRP),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANK));
    
    bm[bm_idx].update_states(pkt);

    if (state==ST_CLOSED)
    {
        state = ST_OPEN;
        PCMC_DBG(sm->je->is_msg( ), "[RM] Rank=%lu is active now\n", id);
    }

    XAW_ptr = (XAW_ptr+1) % sm->XAW;
    last_ACTs[XAW_ptr] = sm->je->getGlobalEventQueue( )->getCurrentTick( );
    return 0;
}

ncycle_t RankMachine::rdwr(Packet* pkt)
{
    uint64_t bm_idx = get_bm_idx(
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANKGRP),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANK));
    
    return bm[bm_idx].update_states(pkt);
}

ncycle_t RankMachine::precharge(Packet* pkt)
{
    uint64_t bm_idx = get_bm_idx(
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANKGRP),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANK));
    
    bm[bm_idx].update_states(pkt);

    /* Update rank state */
    if (is_idle( ))
    {
        state = ST_CLOSED;
        PCMC_DBG(sm->je->is_msg( ), "[RM] Rank=%lu is idle now\n", id);
    }

    return 0;
}

ncycle_t RankMachine::powerdown(Packet* pkt)
{
    for (uint64_t i=0; i<num_all_banks; i++)
        bm[i].update_states(pkt);

    switch (pkt->cmd)
    {
        case CMD_APDE:
            state = ST_APD;
            break;
        case CMD_FPPDE:
            state = ST_FPPD;
            break;
        case CMD_SPPDE:
            state = ST_SPPD;
            break;
        default:
            assert(0);
            break;
    }
    return 0;
}

ncycle_t RankMachine::powerup(Packet* pkt)
{
    /* Wakeup all banks */
    for (uint64_t i=0; i<num_all_banks; i++)
        bm[i].update_states(pkt);

    switch (state)
    {
        case ST_APD:
            state = ST_OPEN;
            break;
        case ST_FPPD:
        case ST_SPPD:
            state = ST_CLOSED;
            break;
        default:
            assert(0);
            break;
    }
    return 0;
}

ncycle_t RankMachine::refresh(Packet* pkt)
{
    uint64_t base_refresh = get_bm_idx(
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANKGRP),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_BANK));

    if (sm->sb_refresh)
    {
        assert(base_refresh<num_banks);
        for (uint64_t bg=0; bg<num_bgs; bg++)
        {
            uint64_t offset = bg*num_banks+base_refresh;
            for (uint64_t b=0; b<sm->banks_per_refresh; b++)
                bm[offset+b].update_states(pkt);
        }
    }
    else
    {
        for (uint64_t i=0; i<sm->banks_per_refresh; i++)
            bm[base_refresh+i].update_states(pkt);
    }

    state = ST_REFRESH;
    XAW_ptr = (XAW_ptr+1) % sm->XAW;
    last_ACTs[XAW_ptr] = sm->je->getGlobalEventQueue( )->getCurrentTick( );
    return 0;
}

ncycle_t RankMachine::sre(Packet* pkt)
{
    for (uint64_t i=0; i<num_all_banks; i++)
        bm[i].update_states(pkt);
    
    state = ST_SREF;

    return 0;
}

ncycle_t RankMachine::srx(Packet* pkt)
{
    for (uint64_t i=0; i<num_all_banks; i++)
        bm[i].update_states(pkt);

    state = ST_CLOSED;

    return 0;
}

/* Definition of BankMachine */
BankMachine::BankMachine(StateMachine* sm_, uint64_t id_, uint64_t rank_id)
: id(id_), sm(sm_), state(ST_CLOSED), issuable_ACT(0), issuable_PRE(0),
issuable_READ(0), issuable_WRITE(0), issuable_REFRESH(0),
issuable_PDE(0), issuable_PDX(0), issuable_SRE(0), issuable_SRX(0)
{
    num_parts = sm->je->info->get_partitions( );
    half_banks = (sm->je->info->is_half_bank( ))? 2 : 1;
    num_rows_per_part = sm->je->info->get_rowsPerMAT( );
    num_rows = half_banks * num_parts * num_rows_per_part;
    open_row = num_rows;

    name = sm->je->get_name( ) + ".rank[" + std::to_string(rank_id) + 
        "].bank[" +  std::to_string(id) + "]";

    /* Init stats */
    stats = new Stats( );

    cycles_act = 0;
    cycles_bckgnd = 0;
    cycles_io = 0;
    
    num_ACT = 0;
    num_PRE = 0;
    num_REFRESH = 0;
    num_READ = 0;
    num_WRITE = 0;
    num_PD = 0;
    num_SREF = 0;
    
    E_act = 0.0;
    E_rd = 0.0;
    E_wr = 0.0;
    E_refresh = 0.0;
}

BankMachine::BankMachine(const BankMachine& obj)
{
    (*this) = obj;
    stats = new Stats( ); // deep-copy for instantiation
}

bool BankMachine::is_idle( )
{
    return (state==ST_CLOSED);
}

bool BankMachine::is_issuable(Packet* pkt)
{
    bool rv = true;
    ncycle_t cur_tick = sm->je->getGlobalEventQueue( )->getCurrentTick( );
    uint64_t row_idx = get_row_idx(
        sm->je->adec->decode_addr(pkt->PADDR, FLD_HALF),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_PART),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_ROW));
        
    switch (pkt->cmd)
    {
        case CMD_ACT:
            if (issuable_ACT>cur_tick || 
                state==ST_APD ||
                state==ST_FPPD ||
                state==ST_SPPD ||
                state==ST_SREF ||
                state!=ST_CLOSED) // need precharge
                rv = false;
            else
                rv = true; 
            break;

        case CMD_READ:
        case CMD_READ_PRE:
            if (issuable_READ>cur_tick || 
                state!=ST_OPEN ||
                open_row!=row_idx)
                rv = false;
            else
                rv = true; 
            break;

        case CMD_WRITE:
        case CMD_WRITE_PRE:
            if (issuable_WRITE>cur_tick || 
                state!=ST_OPEN ||
                open_row!=row_idx)
                rv = false;
            else
                rv = true;
            break;

        case CMD_PRE:
            if (issuable_PRE>cur_tick ||
                (state!=ST_CLOSED && state!=ST_OPEN))
                rv = false;
            else 
                rv = true;
            break;

        case CMD_APDE:
        case CMD_FPPDE:
        case CMD_SPPDE:
            if (issuable_PDE>cur_tick ||
                (state!=ST_CLOSED && state!=ST_OPEN) ||
                 ((pkt->cmd==CMD_FPPDE || pkt->cmd==CMD_SPPDE) &&
                  state==ST_OPEN))
                rv = false;
            else
                rv = true;
            break;

        case CMD_PDX:
            if (issuable_PDX>cur_tick ||
                (state!=ST_APD && 
                 state!=ST_FPPD && 
                 state!=ST_SPPD))
                rv = false;
            else
                rv = true;
            break;

        case CMD_REFRESH: // NOTE: issuable_REFRESH==issuable_ACT
            if (issuable_ACT>cur_tick || state!=ST_CLOSED)
//            if (issuable_ACT>cur_tick || //XXX
//                (state!=ST_CLOSED && state!=ST_OPEN))
                rv = false;
            else
                rv = true; 
            break;

        case CMD_SRE:
            if (issuable_SRE>cur_tick || state!=ST_CLOSED)
                rv = false;
            else
                rv = true;
            break;

        case CMD_SRX:
            if (issuable_SRX>cur_tick || state!=ST_SREF)
                rv = false;
            else
                rv = true;
            break;

        default:
            assert(0);
            break;
    }
    return rv;
}

ncycle_t BankMachine::timely_issuable(Packet* pkt)
{
    ncycle_t issuable = 0;
    if (pkt->cmd==CMD_ACT || pkt->cmd==CMD_REFRESH)
        issuable = issuable_ACT;
    else if (pkt->cmd==CMD_READ || pkt->cmd==CMD_READ_PRE)
        issuable = issuable_READ;
    else if (pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WRITE_PRE)
        issuable = issuable_WRITE;
    else if (pkt->cmd==CMD_PRE)
        issuable = issuable_PRE;
    else if (pkt->cmd==CMD_APDE || pkt->cmd==CMD_FPPDE || pkt->cmd==CMD_SPPDE)
        issuable = issuable_PDE;
    else if (pkt->cmd==CMD_PDX)
        issuable = issuable_PDX;
    else if (pkt->cmd==CMD_SRE)
        issuable = issuable_SRE;
    else if (pkt->cmd==CMD_SRX)
        issuable = issuable_SRX;
    else
        assert(0);
    
    return issuable;
}

ncycle_t BankMachine::update_states(Packet* pkt)
{
    assert(is_issuable(pkt));
    ncycle_t rv = 0;

    /* Update states of ACCESSED banks */
    switch (pkt->cmd)
    {
        case CMD_ACT:
            this->activate(pkt);
            break;

        case CMD_READ:
        case CMD_READ_PRE:
            rv = this->read(pkt);
            break;

        case CMD_WRITE:
        case CMD_WRITE_PRE:
            rv = this->write(pkt);
            break;

        case CMD_PRE:
            this->precharge(pkt);
            break;

        case CMD_APDE:
        case CMD_FPPDE:
        case CMD_SPPDE:
            this->powerdown(pkt);
            break;

        case CMD_PDX:
            this->powerup(pkt);
            break;

        case CMD_REFRESH:
            this->refresh(pkt);
            break;

        case CMD_SRE:
            this->sre(pkt);
            break;

        case CMD_SRX:
            this->srx(pkt);
            break;

        default:
            assert(0);
            break;
    }

    PCMC_DBG(sm->je->is_msg( ), "[BM] uCMD [0x%lx, ID=%lx, CMD=%s]"
        " will access bank=%lu(%lu, %lu)."
        " Timings: issue-ACT @ %lu, issue-PRE @ %lu,"
        " issue-RD @ %lu, issue-WR @ %lu, issue-REFRESH @ %lu,"
        " issue-PD @ %lu, issue-POWERUP @ %lu\n", 
        pkt->LADDR, pkt->req_id, sm->je->get_cmd_str(pkt).c_str( ), 
        id, id/(sm->je->info->get_banks( )), 
        id%(sm->je->info->get_banks( )), 
        issuable_ACT, issuable_PRE, issuable_READ, issuable_WRITE,
        issuable_REFRESH, issuable_PDE, issuable_PDX);

    return rv;
}

void BankMachine::postupdate_states(Packet* pkt)
{
    if (pkt->cmd==CMD_READ_PRE ||
        pkt->cmd==CMD_WRITE_PRE ||
        pkt->cmd==CMD_REFRESH)
    {
        state = ST_CLOSED;
        open_row = num_rows;
        
        PCMC_DBG(sm->je->is_msg( ), "[BM] bank=(%lu, %lu) will"
            " be auto-prechareged by CMD [0x%lx, ID=%lx, CMD=%s]\n", 
            id/(sm->je->info->get_banks( )), 
            id%(sm->je->info->get_banks( )), pkt->LADDR, 
            pkt->req_id, sm->je->get_cmd_str(pkt).c_str( ));
    }
    else
        assert(0);
}

void BankMachine::notify(Packet* pkt, bool is_same_bankgroup)
{
    /* Notify all banks except the accessed one */
    ncycle_t tRRD = (is_same_bankgroup)? sm->tRRD_L : sm->tRRD_S;
    ncycle_t tCCD = (is_same_bankgroup)? sm->get_tCCD_L(pkt->cmd) : sm->tCCD_S;
    ncycle_t tWTR = (is_same_bankgroup)? sm->tWTR_L : sm->tWTR_S;

    bool updated = true;
    if (pkt->cmd==CMD_READ || pkt->cmd==CMD_READ_PRE)
    {
        issuable_READ = MAX(issuable_READ, sm->get_tick_after(
            MAX(sm->tBURST, tCCD)));

        issuable_WRITE = MAX(issuable_WRITE, sm->get_tick_after(
            sm->tAL + sm->tCL + sm->tBURST + sm->tRTRS - sm->tCWL));
    }
    else if (pkt->cmd==CMD_WRITE || pkt->cmd==CMD_WRITE_PRE)
    {
        issuable_READ = MAX(issuable_READ, sm->get_tick_after(
            sm->tCWL + sm->tBURST + tWTR));

        issuable_WRITE = MAX(issuable_WRITE, sm->get_tick_after(
            MAX(sm->tBURST, tCCD)));
    }
    else if (pkt->cmd==CMD_REFRESH)
    {
        issuable_ACT = MAX(issuable_ACT, sm->get_tick_after(sm->tRFC));
        issuable_REFRESH = MAX(issuable_REFRESH, sm->get_tick_after(sm->tRFC));
    }
    else if (pkt->cmd==CMD_ACT)
        issuable_ACT = MAX(issuable_ACT, sm->get_tick_after(tRRD));
    else if (pkt->cmd==CMD_PRE)
        issuable_PRE = MAX(issuable_PRE, sm->get_tick_after(sm->tPPD));
    else
        updated = false;

    if (updated)
    {
        PCMC_DBG(sm->je->is_msg( ), "[BM] Notify banks in"
            " %s BGs, bank=%lu(%lu, %lu), except the accessed one."
            " Timings: issue-ACT @ %lu, issue-PRE @ %lu,"
            " issue-RD @ %lu, issue-WR @ %lu, issue-REFRESH @ %lu,"
            " issue-PD @ %lu, issue-POWERUP @ %lu\n", 
            (is_same_bankgroup)? "same":"diff",
            id, id/(sm->je->info->get_banks( )), 
            id%(sm->je->info->get_banks( )), 
            issuable_ACT, issuable_PRE, issuable_READ, 
            issuable_WRITE, issuable_REFRESH, 
            issuable_PDE, issuable_PDX);
    }
}

void BankMachine::update_stats(ncycle_t cycles)
{
    /* Update bank-related stats */
    switch (state)
    {
        case ST_APD:
        case ST_FPPD:
        case ST_SPPD:
        case ST_REFRESH:
        case ST_CLOSED:
        case ST_SREF:
            cycles_bckgnd += cycles;
            break;
        case ST_OPEN:
            cycles_act += cycles;
            break;
        default:
            assert(0);
            break;
    }
}

void BankMachine::calculate_stats( )
{
    uint64_t bus_width = sm->je->info->get_DQs( ) * sm->je->info->get_devs( ) / 8;
    double max_bw = (double)(sm->je->info->get_ctrl_freq( ))*bus_width*2; //2 for DDR
    if (cycles_act!=0)
        util_bw = (double)cycles_io / (cycles_act+cycles_bckgnd);
    else
        util_bw = 0.0;
    bandwidth = util_bw * max_bw / (double)(1ull<<20); // B/s->MB/s

    uint64_t num_devs = sm->je->info->get_devs( );
    E_act *= (sm->VDD * num_devs);
    E_rd *= (sm->VDD * num_devs);
    E_wr *= (sm->VDD * num_devs);
    E_refresh *= (sm->VDD * num_devs);
}

ncycle_t BankMachine::activate(Packet* pkt)
{
    assert(state==ST_CLOSED || open_row==num_rows);
    uint64_t row_idx = get_row_idx(
        sm->je->adec->decode_addr(pkt->PADDR, FLD_HALF),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_PART),
        sm->je->adec->decode_addr(pkt->PADDR, FLD_ROW));

    issuable_ACT = MAX(issuable_ACT, sm->get_tick_after(
        sm->tRAS + sm->tRP));
    issuable_READ = MAX(issuable_READ, sm->get_tick_after(
        sm->tRCD - sm->tAL)); 
    issuable_WRITE = MAX(issuable_WRITE, sm->get_tick_after(
        sm->tRCD - sm->tAL));
    issuable_PRE = MAX(issuable_PRE, sm->get_tick_after(sm->tRAS));
    issuable_PDE = MAX(issuable_PDE, sm->get_tick_after(sm->tRAS));

    state = ST_OPEN;
    open_row = row_idx;

    /* Update stats */
    num_ACT += 1;
    E_act += sm->E_act_inc;
    
    return 0;
}

ncycle_t BankMachine::precharge(Packet* /*pkt*/)
{
    issuable_ACT = MAX(issuable_ACT, sm->get_tick_after(sm->tRP));
    issuable_REFRESH = MAX(issuable_PRE, sm->get_tick_after(sm->tRP));
    issuable_SRE = MAX(issuable_SRE, sm->get_tick_after(sm->tRP));
    issuable_PDE = MAX(issuable_PDE, sm->get_tick_after(sm->tRP));

    state = ST_CLOSED;
    open_row = num_rows;

    num_PRE += 1;

    return 0;
}

ncycle_t BankMachine::read(Packet* pkt)
{
    if (pkt->cmd==CMD_READ_PRE)
    {
        issuable_ACT = MAX(issuable_ACT, sm->get_tick_after(
            sm->tAL + sm->tBURST + sm->tRTP + sm->tRP));
        issuable_REFRESH = MAX(issuable_REFRESH, sm->get_tick_after(
            sm->tAL + sm->tRTP + sm->tRP));
        issuable_SRE = MAX(issuable_SRE, sm->get_tick_after(
            sm->tAL + sm->tRTP + sm->tRP));
        num_PRE += 1;
    }
    else
    {
        issuable_READ = MAX(issuable_READ, sm->get_tick_after(
            MAX(sm->tBURST, sm->get_tCCD_L(CMD_READ))));
        issuable_WRITE = MAX(issuable_WRITE,  sm->get_tick_after(
            sm->tCL + sm->tBURST + sm->tRTRS - sm->tCWL));
        issuable_PRE = MAX(issuable_PRE, sm->get_tick_after(
            sm->tAL+sm->tRTP));
    }

    issuable_PDE = MAX(issuable_PDE, sm->get_tick_after(sm->tRDPDEN));

    /* Update stats */
    num_READ += 1;
    cycles_io += sm->tBURST;
    E_rd += sm->E_rd_inc;

    /* Return data response latency */
    ncycle_t rv = sm->tCL + sm->tBURST;
    return rv;
}

ncycle_t BankMachine::write(Packet* pkt)
{
    if (pkt->cmd==CMD_WRITE_PRE)
    {
        issuable_ACT = MAX(issuable_ACT, sm->get_tick_after(
            sm->tCWL + sm->tBURST + sm->tWR + sm->tRP));
        issuable_REFRESH = MAX(issuable_REFRESH, sm->get_tick_after(
            sm->tCWL + sm->tBURST + sm->tWR + sm->tRP));
        issuable_SRE = MAX(issuable_SRE, sm->get_tick_after(
            sm->tCWL + sm->tBURST + sm->tWR + sm->tRP));
        issuable_PDE = MAX(issuable_PDE, sm->get_tick_after(sm->tWRAPDEN));
        num_PRE += 1;
    }
    else
    {
        issuable_READ = MAX(issuable_READ, sm->get_tick_after(
            sm->tCWL + sm->tBURST + sm->tWTR_L));
        issuable_WRITE = MAX(issuable_WRITE, sm->get_tick_after(
            MAX(sm->tBURST, sm->get_tCCD_L(CMD_WRITE))));
        issuable_PRE = MAX(issuable_PRE, sm->get_tick_after(
            sm->tCWL + sm->tBURST + sm->tWR));
        issuable_PDE = MAX(issuable_PDE, sm->get_tick_after(sm->tWRPDEN));
    }

    /* Update stats */
    num_WRITE += 1;
    cycles_io += sm->tBURST;
    E_wr += sm->E_wr_inc;

    ncycle_t rv = sm->tCWL + sm->tBURST + sm->tWR;
    return rv;
}

ncycle_t BankMachine::powerdown(Packet* pkt)
{
    issuable_PDX = MAX(issuable_PDX, sm->get_tick_after(sm->tPD));

    if (state==ST_OPEN)
    {
        assert(pkt->cmd==CMD_APDE);
        state = ST_APD;
    }
    else if (state==ST_CLOSED)
    {
        if (pkt->cmd==CMD_APDE || pkt->cmd==CMD_FPPDE)
            state = ST_FPPD;
        else if (pkt->cmd==CMD_SPPDE)
            state = ST_SPPD;
        else
            assert(0);
    }
    else
        assert(0);

    num_PD += 1;

    return 0;
}

ncycle_t BankMachine::powerup(Packet* /*pkt*/)
{
    issuable_ACT = MAX(issuable_ACT, sm->get_tick_after(sm->tXP));
    issuable_PRE = MAX(issuable_PRE, sm->get_tick_after(sm->tXP));
    issuable_WRITE = MAX(issuable_WRITE, sm->get_tick_after(sm->tXP));
    issuable_PDE = MAX(issuable_PDE, sm->get_tick_after(sm->tXP));
    if (state==ST_SPPD)
        issuable_READ = MAX(issuable_READ, sm->get_tick_after(sm->tXPDLL));
    else
        issuable_READ = MAX(issuable_READ, sm->get_tick_after(sm->tXP));

    if (state==ST_APD)
        state = ST_OPEN;
    else
        state = ST_CLOSED;

    return 0;
}

ncycle_t BankMachine::refresh(Packet* /*pkt*/)
{
    state = ST_REFRESH;
    issuable_ACT = MAX(issuable_ACT, sm->get_tick_after(sm->tRFC));
    issuable_REFRESH = MAX(issuable_REFRESH, sm->get_tick_after(sm->tRFC));
    issuable_SRE = MAX(issuable_SRE, sm->get_tick_after(sm->tRFC));
    issuable_PDE = MAX(issuable_PDE, sm->get_tick_after(sm->tRFC));

    /* Update stats */
    num_REFRESH += 1;
    E_refresh += sm->E_refresh_inc;

    return 0;
}

ncycle_t BankMachine::sre(Packet* /*pkt*/)
{
    state = ST_SREF;
    issuable_SRX = MAX(issuable_SRX, sm->get_tick_after(sm->tCKESR));

    num_SREF += 1;

    return 0;
}

ncycle_t BankMachine::srx(Packet* /*pkt*/)
{
    state = ST_CLOSED;
    issuable_ACT = MAX(issuable_ACT, sm->get_tick_after(sm->tXS));
    issuable_PRE = MAX(issuable_PRE, sm->get_tick_after(sm->tXS));
    issuable_REFRESH = MAX(issuable_REFRESH, sm->get_tick_after(sm->tXS));
    issuable_WRITE = MAX(issuable_WRITE, sm->get_tick_after(sm->tXS));
    issuable_PDE = MAX(issuable_PDE, sm->get_tick_after(sm->tXS));
    issuable_SRE = MAX(issuable_SRE, sm->get_tick_after(sm->tXS));
    issuable_READ = MAX(issuable_READ, sm->get_tick_after(sm->tXSDLL));
    
    return 0;
}

/* Stats registration & print */
void StateMachine::register_stats( )
{
    for (uint64_t r=0; r<num_ranks; r++)
        rm[r].register_stats( );
}

void RankMachine::register_stats( )
{
    for (uint64_t b=0; b<num_bgs*num_banks; b++)
        bm[b].register_stats( );

    ADD_STATS(name, num_ACT);
    ADD_STATS(name, num_PRE);
    ADD_STATS(name, num_REFRESH);
    ADD_STATS(name, num_READ);
    ADD_STATS(name, num_WRITE);
    ADD_STATS(name, num_PD);
    ADD_STATS(name, num_SREF);

    ADD_STATS_N_UNIT(name, E_stnby, "nJ");
    ADD_STATS_N_UNIT(name, E_refresh, "nJ");
    ADD_STATS_N_UNIT(name, E_act, "nJ");
    ADD_STATS_N_UNIT(name, E_rd, "nJ");
    ADD_STATS_N_UNIT(name, E_wr, "nJ");
    ADD_STATS_N_UNIT(name, E_bckgnd, "nJ");
    ADD_STATS_N_UNIT(name, E_total, "nJ");
    
    ADD_STATS_N_UNIT(name, avg_bandwidth, "MB/s");
    ADD_STATS(name, util_bw);
}

void BankMachine::register_stats( )
{
    ADD_STATS_N_UNIT(name, E_refresh, "nJ");
    ADD_STATS_N_UNIT(name, E_act, "nJ");
    ADD_STATS_N_UNIT(name, E_rd, "nJ");
    ADD_STATS_N_UNIT(name, E_wr, "nJ");
    
    ADD_STATS_N_UNIT(name, bandwidth, "MB/s");
    ADD_STATS(name, util_bw);
}

void StateMachine::print_stats(std::ostream& os)
{
    for (uint64_t r=0; r<num_ranks; r++)
        rm[r].print_stats(os);
}

void RankMachine::print_stats(std::ostream& os)
{
    for (uint64_t b=0; b<num_bgs*num_banks; b++)
        bm[b].print_stats(os);
    stats->print(os);
}

void BankMachine::print_stats(std::ostream& os)
{
    stats->print(os);
}

