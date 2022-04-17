#include "base/MemoryControlSystem.h"
#include "base/EventQueue.h"
#include "base/Packet.h"
#include "base/PipeBufferv2.h"
#include "base/Stats.h"
#include "base/MemInfo.h"
#include "base/PCMInfo.h"
#include "ReadModifyWrite/ReadModifyWrite.h"
#include "AITManager/AITManager.h"
#include "DataCache/DataCache.h"
#include "TraceGen/TraceGen.h"

#define NO_MERGE_STR            0
#define NEED_MERGE_STR          std::numeric_limits<uint64_t>::max( )
#define PRIORITY_DBUF_RETIRE    10
using namespace PCMCsim;

typedef struct _signals_t
{
    Packet* pkt;
} signals_t;

ReadModifyWrite::ReadModifyWrite(MemoryControlSystem* memsys_, std::string cfg_header): 
Component(memsys_), dcache(NULL), aitm(NULL), errMgr(NULL), 
ucmde(NULL), dpu(NULL), mcu(NULL), need_wakeup(true), num_blkmgr_ctrl(0),
src_id_IA(0), st_inArbtr(ST_NORMAL), st_hzdCheck(NO_STALL),
flushRMW_RD(false), flushRMW_WR(false), flushCntRMW_RD(0), flushCntRMW_WR(0),
starvCntRMW_RD(0), starvCntRMW_WR(0), wbuffer_idx_dpu(0), wake_outArbtr(0), 
last_wake_outArbtr(0), free_outArbtr(0), wake_wdcache(0), 
last_wake_wdcache(0), free_wdcache(0), wake_rdbuf(0), last_wake_rdbuf(0), 
free_rdbuf(0), wake_wdbuf(0), last_wake_wdbuf(0), free_wdbuf(0) 
{
    assert(memsys);
    cp_name = cfg_header;
    dbg_msg = memsys->getParamBOOL(cp_name+".dbg_msg", false);
    ticks_per_cycle= memsys->getParamUINT64("global.ticks_per_cycle", 1);

    PAGE_OFFSET = memsys->adec->PAGE_OFFSET;
    HOST_TX_SIZE = memsys->info->HOST_TX_SIZE;
    PAGE_SIZE = memsys->info->PAGE_SIZE;
    META_SIZE = memsys->info->META_SIZE;

    /* Input arbiter setting */
    numReqPorts = NUM_SRC;
    
    cmdq.resize(numReqPorts);
    size_cmdq.resize(numReqPorts);
    for (uint64_t i=0; i<numReqPorts; i++)
        size_cmdq[i] = memsys->getParamUINT64(cp_name+".size_cmdq", 8);
    assert(size_cmdq[0]>=2 && size_cmdq[0]%2==0);

    servCntrs.resize(numReqPorts, 0);
    servWeights.resize(numReqPorts, 1);
    for (uint64_t i = 0; i < numReqPorts; i++)
    {
        std::string param_name = cp_name+".servWeights[" + std::to_string(i) + "]";
        servWeights[i] = memsys->getParamUINT64(param_name, 1);
        assert(servWeights[i]>0);
    }

    /* Output arbiter setting */
    maxLevelRMWQ_RD = memsys->getParamUINT64(cp_name+".maxLevelRMWQ_RD", 32);
    maxFlushRMW_RD = memsys->getParamUINT64(cp_name+".maxFlushRMW_RD", 16);
    maxStarvRMW_RD = memsys->getParamUINT64(cp_name+".maxStarvRMW_RD", 16);
    maxLevelRMWQ_WR = memsys->getParamUINT64(cp_name+".maxLevelRMWQ_WR", 32);
    maxFlushRMW_WR = memsys->getParamUINT64(cp_name+".maxFlushRMW_WR", 16);
    maxStarvRMW_WR = memsys->getParamUINT64(cp_name+".maxStarvRMW_WR", 16);

    /* RMWQ & DBUF param setting */
    size_rmwq = memsys->getParamUINT64(cp_name+".size_rmwq", 64);    
    assert(size_rmwq>0);

    size_dbuf.resize(numReqPorts);
    base_dbuf.resize(numReqPorts);
    for (uint64_t i=0; i<numReqPorts; i++)
    {
        if (i==SRC_HOST)
            size_dbuf[i] = memsys->getParamUINT64(cp_name+".size_dbuf.HostRMW", 64);
        else if (i==SRC_WDTP)
            size_dbuf[i] = memsys->getParamUINT64(cp_name+".size_dbuf.WDTP", 4);
        else if (i==SRC_RDTP)
            size_dbuf[i] = memsys->getParamUINT64(cp_name+".size_dbuf.RDTP", 1);
        else if (i==SRC_PTRSCB)
            size_dbuf[i] = memsys->getParamUINT64(cp_name+".size_dbuf.PTRSCB", 1);
        else // block move (BLKMV) and block swap (BLKSWP)
            size_dbuf[i] = memsys->getParamUINT64(cp_name+".size_dbuf.BLKMGR", 64);
        assert(size_dbuf[i]>0);

        size_all_dbuf+=size_dbuf[i];
        if (i+1<numReqPorts)
            base_dbuf[i+1] = size_all_dbuf;
    }

    dbuf.resize(size_all_dbuf);
    for (uint64_t i=0; i <size_all_dbuf; i++)
        dbuf[i] = NULL;

    wcmd_issued.pkt = NULL;
    wcmd_issued.link.first = -1;
    wcmd_issued.link.second = -1;
    wcmd_issued.wdata_issued = false;

    /* Interrupt threshold */
    intr_th_WLV = memsys->getParamUINT64(cp_name+".intr_th_WLV", 1024);
    max_pwcnt = (uint64_t)1 << memsys->getParamUINT64("global.meta.pwcnt", 22); 

    /* Timing variables */
    tCMD = memsys->getParamUINT64(cp_name+".tCMD", 1);
    tDBUF = memsys->getParamUINT64(cp_name+".tDBUF", 1);
    tBURST = memsys->getParamUINT64(cp_name+".tBURST", 4);

    /* Pipeline buffers on main path */
    pipe_hzdCheck = new PipeBufferv2( );
    
    /* Hard-fixed params */
    size_nonHzdHostRD_cmdq = 1;
    size_rdq.resize(NUM_RDQ); // not specified 
    rdq.resize(NUM_RDQ);

    /* Stats registration */
    register_stats( );
}

ReadModifyWrite::~ReadModifyWrite( )
{
    delete pipe_hzdCheck;
    
    for (uint64_t i=0; i<size_all_dbuf; i++)
    {
        if (dbuf[i]==NULL) continue;

        delete dbuf[i];
    }
}

ReadModifyWrite::rmwqe_t::rmwqe_t( )
{
    pkt = NULL;
    issuable = false;
    RD_issued = false;
    RD_only = false;
    remap_link.first = -1;
    remap_link.second = -1;
}

void ReadModifyWrite::rmwqe_t::link(std::list<rmwqe_t>::iterator entry)
{
    remap_link.first = entry->pkt->src_id;
    remap_link.second = entry->pkt->req_id;

    entry->remap_link.first = pkt->src_id;
    entry->remap_link.second = pkt->req_id;
}

ReadModifyWrite::dbe_t::dbe_t(uint64_t num_data)
{
    assert(num_data>0);
    alloc = true;
    dcache_read = 0;
    meta_issued = false;
    dvalid.resize(num_data, false);
    mvalid = false;
    pkt = NULL;
    HostHzdPkt_RD = NULL;
}

ReadModifyWrite::dbe_t::~dbe_t( )
{
}

bool ReadModifyWrite::isReady(Packet* pkt)
{
    bool rv = false;
    if (cmdq[pkt->src_id].size( )<size_cmdq[pkt->src_id])
        rv = true;

    /* Ensuring wdata is completely out to prevent ID confliction */
    if (pkt->src_id==SRC_BLKMGR && 
        st_inArbtr==ST_NORMAL && 
        num_blkmgr_ctrl>0)
        rv = false;

    return rv;
}

void ReadModifyWrite::recvRequest(Packet* pkt, ncycle_t delay)
{
    assert(pkt->src_id>=0);

    cmdq[pkt->src_id].push(pkt);
    pkt->recvTick = geq->getCurrentTick( )+delay*ticks_per_cycle;

    /* Trigger protocol for BLKMV/SWP */
    if (pkt->src_id==SRC_HOST)
        num_HOST+=1;
    else if (pkt->src_id==SRC_BLKMGR)
    {
        num_blkmgr_ctrl+=1;
        num_BLKMGR+=1;
        if (st_inArbtr==ST_NORMAL)
            st_inArbtr = ST_FORBID_BLKMGR;

        PCMC_DBG(dbg_msg, "[RMW] A new BLKMGR comes (ID=%d), "
            "flush cmdq[AT] first to prevent from writing "
            "data on previously mapped PAs\n", (int)pkt->req_id);
    }

    if (need_wakeup)
    {
        need_wakeup = false;
        registerCallback((CallbackPtr)&ReadModifyWrite::execute, delay);
    }
}

void ReadModifyWrite::recvResponse(Packet* pkt, ncycle_t delay)
{
    /* 
     * It is always called from DPU/DCACHE after all RMW-events 
     * are processed according to data structure of the event queue
     */
    if (pkt->cmd==CMD_PKT_DEL)
    {
        assert(pkt->owner==this);
        delete pkt;
    }
    else if (pkt->cmd==CMD_WACK_ID)
        wack(pkt, CMD_WACK_ID);
    else if (pkt->from==dynamic_cast<Component*>(dpu) ||
             pkt->from==dynamic_cast<Component*>(dcache))
    {
        /* Will process received data according to source */
        Component::recvResponse(pkt, delay);
    }
    else
        assert(0);
}

void ReadModifyWrite::handle_events(ncycle_t curr_tick)
{
    prepare_events(curr_tick);

    if (!await_resp.empty( ))
        handle_await_resps( );

    if (!await_cb.empty( ))
        handle_await_callbacks( );
}

bool ReadModifyWrite::is_cmdq_avail( )
{
    bool rv = false;
    for (uint64_t i = 0; i < NUM_SRC; i++)
    {
        if (cmdq[i].size( )>0)
        {
            rv = true;
            break;
        }
    }
    return rv;
}

bool ReadModifyWrite::is_cmdq_empty( )
{
    for (uint64_t i = 0; i < NUM_SRC; i++)
    {
        if (cmdq[i].empty( )==false)
            return false;
    }
    return true;
}

void ReadModifyWrite::execute( )
{
    /* Execute main path sub-modules */
    hzdCheck_exec( );

    /* Advance pipeline buffers */
    cmdq_proceed( );
    pipe_hzdCheck->proceed( );

    /* Schedule main path events if there remains command in queues */
    if (pipe_hzdCheck->isEmpty( )==false ||
        is_cmdq_avail( ))
    {
        registerCallback((CallbackPtr)&ReadModifyWrite::execute, 1);
    }
    else
        need_wakeup = true;
}

void ReadModifyWrite::cmdq_proceed( )
{
    if (pipe_hzdCheck->isAcceptable( ) &&
        is_cmdq_avail( ))
    {
        bool isIssued = false;
        if (cmdq[src_id_IA].size( )>0)
        {
            Packet* pkt = cmdq[src_id_IA].front( );
            cmdq[src_id_IA].pop( );
            isIssued = true;
            
            signals_t* new_pb_signals = new signals_t;
            new_pb_signals->pkt = pkt;
            pipe_hzdCheck->input = new_pb_signals;

            PCMC_DBG(dbg_msg, "[RMW] Get req [0x%lx, ID=%lx, CMD=%c]"
                " from SRC=%lu (q-size=%lu)\n", pkt->LADDR, pkt->req_id,
                ((pkt->cmd==CMD_READ)? 'R':'W'), src_id_IA, 
                cmdq[src_id_IA].size( ));
        }

        inArbtr_exec(isIssued);
    }
}

void ReadModifyWrite::inArbtr_exec(bool isIssued)
{
    servCntrs[src_id_IA] = (isIssued)? (servCntrs[src_id_IA]+1):servCntrs[src_id_IA];

    /* Ensure & dst are issued together */
    if (isIssued && src_id_IA==SRC_BLKMGR)
        issued_blks += 1;

    if (issued_blks<2 && st_inArbtr==ST_FORBID_HOST)
        return;
    
    issued_blks = 0;
    if (servCntrs[src_id_IA]>=servWeights[src_id_IA] || // for RR, weight=1
        cmdq[src_id_IA].size( )==0)                     // in case of <TH, but empty
    {
        if (servCntrs[src_id_IA]>=servWeights[src_id_IA])
            servCntrs[src_id_IA] = 0;

        if (cmdq[src_id_IA].size( )==0)
        {
            if (src_id_IA==SRC_HOST && st_inArbtr==ST_FORBID_BLKMGR)
            {
                st_inArbtr = ST_FORBID_HOST;
                PCMC_DBG(dbg_msg, "[RMW] Now cmdq[AT] is empty, block cmdq[AT]"
                    " and reflect PA remap in cmdq[BLKMGR]\n"); 
            }
            else if (src_id_IA==SRC_BLKMGR && st_inArbtr==ST_FORBID_HOST)
            {
                st_inArbtr = ST_NORMAL;
                PCMC_DBG(dbg_msg, "[RMW] Now cmdq[BLKMGR] is empty, let"
                    " cmdq[AT] be released\n");
            }
        }

        /* Traverse command queue based on the selected one */
        for (uint64_t i=1; i<=NUM_SRC; i++)
        {
            uint64_t tmp_port = (src_id_IA+i)%NUM_SRC;
            
            if ((tmp_port==SRC_HOST && st_inArbtr==ST_FORBID_HOST) ||
                (tmp_port==SRC_BLKMGR && st_inArbtr==ST_FORBID_BLKMGR))
            {
                assert(cmdq[tmp_port].size( )==0);
                continue;
            }

            if (cmdq[tmp_port].size( )>0)
            {
                src_id_IA = tmp_port;

                if (src_id_IA==SRC_HOST)
                    num_HOST+=1;
                else if (src_id_IA==SRC_BLKMGR)
                    num_BLKMGR+=1;

                break;
            }
        }
    }
}

void ReadModifyWrite::hzdCheck_exec( )
{
    int rear = pipe_hzdCheck->getRearIdx( );
    if (pipe_hzdCheck->isEmpty( )==false && pipe_hzdCheck->buffer[rear].stage==0)
    {
        assert(st_hzdCheck==NO_STALL);

        /* Judge hazardness */
        signals_t* pb_signals = (signals_t*)(pipe_hzdCheck->buffer[rear].signals);
        Packet* proc_pkt = pb_signals->pkt;
        int64_t match_dbe = -1;
        int64_t hzd_idx = find_hzd(proc_pkt, match_dbe);
        if (hzd_idx>=0)
            hzd_handle(hzd_idx, proc_pkt);
        else if (match_dbe<0)
            non_hzd_handle(proc_pkt);
        else 
            unmergeable_handle(match_dbe, proc_pkt);
    }
    else if (pipe_hzdCheck->isEmpty( )==false)
    {
        signals_t* pb_signals = (signals_t*)(pipe_hzdCheck->buffer[rear].signals);
        Packet* proc_pkt = pb_signals->pkt;
        stall_handle(proc_pkt);
    }

    if (last_wake_outArbtr==wake_outArbtr &&
        (nonHzdHostRD_cmdq.empty( )==false || 
        cnt_rmwq_RD( ).first>0 ||
        cnt_rmwq_WR( ).first>0))
    {
        wake_outArbtr = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_outArbtr = std::max(wake_outArbtr, free_outArbtr);
        registerCallback((CallbackPtr)&ReadModifyWrite::issue_cmd, 1);
    }
}

void ReadModifyWrite::non_hzd_handle(Packet* proc_pkt)
{
    if (proc_pkt->cmd==CMD_READ)
    {
        /* Non-hazard host read */
        assert(proc_pkt->src_id==SRC_HOST);
        if (nonHzdHostRD_cmdq.size( )>=size_nonHzdHostRD_cmdq)
        {
            pipe_hzdCheck->prgm_stall = true;
            st_hzdCheck = FULL_HOSTRDQ;
            
            PCMC_DBG(dbg_msg, "[RMW] NONHZD HOST-RD [0x%lx, ID=%lx] is "
                "stalled due to stalled outArbtr\n", proc_pkt->LADDR,
                proc_pkt->req_id);
        }
        else
        {
            nonHzdHostRD_cmdq.push(proc_pkt);
            signals_t* pb_signals = (signals_t*)(pipe_hzdCheck->dequeue_rear( ));
            st_hzdCheck = NO_STALL;
            pipe_hzdCheck->prgm_stall = false;
            delete pb_signals;
            
            PCMC_DBG(dbg_msg, "[RMW] NONHZD HOST-RD [0x%lx, ID=%lx] is "
                "passed to outArbtr\n", proc_pkt->LADDR, proc_pkt->req_id);
        }
    }
    else if (proc_pkt->cmd==CMD_WRITE)
    {
        /* Non-hazard write cmd->Alloc new entry to RMWQ & DBUF */
        if (rmwq.size( )>=size_rmwq ||
            cnt_dbuf(proc_pkt->src_id)>=size_dbuf[proc_pkt->src_id])
        {
            pipe_hzdCheck->prgm_stall = true;
            st_hzdCheck = UNALLOCABLE;

            PCMC_DBG(dbg_msg, "[RMW] NONHZD WR [0x%lx, SRC=%ld, ID=%lx] "
                "is stalled due to unallocable RMWQ/DBUF\n", proc_pkt->LADDR,
                proc_pkt->src_id, proc_pkt->req_id);
        }
        else 
        {
            insert_rmwq(proc_pkt);
            int64_t dbe_idx = alloc_dbuf(proc_pkt);
            if (proc_pkt->src_id==SRC_HOST)
            {
                st_hzdCheck = req_wdata(proc_pkt, dbe_idx);
            
                PCMC_DBG(dbg_msg, "[RMW] NONHZD WR [0x%lx, SRC=%ld, ID=%lx] "
                    "starts waiting HOST wdata\n", proc_pkt->LADDR,
                    proc_pkt->src_id, proc_pkt->req_id);
            }
            else
            {
                signals_t* pb_signals = (signals_t*)(pipe_hzdCheck->dequeue_rear( ));
                st_hzdCheck = NO_STALL;
                pipe_hzdCheck->prgm_stall = false;
                delete pb_signals;
                
                PCMC_DBG(dbg_msg, "[RMW] NONHZD WR [0x%lx, SRC=%ld, ID=%lx] "
                    "is successfully allocated to RMWQ&DBUF\n", proc_pkt->LADDR,
                    proc_pkt->src_id, proc_pkt->req_id);
            }
        }
    }
    else
        assert(0);
}

void ReadModifyWrite::hzd_handle(int64_t hit_idx, Packet* proc_pkt)
{
    assert(hit_idx>=0);
    rmwqeIter_t hit_rmwqe = find_rmwq(dbuf[hit_idx]->pkt);
    assert(hit_rmwqe!=rmwq.end( ));

    PCMC_DBG(dbg_msg, "[RMW] HZD req [0x%lx, SRC=%ld, ID=%lx, CMD=%c] "
        "is merged to DBUF[%ld]=[0x%lx, SRC=%ld, ID=%lx]\n", proc_pkt->LADDR,
        proc_pkt->src_id, proc_pkt->req_id, ((proc_pkt->cmd==CMD_READ)? 'R':'W'),
        hit_idx, dbuf[hit_idx]->pkt->LADDR, dbuf[hit_idx]->pkt->src_id,
        dbuf[hit_idx]->pkt->req_id);
   
    /* Merging hazard cmd */
    if (proc_pkt->cmd==CMD_READ)
    {
        assert(proc_pkt->src_id==SRC_HOST && dbuf[hit_idx]->HostHzdPkt_RD==NULL);
        dbuf[hit_idx]->HostHzdPkt_RD = proc_pkt;

        signals_t* pb_signals = (signals_t*)(pipe_hzdCheck->dequeue_rear( ));
        st_hzdCheck = NO_STALL;
        pipe_hzdCheck->prgm_stall = false;
        delete pb_signals;
    }
    else if (proc_pkt->src_id==SRC_HOST)
    {
        proc_pkt->dbe_idx = hit_idx;
        st_hzdCheck = req_wdata(proc_pkt, hit_idx);
    }
    else if (proc_pkt->src_id==SRC_BLKMGR)
    {
        /* If pkt is BLKSWP/MV-dst, update PADDR_MAP */
        assert(proc_pkt->swap || proc_pkt->req_id%2==1);
        if (hit_rmwqe->pkt->LADDR==INVALID_ADDR)
            hit_rmwqe->pkt->LADDR = memsys->adec->mask_host_offset(proc_pkt->LADDR);

        hit_rmwqe->pkt->PADDR_MAP = proc_pkt->PADDR_MAP;
        hit_rmwqe->pkt->swap = proc_pkt->swap;
        rmwqeIter_t r_it = find_n_link(hit_rmwqe);
        assert(proc_pkt->swap || r_it!=rmwq.end( ));

        wack(proc_pkt, CMD_WACK_ID);
        wack(proc_pkt, CMD_PKT_DEL);
        
        signals_t* pb_signals = (signals_t*)(pipe_hzdCheck->dequeue_rear( ));
        st_hzdCheck = NO_STALL;
        pipe_hzdCheck->prgm_stall = false;
        delete pb_signals;
    }
    else if (proc_pkt->src_id==SRC_WDTP ||
        proc_pkt->src_id==SRC_RDTP ||
        proc_pkt->src_id==SRC_PTRSCB)
    {
        /* TODO: response to error manager */
        assert(0);
    }

    /* If pkt is urgent, mark merged one urgent as well */
    if (hit_rmwqe->pkt->urgent==false && proc_pkt->urgent)
    {
        hit_rmwqe->pkt->urgent = true;
        rmwq.splice(rmwq.begin( ), rmwq, hit_rmwqe);
    }

    check_issuable(hit_idx);
}

void ReadModifyWrite::unmergeable_handle(int64_t match_dbe, Packet* pkt)
{
    /* Handle the case when cmd is unable to merge */
    assert(match_dbe>=0);
    st_hzdCheck = HZD_UNMERGEABLE;
    pipe_hzdCheck->prgm_stall = true;

    PCMC_DBG(dbg_msg, "[RMW] Cannot merge [0x%lx, SRC=%ld, ID=%lx, CMD=%c]"
        " to DBUF[%ld]=[0x%lx, SRC=%ld, ID=%lx], because ", pkt->LADDR,
        pkt->src_id, pkt->req_id, ((pkt->cmd==CMD_READ)? 'R':'W'),
        match_dbe, dbuf[match_dbe]->pkt->LADDR, dbuf[match_dbe]->pkt->src_id,
        dbuf[match_dbe]->pkt->req_id);
    rmwqeIter_t r_it = (dbg_msg)? find_rmwq(dbuf[match_dbe]->pkt) : rmwq.end( ); 
    if (pkt->cmd==CMD_READ && dbuf[match_dbe]->HostHzdPkt_RD)
    {
        PCMC_DBG(dbg_msg, "HOST-RD HZD is merged previously\n");
    }
    else if (r_it->issuable)
    {
        PCMC_DBG(dbg_msg, "CMD of BLKMGR is ready to be issued\n");
    }
    else
        assert(0);
}

void ReadModifyWrite::stall_handle(Packet* pkt)
{
    assert(pkt);
    assert(pipe_hzdCheck->prgm_stall);

    if (st_hzdCheck==FULL_HOSTRDQ)
    {
        if (nonHzdHostRD_cmdq.size( )<size_nonHzdHostRD_cmdq)
        {
            nonHzdHostRD_cmdq.push(pkt);
            signals_t* pb_signals = (signals_t*)(pipe_hzdCheck->dequeue_rear( ));
            pipe_hzdCheck->prgm_stall = false;
            st_hzdCheck = NO_STALL;
            delete pb_signals;

            PCMC_DBG(dbg_msg, "[RMW] outArbtr now allow HOST-RD "
                "[0x%lx, ID=%lx] gets in\n", pkt->LADDR, pkt->req_id);
        }
    }
    else if (st_hzdCheck==UNALLOCABLE)
    {
        if (rmwq.size( )<size_rmwq &&
            cnt_dbuf(pkt->src_id)<size_dbuf[pkt->src_id])
        {
            insert_rmwq(pkt);
            int64_t dbe_idx = alloc_dbuf(pkt);
            if (pkt->src_id==SRC_HOST)
                st_hzdCheck = req_wdata(pkt, dbe_idx);
            else
            {
                signals_t* pb_signals = (signals_t*)(pipe_hzdCheck->dequeue_rear( ));
                pipe_hzdCheck->prgm_stall = false;
                st_hzdCheck = NO_STALL;
                delete pb_signals;
            }

            PCMC_DBG(dbg_msg, "[RMW] RMWQ/DBUF is now avail. to be newly alloc."
                " for [0x%lx, SRC=%ld, ID=%lx]\n", pkt->LADDR, pkt->src_id, pkt->req_id);
        }
    }
    else if (st_hzdCheck==WAIT_DCACHE_WDATA)
    {
        /* Just wait for wdata coming from DCACHE */
        PCMC_DBG(dbg_msg, "[RMW] Waiting for wdata of [0x%lx, SRC=0, ID=%lx]" 
            " from DCACHE\n", pkt->LADDR, pkt->req_id);
    }
    else if (st_hzdCheck==HZD_UNMERGEABLE)
    {
        int64_t match_dbe = -1;
        int64_t hzd_idx = find_hzd(pkt, match_dbe);
        if (hzd_idx>=0)
            hzd_handle(hzd_idx, pkt);
        else if (match_dbe<0)
            non_hzd_handle(pkt);
    }
    else
        assert(0);
}

int ReadModifyWrite::req_wdata(Packet* pkt, int64_t dbe_idx)
{
    assert(dbe_idx>=0 && pkt->cmd==CMD_WRITE);
    /* Stall pipeline if previous req is still getting wdata */
    if (host_wdata_wait_pkt)
    {
        PCMC_DBG(dbg_msg, "[RMW] Pipeline is stalled by [0x%lx, ID=%lx] "
            "to DCACHE-RDQ\n", host_wdata_wait_pkt->LADDR, 
            host_wdata_wait_pkt->req_id);

        pipe_hzdCheck->prgm_stall = true;
        return WAIT_DCACHE_WDATA;
    }

    /* Ready to get wdata and let next req come in */
    host_wdata_wait_pkt = pkt;
    signals_t* pb_signals = (signals_t*)(pipe_hzdCheck->dequeue_rear( ));
    pipe_hzdCheck->prgm_stall = false;
    delete pb_signals;
    
    dbuf[dbe_idx]->dcache_read += 1;
    pkt->from = this;
    dcache->recvResponse(pkt, tCMD);

    PCMC_DBG(dbg_msg, "[RMW] Get HOST-WR wdata [0x%lx, ID=%lx] "
        "from DCACHE\n", pkt->LADDR, pkt->req_id);

    return NO_STALL;
}

void ReadModifyWrite::issue_cmd( )
{
    bool isIssued = false;
    uint64_t num_RMW_RD = 0;
    uint64_t num_RMW_WR = 0;
    if (ucmde->isReady(NULL))
    {
        isIssued = true;
        std::pair<uint64_t, rmwqeIter_t> cnt_pair_RD = cnt_rmwq_RD( );
        std::pair<uint64_t, rmwqeIter_t> cnt_pair_WR = cnt_rmwq_WR( );
        num_RMW_RD = cnt_pair_RD.first;
        num_RMW_WR = cnt_pair_WR.first;
        int OUT_CMD_SEL = outArbtr_exec(num_RMW_RD, num_RMW_WR);
        
        /* Finally get output pkt on selected port */
        Packet* out_pkt = NULL;
        if (OUT_CMD_SEL<0)
            isIssued = false; // scheduled but issuable sets to 'false'
        else if (OUT_CMD_SEL==OUT_NONHZD_HOSTRD || OUT_CMD_SEL==OUT_RMW_RD)
        {
            if (OUT_CMD_SEL==OUT_NONHZD_HOSTRD)
            {
                /* Host read for direct dispath is available */
                out_pkt = nonHzdHostRD_cmdq.front( );
                nonHzdHostRD_cmdq.pop( );
            }
            else if (OUT_CMD_SEL==OUT_RMW_RD)
            {
                rmwqeIter_t ref_e = cnt_pair_RD.second;
                ref_e->RD_issued = true;
                
                out_pkt = new Packet( );
                *out_pkt = *(ref_e->pkt); 
                out_pkt->cmd = CMD_READ;
                out_pkt->owner = this;

                num_RMW_RD -= 1;
            }
        }
        else if (OUT_CMD_SEL==OUT_RMW_WR && 
            wbuffer_idx_dpu>=0 &&   // wbuffer id updated
            wcmd_issued.wdata_issued==false && 
            wait_redir==false)      // dbuf entry retired
        {
            rmwqeIter_t ref_e = cnt_pair_WR.second;
            ref_e->pkt->buffer_idx = wbuffer_idx_dpu;
            wbuffer_idx_dpu = -1;

            /* To prevent memory free conflict w/ retire_dbuf */
            out_pkt = new Packet( );
            *out_pkt = *(ref_e->pkt);

            assert(wcmd_issued.pkt==NULL && wcmd_issued.link.first==-1);
            wcmd_issued.pkt = ref_e->pkt;
            wcmd_issued.link.first = ref_e->remap_link.first;
            wcmd_issued.link.second = ref_e->remap_link.second;

            rmwq.erase(ref_e);
            num_RMW_WR -= 1;

            /* Erase corresponding BLKMV src req */
            if (wcmd_issued.link.first>=0 &&
                wcmd_issued.pkt->swap==false)
            {
                int64_t link_idx = find_dbuf(wcmd_issued.link.first,
                                             wcmd_issued.link.second);
                rmwqeIter_t link_rmwqe = find_rmwq(dbuf[link_idx]->pkt);
                assert(link_rmwqe->RD_only);
                rmwq.erase(link_rmwqe);
            }
            
            /* Ready to issue data of WCMD */
            if (last_wake_rdbuf==wake_rdbuf)
            {
                wake_rdbuf = geq->getCurrentTick( )+1*ticks_per_cycle;
                wake_rdbuf = std::max(wake_rdbuf, free_rdbuf);
                registerCallbackAt((CallbackPtr)&ReadModifyWrite::dbuf_rmux, wake_rdbuf);
            }
        }
        else if (wbuffer_idx_dpu<0 || wait_redir || wcmd_issued.wdata_issued)
            isIssued = false;
        else
            assert(0);

        /* Issue command */
        if (isIssued)
        {
            assert(out_pkt);

            /* Stats update */
            ncycle_t tmp_lat = geq->getCurrentTick( )-out_pkt->recvTick;
            if (tmp_lat>max_issue_lat)
                max_issue_lat = tmp_lat;
            if (tmp_lat<min_issue_lat)
                min_issue_lat = tmp_lat;
            avg_issue_lat = (avg_issue_lat*(num_issue-1)+tmp_lat)/(double)num_issue;

            if (OUT_CMD_SEL==OUT_NONHZD_HOSTRD)
            {
                if (max_issue_nonhzdrd_lat<tmp_lat)
                    max_issue_nonhzdrd_lat = tmp_lat;
                if (min_issue_nonhzdrd_lat>tmp_lat)
                    min_issue_nonhzdrd_lat = tmp_lat;

                avg_issue_nonhzdrd_lat = (avg_issue_nonhzdrd_lat*
                        (num_issue_nonHzdHostRD-1)+tmp_lat)/(double)num_issue_nonHzdHostRD;
            }
            else if (OUT_CMD_SEL==OUT_RMW_RD)
            {
                if (max_issue_rmwrd_lat<tmp_lat)
                    max_issue_rmwrd_lat = tmp_lat;
                if (min_issue_rmwrd_lat>tmp_lat)
                    min_issue_rmwrd_lat = tmp_lat;

                avg_issue_rmwrd_lat = (avg_issue_rmwrd_lat*
                        (num_issue_RMW_RD-1)+tmp_lat)/(double)num_issue_RMW_RD;
            }
            else
            {
                if (max_issue_wr_lat<tmp_lat)
                    max_issue_wr_lat = tmp_lat;
                if (min_issue_wr_lat>tmp_lat)
                    min_issue_wr_lat = tmp_lat;

                avg_issue_wr_lat = (avg_issue_wr_lat*
                        (num_issue_RMW_WR-1)+tmp_lat)/(double)num_issue_RMW_WR;
            }

            out_pkt->from = this;
            ucmde->recvRequest(out_pkt, tCMD);

            PCMC_DBG(dbg_msg, "[RMW] Issue CMD [0x%lx, SRC=%ld, ID=%lx, CMD=%c] "
                "selected by %s(rmwq.size=%lu, rmw-rd=%lu, rmw-wr=%lu)\n", 
                out_pkt->LADDR, out_pkt->src_id, 
                out_pkt->req_id, ((out_pkt->cmd==CMD_READ)? 'R':'W'), 
                (OUT_CMD_SEL==OUT_RMW_RD)? "RMW-RD":(OUT_CMD_SEL==OUT_RMW_WR)? 
                "RMW-WR":"HOST-RD", rmwq.size( ), num_RMW_RD, num_RMW_WR);
        }
    }

    /* Self schedule */
    last_wake_outArbtr = wake_outArbtr;
    free_outArbtr = (isIssued==false)? free_outArbtr:(wake_outArbtr+1*ticks_per_cycle);
    if (nonHzdHostRD_cmdq.empty( )==false || 
        num_RMW_RD>0 || num_RMW_WR>0)
    {
        wake_outArbtr = geq->getCurrentTick( )+1*ticks_per_cycle;
        registerCallback((CallbackPtr)&ReadModifyWrite::issue_cmd, 1);
    }
}

void ReadModifyWrite::check_meta(Packet* pkt)
{
    DataBlock pwcnt_blk = memsys->mdec->get_meta(pkt->buffer_meta, META_PWCNT);
    DataBlock wlvp_blk = memsys->mdec->get_meta(pkt->buffer_meta, META_WLVP);
    uint64_t pwcnt = pwcnt_blk.unwrap_u64( );
    uint64_t wlvp = wlvp_blk.unwrap_u64( );

    pwcnt+=1;
    if (pwcnt%intr_th_WLV==0 || wlvp!=0)
    {
        if (mcu && mcu->isReady(NULL)) 
        {
            /* Interrupt MCU directly */
            Packet* wlv_pkt = new Packet( );
            wlv_pkt->owner = this;
            wlv_pkt->from = this;
            wlv_pkt->cmd = CMD_IRQ;
            wlv_pkt->LADDR = pkt->LADDR; 
            wlv_pkt->PADDR = pkt->PADDR;
            mcu->recvRequest(wlv_pkt, tCMD);
            wlvp = 0;

            /* Update related stats */
            ncycle_t tmp_WLV = (num_WLV>0)? (geq->getCurrentTick( )-trigger_WLV):0; 
            if (tmp_WLV>max_interval_WLV && num_WLV>0)
                max_interval_WLV = tmp_WLV;
            if (tmp_WLV<min_interval_WLV && num_WLV>0)
                min_interval_WLV = tmp_WLV;
            
            avg_interval_WLV = (num_WLV>0)? 
                ((avg_interval_WLV*num_WLV+tmp_WLV)/(double)(num_WLV+1)):0;
            num_WLV+=1;

            trigger_WLV = geq->getCurrentTick( );
        }
        else
            wlvp = 1;

        wlvp_blk.wrap_u64(wlvp);
        memsys->mdec->set_meta(pkt->buffer_meta, META_WLVP, wlvp_blk);
    }

    if (pwcnt==max_pwcnt)
        pwcnt = 0;

    pwcnt_blk.wrap_u64(pwcnt);
    memsys->mdec->set_meta(pkt->buffer_meta, META_PWCNT, pwcnt_blk);
}

ncycle_t ReadModifyWrite::issue_wdata(Packet* pkt, int64_t dbe_idx)
{
    assert(pkt && dbe_idx>=0 && dbuf[dbe_idx]->pkt==pkt); 
    
    ncycle_t LAT = tDBUF + tBURST;
    
    /* Merge meta & data of wcmd for issuing wdata */
    int64_t link_idx = -1;
    pkt->buffer_meta = dbuf[dbe_idx]->meta;
    dbuf[dbe_idx]->meta_issued = true;
    if (wcmd_issued.link.first>=0)
    {
        id_t link_src_id = wcmd_issued.link.first;
        id_t link_req_id = wcmd_issued.link.second;
        link_idx = find_dbuf(link_src_id, link_req_id);
        assert(link_idx>=0);
        assert(dbuf[link_idx]->data.getSize( )==PAGE_SIZE);
        
        pkt->buffer_data = dbuf[link_idx]->data;
        LAT += tDBUF;
        
        PCMC_DBG(dbg_msg, "issue wdata of [0x%lx, SRC=%ld, ID=%lx]"
            ", where DBUF[%ld].meta and DBUF[%ld].data are merged\n",
            pkt->LADDR, pkt->src_id, pkt->req_id, dbe_idx, link_idx);
    }
    else
    {
        assert(dbuf[dbe_idx]->data.getSize( )==PAGE_SIZE);
        pkt->buffer_data = dbuf[dbe_idx]->data;
        
        PCMC_DBG(dbg_msg, "issue wdata of [0x%lx, SRC=%ld, ID=%lx]\n",
            pkt->LADDR, pkt->src_id, pkt->req_id);
    }

    /* Check metadata for WLV & WDT */
    check_meta(pkt);

    /* Dispatch data to DPU (and DCACHE if possible) */
    Packet* wpkt = new Packet( );
    *wpkt = *pkt;
    wpkt->isDATA = true;
    wpkt->owner = this;
    wpkt->from = this;
    dpu->recvRequest(wpkt, LAT);

    return LAT;
}

bool ReadModifyWrite::resp_HostHzd_RD(Packet* pkt, int64_t dbe_idx)
{
    bool wait = false;
    ncycle_t LAT = tDBUF + tBURST;

    if (dbuf[dbe_idx]->HostHzdPkt_RD)
    {
        if (geq->getCurrentTick( )>=free_wdcache)
        {
            PCMC_DBG(dbg_msg, "=> HOST-RD HZD [0x%lx, ID=%lx] are "
                "responded as well\n", dbuf[dbe_idx]->HostHzdPkt_RD->LADDR, 
                dbuf[dbe_idx]->HostHzdPkt_RD->req_id);

            dbuf[dbe_idx]->HostHzdPkt_RD->wdcache_latency = LAT;
            dbuf[dbe_idx]->HostHzdPkt_RD->buffer_data = pkt->buffer_data;
            dcache_wmux(dbuf[dbe_idx]);
        }
        else
        {
            wait = true;
            
            PCMC_DBG(dbg_msg, "HOST-RD HZD [0x%lx, ID=%lx] is waiting for "
                "DCACHE WR PORT\n", dbuf[dbe_idx]->HostHzdPkt_RD->LADDR, 
                dbuf[dbe_idx]->HostHzdPkt_RD->req_id);
        }
    }
   
    return wait;
}

int ReadModifyWrite::outArbtr_exec(uint64_t num_RMW_RD, uint64_t num_RMW_WR)
{
    int OUT_CMD_SEL = OUT_NONHZD_HOSTRD;
    bool nonHzdHostRD_avail = (nonHzdHostRD_cmdq.size( )>0)? true : false; 
    bool wasFlush = false;
    bool checkCandidates = false;
    
    /* Apply policy to determine the output req */
    if (flushRMW_RD)
    {
        wasFlush = true;
        if (flushCntRMW_RD>=maxFlushRMW_RD || num_RMW_RD==0)
        {
            flushRMW_RD = false;
            flushCntRMW_RD = 0;
        }
        else
        {
            OUT_CMD_SEL = OUT_RMW_RD;
            flushCntRMW_RD += 1;
        }
    }
    else if (flushRMW_WR)
    {
        wasFlush = true;
        if (flushCntRMW_WR>=maxFlushRMW_WR || num_RMW_WR==0)
        {
            flushRMW_WR = false;
            flushCntRMW_WR = 0;
        }
        else
        {
            OUT_CMD_SEL = OUT_RMW_WR;
            flushCntRMW_WR += 1;
        }
    }

    if (wasFlush==false ||
        (OUT_CMD_SEL==OUT_NONHZD_HOSTRD && nonHzdHostRD_avail==false))
        checkCandidates = true;
    
    bool serv_starv_RMW_RD = false;
    bool serv_starv_RMW_WR = false;
    if (checkCandidates)
    {
        if (num_RMW_RD>=maxLevelRMWQ_RD)
        {
            OUT_CMD_SEL = OUT_RMW_RD;
            starvCntRMW_RD = 0;
            starvCntRMW_WR = (num_RMW_WR>0)? (starvCntRMW_WR+1):0;
            flushRMW_RD = true;
        }
        else if (num_RMW_WR>=maxLevelRMWQ_WR)
        {
            OUT_CMD_SEL = OUT_RMW_WR;
            starvCntRMW_WR = 0;
            starvCntRMW_RD = (num_RMW_RD>0)? (starvCntRMW_RD+1):0;
            flushRMW_WR = true;
        }
        else if (starvCntRMW_RD>=maxStarvRMW_RD)
        {
            assert(num_RMW_RD>0);
            OUT_CMD_SEL = OUT_RMW_RD;
            starvCntRMW_RD = 0;
            starvCntRMW_WR = (num_RMW_WR>0)? (starvCntRMW_WR+1):0;
            serv_starv_RMW_RD = true;
        }
        else if (num_RMW_WR>0 && starvCntRMW_WR>=maxStarvRMW_WR)
        {
            assert(num_RMW_WR>0);
            OUT_CMD_SEL = OUT_RMW_WR;
            starvCntRMW_WR = 0;
            starvCntRMW_RD = (num_RMW_RD>0)? (starvCntRMW_RD+1):0;
            serv_starv_RMW_RD = false;
        }
        else if (OUT_CMD_SEL==OUT_NONHZD_HOSTRD)
        {
            if (nonHzdHostRD_avail==false)
            {
                if (num_RMW_RD>0)
                {
                    OUT_CMD_SEL = OUT_RMW_RD;
                    starvCntRMW_RD = 0;
                    starvCntRMW_WR = (num_RMW_WR>0)? (starvCntRMW_WR+1):0;
                }
                else if (num_RMW_WR>0)
                {
                    OUT_CMD_SEL = OUT_RMW_WR;
                    starvCntRMW_WR = 0;
                    starvCntRMW_RD = (num_RMW_RD>0)? (starvCntRMW_RD+1):0;
                }
                else
                    OUT_CMD_SEL = -1;
            }
            else
            {
                starvCntRMW_RD = (num_RMW_RD>0)? (starvCntRMW_RD+1):0;
                starvCntRMW_WR = (num_RMW_WR>0)? (starvCntRMW_WR+1):0;
            }
        }
    }

    /* Record issue stats */
    num_issue+=1;
    if (OUT_CMD_SEL==OUT_NONHZD_HOSTRD)
        num_issue_nonHzdHostRD+=1;
    else if (OUT_CMD_SEL==OUT_RMW_RD)
    {
        num_issue_RMW_RD+=1;
        if (serv_starv_RMW_RD)
            num_starv_RMW_RD+=1;
        else if (flushRMW_RD)
            num_flush_RMW_RD+=1;
    }
    else if (OUT_CMD_SEL==OUT_RMW_WR)
    {
        num_issue_RMW_WR+=1;
        if (serv_starv_RMW_WR)
            num_starv_RMW_WR+=1;
        else if (flushRMW_WR)
            num_flush_RMW_WR+=1;
    }

    return OUT_CMD_SEL;
}

void ReadModifyWrite::handle_await_resps( )
{
    /* Process packets, which are from DPU and DCACHE */
    std::list<LocalEvent*>::iterator e_it = await_resp.begin( );
    for ( ; e_it!=await_resp.end( ); )
    {
        Packet* pkt = (*e_it)->pkt;
        if (pkt->from==dynamic_cast<Component*>(dpu))
            handle_dpu_resps(pkt);
        else if (pkt->from==dynamic_cast<Component*>(dcache))
            handle_dcache_resps(pkt);
        else
            assert(0);

        /* Free event */
        delete (*e_it);
        e_it = await_resp.erase(e_it);
    }

    /* Schedule DCACHE write event */
    if (last_wake_wdcache==wake_wdcache &&
        rdq[RDQ_DPU].empty( )==false && 
        rdq[RDQ_DPU].front( )->need_redirect)
    {
        wake_wdcache = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_wdcache = std::max(wake_wdcache, free_wdcache);
        registerCallbackAt((CallbackPtr)&ReadModifyWrite::dcache_wmux, wake_wdcache);
    }

    /* Schedule DBUF RDPORT event */
    if (last_wake_rdbuf==wake_rdbuf && 
        isMergeRequired( ))
    {
        wake_rdbuf = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_rdbuf = std::max(wake_rdbuf, free_rdbuf);
        registerCallbackAt((CallbackPtr)&ReadModifyWrite::dbuf_rmux, wake_rdbuf);
    }

    /* Schedule DBUF WRPORT event */
    if (last_wake_wdbuf==wake_wdbuf)
    {
        Packet* str_pkt = isStorableExist( );
        if (str_pkt)
        {
            wake_wdbuf = std::max(str_pkt->merge_end_tick, free_wdbuf);
            wake_wdbuf = std::max(wake_wdbuf, geq->getCurrentTick( )+1*ticks_per_cycle);
            registerCallbackAt((CallbackPtr)&ReadModifyWrite::dbuf_wmux, wake_wdbuf);
        }
    }
}

void ReadModifyWrite::handle_dpu_resps(Packet* pkt)
{
    assert(pkt);
    if (pkt->cmd==CMD_WRITE)
    {
        /* Get new wbuffer ID for new wcmd-issue */
        assert(wbuffer_idx_dpu<0);
        PCMC_DBG(dbg_msg, "[RMW] Update DPU wbuffer-ID=%lx\n", pkt->buffer_idx);

        wbuffer_idx_dpu = pkt->buffer_idx;
        delete pkt;
    }
    else if (pkt->cmd==CMD_READ)
    {
        /* Store read response temporally (not exceed ID #) */
        rdq_push(pkt, RDQ_DPU);
        PCMC_DBG(dbg_msg, "[RMW] Push RD resp. of [0x%lx, SRC=%ld, ID=%lx]"
            " to DPU-RDQ(size=%lu)\n", pkt->LADDR, pkt->src_id, 
            pkt->req_id, rdq[RDQ_DPU].size( ));

        /* Whether redirection & merge are required */
        int64_t found_idx = find_dbuf(pkt);
        if (found_idx<0)
        {
            /* Non-hazard RD */
            assert(find_dbuf(pkt)<0);
            pkt->need_redirect = true;
        }
    }
    else
        assert(0);
}

void ReadModifyWrite::handle_dcache_resps(Packet* pkt)
{
    assert(pkt);
    rdq_push(pkt, RDQ_DCACHE);
    host_wdata_wait_pkt = NULL;
    
    PCMC_DBG(dbg_msg, "[RMW] Push HOST-WR [0x%lx, ID=%lx]" 
        "to DCACHE-RDQ(size=%lu)\n", pkt->LADDR, pkt->req_id, rdq[RDQ_DCACHE].size( ));

    if (st_hzdCheck==WAIT_DCACHE_WDATA)
    {
        assert(pipe_hzdCheck->isEmpty( )==false && pipe_hzdCheck->prgm_stall);
        signals_t* pb_signals = (signals_t*)
            (pipe_hzdCheck->buffer[pipe_hzdCheck->getRearIdx( )].signals);
        req_wdata(pb_signals->pkt, pb_signals->pkt->dbe_idx);
    }
}

void ReadModifyWrite::dcache_wmux(dbe_t* hzd_dbe)
{
    ncycle_t LAT = 1;
    bool dpu_pop = false;
    bool pass = false;
    if (geq->getCurrentTick( )<free_wdcache)
    {
        pass = true;
        LAT = (free_wdcache-(geq->getCurrentTick( )))/ticks_per_cycle;
        assert((int64_t)LAT>0);
    }
    else if (hzd_dbe)
    {
        wake_wdcache = geq->getCurrentTick( );
        LAT = hzd_dbe->HostHzdPkt_RD->wdcache_latency;
        assert(LAT>0);
        PCMC_DBG(dbg_msg, "[RMW] HZD HOST-RD [0x%lx, ID=%lx] is"
            " responded to DCACHE\n", hzd_dbe->HostHzdPkt_RD->LADDR,
            hzd_dbe->HostHzdPkt_RD->req_id);

        hzd_dbe->HostHzdPkt_RD->from = this;
        dcache->recvResponse(hzd_dbe->HostHzdPkt_RD, LAT);
        hzd_dbe->HostHzdPkt_RD = NULL;
    }
    else if (rdq[RDQ_DPU].empty( )==false && rdq[RDQ_DPU].front( )->need_redirect)
    {
        PCMC_DBG(dbg_msg, "[RMW] NONHZD HOST-RD [0x%lx, ID=%lx] is"
            " redirected to DCACHE(dpuqsize=%ld)\n", rdq[RDQ_DPU].front( )->LADDR,
            rdq[RDQ_DPU].front( )->req_id, rdq[RDQ_DPU].size( ));
        LAT = tBURST;
        rdq[RDQ_DPU].front( )->from = this;
        dcache->recvResponse(rdq[RDQ_DPU].front( ), LAT);
        rdq_pop(RDQ_DPU);
        dpu_pop = true;
    }
    else
        assert(0);

    /* Self-schedule (check redirection predicate in rdq is enough) */
    last_wake_wdcache = wake_wdcache;
    free_wdcache = (pass)? free_wdcache:(geq->getCurrentTick( )+LAT*ticks_per_cycle);
    if (rdq[RDQ_DPU].empty( )==false && rdq[RDQ_DPU].front( )->need_redirect)
    {
        wake_wdcache = geq->getCurrentTick( )+LAT*ticks_per_cycle;
        registerCallback((CallbackPtr)&ReadModifyWrite::dcache_wmux, LAT);
    }

    /* Schedule DBUF RDPORT & WRPORT events since rdq status changed */
    if (dpu_pop && last_wake_rdbuf==wake_rdbuf && isMergeRequired( ))
    {
        wake_rdbuf = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_rdbuf = std::max(wake_rdbuf, free_rdbuf);
        registerCallbackAt((CallbackPtr)&ReadModifyWrite::dbuf_rmux, wake_rdbuf);
    }

    if (dpu_pop && last_wake_wdbuf==wake_wdbuf)
    {
        Packet* str_pkt = isStorableExist( );
        if (str_pkt)
        {
            wake_wdbuf = std::max(str_pkt->merge_end_tick, free_wdbuf);
            wake_wdbuf = std::max(wake_wdbuf, geq->getCurrentTick( )+1*ticks_per_cycle);
            registerCallbackAt((CallbackPtr)&ReadModifyWrite::dbuf_wmux, wake_wdbuf);
        }
    }
}

void ReadModifyWrite::rdq_push(Packet* pkt, int rdq_id)
{
    rdq[rdq_id].push(pkt);
    if (rdq[rdq_id].size( )==1) // if the inserted one is on head
        check_n_set_merge(rdq_id);
}

void ReadModifyWrite::rdq_pop(int rdq_id)
{
    rdq[rdq_id].pop( );
    check_n_set_merge(rdq_id);
}

void ReadModifyWrite::check_n_set_merge(int rdq_id)
{
    /* 
     * Check and set data merge requirement on updated read_queue 
     * Needs to be placed next to read queue update events
     */
    int64_t ref_idx = -1;
    Packet* ref_pkt = NULL;
    if (rdq[rdq_id].empty( ))
        return;
    
    ref_pkt = rdq[rdq_id].front( );
    if (ref_pkt->merge_end_tick==NO_MERGE_STR)
    {
        ref_idx = find_dbuf(ref_pkt);
        if (ref_idx<0) // only redirection is required
            return;

        for (uint64_t i=0; i<PAGE_SIZE/HOST_TX_SIZE; i++)
        {
            if (dbuf[ref_idx]->dvalid[i]==true)
            {
                ref_pkt->merge_end_tick = NEED_MERGE_STR;
                break;
            }
        }
    }
}

ReadModifyWrite::rmwqeIter_t ReadModifyWrite::find_rmwq(Packet* pkt)
{
    rmwqeIter_t r_it = rmwq.begin( );
    for ( ; r_it!=rmwq.end( ); r_it++)
    {
        if (r_it->pkt==pkt)
            break;
    }
    return r_it;
}

ReadModifyWrite::rmwqeIter_t ReadModifyWrite::insert_rmwq(Packet* pkt)
{
    rmwqeIter_t r_it = rmwq.end( );
    rmwqe_t new_entry;
    new_entry.pkt = pkt;
    if (pkt->urgent)
    {
        /* Insert in front of q_it */
        rmwqeIter_t q_it = rmwq.begin( );
        for ( ; q_it!=rmwq.begin( ); q_it++)
        {
            if (q_it->pkt->urgent==false)
                break;
        }
        r_it = rmwq.insert(q_it, new_entry);
    }
    else
    {
        rmwq.push_back(new_entry);
        r_it = --rmwq.end( );
    }

    assert(r_it!=rmwq.end( ));
    if (r_it->pkt->src_id==SRC_BLKMGR && 
        r_it->pkt->swap==false &&
        r_it->pkt->req_id%2==0)
        r_it->RD_only = true;

    if (pkt->src_id==SRC_BLKMGR)
        find_n_link(r_it);

    return r_it;
}

ReadModifyWrite::rmwqeIter_t ReadModifyWrite::find_n_link(rmwqeIter_t ref_rmwqe)
{
    assert(ref_rmwqe->pkt->PADDR_MAP!=INVALID_ADDR);
    rmwqeIter_t r_it = rmwq.begin( );
    for ( ; r_it!=rmwq.end( ); r_it++)
    {
        if (ref_rmwqe==r_it)
            continue;

        if ((ref_rmwqe->pkt->swap &&
             r_it->pkt->PADDR_MAP==ref_rmwqe->pkt->PADDR) ||
            (ref_rmwqe->pkt->swap==false && r_it->RD_only &&
             r_it->pkt->LADDR>>PAGE_OFFSET==
                ref_rmwqe->pkt->LADDR>>PAGE_OFFSET))
        {
            r_it->link(ref_rmwqe);
            break;
        }
    }
    return r_it;
}

std::pair<uint64_t, ReadModifyWrite::rmwqeIter_t> ReadModifyWrite::cnt_rmwq_RD( )
{
    std::pair<uint64_t, rmwqeIter_t> ret(0, rmwq.end( ));
    rmwqeIter_t q_it = rmwq.begin( );
    for ( ; q_it!=rmwq.end( ); q_it++)
    {
        if (q_it->RD_issued==false)
        {
            ret.first += 1;
            if (ret.first==1)
                ret.second = q_it;
        }
    }
    return ret;
}

std::pair<uint64_t, ReadModifyWrite::rmwqeIter_t> ReadModifyWrite::cnt_rmwq_WR( )
{
    std::pair<uint64_t, rmwqeIter_t> ret(0, rmwq.end( ));
    rmwqeIter_t q_it = rmwq.begin( );
    for ( ; q_it!=rmwq.end( ); q_it++)
    {
        if (q_it->RD_only)
            continue;

        if (q_it->issuable)
        {
            ret.first += 1;
            if (ret.first==1)
                ret.second = q_it;
        }
    }
    return ret;
}

void ReadModifyWrite::dbuf_rmux( )
{
    PCMC_DBG(dbg_msg, "[RMW] At DBUF-RMUX, ");

    Packet* pkt = wcmd_issued.pkt;
    int64_t found_idx = (pkt)? find_dbuf(pkt) : -1;
    bool pend = false; // wait for ready or DCACHE-WPORT becoming avail.
    ncycle_t LAT = 1;
    assert(pkt==NULL || found_idx>=0);

    /* Data-issue has the highest priority cause its cmd is the oldest */
    bool wdata_ready = false;
    if (pkt!=NULL && wcmd_issued.wdata_issued==false && 
        (wait_redir || dpu->isReady(pkt)))
        wdata_ready = true;

    if (wdata_ready)
    {
        /* Issue write data & respond host hazard read */
        if (wait_redir==false)
        {
            LAT = issue_wdata(pkt, found_idx);
            wait_redir = resp_HostHzd_RD(pkt, found_idx);
        }
        else
        {
            wait_redir = resp_HostHzd_RD(pkt, found_idx);
            LAT = (wait_redir)? LAT:(tDBUF+tBURST);
        }

        if (wait_redir==false)
        {
            wcmd_issued.wdata_issued = true;
            registerCallback((CallbackPtr)&ReadModifyWrite::retire_dbuf,
                LAT, PRIORITY_DBUF_RETIRE);
//            retire_dbuf( ); //XXX
        }
        else
            pend = true;
    }
    else
    {
        /* Get merging data from DPU-RDQ side & DCACHE-RDQ side */
        Packet* mg_pkt = isMergeRequired( );
        if (mg_pkt)
        {
            mg_pkt->merge_end_tick = geq->getCurrentTick( )+tDBUF*ticks_per_cycle;
            LAT = tDBUF;

            PCMC_DBG(dbg_msg, "data of [0x%lx, SRC=%ld, ID=%lx] is read for "
                "merging on %s-RDQ\n", mg_pkt->LADDR, mg_pkt->src_id, mg_pkt->req_id,
                ((mg_pkt==rdq[RDQ_DPU].front( ))? "DPU":"DCACHE"));
        }
        else if (pkt!=NULL && wdata_ready==false)
        {
            pend = true;
            PCMC_DBG(dbg_msg, "wdata is not ready to issue "
                "wait to tick-%lu\n", free_wdcache);
        }
    }

    /* Self schedule */
    last_wake_rdbuf = wake_rdbuf;
    free_rdbuf = (pend)? free_rdbuf:(geq->getCurrentTick( )+LAT*ticks_per_cycle);
    if ((wcmd_issued.pkt!=NULL && wcmd_issued.wdata_issued==false) || // host hzd not resp yet
         isMergeRequired( ))                                          // read for merge needed
    {
        wake_rdbuf = geq->getCurrentTick( )+LAT*ticks_per_cycle;
        registerCallback((CallbackPtr)&ReadModifyWrite::dbuf_rmux, LAT);
    }

    /* Schedule DBUF WRPORT */
    if (last_wake_wdbuf==wake_wdbuf)
    {
        Packet* str_pkt = isStorableExist( );
        if (str_pkt)
        {
            wake_wdbuf = std::max(str_pkt->merge_end_tick, free_wdbuf);
            wake_wdbuf = std::max(wake_wdbuf, geq->getCurrentTick( )+LAT*ticks_per_cycle);
            registerCallbackAt((CallbackPtr)&ReadModifyWrite::dbuf_wmux, wake_wdbuf);
        }
    }
}

void ReadModifyWrite::dbuf_wmux( )
{
    /* Perform merge & store */
    PCMC_DBG(dbg_msg, "[RMW] At DBUF-WMUX, ");
    
    Packet* pkt = NULL;
    ncycle_t LAT = tDBUF;
    int64_t found_idx = -1;
    bool dpu_pop = true;
    if (rdq[RDQ_DPU].empty( )==false &&
        rdq[RDQ_DPU].front( )->need_redirect==false &&
        rdq[RDQ_DPU].front( )->merge_end_tick<=geq->getCurrentTick( ))
    {
        pkt = rdq[RDQ_DPU].front( );
        found_idx = find_dbuf(pkt);
        
        assert(pkt->owner==this);
        PCMC_DBG(dbg_msg, "DPU rdata of [0x%lx, SRC=%ld, ID=%lx] "
            "stored into DBUF[%ld]\n", pkt->LADDR, pkt->src_id,
            pkt->req_id, found_idx);

        if (dbuf[found_idx]->data.getSize( )==0)
            init_dbuf_data(found_idx);

        assert(pkt->mvalid);
        dbuf[found_idx]->mvalid = true;
        dbuf[found_idx]->meta = pkt->buffer_meta;
        for (uint64_t i=0; i<PAGE_SIZE/HOST_TX_SIZE; i++)
        {
            if (dbuf[found_idx]->dvalid[i])
                continue;
            
            dbuf[found_idx]->dvalid[i] = true;
            for (uint64_t b=0; b<HOST_TX_SIZE;b++)
            {
                uint8_t getData = pkt->buffer_data.getByte(i*HOST_TX_SIZE+b);
                dbuf[found_idx]->data.setByte(i*HOST_TX_SIZE+b, getData);
            }
        }

        rdq_pop(RDQ_DPU);
        delete pkt;
    }
    else if (rdq[RDQ_DCACHE].empty( )==false &&
        rdq[RDQ_DCACHE].front( )->merge_end_tick<=geq->getCurrentTick( ))
    {
        pkt = rdq[RDQ_DCACHE].front( );
        found_idx = find_hostWR(pkt);

        PCMC_DBG(dbg_msg, "HOST wdata of [0x%lx, SRC=%ld, ID=%lx] is "
            "stored into DBUF[%ld]\n", pkt->LADDR, pkt->src_id,
            pkt->req_id, found_idx);
       
        if (dbuf[found_idx]->data.getSize( )==0)
            init_dbuf_data(found_idx);

        for (uint64_t i=0; i<PAGE_SIZE/HOST_TX_SIZE; i++)
        {
            if (pkt->dvalid[i]==false)
                continue;

            dbuf[found_idx]->dvalid[i] = true;
            for (uint64_t b=0; b<HOST_TX_SIZE;b++)
            {
                uint8_t getData = pkt->buffer_data.getByte(i*HOST_TX_SIZE+b);
                dbuf[found_idx]->data.setByte(i*HOST_TX_SIZE+b, getData);
            }
        }

        /* Mark-up DCACHE-RD complete & respond if it's a MERGED req */
        dbuf[found_idx]->dcache_read -= 1;
        if (dbuf[found_idx]->pkt!=pkt)
        {
            wack(pkt, CMD_WACK_ID);
            wack(pkt, CMD_PKT_DEL);
        }

        rdq_pop(RDQ_DCACHE);
        dpu_pop = false;
    }
    else
        assert(0);

    check_issuable(found_idx);

    /* Schedule DCACHE write event */
    if (dpu_pop && last_wake_wdcache==wake_wdcache && 
        rdq[RDQ_DPU].empty( )==false && 
        rdq[RDQ_DPU].front( )->need_redirect)
    {
        wake_wdcache = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_wdcache = std::max(wake_wdcache, free_wdcache);
        registerCallbackAt((CallbackPtr)&ReadModifyWrite::dcache_wmux, wake_wdcache);
    }
    
    /* Schedule DBUF RDPORT event */
    if (last_wake_rdbuf==wake_rdbuf && 
        isMergeRequired( ))
    {
        wake_rdbuf = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_rdbuf = std::max(wake_rdbuf, free_rdbuf);
        registerCallbackAt((CallbackPtr)&ReadModifyWrite::dbuf_rmux, wake_rdbuf);
    }

    /* Schedule issue command due to the call of check_issuable */
    if (last_wake_outArbtr==wake_outArbtr &&
        (nonHzdHostRD_cmdq.empty( )==false || 
        cnt_rmwq_RD( ).first>0 ||
        cnt_rmwq_WR( ).first>0))
    {
        wake_outArbtr = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_outArbtr = std::max(wake_outArbtr, free_outArbtr);
        registerCallback((CallbackPtr)&ReadModifyWrite::issue_cmd, 1);
    }

    /* Self schedule */
    last_wake_wdbuf = wake_wdbuf;
    free_wdbuf = geq->getCurrentTick( )+LAT*ticks_per_cycle;
    Packet* str_pkt = isStorableExist( );
    if (str_pkt)
    {
        wake_wdbuf = std::max(geq->getCurrentTick( )+LAT*ticks_per_cycle, str_pkt->merge_end_tick);
        registerCallback((CallbackPtr)&ReadModifyWrite::dbuf_wmux, LAT);
    }
}

void ReadModifyWrite::check_issuable(int64_t dbe_idx)
{
    /* Check whether corresponding RMWQ entry is issuable */
    uint64_t i = 0;
    for ( ; i<PAGE_SIZE/HOST_TX_SIZE; i++)
    {
        if (dbuf[dbe_idx]->dvalid[i]==false)
            break;
    }

    if (i==PAGE_SIZE/HOST_TX_SIZE &&
        dbuf[dbe_idx]->mvalid &&
        dbuf[dbe_idx]->dcache_read==0)
    {
        /* Issuable only if linked entry also has complete data */
        rmwqeIter_t rmwqe = find_rmwq(dbuf[dbe_idx]->pkt);
        assert(rmwqe!=rmwq.end( ));
        if (rmwqe->pkt->PADDR_MAP!=INVALID_ADDR &&
            rmwqe->remap_link.first<0)
        {
            rmwqe->issuable = false; // for unpaired BLKMGR req, bypass
        }
        else if (rmwqe->remap_link.first>=0)
        {
            int64_t link_idx = find_dbuf(rmwqe->remap_link.first, 
                                         rmwqe->remap_link.second);
            for (i=0; i <PAGE_SIZE/HOST_TX_SIZE; i++)
            {
                if (dbuf[link_idx]->dvalid[i]==false)
                    break;
            }
            
            if (i==PAGE_SIZE/HOST_TX_SIZE &&
                dbuf[link_idx]->mvalid &&
                dbuf[link_idx]->dcache_read==0)
            {
                rmwqeIter_t link_rmwqe = find_rmwq(dbuf[link_idx]->pkt);
                assert(link_rmwqe!=rmwq.end( ));
                rmwqe->issuable = true;
                link_rmwqe->issuable = true;
            }
            else
                rmwqe->issuable = false;
        }
        else
            rmwqe->issuable = true;
    }
}

Packet* ReadModifyWrite::isStorableExist( )
{
    /* Check if a storable one exists */
    Packet* ret = NULL;
    for (uint64_t n=0; n<2; n++)
    {
        Packet* tmp_pkt = NULL;
        if (n==0 && rdq[RDQ_DPU].empty( )==false && 
            rdq[RDQ_DPU].front( )->need_redirect==false &&
            rdq[RDQ_DPU].front( )->merge_end_tick!=NEED_MERGE_STR)
        {
            tmp_pkt = rdq[RDQ_DPU].front( );
        }
        else if (n==1 && rdq[RDQ_DCACHE].empty( )==false &&
            rdq[RDQ_DCACHE].front( )->merge_end_tick!=NEED_MERGE_STR)
        {
            tmp_pkt = rdq[RDQ_DCACHE].front( );
        }
        else
            continue;

        if (tmp_pkt!=NULL && (ret==NULL || 
            (ret!=NULL && tmp_pkt->merge_end_tick<ret->merge_end_tick)))
            ret = tmp_pkt;
    }

    return ret;
}

Packet* ReadModifyWrite::isMergeRequired( )
{
    /* Check data merge requirement on each read-queue */
    Packet* ret = NULL;
    for (uint64_t n=0; n<2; n++)
    {
        Packet* ref_pkt = NULL;
        if (rdq[n].empty( )==false)
            ref_pkt = rdq[n].front( );
        else
            continue;

        if (ref_pkt->merge_end_tick==NEED_MERGE_STR)
        {
            ret = ref_pkt;
            break;
        }
    }

    return ret;
}

int64_t ReadModifyWrite::alloc_dbuf(Packet* pkt)
{
    assert(pkt);
    int64_t idx = get_allocable_dbuf(pkt->src_id);
    assert(idx>=0 && dbuf[idx]==NULL);

    dbe_t* new_dbe = new dbe_t(PAGE_SIZE/HOST_TX_SIZE);
    new_dbe->pkt = pkt;
    dbuf[idx] = new_dbe;

    pkt->dbe_idx = idx;

    return idx;
}

int64_t ReadModifyWrite::get_allocable_dbuf(id_t src_id)
{
    int64_t rv = -1;
    uint64_t base_idx = base_dbuf[src_id];
    uint64_t tmp_size = size_dbuf[src_id];
    for (uint64_t i=base_idx; i<tmp_size+base_idx; i++)
    {
        if (dbuf[i]==NULL)
        {
            rv = (int64_t)i;
            break;
        }
    }
    return rv;
}

int64_t ReadModifyWrite::find_hzd(Packet* pkt, int64_t& match_dbe)
{
    int64_t rv = -1;
    match_dbe = -1; //init

    /* BLKMV-src is NOT used for hazard checking */
    if (pkt->src_id==SRC_BLKMGR && 
        pkt->swap==false && pkt->req_id%2==0) // BLKMV-src
        return rv;

    uint64_t LADDR = pkt->LADDR;
    uint64_t PADDR = pkt->PADDR;
    for (uint64_t i=0; i<size_all_dbuf; i++)
    {
        if (dbuf[i]==NULL)
            continue;

        bool logical_hit = 
            (dbuf[i]->pkt->LADDR>>PAGE_OFFSET==LADDR>>PAGE_OFFSET)? true : false;
        bool physical_hit = (dbuf[i]->pkt->PADDR==PADDR)? true : false;

        if (logical_hit==false && physical_hit==false)
            continue;

        if (logical_hit==false && 
            ((pkt->swap==false && pkt->PADDR_MAP!=INVALID_ADDR) ||
             (dbuf[i]->pkt->swap==false && dbuf[i]->pkt->PADDR_MAP!=INVALID_ADDR)))
        {
            /* 
             * If logical miss, one of the comparing addresses is BLKMV-dst 
             * the other one must be ERR req, whose LADDR is INVALID
             */
            assert(dbuf[i]->pkt->LADDR==INVALID_ADDR || LADDR==INVALID_ADDR);
            continue; //XXX we do not make them hazard in this ver.
        }

        rmwqeIter_t r_it = find_rmwq(dbuf[i]->pkt);
        if (r_it==rmwq.end( ) ||    // cmd is issued wdata remains
            r_it->RD_only ||        // BLKMV-src is NOT merging object
            (r_it->issuable &&
             r_it->pkt->PADDR_MAP!=INVALID_ADDR)) 
            continue;

        match_dbe = (int64_t)i;
        if (pkt->cmd==CMD_READ &&   // host-RD is already merged, WAIT
            dbuf[i]->HostHzdPkt_RD)
            break; 

        /* Allow req being merged to non-BLKMGR */
        if (r_it->issuable)
            r_it->issuable = false;

        rv = (int64_t)i;
        break;
    }

    if (match_dbe>=0)
    {
        if (pkt->cmd==CMD_WRITE)
        {
            num_WAW+=1;
            if (rv>=0)
                num_mg_WAW+=1;
        }
        else
        {
            num_RAW+=1;
            if (rv>=0)
                num_mg_RAW+=1;
        }
    }

    return rv;
}

int64_t ReadModifyWrite::find_dbuf(id_t src_id, id_t req_id)
{
    assert(src_id>=0 && req_id>=0);
    
    int64_t rv = -1;
    uint64_t base_idx = base_dbuf[src_id];
    uint64_t tmp_size = size_dbuf[src_id];
    for (uint64_t i=base_idx; i<tmp_size+base_idx; i++)
    {
        if (dbuf[i]==NULL)
            continue;

        if (dbuf[i]->pkt->req_id==req_id)
        {
            rv = (int64_t)i;
            break;
        }
    }
    return rv;
}

int64_t ReadModifyWrite::find_dbuf(Packet* pkt)
{
    assert(pkt);
    int64_t rv = pkt->dbe_idx;
    return rv;
}

int64_t ReadModifyWrite::find_hostWR(Packet* pkt)
{
    assert(pkt);
    return find_dbuf(pkt);
}

uint64_t ReadModifyWrite::cnt_dbuf(id_t src_id)
{
    assert(src_id>=0);

    uint64_t rv = 0;
    uint64_t base_idx = base_dbuf[src_id];
    uint64_t tmp_size = size_dbuf[src_id];
    for (uint64_t i=base_idx; i<tmp_size+base_idx; i++)
    {
        if (dbuf[i]!=NULL)
            rv += 1;
    }
    return rv;
}

void ReadModifyWrite::init_dbuf_data(int64_t dbe_idx)
{
    assert(dbe_idx>=0);
    dbuf[dbe_idx]->data.setSize(PAGE_SIZE);
    dbuf[dbe_idx]->meta.setSize(META_SIZE);
}

void ReadModifyWrite::retire_dbuf( )
{
    /* Retire DBUF entry and delete original packets */ 
    assert(wcmd_issued.pkt);
    Packet* pkt = wcmd_issued.pkt;
    int64_t dbe_idx = (pkt)? find_dbuf(pkt) : -1;
    bool retire_allow = true;
    assert(wcmd_issued.wdata_issued);

    id_t link_src_id = wcmd_issued.link.first;
    id_t link_req_id = wcmd_issued.link.second;
    int64_t link_idx = (link_src_id<0)? -1 : find_dbuf(link_src_id, link_req_id);

    if (dbg_msg && (link_idx<0 || 
        (dbuf[link_idx]->meta_issued || (dbuf[link_idx]->pkt->swap==false &&
           dbuf[link_idx]->pkt->req_id%2==0))))
    {
        PCMC_DBG(dbg_msg, "[RMW] Retire DBUF[%ld]=[0x%lx, SRC=%ld, ID=%lx]\n",
            dbe_idx, pkt->LADDR, pkt->src_id, pkt->req_id);
        if (link_idx>=0)
            PCMC_DBG(dbg_msg, "=> linked DBUF[%ld]=[0x%lx, SRC=%ld, ID=%lx] "
                "are retired too\n", link_idx, dbuf[link_idx]->pkt->LADDR, 
                dbuf[link_idx]->pkt->src_id, dbuf[link_idx]->pkt->req_id);
    }

    if (link_idx>=0)
    {
        if (dbuf[link_idx]->meta_issued || 
            (dbuf[link_idx]->pkt->swap==false &&
             dbuf[link_idx]->pkt->req_id%2==0))
        {
            /* For BLKMV-src, retire ID first */
            if (dbuf[link_idx]->pkt->swap==false)
            {
                assert(dbuf[link_idx]->pkt->owner==aitm);
                wack(dbuf[link_idx]->pkt, CMD_WACK_ID);
                wack(dbuf[link_idx]->pkt, CMD_PKT_DEL);
            }

            delete dbuf[link_idx];
            dbuf[link_idx] = NULL;
            num_blkmgr_ctrl-=2;
            assert(num_blkmgr_ctrl>=0);
        }
        else
            retire_allow = false;
    }
    
    if (retire_allow)
    {
        wack(dbuf[dbe_idx]->pkt, CMD_PKT_DEL); 
        delete dbuf[dbe_idx];
        dbuf[dbe_idx] = NULL;
    }
    
    wcmd_issued.pkt = NULL;
    wcmd_issued.link.first = -1;
    wcmd_issued.link.second = -1;
    wcmd_issued.wdata_issued = false;
}

void ReadModifyWrite::wack(Packet* pkt, cmd_t type)
{
    assert(type==CMD_WACK_ID || type==CMD_PKT_DEL);
    pkt->cmd = type;
    pkt->from = this;
    if (pkt->owner==aitm)
        aitm->recvResponse(pkt);
    else if (pkt->owner==dcache)
        dcache->recvResponse(pkt);
    else
        assert(0);
}

/*========== Below is stats setting ==========*/
void ReadModifyWrite::register_stats( )
{
    uint64_t u64_zero = 0;
    uint64_t u64_max = std::numeric_limits<uint64_t>::max( );
    double df_init = 0.0;
    RESET_STATS(num_HOST, u64_zero);
    RESET_STATS(num_BLKMGR, u64_zero);

    RESET_STATS(num_WAW, u64_zero);
    RESET_STATS(num_mg_WAW, u64_zero);
    RESET_STATS(num_RAW, u64_zero);
    RESET_STATS(num_mg_RAW, u64_zero);

    RESET_STATS(num_issue_RMW_RD, u64_zero);
    RESET_STATS(num_issue_RMW_WR, u64_zero);
    RESET_STATS(num_issue_nonHzdHostRD, u64_zero);
    RESET_STATS(num_flush_RMW_RD, u64_zero);
    RESET_STATS(num_flush_RMW_WR, u64_zero);
    RESET_STATS(num_starv_RMW_RD, u64_zero);
    RESET_STATS(num_starv_RMW_WR, u64_zero);

    RESET_STATS(num_WLV, u64_zero);
    RESET_STATS(trigger_WLV, u64_zero);
    RESET_STATS(max_interval_WLV, u64_zero);
    RESET_STATS(min_interval_WLV, u64_max);
    RESET_STATS(avg_interval_WLV, df_init);

    num_issue = u64_zero;
    RESET_STATS(max_issue_lat, u64_zero);
    RESET_STATS(min_issue_lat, u64_max);
    RESET_STATS(avg_issue_lat, df_init);
    
    RESET_STATS(max_issue_nonhzdrd_lat, u64_zero);
    RESET_STATS(min_issue_nonhzdrd_lat, u64_max);
    RESET_STATS(avg_issue_nonhzdrd_lat, df_init);
    
    RESET_STATS(max_issue_rmwrd_lat, u64_zero);
    RESET_STATS(min_issue_rmwrd_lat, u64_max);
    RESET_STATS(avg_issue_rmwrd_lat, df_init);
    
    RESET_STATS(max_issue_wr_lat, u64_zero);
    RESET_STATS(min_issue_wr_lat, u64_max);
    RESET_STATS(avg_issue_wr_lat, df_init);
    
    ADD_STATS(cp_name, num_HOST);
    ADD_STATS(cp_name, num_BLKMGR);
    ADD_STATS(cp_name, num_WAW);
    ADD_STATS(cp_name, num_mg_WAW);
    ADD_STATS(cp_name, num_RAW);
    ADD_STATS(cp_name, num_mg_RAW);
    ADD_STATS(cp_name, num_issue_RMW_RD);
    ADD_STATS(cp_name, num_issue_RMW_WR);
    ADD_STATS(cp_name, num_issue_nonHzdHostRD);
    ADD_STATS(cp_name, num_flush_RMW_RD);
    ADD_STATS(cp_name, num_flush_RMW_WR);
    ADD_STATS(cp_name, num_starv_RMW_RD);
    ADD_STATS(cp_name, num_starv_RMW_WR);
    ADD_STATS(cp_name, num_WLV);
    ADD_STATS_N_UNIT(cp_name, max_interval_WLV, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_interval_WLV, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_interval_WLV, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_issue_nonhzdrd_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_issue_nonhzdrd_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_issue_nonhzdrd_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_issue_rmwrd_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_issue_rmwrd_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_issue_rmwrd_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_issue_wr_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_issue_wr_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_issue_wr_lat, "cycles");
}

void ReadModifyWrite::calculate_stats( )
{
    if (num_WLV>0)
    {
        max_interval_WLV /= ticks_per_cycle;
        min_interval_WLV /= ticks_per_cycle;
        avg_interval_WLV /= ticks_per_cycle;
    }

    if (num_issue>0)
    {
        max_issue_lat /= ticks_per_cycle;
        min_issue_lat /= ticks_per_cycle;
        avg_issue_lat /= ticks_per_cycle;
    }

    if (num_issue_nonHzdHostRD>0)
    {
        max_issue_nonhzdrd_lat /= ticks_per_cycle;
        min_issue_nonhzdrd_lat /= ticks_per_cycle;
        avg_issue_nonhzdrd_lat /= ticks_per_cycle;
    }

    if (num_issue_RMW_RD>0)
    {
        max_issue_rmwrd_lat /= ticks_per_cycle;
        min_issue_rmwrd_lat /= ticks_per_cycle;
        avg_issue_rmwrd_lat /= ticks_per_cycle;
    }

    if (num_issue_RMW_WR>0)
    {
        max_issue_wr_lat /= ticks_per_cycle;
        min_issue_wr_lat /= ticks_per_cycle;
        avg_issue_wr_lat /= ticks_per_cycle;
    }
}

