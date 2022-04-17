#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "base/Stats.h"
#include "Parsers/Parser.h"
#include "RequestReceiver/RequestReceiver.h"
#include "DataCache/DataCache.h"

#define PRIORITY_INSERT_BUFFER 10
using namespace PCMCsim;

RequestReceiver::RequestReceiver(MemoryControlSystem* memsys_, std::string cfg_header)
:Component(memsys_), dcache(NULL), mux_st(PRIORITY_READ), wake_req_mux(0),
last_wake_req_mux(0), wake_resp_mux(0), last_wake_resp_mux(0), free_resp(0)
{
    cp_name = cfg_header;

    /* Parameter load */
    dbg_msg = memsys->getParamBOOL(cp_name+".dbg_msg", false);
    ticks_per_cycle= memsys->getParamUINT64("global.ticks_per_cycle", 1);
    maxIssue_RD = memsys->getParamUINT64(cp_name+".maxIssue_RD", 2);
    maxIssue_WR = memsys->getParamUINT64(cp_name+".maxIssue_WR", 2);
    buffer_size_bits = memsys->getParamUINT64(cp_name+".buffer_size_bits", 7);
    buffer_size = 1<<buffer_size_bits;

    drain_mode = memsys->getParamBOOL(cp_name+".drain_mode", true);
    uint64_t num_qs = (drain_mode)? NUM_QTYPE:1;
    credits.resize(num_qs, 0);
    FIFO.resize(num_qs);
    if (drain_mode)
    {
        credits[RDQ] = buffer_size;
        credits[WRQ] = buffer_size;
    }
    else
        credits[0] = buffer_size;

    tCMD = memsys->getParamUINT64(cp_name+".tCMD", 1);
    tDATA = memsys->getParamUINT64(cp_name+".tDATA_MEM", 2);
    tRESP = memsys->getParamUINT64(cp_name+".tDATA_HOST", 1);

    /* Data structure init */
    for (uint64_t id = 0 ; id < buffer_size; id++)
    {
        avail_RID.push_back( id );
        avail_WID.push_back( (id|(1<<buffer_size_bits)) );
    }

    DataBlock e_data;
    entry_t e_init{0, 0, e_data};
    rbuffer.resize(buffer_size, e_init);
    wbuffer.resize(buffer_size, e_init);
    rbuffer_WAR_wid.resize(buffer_size, e_init);
    wbuffer_WAR_rid.resize(buffer_size, e_init);

    /* Stats value setting */
    register_stats( );
}

RequestReceiver::~RequestReceiver( )
{

}

void RequestReceiver::recvRequest(Packet* pkt, ncycle_t delay)
{
    credit_calc(pkt->cmd, true);
    registerCallback((CallbackPtr)&RequestReceiver::ID_remap, 
                     delay, 0, reinterpret_cast<void*>(pkt));
}

void RequestReceiver::recvResponse(Packet* pkt, ncycle_t delay)
{
    if (pkt->cmd==CMD_WRITE)
        resp_handle(pkt);
    else if (pkt->cmd==CMD_READ)
        Component::recvResponse(pkt, delay);
    else
        assert(0);
}

bool RequestReceiver::isReady(Packet* pkt)
{
    bool rv = true;
    uint64_t credit = std::numeric_limits<uint64_t>::max( );
    if (drain_mode==false)
        credit = credits[0];
    else if (pkt->cmd==CMD_READ)
        credit = credits[RDQ];
    else if (pkt->cmd==CMD_WRITE)
        credit = credits[WRQ];
    else
        assert(0);
    
    assert(credit < std::numeric_limits<uint64_t>::max( ));
    if (credit == 0 || CAM_stalls.empty( )==false)
        rv = false;

    return rv;
}

void RequestReceiver::handle_events(ncycle_t curr_tick)
{
    /* Get events of current cycle */
    prepare_events(curr_tick);

    /* Response handle */
    while (!await_resp.empty( ))
    {
        LocalEvent* tmp_et = await_resp.front( );
        Packet* pkt = tmp_et->pkt;
        resp_handle( pkt );

        await_resp.pop_front( );
        delete tmp_et;
    }

    /* Callback handle */
    if (!await_cb.empty( ))
        handle_await_callbacks( );

    /* Request handle */
    if (!await_req.empty( ))
        assert(0);
}

void RequestReceiver::insert_buffer(Packet* pkt)
{
    assert(get_bid(pkt)<buffer_size);
    if (pkt->cmd == CMD_READ)
    {
        rbuffer[get_bid(pkt)].addr = pkt->LADDR;
        rbuffer[get_bid(pkt)].valid = true;
    }
    else if (pkt->cmd == CMD_WRITE)
    {
        wbuffer[get_bid(pkt)].addr = pkt->LADDR;
        wbuffer[get_bid(pkt)].valid = true;
        wbuffer[get_bid(pkt)].wdata = pkt->buffer_data;
    }
    else
        assert(0);

    int qid = (drain_mode==false)? 0:((pkt->cmd==CMD_READ)? RDQ:WRQ);
    FIFO[qid].push_back(pkt);
    assert(FIFO[qid].size( )<=buffer_size);

    PCMC_DBG(dbg_msg, "[ReqRecv] Insert [0x%lx, ID=%lx, CMD=%c]"
        " to FIFO[%s] (size=%ld)\n", pkt->LADDR, pkt->req_id, 
        (pkt->cmd==CMD_READ)? 'R':'W', 
        (!drain_mode)? "":((pkt->cmd==CMD_READ)? "RD":"WR"),
        FIFO[qid].size( ));

    /* Schedule REQ-MUX event */
    if (last_wake_req_mux==wake_req_mux)
    {
        wake_req_mux = geq->getCurrentTick( )+1*ticks_per_cycle;
        registerCallback((CallbackPtr)&RequestReceiver::req_mux, 1);
    }
}

void RequestReceiver::resp_handle( Packet* pkt )
{
    if (pkt->cmd == CMD_READ)
    {
        PCMC_DBG(dbg_msg, "[ReqRecv] Ready to respond RD [0x%lx, ID=%lx]"
            " (qsize=%ld), CAM-stall#=%lu\n", pkt->LADDR, pkt->req_id,
            respq.size( ), CAM_stalls.size( ));
        
        /* WAR unlink */
        unlink_WAR(pkt->req_id, pkt->LADDR);

        /* Push into respq for response */
        pkt->req_id = ID_unmap(pkt->req_id, CMD_READ, pkt);
        credit_calc(CMD_READ, false);
        respq.push(pkt);
    }
    else if (pkt->cmd == CMD_WRITE) // can send data to dcache
    {
        PCMC_DBG(dbg_msg, "[ReqRecv] Send wdata of [0x%lx, ID=%lx] "
            "to DCACHE\n", pkt->LADDR, pkt->req_id);

        /* 
         * Got buffer id from dcache, then issue data 
         * Get data from wbuffer in case it is updated by WAW
         */
        pkt->buffer_data = wbuffer[get_bid(pkt->buffer_idx)].wdata;
        pkt->from = this;
        dcache->recvResponse(pkt, tDATA);

        /* BID retirement */
        assert(pkt->buffer_idx>=0);
        ID_unmap(pkt->buffer_idx, CMD_WRITE, pkt);
        credit_calc(CMD_WRITE, false);
        pkt->buffer_idx = -1;
    }
    else
        assert(0);

    /* Schedule response MUX event */
    if (last_wake_resp_mux!=wake_resp_mux)
        return;
    else if (respq.empty( )==false)
    {
        wake_resp_mux = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_resp_mux = std::max(wake_resp_mux, free_resp);
        registerCallbackAt((CallbackPtr)&RequestReceiver::resp_mux, wake_resp_mux);
    }
}

void RequestReceiver::credit_calc( cmd_t cmd, bool in )
{
    if (drain_mode==false)
    {
        if (in)
            credits[0]--;
        else
            credits[0]++;
        assert(credits[0]<=buffer_size);
    }
    else if (cmd == CMD_WRITE) // WRITE
    {
        if (in) // in
            credits[WRQ]--;
        else
            credits[WRQ]++;
        assert(credits[WRQ]<=buffer_size);
    }
    else
    {
        if (in)
            credits[RDQ]--;
        else
            credits[RDQ]++;
        assert(credits[RDQ]<=buffer_size);
    }
}

void RequestReceiver::ID_remap(Packet* pkt)
{
    uint64_t newID = 0;
    uint64_t originID = 0;
    if (pkt->cmd == CMD_READ)
    {
        assert(!avail_RID.empty( ));
        newID = avail_RID.front( );
        avail_RID.pop_front( );
        alloc_RID.insert(std::pair<uint64_t, uint64_t>(newID, pkt->req_id));
    }
    else if (pkt->cmd == CMD_WRITE)
    {
        assert(!avail_WID.empty( ));
        newID = avail_WID.front( );
        avail_WID.pop_front( );
        alloc_WID.insert(std::pair<uint64_t, uint64_t>(newID, pkt->req_id));
    }
    else
        assert(0);
    
    originID = pkt->req_id;
    pkt->req_id = newID;

    pkt->recvTick_recvr = geq->getCurrentTick( );
    pkt->recvTick = geq->getCurrentTick( );

    PCMC_DBG(dbg_msg, "[ReqRecv] Remap ID: HostID=0x%lx=>%s=0x%lx of [LADDR=0x%lx]\n",
        originID, (pkt->cmd==CMD_READ)? "RID":"WID", newID, pkt->LADDR);
    
    registerCallback((CallbackPtr)&RequestReceiver::CAM_check, 
                     1, 0, reinterpret_cast<void*>(pkt));
}

uint64_t RequestReceiver::ID_unmap(uint64_t ID, cmd_t type, Packet* pkt) 
{
     uint64_t hostID = 0;
     ncycle_t tmp_lat = geq->getCurrentTick( )-pkt->recvTick_recvr;

    if (type == CMD_READ)
    {
        avail_RID.push_back( ID );
        
        if (alloc_RID.count(ID)==0)
            assert(0);
        hostID = alloc_RID[ID];
        alloc_RID.erase(ID);
        rbuffer[ID].valid = false;

        /* Stats update */
        if (tmp_lat>max_rd_lat)
            max_rd_lat = tmp_lat;
        if (tmp_lat<min_rd_lat)
            min_rd_lat = tmp_lat;

        avg_rd_lat = (avg_rd_lat*num_reads+tmp_lat)/(double)(num_reads+1);
        num_reads += 1;
    }
    else if (type == CMD_WRITE)
    {
        avail_WID.push_back( ID );

        if (alloc_WID.count(ID)==0)
            assert(0);
        hostID = alloc_WID[ID];
        alloc_WID.erase(ID);
        wbuffer[get_bid(ID)].valid = false;

        /* Stats update */
        if (tmp_lat>max_wr_lat)
            max_wr_lat = tmp_lat;
        if (tmp_lat<min_wr_lat)
            min_wr_lat = tmp_lat;

        avg_wr_lat = (avg_wr_lat*num_writes+tmp_lat)/(double)(num_writes+1);
        num_writes += 1;
    }
    else
        assert(0);

    PCMC_DBG(dbg_msg, "[ReqRecv] Retire %sID=%lx\n", 
        (type==CMD_READ)? "R":"W", ID);
        
    return hostID;
}

uint64_t RequestReceiver::get_bid(Packet* pkt)
{
    uint64_t rv = 0;
    uint64_t ref1 = 1;
    if (pkt->cmd==CMD_READ)
        rv = (uint64_t)(pkt->req_id);
    else if (pkt->cmd==CMD_WRITE)
        rv = (uint64_t)(pkt->req_id) & (~(ref1<<buffer_size_bits));
    else
        assert(0);
    return rv;
}

uint64_t RequestReceiver::get_bid(uint64_t ID)
{
    uint64_t rv = 0;
    uint64_t ref1 = 1;
    if (ID>>buffer_size_bits) // WRITE
        rv = ID & (~(ref1<<buffer_size_bits));
    else
        rv = ID;
    return rv;
}

/* NOTE: this function is executed once per cycle */
void RequestReceiver::CAM_check(Packet* pkt)
{
    /* Check read & write buffer (1-cycle) */
    uint64_t rbid = buffer_size;
    uint64_t wbid = buffer_size;
    bool wasWAR = false;
    
    if (pkt->cmd==CMD_WRITE)
    {
        for (rbid=0; rbid<buffer_size; rbid++)
        {
            /* 
             * Treat as WAW if there's already WCMD in WAR buffer 
             * Hence, multiple WAR on same LA is impossible 
             */
            if (rbuffer[rbid].valid && rbuffer[rbid].addr==pkt->LADDR)
            {
                if (rbuffer_WAR_wid[rbid].valid)
                {
                    wasWAR = true;
                    wbid = get_bid(rbuffer_WAR_wid[rbid].id);
                    assert(wbuffer[wbid].addr==pkt->LADDR);
                }
                break;
            }
        }
    }

    if (wasWAR==false && rbid==buffer_size)
    {
        for (wbid=0; wbid<buffer_size; wbid++)
        {
            /* MUST ensure all command addresses are unique */
            if (wbuffer[wbid].valid && wbuffer[wbid].addr==pkt->LADDR)
            {
                assert(rbid==buffer_size);
                break;
            }
        }
    }
    
    if (dbg_msg && drain_mode)
    {
        if (pkt->cmd==CMD_WRITE)
            PCMC_DBG(dbg_msg, "[ReqRecv] CAM check on WR [0x%lx, ID=%lx], "
                "WAR-HZD=%s, WAW-HZD=%s\n", pkt->LADDR, pkt->req_id, 
                (rbid<buffer_size)? "true":"false", 
                (wbid<buffer_size)? "true":"false");
        else if (pkt->cmd==CMD_READ)
            PCMC_DBG(dbg_msg, "[ReqRecv] CAM check on RD [0x%lx, ID=%lx], "
                "RAW-HZD=%s\n", pkt->LADDR, pkt->req_id, 
                (wbid<buffer_size)? "true":"false");
    }

    /* Handle hazard (1-cycle) */
    if (pkt->cmd==CMD_WRITE)
    {
        if (drain_mode)
        {
            if (rbid<buffer_size && wasWAR==false)
                hazard_handle(pkt, WAR, rbid);
            else if (wbid<buffer_size)
                hazard_handle(pkt, WAW, wbid);

            if (wbid==buffer_size)
            {
                insert_buffer(pkt);
//                registerCallback((CallbackPtr)&RequestReceiver::insert_buffer,
//                    1, PRIORITY_INSERT_BUFFER, reinterpret_cast<void*>(pkt));
            }
        }
        else
            insert_buffer(pkt);
    }
    else if (pkt->cmd==CMD_READ)
    {
        if (drain_mode && wbid<buffer_size)
            hazard_handle(pkt, RAW, wbid);
        else if (drain_mode)
        {
            insert_buffer(pkt);
//            registerCallback((CallbackPtr)&RequestReceiver::insert_buffer,
//                1, PRIORITY_INSERT_BUFFER, reinterpret_cast<void*>(pkt));
        }
        else
            insert_buffer(pkt); 
    }
    else
        assert(0);


    if (rbid==buffer_size && wbid==buffer_size)
        num_xHZD+=1;
}

void RequestReceiver::hazard_handle(Packet* pkt, hzd_t hzd, uint64_t hit_bid)
{
    PCMC_DBG(dbg_msg, "[ReqRecv] Handle %s-HZD of [0x%lx, ID=%lx], "
        "where hit-ID=%lx\n", (hzd==WAR)? "WAR":((hzd==WAW)? "WAW":"RAW"),
        pkt->LADDR, pkt->req_id, hit_bid);

    if (hzd == WAR)
    {
        assert(wbuffer_WAR_rid[get_bid(pkt)].valid==false); //XXX
        wbuffer_WAR_rid[get_bid(pkt)].id = hit_bid;
        wbuffer_WAR_rid[get_bid(pkt)].valid = true;
        rbuffer_WAR_wid[hit_bid].id = pkt->req_id;
        rbuffer_WAR_wid[hit_bid].valid = true;

        num_WAR+=1;
    }
    else if (hzd == WAW)
    {
        /* Insert new data to hit WID */
        wbuffer[hit_bid].wdata = pkt->buffer_data;
        assert(wbuffer[hit_bid].wdata.getSize( )>0);

        /* Retire input BID and response directly */
        ID_unmap(pkt->req_id, CMD_WRITE, pkt);
        credit_calc(CMD_WRITE, false);
        
        pkt->cmd = CMD_PKT_DEL;
        pkt->owner->recvResponse(pkt);
        
        num_WAW+=1;
    }
    else if (hzd == RAW)
    {
        /* Ready to respond with data in wbuffer */
        pkt->buffer_data = wbuffer[hit_bid].wdata;
        assert(pkt->buffer_data.getSize( )>0);

        /* Check whether directly served cmd can be responded now */
        CAM_stalls.push( pkt );
        pkt->req_id = ID_unmap(pkt->req_id, CMD_READ, pkt);
        credit_calc(CMD_READ, false);
        
        num_RAW+=1;
    }
    else
        assert(0);

    /* Schedule response MUX if resp-port is available for stalled req */
    if (last_wake_resp_mux==wake_resp_mux &&
        CAM_stalls.empty( )==false)
    {
        wake_resp_mux = geq->getCurrentTick( )+1*ticks_per_cycle;
        wake_resp_mux = std::max(wake_resp_mux, free_resp);
        registerCallbackAt((CallbackPtr)&RequestReceiver::resp_mux, wake_resp_mux);
    }
}

bool RequestReceiver::isIssuable(cmd_t type)
{
    bool rv = true;
    std::list<Packet*>* fifo = (drain_mode==false)? &FIFO[0]:
                                ((type==CMD_READ)? &FIFO[RDQ] : &FIFO[WRQ]);
    Packet* head_pkt = fifo->front( );
    
    if (fifo->empty( ))
        return false;

    /* Check availability of dcache or FIFO */
    if (!dcache->isReady(head_pkt) ||
        (drain_mode && type==CMD_WRITE && 
         wbuffer_WAR_rid[get_bid(head_pkt)].valid))
    {
        rv = false;
//        if (dcache->isReady(head_pkt))
//            std::cout << "Pended due to WAR hazard" << std::endl;
//        else
//            std::cout << "DCACHE is not ready" << std::endl;
    }

    return rv;
}

/* NOTE: this function is executed once per cycle */
void RequestReceiver::req_mux( )
{
    bool is_issue = true;
    if (drain_mode)
    {
        static uint64_t cnt_rd = 0;
        static uint64_t cnt_wr = 0;
        bool wr_issuable = isIssuable(CMD_WRITE);
        bool rd_issuable = isIssuable(CMD_READ);

        if (mux_st==PRIORITY_READ)
        {
            if (rd_issuable)
            {
                req_issue(CMD_READ);
                cnt_rd+=1;
                if (cnt_rd>=maxIssue_RD)
                {
                    mux_st = PRIORITY_WRITE;
                    cnt_rd = 0;
                }
            }
            else if (wr_issuable)
            {
                req_issue(CMD_WRITE);
                cnt_wr+=1;
                if (cnt_wr>=maxIssue_WR)
                    cnt_wr = 0;
            }
            else
                is_issue = false;
        }
        else if (mux_st==PRIORITY_WRITE)
        {
            if (wr_issuable)
            {
                req_issue(CMD_WRITE);
                cnt_wr+=1;
                if (cnt_wr>=maxIssue_WR)
                {
                    mux_st = PRIORITY_READ;
                    cnt_wr = 0;
                }
            }
            else if (rd_issuable)
            {
                req_issue(CMD_READ);
                cnt_rd+=1;
                if (cnt_rd>=maxIssue_RD)
                    cnt_rd = 0;
            }
            else
                is_issue = false;
        }
        else
            assert(0);
    }
    else if (isIssuable( ))
        req_issue( );
    else
        is_issue = false;

    /* Schedule event if req exists in FIFO */
    last_wake_req_mux = wake_req_mux;
    ncycle_t LAT = (is_issue==false)? 1 : tCMD;
    if ((drain_mode==false && FIFO[0].empty( )==false) ||
        (drain_mode && (FIFO[RDQ].empty( )==false || FIFO[WRQ].empty( )==false)))
    {
        wake_req_mux = geq->getCurrentTick( ) + LAT*ticks_per_cycle;
        registerCallback((CallbackPtr)&RequestReceiver::req_mux, LAT); 
    }
}

void RequestReceiver::req_issue(cmd_t type)
{
    Packet* pkt = NULL;
    int qid = (drain_mode==false)? 0:((type==CMD_READ)? RDQ:WRQ);
    assert(FIFO[qid].empty( )==false);

    /* Get a command from the FIFO head */
    pkt = FIFO[qid].front( );
    FIFO[qid].pop_front( );

    if (drain_mode && type==CMD_WRITE)
        assert(wbuffer_WAR_rid[get_bid(pkt)].valid==false);

    /* Stats update */
    ncycle_t tmp_lat = geq->getCurrentTick( )-pkt->recvTick;
    if (tmp_lat>max_issue_lat)
        max_issue_lat = tmp_lat;
    if (tmp_lat<min_issue_lat)
        min_issue_lat = tmp_lat;

    avg_issue_lat = 
        (avg_issue_lat*num_issue+tmp_lat)/(double)(num_issue+1);
    num_issue += 1;

    if (pkt->cmd==CMD_READ)
    {
        if (tmp_lat>max_rd_issue_lat)
            max_rd_issue_lat = tmp_lat;
        if (tmp_lat<min_rd_issue_lat)
            min_rd_issue_lat = tmp_lat;

        avg_rd_issue_lat = 
            (avg_rd_issue_lat*num_rd_issue+tmp_lat)/(double)(num_rd_issue+1);
        num_rd_issue += 1;
    }
    else
    {
        if (tmp_lat>max_wr_issue_lat)
            max_wr_issue_lat = tmp_lat;
        if (tmp_lat<min_wr_issue_lat)
            min_wr_issue_lat = tmp_lat;

        avg_wr_issue_lat = 
            (avg_wr_issue_lat*num_wr_issue+tmp_lat)/(double)(num_wr_issue+1);
        num_wr_issue += 1;
    }

    /* Issue command */
    dcache->recvRequest(pkt, tCMD); 

    PCMC_DBG(dbg_msg, "[ReqRecv] Issue [0x%lx, CMD=%c, ID=%lx] to DCACHE\n",
        pkt->LADDR, (pkt->cmd==CMD_READ)? 'R':'W', pkt->req_id);
}

void RequestReceiver::unlink_WAR(uint64_t rbid, uint64_t LADDR)
{
    if (rbuffer_WAR_wid[rbid].valid==false)
        return;

    PCMC_DBG(dbg_msg, "[ReqRecv] Unlink WAR pair RID=%lx-WID=%lx\n",
        rbid, rbuffer_WAR_wid[rbid].id);

    uint64_t WAR_wid = rbuffer_WAR_wid[rbid].id;
    rbuffer_WAR_wid[rbid].valid = false;
    wbuffer_WAR_rid[get_bid(WAR_wid)].valid = false;

    /* Re-Check RD buffer whether RAR requests exist or not */
    uint64_t i = 0;
    for ( ; i<buffer_size; i++)
    {
        if (i==rbid) continue;
        if (rbuffer[i].valid && rbuffer[i].addr==LADDR)
        {
            assert(rbuffer_WAR_wid[i].valid==false);
            break;
        }
    }

    if (i<buffer_size)
    {
        std::list<Packet*>::iterator f_it = FIFO[WRQ].begin( );
        for ( ; f_it!=FIFO[WRQ].end( ); f_it++)
        {
            if (WAR_wid==(uint64_t)((*f_it)->req_id))
            {
                hazard_handle((*f_it), WAR, i);
                break;
            }
        }
        assert(f_it!=FIFO[WRQ].end( ));
    }
}

void RequestReceiver::resp_mux( )
{
    /* MUX between respq & CAM_stalls that is served by wbuffer */
    assert(geq->getCurrentTick( )>=free_resp);

    Packet* pkt = NULL;
    if (respq.empty( )==false)
    {
        /* respq has higher priority */
        pkt = respq.front( );
        respq.pop( );
    }
    else if (CAM_stalls.empty( )==false)
    {
        pkt = CAM_stalls.front( );
        CAM_stalls.pop( );
    }
    else
        assert(0);
    
    PCMC_DBG(dbg_msg, "[ReqRecv] Respond RD [0x%lx, ID=%lx] (qsize=%ld, "
        "CAM-stall#=%lu)\n", pkt->LADDR, pkt->req_id, respq.size( ), 
        CAM_stalls.size( ));
    
    parser->recvResponse(pkt, tRESP);

    last_wake_resp_mux = wake_resp_mux;
    ncycle_t LAT = (pkt)? tRESP : 1;
    free_resp = (pkt)? (wake_resp_mux+tRESP*ticks_per_cycle):free_resp;
    if (respq.empty( )==false || CAM_stalls.empty( )==false)
    {
        wake_resp_mux = geq->getCurrentTick( )+LAT*ticks_per_cycle;
        registerCallback((CallbackPtr)&RequestReceiver::resp_mux, LAT);
    }
}

/*========== Below is stats setting ==========*/
void RequestReceiver::register_stats( )
{
    /* RESET phase */
    uint64_t u64_zero = 0;
    uint64_t u64_max = std::numeric_limits<uint64_t>::max( );
    double df_init = 0.0;
    RESET_STATS(num_WAR, u64_zero);
    RESET_STATS(num_WAW, u64_zero);
    RESET_STATS(num_RAW, u64_zero);
    RESET_STATS(num_xHZD, u64_zero);
    RESET_STATS(num_reads, u64_zero);
    RESET_STATS(num_writes, u64_zero);

    RESET_STATS(max_rd_lat, u64_zero);
    RESET_STATS(min_rd_lat, u64_max);
    RESET_STATS(avg_rd_lat, df_init);

    RESET_STATS(max_wr_lat, u64_zero);
    RESET_STATS(min_wr_lat, u64_max);
    RESET_STATS(avg_wr_lat, df_init);

    RESET_STATS(max_issue_lat, u64_zero);
    RESET_STATS(min_issue_lat, u64_max);
    RESET_STATS(avg_issue_lat, df_init);
    RESET_STATS(max_rd_issue_lat, u64_zero);
    RESET_STATS(min_rd_issue_lat, u64_max);
    RESET_STATS(avg_rd_issue_lat, df_init);
    RESET_STATS(max_wr_issue_lat, u64_zero);
    RESET_STATS(min_wr_issue_lat, u64_max);
    RESET_STATS(avg_wr_issue_lat, df_init);
    num_issue = 0;
    num_rd_issue = 0;
    num_wr_issue = 0;

    /* Register phase */
    ADD_STATS(cp_name, num_WAR);
    ADD_STATS(cp_name, num_WAW);
    ADD_STATS(cp_name, num_RAW);
    ADD_STATS(cp_name, num_xHZD);
    ADD_STATS(cp_name, num_reads);
    ADD_STATS(cp_name, num_writes);
    ADD_STATS_N_UNIT(cp_name, max_rd_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_rd_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_rd_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_wr_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_wr_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_wr_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_rd_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_rd_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_rd_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, max_wr_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, min_wr_issue_lat, "cycles");
    ADD_STATS_N_UNIT(cp_name, avg_wr_issue_lat, "cycles");
}

void RequestReceiver::calculate_stats( )
{
    if (num_reads>0)
    {
        max_rd_lat /= ticks_per_cycle;
        min_rd_lat /= ticks_per_cycle;
        avg_rd_lat /= ticks_per_cycle;
    }

    if (num_writes>0)
    {
        max_wr_lat /= ticks_per_cycle;
        min_wr_lat /= ticks_per_cycle;
        avg_wr_lat /= ticks_per_cycle;
    }

    if (num_issue>0)
    {
        max_issue_lat /= ticks_per_cycle;
        min_issue_lat /= ticks_per_cycle;
        avg_issue_lat /= ticks_per_cycle;
    }

    if (num_rd_issue>0)
    {
        max_rd_issue_lat /= ticks_per_cycle;
        min_rd_issue_lat /= ticks_per_cycle;
        avg_rd_issue_lat /= ticks_per_cycle;
    }

    if (num_wr_issue>0)
    {
        max_wr_issue_lat /= ticks_per_cycle;
        min_wr_issue_lat /= ticks_per_cycle;
        avg_wr_issue_lat /= ticks_per_cycle;
    }
}

