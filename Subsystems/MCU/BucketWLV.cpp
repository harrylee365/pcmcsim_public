#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "base/Stats.h"
#include "base/MemInfo.h"
#include "base/PCMInfo.h"
#include "AITManager/AITManager.h"
#include "Subsystems/MCU/BucketWLV.h"

using namespace PCMCsim;

BucketWLV::BucketWLV(MemoryControlSystem* memsys_, 
    std::string cfg_header, ncycle_t ticks_per_cycle_)
:MicroControlUnit(memsys_), tdec(memsys_->tdec)
{
    cp_name = cfg_header;
    dbg_msg = memsys->getParamBOOL(cp_name+".dbg_msg", true);
    if (ticks_per_cycle_==0)
        ticks_per_cycle= memsys->getParamUINT64(cp_name+".ticks_per_cycle", 1);
    else 
        ticks_per_cycle = ticks_per_cycle_;

    /* Init algorithm */
    num_buckets = memsys->getParamUINT64(cp_name+".num_buckets", 64);
    max_bucket_cnt = memsys->getParamUINT64(cp_name+".max_bucket_cnt", 4);
    memsys->aitm->get_gpc_info(gpc_cline_offset, gpc_byte_offset);
    BLOCK_OFFSET = memsys->adec->BLOCK_OFFSET;
    NUM_BLOCKS = memsys->info->get_capacity_bits( ) / (8*memsys->info->BLOCK_SIZE);

    /* Sanity check */
    if (NUM_BLOCKS<=2)
    {
        std::cerr << "[BucketWLV] Block number is too small! "
            << "Check the capacity and block size" << std::endl;
        assert(0);
        exit(1);
    }

    if (max_bucket_cnt<2)
    {
        std::cerr << "[BucketWLV] max-bucket-cnt is too small! "
            << "Value should be larger than 1" << std::endl;
        assert(0);
        exit(1);
    }

    if (num_buckets<4)
    {
        std::cerr << "[BucketWLV] num-buckets is too small!" << std::endl;
        assert(0);
        exit(1);
    }

    tWLV = memsys->getParamUINT64(cp_name+".tWLV", 100);

    th_wlv = max_bucket_cnt / 2;
    base_bucket = 0;
    oldest_free_bucket = 0;

    wr_cntr = new uint64_t[NUM_BLOCKS];
    alloc_list.resize(num_buckets);
    free_list.resize(num_buckets);

    /* Assume that all LBAs are linearly mapped */
    for (uint64_t i=0; i<NUM_BLOCKS; i++)
    {
        BlockAddress in_blk(i, i);
        alloc_list[base_bucket].push_front(in_blk);
    }

    /* Overprovision free blocks for fast-move */
    uint64_t added_blocks = NUM_BLOCKS*(memsys->op_rate-1);
    for (uint64_t i=0; i<added_blocks; i++)
    {
        BlockAddress in_blk(NUM_BLOCKS+i);
        free_list[base_bucket].push_front(in_blk);
    }
    NUM_BLOCKS += added_blocks;
}

BucketWLV::~BucketWLV( )
{
    delete[] wr_cntr;
}

bool BucketWLV::isReady(Packet* /*pkt*/)
{
    bool rv = true;
    if (state==ST_BUSY)
        rv = false;

    return rv;
}

void BucketWLV::recvRequest(Packet* pkt, ncycle_t delay)
{
    register_isr(pkt);
    Component::recvRequest(pkt, delay);
}

void BucketWLV::recvResponse(Packet* pkt, ncycle_t delay)
{
    if (pkt->cmd==CMD_PKT_DEL)
    {
        assert(pkt->owner==this);
        delete pkt;
    }
    else
        assert(0);
}

void BucketWLV::handle_events(ncycle_t curr_tick)
{
    prepare_events(curr_tick);

    if (!await_resp.empty( ))
        handle_await_resps( );

    if (!await_req.empty( ))
        handle_await_reqs( );
}

void BucketWLV::handle_await_reqs( )
{
    std::list<LocalEvent*>::iterator e_it = await_req.begin( );
    for ( ; e_it!=await_req.end( ); )
    {
        Packet* pkt = (*e_it)->pkt;
        lookup_isr(pkt);

        /* Free event */
        delete (*e_it);
        e_it = await_req.erase(e_it);
    }
}

void BucketWLV::handle_await_resps( )
{
    std::list<LocalEvent*>::iterator e_it = await_req.begin( );
    for ( ; e_it!=await_req.end( ); )
    {
        // TODO: handling of response from DRAM-AIT

        /* Free event */
        delete (*e_it);
        e_it = await_req.erase(e_it);
    }
}

void BucketWLV::register_isr(Packet* pkt)
{
    if (pkt->cmd==CMD_IRQ)
    {
        ISRWrapper isr_w;
        isr_w.arg = reinterpret_cast<void*>(pkt);
        
        if (pkt->from==aitm)
        {
            assert(state==ST_BUSY);
            isr_w.isr = std::bind(&BucketWLV::handle_aitm_irq,
                                  this, std::placeholders::_1);
        }
        else // from RMW
        {
            state = ST_BUSY;
            isr_w.isr = std::bind(&BucketWLV::run_wlv, 
                                  this, std::placeholders::_1);
        }

        isr_map.insert(std::pair<Packet*, ISRWrapper>(pkt, isr_w));
    }
    else
        assert(0);
}

void BucketWLV::run_wlv(void* args)
{
    /* Extract address information */
    Packet* pkt = reinterpret_cast<Packet*>(args);
    uint64_t LBA = pkt->LADDR>>BLOCK_OFFSET;
    uint64_t PBA = pkt->PADDR>>BLOCK_OFFSET;

    PCMC_DBG(dbg_msg, "[BucketWLV] Run WLV for CMD [LA=0x%lx, PA=0x%lx, "
        "LBA=0x%lx, PBA=0x%lx]\n", pkt->LADDR, pkt->PADDR, LBA, PBA);

    /* Find the bucket that has PBA */
    BlockAddress a_block;
    uint64_t bidx = get_bucket_idx(PBA);
    std::list<BlockAddress>::iterator a_it = alloc_list[bidx].begin( );
    for ( ; a_it!=alloc_list[bidx].end( ); a_it++)
    {
        if (a_it->PBA==PBA)
        {
            assert(a_it->LBA==LBA);
            a_block = *a_it;
            alloc_list[bidx].erase(a_it);
            break;
        }
    }
    assert(a_it!=alloc_list[bidx].end( ));

    /* Execute algorithm */
    bool bucket_wrap_ready = false;
    if (bidx!=(base_bucket+num_buckets-1)%num_buckets)
    {
        /* Move to next bucket if threshold reaches */
        wr_cntr[PBA] += 1;
        if (wr_cntr[PBA]%max_bucket_cnt==0)
        {
            alloc_list[bidx].erase(a_it);
            push_block(alloc_list, (bidx+1)%num_buckets, a_block, IS_HEAD); 
            
            PCMC_DBG(dbg_msg, "[BucketWLV] Move PBA=0x%lx to bucket-%lu\n",
                PBA, (bidx+1)%num_buckets);
        }
    }
    else
    {
        /* Accessed block is on upper bucket boundary */
        assert(wr_cntr[PBA]>0);
        uint64_t wrap_cnt = wr_cntr[PBA] % max_bucket_cnt;
        if (wrap_cnt>0 && // remap right after reaching upper boundary
            wrap_cnt<th_wlv)
        {
            /* Simply increment write counter */
            wr_cntr[PBA] += 1;
        }
        else
        {
            /* Get free block for MOVE or SWAP */
            using namespace std;
            pair<BlockAddress, bool> ret_pair = get_free_block( );
            bool need_bucket_wrap = ret_pair.second;

            /* Get address info and update LBAs */
            uint64_t LBA_MAP = ret_pair.first.LBA;
            uint64_t PBA_MAP = ret_pair.first.PBA;

            ret_pair.first.LBA = LBA;
            a_block.LBA = LBA_MAP;

            // XXX: Reserve push_block(free_list, bidx, ....) for original Bucket WL
            if (LBA_MAP==INVALID_ADDR)  // MOVE
                push_block(free_list, bidx, a_block, IS_HEAD);
            else                        // SWAP
            {
                /* Update bucket of old block info */
                wr_cntr[PBA] += 1;
                if (wr_cntr[PBA]%max_bucket_cnt==0)
                {
                    /* Count reaches max val -> move to next bucket */
                    push_block(alloc_list, (bidx+1)%num_buckets,
                        a_block, IS_HEAD);

                    /* 
                     * Buckets require wrapping(=base bucket++) if:
                     * 1. next bucket is base bucket
                     * 2. next bucket has only this block
                     */
                    if ((bidx+1)%num_buckets==base_bucket &&
                        alloc_list[base_bucket].size( )==1)
                    {
                        need_bucket_wrap = true;
                    }
                }
                else // re-push to same bucket
                    push_block(alloc_list, bidx, a_block, IS_HEAD);
            }

            /* Determine bucket of block info using counter value */
            wr_cntr[PBA_MAP] += 1;
            if (wr_cntr[PBA_MAP]%max_bucket_cnt==0)
            {
                /* Count of block reaches max val -> Move to next bucket */
                push_block(alloc_list, (base_bucket+1)%num_buckets, 
                    ret_pair.first, IS_HEAD);

                /* Checks if buckets require wrapping */
                if (alloc_list[base_bucket].empty( ) || need_bucket_wrap)
                    bucket_wrap_ready = true;
            }
            else
                push_block(alloc_list, base_bucket, ret_pair.first, IS_HEAD);

            issue_wlv_req(LBA, PBA, LBA_MAP, PBA_MAP);

            PCMC_DBG(dbg_msg, "[BucketWLV] Remap LBA=0x%lx to PBA=0x%lx, "
                "and LBA=0x%lx to PBA=0x%lx\n", LBA, PBA_MAP, LBA_MAP, PBA);
        }
    }

    if ((free_list[base_bucket].empty( ) && 
         alloc_list[base_bucket].empty( )) ||
        bucket_wrap_ready)
    {
        base_bucket = (base_bucket+1) % num_buckets;
    }

    /* Delete packet */
    pkt->cmd = CMD_PKT_DEL;
    pkt->owner->recvResponse(pkt);
}

void BucketWLV::issue_wlv_req(uint64_t LBA, uint64_t PBA, uint64_t LBA_MAP, uint64_t PBA_MAP)
{
    assert(PBA!=INVALID_ADDR && PBA_MAP!=INVALID_ADDR && LBA!=INVALID_ADDR);

    Packet* wlv_pkt = new Packet( );
    wlv_pkt->owner = this;
    wlv_pkt->from = this;
    wlv_pkt->cmd = CMD_WLV;
    wlv_pkt->LADDR = LBA;
    wlv_pkt->PADDR = PBA;
    wlv_pkt->LADDR_MAP = LBA_MAP;
    wlv_pkt->PADDR_MAP = PBA_MAP;
    wlv_pkt->src_id = SRC_MCU;
    wlv_pkt->dest = aitm;

    if (LBA_MAP!=INVALID_ADDR)
        wlv_pkt->swap = true;

    xbar->recvRequest(wlv_pkt, tWLV);
}

void BucketWLV::handle_aitm_irq(void* args)
{
    Packet* pkt = reinterpret_cast<Packet*>(args);
    state = ST_IDLE;

    PCMC_DBG(dbg_msg, "[BucketWLV] Ready to get next WLV request from RMW\n");

    /* Delete packet */
    pkt->cmd = CMD_PKT_DEL;
    pkt->owner->recvResponse(pkt);
}

std::pair<BlockAddress, bool>
BucketWLV::get_free_block( )
{
    std::pair<BlockAddress, bool> ret;
    ret.first.PBA = INVALID_ADDR;
    ret.second = false;

    if (free_list[base_bucket].empty( )==false)
    {
        /* Get a free block directly (MOVE BLOCK) */
        ret.first = pop_block(free_list, base_bucket, IS_TAIL);
        num_move += 1;

        PCMC_DBG(dbg_msg, "[BucketWLV] Get a free block directly "
            "(LBA=0x%lx, PBA=0x%lx)\n", ret.first.LBA, ret.first.PBA);
    }
    else
    {
        /* Get coldest block in alloc list (SWAP BLOCK) */
        assert(alloc_list[base_bucket].empty( )==false);
        ret.first = pop_block(alloc_list, base_bucket, IS_TAIL);

        //XXX: followings are original Bucket WL
//        /* Get oldest free block to store data of coldest block */
//        uint64_t str_bidx = oldest_free_bucket;
//        BlockAddress str_block = pop_block(free_list, oldest_free_bucket, IS_TAIL);
//
//        /* Determine bucket of block info using counter value */
//        wr_cntr[str_block.PBA] += 1;
//        if (wr_cntr[str_block.PBA]%max_bucket_cnt==0)
//        {
//            /* Count of block reaches max val -> Move to next bucket */
//            push_block(alloc_list, (str_bidx+1)%num_buckets, 
//                       str_block, IS_HEAD);
//
//            /* 
//             * Buckets require wrapping(=base bucket++) if:
//             * 1. next bucket is base bucket
//             * 2. next bucket has only this block (not sacrifice majority)
//             */
//            if ((str_bidx+1)%num_buckets==base_bucket &&
//                alloc_list[base_bucket].size( )==1)
//            {
//                ret.second = true;
//            }
//        }
//        else
//            push_block(alloc_list, str_bidx, str_block, IS_HEAD);

        num_swap += 1;
    
        PCMC_DBG(dbg_msg, "[BucketWLV] Get coldest block from "
            "allocated list (LBA=0x%lx, PBA=0x%lx)\n", 
            ret.first.LBA, ret.first.PBA);
    }

    assert(ret.first.PBA!=INVALID_ADDR);
    return ret;
}

void BucketWLV::push_block(std::vector<std::list<BlockAddress>>& dest_list,
                          uint64_t dest_bidx, BlockAddress in_block, int op_pos)
{
    assert(in_block.PBA!=INVALID_ADDR);

    /* Push block */
    if (op_pos==IS_TAIL)
        dest_list[dest_bidx].push_back(in_block);
    else if (op_pos==IS_HEAD)
        dest_list[dest_bidx].push_front(in_block);
    else
        assert(0);

    /* Update oldest bucket index: inserted block can be oldest */
    if (&dest_list==&free_list)
    {
        /* 
         * New oldest free for one of followings:
         *  bidx > oldest >= base 
         *  base > bidx > oldest
         *  oldest >= base > bidx
         */
        if ((dest_bidx>oldest_free_bucket && 
             oldest_free_bucket>=base_bucket) ||
            (base_bucket>dest_bidx && 
             dest_bidx>oldest_free_bucket) ||
            (oldest_free_bucket>=base_bucket &&
             base_bucket>dest_bidx))
        {
            oldest_free_bucket = dest_bidx;
        }
    }
}

BlockAddress BucketWLV::pop_block(std::vector<std::list<BlockAddress>>& src_list,
                         uint64_t src_bidx, int op_pos)
{
    assert(src_list[src_bidx].empty( )==false);
    BlockAddress ret_block;

    /* Pop block */
    if (op_pos==IS_TAIL)
    {
        ret_block = src_list[src_bidx].back( );
        src_list[src_bidx].pop_back( );
    }
    else if (op_pos==IS_HEAD)
    {
        ret_block = src_list[src_bidx].front( );
        src_list[src_bidx].pop_front( );
    }
    else
        assert(0);
    assert(ret_block.PBA!=INVALID_ADDR);

    /* Update oldest free bucket during pop */
    if ((&src_list==&free_list) && 
        src_bidx==oldest_free_bucket &&
        free_list[oldest_free_bucket].empty( ))
    {
        if (oldest_free_bucket>base_bucket)
        {
            /* Normal: oldest free bucket idx value > base */
            while (oldest_free_bucket>base_bucket)
            {
                if (free_list[oldest_free_bucket].empty( )==false)
                    break;
                else
                    oldest_free_bucket -= 1;
            }
        }
        else if (oldest_free_bucket<base_bucket)
        {
            /* Oldest free bucket idx value < base (wrapped) */
            bool found = false;
            while (oldest_free_bucket>0) // traverse down to 0-idx 
            {
                if (free_list[oldest_free_bucket].empty( )==false)
                {
                    found = true;
                    break;
                }
                else
                {
                    oldest_free_bucket -= 1;
                    if (oldest_free_bucket==0 && 
                        free_list[0].empty( )==false)
                    {
                        found = true;
                        break;
                    }
                }
            }

            if (found==false) // find by wrapping index value
            {
                oldest_free_bucket = num_buckets-1;
                while (oldest_free_bucket>base_bucket)
                {
                    if (free_list[oldest_free_bucket].empty( )==false)
                        break;
                    else
                        oldest_free_bucket -= 1;
                }
            }
        }
    }

    return ret_block;
}

uint64_t BucketWLV::get_bucket_idx(uint64_t PBA)
{
    return ((wr_cntr[PBA]/max_bucket_cnt)%num_buckets);
}

