#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "MemoryModules/DummyMemory/DummyAIT.h"

using namespace PCMCsim;

DummyAIT::DummyAIT(MemoryControlSystem* memsys_, std::string cfg_header)
:DummyMemory(memsys_)
{
    cp_name = cfg_header;
    ticks_per_cycle = memsys->getParamUINT64("global.ticks_per_cycle", 1);

    tx_line_bits = memsys->getParamUINT64(cfg_header+".tx_line_bits", 6);
    byte_offset = memsys->getParamUINT64(cfg_header+".byte_offset", 3);
    assert(tx_line_bits>=byte_offset);

    tRD = memsys->getParamUINT64(cfg_header+".tRD", 10);
    tWR = memsys->getParamUINT64(cfg_header+".tWR", 20);

    data_enable = memsys->getParamBOOL(cp_name+".data_enable", false);
    meta_enable = false;
    true_enable = false;
}

void DummyAIT::handle_events(ncycle_t curr_tick)
{
    /* Get events from queue */
    prepare_events(curr_tick);

    /* Don't change following order */
    if (!await_resp.empty( ))
        handle_await_resps( );

    if (!await_req.empty( ))
        handle_await_reqs( );
}

void DummyAIT::handle_await_reqs( )
{
    std::list<LocalEvent*>::iterator e_it = await_req.begin( );
    for ( ; e_it!=await_req.end( ); )
    {

        Packet* pkt = (*e_it)->pkt;
        ncycle_t LAT = (pkt->cmd==CMD_READ)? tRD : tWR;
        
        refer_data(pkt);
        pkt->from = this;
        parent->recvResponse(pkt, LAT);

        /* Free event */
        delete (*e_it);
        e_it = await_req.erase(e_it);
    }
}

void DummyAIT::handle_await_resps( )
{
    std::list<LocalEvent*>::iterator e_it = await_resp.begin( );
    for ( ; e_it!=await_resp.end( ); )
    {
        /* Path of getting write data */
        Packet* pkt = (*e_it)->pkt;

        assert(pkt->cmd==CMD_WRITE);
        pkt->cmd = CMD_PKT_DEL;
        pkt->owner->recvResponse(pkt);

        /* Free event */
        delete (*e_it);
        e_it = await_resp.erase(e_it);
    }
}

void DummyAIT::refer_data(Packet* pkt) 
{
    static uint64_t tx_line_bytes = 1 << tx_line_bits;
    static uint64_t width_block = 1 << byte_offset;
    static uint64_t num_blocks = 1 << (tx_line_bits-byte_offset); 

    /* Linearly map address as data initially */
    uint64_t mem_addr = get_addr(pkt->LADDR);
    if (pkt->cmd==CMD_READ)
    {
        if (get_stored(pkt)==false)
        {
            pkt->buffer_data.setSize(tx_line_bytes);
            for (uint64_t b=0; b<num_blocks; b++)
            {
                uint64_t dataFull = (mem_addr<<byte_offset) | b;
                for (uint64_t i=0; i<width_block; i++)
                {
                    uint8_t dataByte = (dataFull>>8*i) & 0xFF;
                    pkt->buffer_data.setByte(b*width_block+i, dataByte);
                }
            }
            if (data_enable)
                set_stored(pkt->LADDR, true_enable, &(pkt->buffer_data));
        }
    }
    else if (pkt->cmd==CMD_WRITE)
    {
        if (pkt->buffer_data.getSize( )==width_block)
        {
            DataBlock tmp_data = get_stored(pkt->PADDR);
            if (tmp_data.getSize( )==0)
                tmp_data.setSize(tx_line_bytes);
            
            /* Partial write (data mask) */
            uint64_t blk = get_block(pkt->LADDR);
            for (uint64_t i=0; i<width_block; i++)
                tmp_data.setByte(blk*width_block+i, 
                    pkt->buffer_data.getByte(i));
       
            pkt->buffer_data = tmp_data;
        }
        else if (pkt->buffer_data.getSize( )==tx_line_bytes)
        {
            /* Full write */
        }
        else
            assert(0);
        
        if (data_enable)
            set_stored(pkt->LADDR, true_enable, &(pkt->buffer_data));
    }
    else
        assert(0);
}

uint64_t DummyAIT::get_addr(uint64_t addr)
{
    return addr >> tx_line_bits;
}

uint64_t DummyAIT::get_block(uint64_t addr)
{
    static uint64_t num_blocks =  1 << (tx_line_bits-byte_offset); 
    return ((addr >> (tx_line_bits-byte_offset)) % num_blocks);
}
