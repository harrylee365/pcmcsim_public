#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "base/MemInfo.h"
#include "MemoryModules/DummyMemory/DummyJedecMEM.h"

using namespace PCMCsim;

DummyJedecMEM::DummyJedecMEM(MemoryControlSystem* memsys_, std::string cfg_header, MemInfo* info)
:DummyMemory(memsys_)
{
    cp_name = cfg_header;
    ticks_per_cycle = memsys->getParamUINT64("global.ticks_per_cycle", 1);
    data_enable = memsys->getParamBOOL(cp_name+".data_enable", false);
    meta_enable = (data_enable)? memsys->getParamBOOL(cp_name+".meta_enable", false) : false;
    true_enable = memsys->getParamBOOL(cp_name+".true_enable", false);

    if (info==NULL)
    {
        std::cerr << "[DummyJedec] Error! MemInfo is not defined yet!" << std::endl;
        assert(0);
        exit(1);
    }
    PAGE_SIZE = info->PAGE_SIZE;
    META_SIZE = info->META_SIZE;

    ready = true;
}

void DummyJedecMEM::recvRequest(Packet* pkt, ncycle_t delay)
{
    switch (pkt->cmd)
    {
        case CMD_ACT:
        case CMD_PRE:
        case CMD_REFRESH:
        case CMD_APDE:
        case CMD_FPPDE:
        case CMD_SPPDE:
        case CMD_PDX:
        case CMD_SRE:
        case CMD_SRX:
            Component::recvRequest(pkt, 1);
            break;

        case CMD_READ:
        case CMD_READ_PRE:
            if (get_stored(pkt)==false)
            {
                if (pkt->buffer_data.getSize( )!=PAGE_SIZE)
                    pkt->buffer_data.setSize(PAGE_SIZE);
                if (pkt->buffer_meta.getSize( )!=META_SIZE)
                    pkt->buffer_meta.setSize(META_SIZE);
                if (data_enable)
                    set_stored(pkt->PADDR, true_enable, 
                        &(pkt->buffer_data), &(pkt->buffer_meta));
            }
            pkt->mvalid = true;
            pkt->from = this;
            pkt->cmd = CMD_READ;
            parent->recvResponse(pkt, delay);
            break;

        case CMD_WRITE:
        case CMD_WRITE_PRE:
            if (pkt->isDATA)
            {
                /* Partial write if byte enable is active */
                if (pkt->byte_enable.empty( )==false)
                {
                    DataBlock tmp_data = get_stored(pkt->PADDR);
                    if (tmp_data.getSize( )==0)
                        tmp_data.setSize(PAGE_SIZE);
                    else
                        assert(tmp_data.getSize( )==PAGE_SIZE);

                    std::list<uint64_t>::iterator e_it = pkt->byte_enable.begin( );
                    for ( ; e_it!=pkt->byte_enable.end( ); )
                    {
                        uint64_t byte_idx = *e_it;
                        assert(byte_idx<PAGE_SIZE);
                        tmp_data.setByte(byte_idx, 
                            pkt->buffer_data.getByte(byte_idx));
                        e_it = pkt->byte_enable.erase(e_it);
                    }

                    pkt->buffer_data = tmp_data;
                }

                if (data_enable)
                    set_stored(pkt->PADDR, true_enable, 
                        &(pkt->buffer_data), &(pkt->buffer_meta));
            }

            Component::recvRequest(pkt, delay);
            break;

        default:
            assert(0);
            break;
    }
}

void DummyJedecMEM::handle_events(ncycle_t curr_tick)
{
    /* Get events from queue */
    prepare_events(curr_tick);

    if (!await_req.empty( ))
        handle_await_reqs( );
}

void DummyJedecMEM::handle_await_reqs( )
{
    std::list<LocalEvent*>::iterator e_it = await_req.begin( );
    for ( ; e_it!=await_req.end( ); )
    {
        Packet* pkt = (*e_it)->pkt;
        
        /* WRITE | WRITE_PRE | (!=READ && !=READ_PRE) */
        pkt->cmd = CMD_PKT_DEL;
        pkt->owner->recvResponse(pkt); 

        /* Free event */
        delete (*e_it);
        e_it = await_req.erase(e_it);
    }
}

