#include "base/Packet.h"
#include "base/EventQueue.h"
#include "MemoryModules/DummyMemory/DummyMemory.h"

using namespace PCMCsim;

DummyMemory::~DummyMemory( )
{
    std::map<uint64_t, MemoryPair>::iterator d_it = store_data.begin( );
    for ( ; d_it!=store_data.end( ); d_it++)
    {
        delete d_it->second.first;
        delete d_it->second.second;
    }
    
    d_it = true_data.begin( );
    for ( ; d_it!=true_data.end( ); d_it++)
    {
        delete d_it->second.first;
        delete d_it->second.second;
    }
}

bool DummyMemory::get_stored(Packet* pkt)
{
    bool rv = true;
    uint64_t PA = pkt->PADDR; //(pkt->PADDR_MAP==INVALID_ADDR)? pkt->PADDR_MAP : pkt->PADDR;

    if (store_data.count(PA)>0)
    {
        MemoryPair dpair = store_data[PA];
        assert(dpair.first!=NULL);

        pkt->buffer_data = *(dpair.first);
        if (dpair.second!=NULL)
            pkt->buffer_meta = *(dpair.second);
        else if (meta_enable)
        {
            assert(0);
            exit(1);
        }
    }
    else
        rv = false;
    return rv;
}

DataBlock DummyMemory::get_stored(uint64_t PA)
{
    DataBlock ret;
    if (store_data.count(PA)>0)
    {
        MemoryPair dpair = store_data[PA];
        assert(dpair.first!=NULL);

        ret = *(dpair.first);
    }

    return ret;
}

void DummyMemory::set_stored(uint64_t PA, bool set_true, DataBlock* data, DataBlock* meta)
{
    if (data_enable==false)
        return;

    DataBlock* up_data = NULL;
    DataBlock* up_meta = NULL;

    if (store_data.count(PA)==0)
    {
        up_data = new DataBlock( );
        *up_data = *data;
        if (meta_enable)
        {
            assert(meta!=NULL);
            up_meta = new DataBlock( );
            *up_meta = *meta;
        }

        store_data[PA] = MemoryPair(up_data, up_meta);
    }
    else
    {
        *(store_data[PA].first) = *data;
        if (store_data[PA].second!=NULL)
        {
            assert(meta!=NULL);
            *(store_data[PA].second) = *meta;
        }
    }

    if (true_enable==false || set_true==false)
        return;

    up_data = NULL;
    up_meta = NULL;
    if (true_data.count(PA)==0)
    {
        up_data = new DataBlock( );
        *up_data = *data;
        if (meta_enable)
        {
            assert(meta!=NULL);
            up_meta = new DataBlock( );
            *up_meta = *meta;
        }

        true_data[PA] = MemoryPair(up_data, up_meta);
    }
    else
    {
        *(true_data[PA].first) = *data;
        if (true_data[PA].second!=NULL)
        {
            assert(meta!=NULL);
            *(true_data[PA].second) = *meta;
        }
    }
}

bool DummyMemory::get_true(Packet* pkt)
{
    bool rv = true;
    uint64_t PA = pkt->PADDR; //(pkt->PADDR_MAP==INVALID_ADDR)? pkt->PADDR_MAP : pkt->PADDR;
    if (true_data.count(PA)>0)
    {
        MemoryPair dpair = true_data[PA];
        assert(dpair.first!=NULL);

        pkt->buffer_data = *(dpair.first);
        if (dpair.second!=NULL)
            pkt->buffer_meta = *(dpair.second);
        else if (meta_enable)
        {
            assert(0);
            exit(1);
        }
    }
    else
    {
        if (get_stored(pkt)==false)
            rv = false;
    }

    return rv;
}

