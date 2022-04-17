#include "ReplacePolicy/TrueLRU/TrueLRU.h"

using namespace PCMCsim;

TrueLRU::TrueLRU(uint64_t s, uint64_t w)
: ReplacePolicy(s, w)
{
    init_policy( );
}

TrueLRU::~TrueLRU( )
{
}

void TrueLRU::init_policy( )
{
    tLRU.resize(num_sets);
}

void TrueLRU::update_victim(uint64_t s, int64_t w)
{
    std::list<uint64_t>::iterator it = tLRU[s].begin( );
    for ( ; it!=tLRU[s].end( ); it++)
    {
        if ((*it)==(uint64_t)w)
            break;
    }
    assert(it!=tLRU[s].end( ));
    tLRU[s].splice(tLRU[s].end( ), tLRU[s], it);
}

int64_t TrueLRU::get_victim(uint64_t s)
{
    int64_t rv = -1;
    if (tLRU[s].size( )<num_ways)
    {
        rv = tLRU[s].size( );
        tLRU[s].push_front(rv);
    }
    else
    {
        std::list<uint64_t>::iterator it = tLRU[s].begin( );
        for ( ; it!=tLRU[s].end( ); it++)
        {
            if (rsvd[s][(*it)]==false)
            {
                rv = *it;
                tLRU[s].splice(tLRU[s].end( ), tLRU[s], it);
                break;
            }
        }
    }
    
    return rv;
}

