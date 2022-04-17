#include "ReplacePolicy/ReplacePolicy.h"

using namespace PCMCsim;

ReplacePolicy::ReplacePolicy(uint64_t s, uint64_t w)
: num_sets(s), num_ways(w)
{
    bool noerr = false;
    if (w>=1)
    {
        uint64_t n=w;
        while (n%2==0)
            n = (n>>1);

        if (n==1)
            noerr = true;
    }

    rsvd.resize(num_sets);
    for (uint64_t i=0; i<num_sets; i++)
        rsvd[i].resize(num_ways);

    if (noerr==false) 
    {
        std::cerr << "[ReplacePolicy] The number of ways is not power of 2!" << std::endl;
        assert(0);
        exit(1);
    }
}

ReplacePolicy::~ReplacePolicy( )
{
}

void ReplacePolicy::set_rsvd(uint64_t set, int64_t way)
{
    assert(set<num_sets && (uint64_t)way<num_ways);
    rsvd[set][way] = true;
}

void ReplacePolicy::unset_rsvd(uint64_t set, int64_t way)
{
    assert(set<num_sets && (uint64_t)way<num_ways);
    rsvd[set][way] = false;
}

bool ReplacePolicy::get_rsvd(uint64_t set, int64_t way)
{
    return rsvd[set][way];
}
