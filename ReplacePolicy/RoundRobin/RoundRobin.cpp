#include "ReplacePolicy/RoundRobin/RoundRobin.h"

using namespace PCMCsim;

RoundRobin::RoundRobin(uint64_t s, uint64_t w)
: ReplacePolicy(s, w)
{
    init_policy( );
}

RoundRobin::~RoundRobin( )
{
}

void RoundRobin::init_policy( )
{
    filled_ways.resize(num_sets, 0);
    curr_way = 0;
}

void RoundRobin::update_victim(uint64_t s, int64_t w)
{
    s=s;
    w=w;
}

int64_t RoundRobin::get_victim(uint64_t s)
{
    if (filled_ways[s]<num_ways)
    {
        curr_way = filled_ways[s];
        filled_ways[s]+=1;
    }
    else 
    {
        uint64_t tmp_way = 0;
        for (uint64_t i=1; i<=num_ways; i++)
        {
            tmp_way = (curr_way+i)%num_ways;
            if (rsvd[s][tmp_way]==false)
            {
                curr_way = (int64_t)tmp_way;
                break;
            }
        }

        if (rsvd[s][tmp_way])
            curr_way = -1;
    }


    return curr_way;
}
