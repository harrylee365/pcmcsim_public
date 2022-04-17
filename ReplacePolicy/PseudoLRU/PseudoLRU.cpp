#include "ReplacePolicy/PseudoLRU/PseudoLRU.h"

using namespace PCMCsim;

PseudoLRU::PseudoLRU(uint64_t s, uint64_t w)
: ReplacePolicy(s, w), max_level(0)
{
    init_policy( );
}

PseudoLRU::~PseudoLRU( )
{
}

void PseudoLRU::init_policy( )
{
    /* Valid index range/set: [1, 2*num_ways-1] */
    pLRU.resize(num_sets);
    filled_ways.resize(num_sets, 0);
    for (uint64_t s=0; s<num_sets; s++)
    {
        pLRU[s].resize(2*num_ways, 0);
        recurs_init(s, 1);
    }
}

void PseudoLRU::update_victim(uint64_t s, int64_t w)
{
    int64_t prob_way = recurs_update(s, w, 1, 0);
    assert(prob_way==w);
    prob_way=prob_way;
}

int64_t PseudoLRU::get_victim(uint64_t s)
{
    int64_t victim = -1;
    if (filled_ways[s]<num_ways)
    {
        victim = (int64_t)filled_ways[s];
        filled_ways[s] += 1;
        update_victim(s, victim); 
    }
    else
    {
        for (uint64_t i=0; i<num_ways; i++)
        {
            victim = recurs_trav(s, 1);
            assert(victim>=0);
            update_victim(s, victim);
            if (rsvd[s][victim]==false)
                break;
        }

        if (rsvd[s][victim])
            victim = -1;
    }

    return victim;
}

/* Executed only for initialization! */
int64_t PseudoLRU::recurs_init(uint64_t s, int64_t idx)
{
    /* Base condition */
    if ((uint64_t)idx>=2*num_ways)
        return -1;

    /* Normal cases */
    int64_t lc = recurs_init(s, 2*idx);
    int64_t rc = recurs_init(s, 2*idx+1);
    uint64_t lv = (uint64_t)log2(idx);
    uint64_t ref1 = 1;
    if (lc<0) // leaf-case
    {
        pLRU[s][idx] = idx&(~(ref1<<lv));
        max_level = lv;
    }
    else if (rc<0)
    {
        /* Assumed that a node is balanced */
        assert(0);
    }

    return pLRU[s][idx];
}

/*
 * dstw: destination way
 * lmw: way of left-most child in the sub-tree
 */
int64_t PseudoLRU::recurs_update(uint64_t s, int64_t dstw, int64_t idx, int64_t lmw)
{
    /* Base condition */ 
    if ((uint64_t)idx>=2*num_ways)
        return -1;

    /* Normal cases */
    uint64_t lv = (uint64_t)log2(idx); // calculate level in tree
    uint64_t ref1 = 1;
    uint64_t criterion = ((num_ways/(ref1<<lv))>>1);
    int64_t nval = 0;   // node value
    int64_t cval = -1;  // child value
    int64_t rv = -1;    // return value
    if (dstw<lmw+(int64_t)criterion)
    {
        cval = recurs_update(s, dstw, 2*idx, lmw);
        nval = 1;
    }
    else 
    {
        /* Update left-most way in right sub-tree and traverse down */
        lmw+=(int64_t)criterion;
        cval = recurs_update(s, dstw, 2*idx+1, lmw);
    }
    
    if (cval<0)
        rv = pLRU[s][idx]; // return way# if leaf is touched
    else
    {
        pLRU[s][idx] = nval;
        rv = cval;
    }

    return rv;
}

int64_t PseudoLRU::recurs_trav(uint64_t s, int64_t idx)
{
    /* Traverse heap to find out victim according to 0/1 */
    uint64_t lv = (uint64_t)log2(idx); // calculate level in tree
    int64_t rv = -1;
    if (lv==max_level)
        rv = pLRU[s][idx];
    else if (lv<max_level)
    {
        /* Traverse down to find victim */
        if (pLRU[s][idx]==0) // recently touched right, left will be pseudo LRU
            rv = recurs_trav(s, 2*idx);
        else if (pLRU[s][idx]==1)
            rv = recurs_trav(s, 2*idx+1);
        else
            assert(0);
    }
    else
        assert(0);

    return rv;
}

