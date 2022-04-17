#include "base/Packet.h"

using namespace PCMCsim;

Packet::Packet( )
{
    cmd = CMD_UNDEF;
    req_id = -1;
    src_id = -1;
    dst_id = -1;
    tid = -1;
    
    LADDR = INVALID_ADDR; 
    PADDR = INVALID_ADDR; 
    LADDR_MAP = INVALID_ADDR;
    PADDR_MAP = INVALID_ADDR; 

    buffer_idx = -1;

    mvalid = false;

    poison = false;
    urgent = false;
    swap = false;

    owner = NULL;
    from =NULL;
    dest =NULL;
    
    recvTick = 0;
    recvTick_recvr = 0;
    recvTick_dcache = 0;
    recvTick_ucmde = 0; 

    is_prefetch = false;
    pb_hit = false;

    dbe_idx = -1;    
    isDATA = false;
    need_redirect = false;
    merge_end_tick = 0;
    wdcache_latency = 0;
}

Packet::Packet(uint64_t num_data)
{
    cmd = CMD_UNDEF;
    req_id = -1;
    src_id = -1;
    dst_id = -1;
    tid = -1;
    
    LADDR = INVALID_ADDR; 
    PADDR = INVALID_ADDR; 
    LADDR_MAP = INVALID_ADDR;
    PADDR_MAP = INVALID_ADDR; 

    buffer_idx = -1;

    dvalid.resize(num_data, false); 
    mvalid = false;

    poison = false;
    urgent = false;
    swap = false;

    owner = NULL;
    from =NULL;
    dest =NULL;
    
    recvTick = 0;
    recvTick_recvr = 0;
    recvTick_dcache = 0;
    recvTick_ucmde = 0; 

    is_prefetch = false;
    pb_hit = false;

    dbe_idx = -1;    
    isDATA = false;
    need_redirect = false;
    merge_end_tick = 0;
    wdcache_latency = 0;
}

Packet::~Packet( )
{
}

void Packet::reset_dvalid(uint64_t num_data)
{
    dvalid.clear( );
    dvalid.resize(num_data, false);
    byte_enable.clear( );
}

Packet& Packet::operator=(Packet& rhs)
{
    cmd = rhs.cmd;
    req_id = rhs.req_id;
    src_id = rhs.src_id;
    dst_id = rhs.dst_id;
    tid = rhs.tid;

    LADDR = rhs.LADDR;
    PADDR = rhs.PADDR;
    LADDR_MAP = rhs.LADDR_MAP;
    PADDR_MAP = rhs.PADDR_MAP;

    buffer_idx = rhs.buffer_idx;
    buffer_data = rhs.buffer_data;
    buffer_meta = rhs.buffer_meta;

    dvalid = rhs.dvalid;
    byte_enable = rhs.byte_enable;
    mvalid = rhs.mvalid;
   
    poison = rhs.poison;
    urgent = rhs.urgent;
    swap = rhs.swap;

    owner = rhs.owner;
    from = rhs.from;
    dest = rhs.dest;
    
    recvTick = rhs.recvTick;
    recvTick_recvr = rhs.recvTick;
    recvTick_dcache = rhs.recvTick;
    recvTick_ucmde = rhs.recvTick; 

    is_prefetch = rhs.is_prefetch;
    pb_hit = rhs.pb_hit;

    dbe_idx = rhs.dbe_idx;
    isDATA = rhs.isDATA;
    need_redirect = rhs.need_redirect;
    merge_end_tick = rhs.merge_end_tick;
    wdcache_latency = rhs.wdcache_latency;

    return (*this);
}

