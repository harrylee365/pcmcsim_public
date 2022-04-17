#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "uCMDEngine/FCFS.h"

using namespace PCMCsim;

FCFS::FCFS(MemoryControlSystem* memsys_, AddressDecoder* adec_,
    MemInfo* info_, std::string cfg_header, ncycle_t ticks_per_cycle_, uint64_t id_)
:JedecEngine(memsys_, adec_, info_, cfg_header, ticks_per_cycle_, id_)
{
}

void FCFS::cycle_reqlist( )
{
    Packet* found_pkt = NULL;

    if (find_open_bank(found_pkt)==false)
        find_closed_bank(found_pkt);

    if (found_pkt!=NULL)
        insert_ucmdq(found_pkt);

    JedecEngine::cycle_reqlist( );
}

