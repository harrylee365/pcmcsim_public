#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "base/MemInfo.h"
#include "base/Stats.h"
#include "DataPathUnit/DataPathUnit.h"
#include "uCMDEngine/HynixEngine.h"
#include "TraceGen/TraceGen.h"

using namespace PCMCsim;

HynixEngine::HynixEngine(MemoryControlSystem* memsys_, std::string cfg_header)
:uCMDEngine(memsys_)
{
    cp_name = cfg_header;
    dbg_msg = memsys->getParamBOOL(cp_name+".dbg_msg", true);
    ticks_per_cycle= memsys->getParamUINT64("global.ticks_per_cycle", 1);
}

HynixEngine::~HynixEngine( )
{
}

void HynixEngine::handle_events(ncycle_t /*curr_tick*/) { }
