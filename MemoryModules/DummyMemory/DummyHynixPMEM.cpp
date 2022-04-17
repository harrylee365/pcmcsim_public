#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "base/MemInfo.h"
#include "MemoryModules/DummyMemory/DummyHynixPMEM.h"

using namespace PCMCsim;

DummyHynixPMEM::DummyHynixPMEM(MemoryControlSystem* memsys_, std::string cfg_header, MemInfo* info)
:DummyMemory(memsys_)
{
    cp_name = cfg_header;
    ticks_per_cycle = memsys->getParamUINT64("global.ticks_per_cycle", 1);
    PAGE_SIZE = info->PAGE_SIZE;
    META_SIZE = info->META_SIZE;
}

void DummyHynixPMEM::handle_events(ncycle_t /*curr_tick*/) { }
