#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "Subsystems/MCU/MicroControlUnit.h"

using namespace PCMCsim;

MicroControlUnit::MicroControlUnit(MemoryControlSystem* memsys_)
:Component(memsys_), xbar(NULL), tdram(NULL), aitm(NULL)
{
}

MicroControlUnit::~MicroControlUnit( )
{
}

void MicroControlUnit::lookup_isr(Packet* pkt)
{
    ISRWrapper wrapper = isr_map[pkt];
    wrapper.isr(wrapper.arg);
}
