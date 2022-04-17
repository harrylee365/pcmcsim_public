#include "base/MemoryControlSystem.h"
#include "Subsystems/MCU/MicroControlUnitFactory.h"
#include "Subsystems/MCU/BucketWLV.h"

using namespace PCMCsim;

MicroControlUnit* MicroControlUnitFactory::create_mcu(
    MemoryControlSystem* memsys_, std::string cfg_header, ncycle_t ticks_per_cycle_)
{
    MicroControlUnit* rv = NULL;
    std::string mcu_type = memsys_->getParamSTR(cfg_header+".type");
    if (mcu_type=="BucketWearLeveling")
        rv = new BucketWLV(memsys_, cfg_header+".wlv", ticks_per_cycle_); 
    else
        assert(0);

    return rv;
}
