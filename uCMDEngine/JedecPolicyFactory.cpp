#include "base/MemoryControlSystem.h"
#include "base/MemInfo.h"
#include "uCMDEngine/JedecPolicyFactory.h"
#include "uCMDEngine/FCFS.h"
#include "uCMDEngine/FRFCFS.h"

using namespace PCMCsim;

uCMDEngine* JedecPolicyFactory::create_engine_policy(std::string policy, 
            MemoryControlSystem* memsys_, AddressDecoder* adec_, MemInfo* info_,
            std::string cfg_header, ncycle_t ticks_per_cycle_, uint64_t id_)
{
    uCMDEngine* rv = NULL;
    if (policy=="FCFS")
        rv = new FCFS(memsys_, adec_, info_, cfg_header, ticks_per_cycle_, id_);
    else if (policy=="FRFCFS")
        rv = new FRFCFS(memsys_, adec_, info_, cfg_header, ticks_per_cycle_, id_);
    else
    {
        std::cerr << "[JedecPolicyFactory] Error! Invalid policy is specified!" 
            << std::endl;
        assert(0);
        exit(1);
    }
    return rv;
}

uCMDEngine* JedecPolicyFactory::create_engine_policy(
            MemoryControlSystem* memsys_, AddressDecoder* adec_, MemInfo* info_,
            std::string cfg_header, ncycle_t ticks_per_cycle_, uint64_t id_)
{
    uCMDEngine* rv = NULL;
    std::string header = cfg_header+"["+std::to_string(id_)+"]";
    std::string policy = memsys_->getParamSTR(header+".policy");

    if (policy=="FCFS")
        rv = new FCFS(memsys_, adec_, info_, cfg_header, ticks_per_cycle_, id_);
    else if (policy=="FRFCFS")
        rv = new FRFCFS(memsys_, adec_, info_, cfg_header, ticks_per_cycle_, id_);
    else
    {
        std::cerr << "[JedecPolicyFactory] Error! Invalid policy is specified!" 
            << std::endl;
        assert(0);
        exit(1);
    }
    return rv;
}
