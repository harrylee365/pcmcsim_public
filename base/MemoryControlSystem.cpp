#include "base/MemoryControlSystem.h"
#include "base/EventQueue.h"
#include "base/DataBlock.h"
#include "base/Packet.h"
#include "base/MemInfo.h"
#include "base/PCMInfo.h"
#include "Parsers/Parser.h"
#include "RequestReceiver/RequestReceiver.h"
#include "DataCache/DataCache.h"
#include "AITManager/AITManager.h"
#include "MemoryModules/GPCache/GPCache.h"
#include "ReadModifyWrite/ReadModifyWrite.h"
#include "uCMDEngine/uCMDEngine.h"
#include "uCMDEngine/HynixEngine.h"
#include "uCMDEngine/JedecPolicyFactory.h"
#include "uCMDEngine/JedecEngine.h"
#include "DataPathUnit/DataPathUnit.h"
#include "MemoryModules/DummyMemory/DummyAIT.h"
#include "MemoryModules/DummyMemory/DummyHynixPMEM.h"
#include "MemoryModules/DummyMemory/DummyJedecMEM.h"
#include "Subsystems/XBar/XBar.h"
#include "Subsystems/MCU/MicroControlUnitFactory.h"
#include "Subsystems/MCU/MicroControlUnit.h"

using namespace PCMCsim;

MemoryControlSystem::MemoryControlSystem(const std::string& cfgfile, 
                                         GlobalEventQueue* geq, std::string _path_prefix)
:info(NULL), adec(NULL), mdec(NULL), tdec(NULL), host_itf(NULL), parser(NULL),
recvr(NULL), dcache(NULL), aitm(NULL), rmw(NULL), xbar(NULL), 
ait_mem(NULL), ait_adec(NULL), ait_info(NULL), ait_dmc(NULL)
{
    if (!geq)
    {
        std::cerr << "[MemoryControlSystem] Error! Create event queue!" << std::endl;
        assert(0);
        exit(1);
    }
    else
        this->geq = geq;

    bool print_cfg = false;
    std::ifstream fin(cfgfile.c_str( ));
    if (fin.good( )==false)
    {
        std::cerr << "[MemoryControlSystem] Error! Configuration file can't open!" << std::endl;
        assert(0);
        exit(1);
    }
    else
    {
        std::string line, pName, value, eqSign;
        std::istringstream sline;
        while (getline(fin, line))
        {
            if (line.empty( ))
                continue;

            sline.clear( );
            sline.str(line);
            sline >> pName >> eqSign >> value;
            if (pName.find("#")!=std::string::npos || value.find("#")!=std::string::npos)
                continue;

            if (pName=="print_cfg" && value=="true")
                print_cfg = true;
            else
                params[pName] = value;
        }
        fin.close( );

        if (print_cfg)
        {
            fin.open(cfgfile.c_str( ));
            while (getline(fin, line))
                std::cout << line << std::endl;
            fin.close( );
        }
    }

    if (_path_prefix!="")
        path_prefix = _path_prefix + "/";
}

MemoryControlSystem::~MemoryControlSystem( )
{
    for (uint64_t ch=0; ch<dpu.size( ); ch++)
        delete dpu[ch];

    for (uint64_t ch=0; ch<ucmde.size( ); ch++)
        delete ucmde[ch];

    for (uint64_t ch=0; ch<media.size( ); ch++)
        delete media[ch];

    for (uint64_t ch=0; ch<recvr_dmc.size( ); ch++)
        delete recvr_dmc[ch];

    delete info;
    delete adec;
    delete mdec;
    delete tdec;
    delete parser;
    delete recvr;
    delete dcache;
    delete aitm;
    delete rmw;
    delete xbar;
    delete ait_mem;
    delete ait_adec;
    delete ait_info;
    delete ait_dmc;
}

void MemoryControlSystem::recvRequest(Packet* pkt, ncycle_t delay)
{
    assert(parser);
    parser->recvRequest(pkt, delay);
}

void MemoryControlSystem::recvResponse(Packet* pkt, ncycle_t delay)
{
    assert(host_itf);
    host_itf->recvResponse(pkt, delay);
}

bool MemoryControlSystem::isReady(Packet* pkt)
{
    return parser->isReady(pkt);
}

void MemoryControlSystem::setup_pcmc(Component* host_itf)
{
    this->host_itf = host_itf;
    ncycle_t global_freq = getParamUINT64("global.ticks_per_cycle", 1);

    /* Define memory layout & data format */
    adec = new AddressDecoder(this, "pcm");
    info = new MemInfo(this, "pcm", adec);
    tdec = new AITDecoder(this, "pcm.ait", adec);
    mdec = new MetaDecoder(this, "pcm.meta", info);
    info->set_ctrl_freq(global_freq);
    info->print( );
    adec->print( );

    /* Setup fundamental sub-modules */
    geq->set_dbg_msg(getParamBOOL("geq.dbg_msg", false));
    parser = new Parser(this, "parser");
    recvr = new RequestReceiver(this, "reqRecv");
    dcache = new DataCache(this, "dcache");
    aitm = new AITManager(this, "aitm");
    xbar = new XBar(this, "xbar");
    rmw = new ReadModifyWrite(this, "rmw");

    /* uCMD engine instantiation */
    ucmde.resize(info->get_channels( ));
    dpu.resize(info->get_channels( ));
    media.resize(info->get_channels( ));
    if (getParamBOOL("global.ucmde.is_jedec", false))
    {
        std::string jheader = "ucmde.jedec";
        ucmde[0] = JedecPolicyFactory::create_engine_policy(
                this, adec, info, jheader, global_freq);

        dpu[0] = new DataPathUnit(this, "dpu", global_freq);
        media[0] = new DummyJedecMEM(this, "media.jedec[0]", info);
    }
    else
    {
        std::cerr << "[MemoryControlSystem] Hynix engine is not available "
            << "in public version due to confidential issues" << std::endl;
        assert(0);
        exit(1);
    }

    /* AIT Subsystem setup */
    std::string mcu_header = "mcu";
    bool simple_ait_subsys = getParamBOOL("global.simple_ait_subsys", false);
    bool mcu_enable = getParamBOOL(mcu_header, false);
    if (simple_ait_subsys==false)
    {
        std::string ait_header = "ait";
        ait_adec = new AddressDecoder(this, ait_header);

        /* Setup wear leveling */
        if (mcu_enable)
        {
            /* Resize AIT using overprovision ratio */
            op_rate = getParamUINT64(mcu_header+".op_rate", 2);
            if (op_rate==0 || 
                (op_rate&(op_rate-1))!=0)
            {
                std::cerr << "[MemoryControlSystem] Error! op-rate "
                    << "must be power of 2!" << std::endl;
                assert(0);
                exit(1);
            }

            uint64_t tmp_op_rate = op_rate;
            uint64_t op_bits = 0;
            while((tmp_op_rate&0x1)==0)
            {
                tmp_op_rate >>= 1;
                op_bits += 1;
            }

            addr_field_t last_ait_afld = ait_adec->get_field(NUM_FIELDS-1);
            ait_adec->set_width(last_ait_afld, 
                ait_adec->get_width(last_ait_afld)+op_bits);
            ait_info = new MemInfo(this, ait_header, ait_adec);

            /* Setup MCU algorithm */
            mcu = MicroControlUnitFactory::create_mcu(this, mcu_header, global_freq);
        }
        else
            ait_info = new MemInfo(this, ait_header, ait_adec);

        ait_info->set_ctrl_freq(global_freq);
        ait_info->print( );
        ait_adec->print( );

        /* Setup AIT memory subsystem */
        ait_dmc = JedecPolicyFactory::create_engine_policy(
            this, ait_adec, ait_info, ait_header+".ucmde", global_freq);

        ait_mem = new DummyJedecMEM(this, ait_header+".media[0]", ait_info);

        /* Sanity check for WLV */
        if (mcu && ((DummyJedecMEM*)ait_mem)->is_data_enable( )==false)
        {
            std::cerr << "[MemoryControlSystem] Error! Data is not enabled "
                << "on AIT media!" << std::endl;
            assert(0);
            exit(1);
        }
    }
    else
        ait_mem = new DummyAIT(this, "ait.dummyAIT");

    /* Connect modules according to cfg */
    parser->expand_path(recvr);
    recvr->parser = parser;
    recvr->dcache = dcache;
    dcache->recvr = recvr;
    dcache->aitm = aitm;
    dcache->rmw = rmw;
    aitm->rmw = rmw;

    if (simple_ait_subsys==false)
    {
        aitm->tdram = ait_dmc;
        dynamic_cast<JedecEngine*>(ait_dmc)->master = aitm->tcache;

        aitm->tcache->setParent(aitm);
        aitm->tcache->setChild(xbar);
        ait_dmc->setParent(xbar);

        xbar->expand_port(aitm->tcache);
        xbar->expand_port(ait_dmc);

        ait_dmc->dpu = ait_mem;
        ait_dmc->media = ait_mem;
        ait_mem->setParent(ait_dmc);

        if (mcu_enable)
        {
            xbar->expand_port(mcu);
            static_cast<ReadModifyWrite*>(rmw)->mcu = mcu;
            static_cast<AITManager*>(aitm)->mcu = mcu;

            mcu->xbar = xbar;
            mcu->aitm = aitm;
            mcu->tdram = ait_dmc;
        }
    }
    else
    {
        aitm->tcache->setParent(aitm);
        aitm->tcache->setChild(ait_mem);
        ait_mem->setParent(aitm->tcache);
    }

    rmw->dcache = dcache;
    rmw->ucmde = ucmde[0];
    rmw->dpu = dpu[0];
    rmw->aitm = aitm;
    ucmde[0]->setParent(rmw);
    ucmde[0]->dpu = dpu[0];
    dpu[0]->rmw = rmw;
    dpu[0]->ucmde = ucmde[0];

    ucmde[0]->media = media[0];
    dpu[0]->media = media[0];

    media[0]->setParent(dpu[0]);
}

void MemoryControlSystem::setup_dmc(Component* host_itf)
{
    this->host_itf = host_itf;
    ncycle_t global_freq = getParamUINT64("global.ticks_per_cycle", 1);

    /* Define memory information */
    adec = new AddressDecoder(this, "dram");
    info = new MemInfo(this, "dram", adec);
    info->set_ctrl_freq(global_freq);
    info->print( );
    adec->print( );

    /* Create modules */
    geq->set_dbg_msg(getParamBOOL("geq.dbg_msg", false));
    parser = new Parser(this, "parser");

    recvr_dmc.resize(info->get_channels( ));
    ucmde.resize(info->get_channels( ));
    media.resize(info->get_channels( ));
    for (uint64_t ch=0; ch<info->get_channels( ); ch++)
    {
        std::string header = "dram.reqRecv["+std::to_string(ch)+"]";
        recvr_dmc[ch] = new RequestReceiver(this, header);

        header = "dram.ucmde";
        ucmde[ch] = JedecPolicyFactory::create_engine_policy(
                this, adec, info, header, global_freq, ch);

        static_cast<JedecEngine*>(ucmde[ch])->master = parser;

        media[ch] = new DummyJedecMEM(this, 
            header+".media["+std::to_string(ch)+"]", info);
    }

    /* Connect modules */
    for (uint64_t ch=0; ch<info->get_channels( ); ch++)
    {
        parser->expand_path(recvr_dmc[ch]);
        recvr_dmc[ch]->parser = parser;
        recvr_dmc[ch]->dcache = ucmde[ch];
        ucmde[ch]->setParent(recvr_dmc[ch]);
        ucmde[ch]->dpu = media[ch];
        ucmde[ch]->media = media[ch];
        media[ch]->setParent(ucmde[ch]);
    }
}

double MemoryControlSystem::getParamFLOAT(const std::string& key, double def) const
{
    if (params.find(key)==params.end( ))
        return def;
    
    double rval = 0.0;
    std::istringstream sstr(params.find(key)->second);
    sstr >> rval;
    return rval;
}

uint64_t MemoryControlSystem::getParamUINT64(const std::string& key, uint64_t def) const
{
    if (params.find(key)==params.end( ))
        return def;
    else
    {
        uint64_t rval = 0;
        const std::string& value = params.find(key)->second;
        if (value.find("0x", 0)!=std::string::npos || value.find("0X", 0)!=std::string::npos)
        {
            /* Convert HEX to DEC */
            for (uint32_t i = 2; i < value.length( ); i++)
            {
                rval <<= 4;
                if (value.at(i)>='A' && value.at(i)<='F')
                    rval += (value.at(i) - 'A' + 10);
                else if (value.at(i)>='a' && value.at(i)<='f')
                    rval += (value.at(i) - 'a' + 10);
                else
                    rval += (value.at(i) - '0');
            }
        }
        else if (value.find("^", 1) != std::string::npos)
        {
            /* Get power value */
            std::istringstream sbase(value.substr(0, value.find("^", 1)));
            std::istringstream sexp(value.substr(value.find("^", 1)));
            uint64_t base, exp;
            sbase >> base;
            sexp >> exp;

            rval = 1;
            for (uint32_t i = 0; i < exp; i++)
                rval *= base;
        }
        else
        {
            std::istringstream sstr(params.find(key)->second);
            sstr >> rval;
        }
        return rval;
    }
}

bool MemoryControlSystem::getParamBOOL(const std::string& key, bool def) const
{
    std::string got_str = getParamSTR(key);
    if (got_str=="TRUE" || got_str=="True" || got_str=="true")
        return true;
    else if (got_str=="FALSE" || got_str=="False" || got_str=="false")
        return false;
    else
        return def;
}

std::string MemoryControlSystem::getParamSTR(const std::string& key, std::string def) const
{
    if (params.find(key)==params.end( ))
        return def;
    else
        return params.find(key)->second;
}

void MemoryControlSystem::getParamSRAM(const uint64_t num_sets, 
                                 const uint64_t num_ways, 
                                 double* Esr, double* Erd, double* Ewr)
{
    /* Field definition in the file */
    const uint64_t f_cap = 1;
    const uint64_t f_way = 3;
    const uint64_t f_Esr = 7;
    const uint64_t f_Erd = 8;
    const uint64_t f_Ewr = 9;

    /* Open parameter table file */
    std::string fn = "cc-way-";
    std::string fp = getParamSTR("global.cache_params_path");
    
    if (num_sets==1)
        fn = fn + "full.csv";
    else
        fn = fn + std::to_string(num_ways) + ".csv";

    fp = path_prefix + fp + "/" + fn;
    std::ifstream fs(fp.c_str( ));
    if (fs.good( )==false)
    {
        std::cerr << "[MemoryControlSystem] Error! "
            << "Energy table file is not opened!" << std::endl;
        assert(0);
        exit(1);
    }

    /* Get parameters */
    uint64_t min_num_sets = std::numeric_limits<uint64_t>::max( );
    uint64_t max_num_sets = 0;
    double min_Esr = 0.0;
    double min_Erd = 0.0;
    double min_Ewr = 0.0;
    double max_Esr = 0.0;
    double max_Erd = 0.0;
    double max_Ewr = 0.0;

    uint64_t tmp_cap = 0;
    uint64_t tmp_way = 0;
    double tmp_Esr = 0.0;
    double tmp_Erd = 0.0;
    double tmp_Ewr = 0.0;

    std::string line;
    std::istringstream sline;
    std::getline(fs, line); // get headers
    while (std::getline(fs, line))
    {
        sline.clear( );
        sline.str(line);

        std::string token;
        uint64_t f_tmp = 0;
        while(std::getline(sline, token, ','))
        {
            std::istringstream stoken(token);
            if (f_tmp==f_cap)
                stoken >> tmp_cap;
            else if (f_tmp==f_way)
                stoken >> tmp_way;
            else if (f_tmp==f_Esr)
                stoken >> tmp_Esr;
            else if (f_tmp==f_Erd)
                stoken >> tmp_Erd;
            else if (f_tmp==f_Ewr)
                stoken >> tmp_Ewr;

            f_tmp+=1;
        }

        uint64_t tmp_num_sets = tmp_cap/(64*tmp_way);
        if (tmp_num_sets>max_num_sets)
        {
            max_num_sets = tmp_num_sets;
            max_Esr = tmp_Esr;
            max_Erd = tmp_Erd;
            max_Ewr = tmp_Ewr;
        }
        if (tmp_num_sets<min_num_sets)
        {
            min_num_sets = tmp_num_sets;
            min_Esr = tmp_Esr;
            min_Erd = tmp_Erd;
            min_Ewr = tmp_Ewr;
        }
        
        if (tmp_num_sets==num_sets)
            break;
    }

    /* Get min/max parameter set if the size does not exist in the table */
    if (num_sets>max_num_sets)
    {
        tmp_Esr = max_Esr;
        tmp_Erd = max_Erd;
        tmp_Ewr = max_Ewr;
    }
    else if (num_sets<min_num_sets)
    {
        tmp_Esr = min_Esr;
        tmp_Erd = min_Erd;
        tmp_Ewr = min_Ewr;
    }
    
    *Esr = tmp_Esr;
    *Erd = tmp_Erd;
    *Ewr = tmp_Ewr;
}

GlobalEventQueue* MemoryControlSystem::getGlobalEventQueue( )
{
    return geq;
}

bool MemoryControlSystem::getMemData(Packet* pkt)
{
    bool rv = false;
    uint64_t PA = (pkt->PADDR_MAP==INVALID_ADDR)?
                    pkt->PADDR_MAP : pkt->PADDR;
    if (memoryData.count(PA) > 0)
    {
        MemoryPair data_pair = memoryData[PA];
        pkt->buffer_data = *(data_pair.first);
        pkt->buffer_meta = *(data_pair.second);
        rv = true;
    }
    return rv;
}

void MemoryControlSystem::setMemData(uint64_t PA, DataBlock& data, DataBlock& meta)
{
    DataBlock* newData = NULL;
    DataBlock* newMeta = NULL;
    if (memoryData.count(PA)==0)
    {
        newData = new DataBlock( );
        newMeta = new DataBlock( );
        memoryData[PA] = MemoryPair(newData, newMeta);
        *newData = data;
        *newMeta = meta;
    }
    else
    {
        MemoryPair newPair = memoryData[PA];
        *(newPair.first) = data;
        *(newPair.second) = meta;
    }
}

void MemoryControlSystem::print_stats(std::ostream& os)
{
    os << "==========PCMCsim stats==========" << std::endl;
    os << "LastTick " << geq->getCurrentTick( ) <<std::endl;

    if (recvr)
    {
        recvr->calculate_stats( );
        recvr->print_stats(os);
    }
    if (dcache)
    {
        dcache->calculate_stats( );
        dcache->print_stats(os);
    }
    if (aitm)
    {
        aitm->calculate_stats( );
        aitm->print_stats(os);
    }
    if (ait_dmc)
    {
        ait_dmc->calculate_stats( );
        ait_dmc->print_stats(os);
    }
    if (rmw)
    {
        rmw->calculate_stats( );
        rmw->print_stats(os);
    }
    for (uint64_t ch=0; ch<info->get_channels( ); ch++)
    {
        if (dpu.size( )>0 && dpu[ch])
        {
            dpu[ch]->calculate_stats( );
            dpu[ch]->print_stats(os);
        }

        if (ucmde[ch])
        {
            ucmde[ch]->calculate_stats( );
            ucmde[ch]->print_stats(os);
        }
    }
}

