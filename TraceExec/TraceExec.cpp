#include "TraceExec/TraceExec.h"
#include "TraceGen/TraceGen.h"
#include "base/MemoryControlSystem.h"
#include "base/Packet.h"
#include "base/EventQueue.h"
#include "base/Component.h"
#include "base/MemInfo.h"

using namespace PCMCsim;

TraceExec::TraceExec(int argc, char* argv[])
: trc_gen(NULL), pendPkt(NULL), trc_end(false), issued_trc(0), 
max_trc(0), input_trace(""), config_path(""), stat_path("")
{
    /* Setup according to arguments */
    input_trace = "";
    config_path = "./configs/example_pcmc.cfg";

    for (int i=1; i<argc; )
    {
        std::string arg_str(argv[i]);
        if (argc==2 && (arg_str=="--help" || arg_str=="-h"))
        {
            std::cerr << "Available options are shown as below:\n" 
                << "\t-i, --input: path of input trace file\n"
                << "\t-c, --config: path of config file\n"
                << "\t-s, --statout: directory path of statistics output\n" 
                << "\t-n, --numline: number of lines to simulate\n"
                << std::endl;
            exit(1);
        }

        if (i+1==argc || argv[i+1][0]=='-')
        {
            std::cerr << "Error getting option value! " 
                << "The value of option is unavailable " 
                << "(or do not name the value begun with '-')" 
                << std::endl;
            assert(0);
            exit(1);
        }

        if (arg_str=="-i" || arg_str=="--input")
            input_trace = argv[i+1];
        else if (arg_str=="-c" || arg_str=="--config")
            config_path = argv[i+1];
        else if (arg_str=="-s" || arg_str=="--statout")
        {
            stat_path = argv[i+1];
            stat_os.open(stat_path.c_str( ), std::ofstream::out | std::ofstream::app);
            if (stat_os.is_open( )==false)
            {
                std::cerr << "Unable to open statistics output file!" << std::endl;
                assert(0);
                exit(1);
            }
        }
        else if (arg_str=="-n" || arg_str=="--numline")
            max_trc = atoi(argv[i+1]);
        else 
        {
            std::cerr << "Invalid option! See info with --help/-h" << std::endl;
            assert(0);
            exit(1);
        }

        i+=2;
    }
    
    if (input_trace=="")
    {
        std::cerr << "Please input trace file path with -i" << std::endl;
        assert(0);
        exit(1);
    }

    /* Setup simulation objects */
    trc_gen = new TraceGen(input_trace);
    geq = new GlobalEventQueue( );
    memsys = new MemoryControlSystem(config_path, geq);
    memsys->sys_name = memsys->getParamSTR("global.system", "PCM");
    if (memsys->sys_name=="PCM" || memsys->sys_name=="PRAM")
        memsys->setup_pcmc(dynamic_cast<Component*>(this));
    else if (memsys->sys_name=="DRAM")
        memsys->setup_dmc(dynamic_cast<Component*>(this));
    else
    {
        std::cerr << "Invalid memory is specified - " << memsys->sys_name << std::endl;
        assert(0);
        exit(1);
    }

    ticks_per_cycle= memsys->getParamUINT64("global.ticks_per_cycle", 1);
}

TraceExec::~TraceExec( )
{
    delete geq;
    delete memsys;
    delete trc_gen;
}

void TraceExec::recvResponse(Packet* pkt, ncycle_t /*delay*/)
{
    std::set<Packet*>::iterator s_it = issued_pkt.find(pkt);
    if (s_it!=issued_pkt.end( ))
    {
        delete (*s_it);
        issued_pkt.erase(s_it);
        
        if (trc_end && issued_pkt.empty( ))
            geq->escape = true; 
    }
    else
        assert(0);
}

void TraceExec::handle_events(ncycle_t curr_tick)
{
    prepare_events(curr_tick);

    if (!await_cb.empty( ))
        handle_await_callbacks( );
}

int TraceExec::exec( )
{
    registerCallback((CallbackPtr)&TraceExec::req_issue, 1);
    geq->handle_events( );

    /* Print out stats */
    std::ostream& ref_stream = (stat_os.is_open( ))? stat_os:std::cout;
    ref_stream << "Input-trace-file=" << input_trace << std::endl;
    ref_stream << "Config-path=" << config_path << std::endl;
    memsys->print_stats(ref_stream);

    return 0;
}

void TraceExec::req_issue( )
{
    if (pendPkt==NULL)
    {
        if ((max_trc>0 && issued_trc>=max_trc) || trc_gen->getNextTrcLine( )==false)
        {
            std::cout << "Reached the pre-defined trace maximum number" << std::endl;
            trc_end = true;
        }
        else
            pendPkt = wrap_pkt( );
    }

    if (trc_end) return;

    if (memsys->isReady(pendPkt))
    {
        issued_pkt.insert(pendPkt);
        memsys->recvRequest(pendPkt, 1);
        pendPkt = NULL;
    }

    registerCallback((CallbackPtr)&TraceExec::req_issue, 1);
}

Packet* TraceExec::wrap_pkt( )
{
    Packet* pkt = new Packet(memsys->info->PAGE_SIZE/memsys->info->HOST_TX_SIZE);
    assert(pkt!=NULL);
    pkt->owner = this;
    pkt->src_id = SRC_HOST;
    pkt->req_id = issued_trc;
    pkt->cmd = trc_gen->line_info.cmd_type;
    pkt->LADDR = trc_gen->line_info.LADDR;
    pkt->tid = trc_gen->line_info.id;

    if (pkt->cmd==CMD_WRITE)
    {
        pkt->buffer_data.setSize(memsys->info->HOST_TX_SIZE);
//        for (uint64_t i=0; i<memsys->info->HOST_TX_SIZE; i++)
//            pkt->buffer_data.setByte(i, trc_gen->line_info.data[i]);
    }

    if (pkt->LADDR>memsys->info->get_capacity_bits( ))
        pkt->LADDR = pkt->LADDR % memsys->info->get_capacity_bits( );

    if (memsys->sys_name=="DRAM")
        pkt->PADDR = pkt->LADDR;

    issued_trc++;
    
    return pkt;
}

