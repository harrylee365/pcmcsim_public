#include "TraceGen/TraceGen.h"
#include <arpa/inet.h> //for calling htonl( )

using namespace PCMCsim;

TraceGen::TraceGen(std::string trc_path, uint64_t data_byte, uint64_t meta_byte):
DATA_BYTE(data_byte), META_BYTE(meta_byte), trc_type(0)
{
    /* Setup trace file */
    std::string trcPath = trc_path;
    std::string trcLineStr;
    if (!trcFile.is_open())
    {
        trcFile.open(trcPath.c_str());
        if (!trcFile.is_open())
        {
            std::cerr << "[Error] Failed to open the trace file." << std::endl;
            exit(1);
        }
    }
    
    std::getline(trcFile, trcLineStr);
    std::string header = trcLineStr.substr(0, 4);
    if (header=="NVMV")
        trc_type = NVMV;
    else
    {
        header = trcLineStr.substr(0, 14);
        if (header=="VALIDATE_INPUT")
            trc_type = VALIDATE_INPUT;
        else if (header=="VALIDATE_CPATH")
            trc_type = VALIDATE_CPATH;
        else if (header=="VALIDATE_DPATH")
            trc_type = VALIDATE_DPATH;
        else
        {
            std::cerr << "[Error] Abnormal header is read from the trace file." << std::endl;
            assert(0);
            exit(1);
        }
    }
    
    memset(&line_info, 0, sizeof(trc_line_t));
    line_info.LADDR = INVALID_ADDR;
    line_info.PADDR = INVALID_ADDR;
    if (DATA_BYTE>0)
    {
        line_info.data = new uint8_t[DATA_BYTE];
        memset(line_info.data, 0, sizeof(uint8_t)*DATA_BYTE);
    }
    if (META_BYTE>0)
    {
        line_info.meta = new uint8_t[META_BYTE];
        memset(line_info.meta, 0, sizeof(uint8_t)*META_BYTE);
    }

    std::cout << "TraceGen is successfully initialized! "
        "(path=" << trc_path << ")" << std::endl;
}

TraceGen::~TraceGen( )
{
    if (trcFile.is_open())
        trcFile.close();

    if (line_info.data)
        delete line_info.data;
    if (line_info.meta)
        delete line_info.meta;
}

bool TraceGen::getNextTrcLine( )
{
    std::string trcLineStr;
    std::getline(trcFile, trcLineStr);
    if (trcFile.eof())
    {
        std::cout << "Trace file reached EOF." << std::endl;
        return false;
    }

    std::istringstream lineStream(trcLineStr);
    std::string field_str;
    uint8_t field_ptr = 0;
    bool rv = false;
    while (std::getline(lineStream, field_str, ' '))
    {
        rv = decode_trc(field_str, field_ptr);
        if (rv==false)
            break;
    }
    return rv;

    // TODO For memsim compare
//    std::istringstream iss(trcLineStr);
//    std::string cmd_type;
//    std::string dummy_str;
//    iss >> line_info.cycle >> cmd_type >> std::hex >> 
//        line_info.LADDR >> std::dec >> dummy_str >> line_info.id;
//
//    if (cmd_type=="W")
//        line_info.cmd_type = CMD_WRITE;
//    else
//        line_info.cmd_type = CMD_READ;
//
//    return true;
}

void TraceGen::printTrcLine( )
{
    std::cout << line_info.cycle << " " << ((line_info.cmd_type==CMD_READ)? "R":"W") <<
        std::hex << " LA=0x" << line_info.LADDR << " PA=0x" << line_info.PADDR << " ";
    for (uint64_t i = 0; i < DATA_BYTE; i++)
    {
        if (line_info.data[i] <= 0xf)
            std::cout << "0";
        std::cout << std::hex << (uint32_t)line_info.data[i];
    }
    std::cout << " " << line_info.id << std::dec << std::endl;
}

bool TraceGen::decode_trc(std::string& field_str, uint8_t& field_ptr)
{
    assert(field_str!="");

    if (trc_type==NVMV || trc_type==VALIDATE_INPUT)
    {
        if (field_ptr == 0)
            line_info.cycle = std::atoi(field_str.c_str());
        else if (field_ptr == 1)
        {
            if (field_str == "R")
                line_info.cmd_type = CMD_READ;
            else if (field_str == "W")
                line_info.cmd_type = CMD_WRITE;
            else
            {
                std::cerr << "[Error] Unknown command type!" << std::endl;
                return false;
            }
        }
        else if (field_ptr == 2)
        {
            std::stringstream fmat;
            fmat << std::hex << field_str;
            fmat >> line_info.LADDR;
        }
        else if (field_ptr == 3)
        {
            if (trc_type==VALIDATE_INPUT)
                return true;

            int byte, st, ed;
            for (byte = 0; byte < (int)DATA_BYTE/4; byte++)
            {
                uint32_t* data32bit = reinterpret_cast<uint32_t*>(line_info.data);
                std::stringstream fmat;
                st = 8 * byte;
                ed = st + 8;
                fmat << std::hex << field_str.substr(st, ed - st);
                fmat >> data32bit[byte];
                data32bit[byte] = htonl(data32bit[byte]);
            }
        }
        else if (field_ptr == 4)
            line_info.id = std::atoi(field_str.c_str());
        else
        {
            std::cerr << "[Error] Unknown trace line field detected!" << std::endl;
            return false;
        }
    }
    else if (trc_type==VALIDATE_CPATH)
    {
        if (field_ptr==0)
            line_info.cycle = std::atoi(field_str.c_str( ));
        else if (field_ptr==1)
        {
            if (field_str=="R")
                line_info.cmd_type = CMD_READ;
            else if (field_str=="W")
                line_info.cmd_type = CMD_WRITE;
            else
            {
                std::cerr << "[Error] Unknown command type!" << std::endl;
                return false;
            }
        }
        else if (field_ptr==2)
        {
            std::stringstream fmat;
            fmat << std::hex << field_str;
            fmat >> line_info.id;
        }
        else if (field_ptr==3)
        {
            std::stringstream fmat;
            fmat << std::hex << field_str;
            fmat >> line_info.LADDR;
        }
        else if (field_ptr==4)
        {
            std::stringstream fmat;
            fmat << std::hex << field_str;
            fmat >> line_info.PADDR;
        }
        else
        {
            std::cerr << "[Error] Unknown trace line field detected!" << std::endl;
            return false;
        }
    }
    else if (trc_type==VALIDATE_DPATH)
    {
        if (field_ptr==0)
            line_info.cycle = std::atoi(field_str.c_str( ));
        else if (field_ptr==1)
        {
            if (field_str=="R")
                line_info.cmd_type = CMD_READ;
            else if (field_str=="W")
                line_info.cmd_type = CMD_WRITE;
            else
            {
                std::cerr << "[Error] Unknown command type!" << std::endl;
                return false;
            }
        }
        else if (field_ptr==2)
        {
            std::stringstream fmat;
            fmat << std::hex << field_str;
            fmat >> line_info.id;
        }
        else if (field_ptr==3)
        {
            if (field_str!="-")
            {
                std::stringstream fmat;
                fmat << std::hex << field_str;
                fmat >> line_info.LADDR;
            }
        }
        else if (field_ptr==4 || field_ptr==5)
        {
            int byte, st, ed;
            int loop_bound = (field_ptr==4)? DATA_BYTE/4 : META_BYTE/4;
            for (byte=0; byte<loop_bound; byte++)
            {
                uint32_t* data32bit = (field_ptr==4)? 
                    reinterpret_cast<uint32_t*>(line_info.data) :
                    reinterpret_cast<uint32_t*>(line_info.meta);
                std::stringstream fmat;
                st = 8*byte;
                ed = st + 8;
                fmat << std::hex << field_str.substr(st, ed-st);
                fmat >> data32bit[byte];
                data32bit[byte] = htonl(data32bit[byte]);
            }
        }
    }

    field_ptr += 1;

    return true;
}

