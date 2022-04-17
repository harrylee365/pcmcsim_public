#include "base/MemInfo.h"
#include "base/MemoryControlSystem.h"

using namespace PCMCsim;

AddressDecoder::AddressDecoder(MemoryControlSystem* memsys, std::string cfg_header)
{
    mem_name = cfg_header;
    PAGE_OFFSET = memsys->getParamUINT64(cfg_header+".page_offset", 7); 
    HOST_TX_OFFSET = memsys->getParamUINT64(cfg_header+".host_offset", PAGE_OFFSET);
    BLOCK_OFFSET = memsys->getParamUINT64(cfg_header+"block_offset", 12);

    orders[FLD_CH] = memsys->getParamUINT64(cfg_header+".channel_order", 7);
    orders[FLD_RANK] = memsys->getParamUINT64(cfg_header+".rank_order", 3);
    orders[FLD_BANKGRP] = memsys->getParamUINT64(cfg_header+".bankgroup_order", 1);
    orders[FLD_BANK] = memsys->getParamUINT64(cfg_header+".bank_order", 2);
    orders[FLD_HALF] = memsys->getParamUINT64(cfg_header+".half_order", 6);
    orders[FLD_PART] = memsys->getParamUINT64(cfg_header+".partition_order", 0);
    orders[FLD_ROW] = memsys->getParamUINT64(cfg_header+".row_order", 4);
    orders[FLD_COL] = memsys->getParamUINT64(cfg_header+".col_order", 5);

    /* Sanity check for orders */
    for (int i=0; i<NUM_FIELDS; i++)
    {
        if (orders[i]>=NUM_FIELDS)
        {
            std::cerr << "[Error] Invalid order of bit field (range: 0-"
                << NUM_FIELDS << ")" << std::endl; 
            assert(0);
            exit(1);
        }

        for (int j=0; j<NUM_FIELDS; j++)
        {
            if (i==j) continue;
            if (orders[i]==orders[j])
            {
                std::cerr << "[Error] Repeated order of bit field occurs!" << std::endl;
                assert(0);
                exit(1);
            }
        }
    }

    widths[FLD_RANK] = memsys->getParamUINT64(cfg_header+".rank_bits", 2);
    widths[FLD_BANKGRP] = memsys->getParamUINT64(cfg_header+".bankgroup_bits", 0);
    widths[FLD_BANK] = memsys->getParamUINT64(cfg_header+".bank_bits", 4);
    widths[FLD_HALF] = memsys->getParamUINT64(cfg_header+".half_bits", 1);
    widths[FLD_PART] = memsys->getParamUINT64(cfg_header+".partition_bits", 2);
    widths[FLD_ROW] = memsys->getParamUINT64(cfg_header+".row_bits", 13);
    widths[FLD_COL] = memsys->getParamUINT64(cfg_header+".col_bits", 10);
    widths[FLD_CH] =
        memsys->getParamUINT64(cfg_header+".subchannel_bits", 0) +
        memsys->getParamUINT64(cfg_header+".channel_bits", 0);

    /* Sanity check for widths */
    if (widths[FLD_HALF]>1)
    {
        std::cerr << "[Error] half-bit for selecting left bank and " 
            "right bank can be 0-bit or 1-bit!" << std::endl;
        assert(0);
        exit(1);
    }
}

addr_field_t AddressDecoder::get_field(uint64_t order)
{
    addr_field_t rv = NUM_FIELDS;
    for (uint64_t i=0; i<NUM_FIELDS; i++)
    {
        if (orders[i]==order)
        {
            rv = (addr_field_t)i;
            break;
        }
    }
    return rv;
}

void AddressDecoder::print( )
{
    std::cout << "===============Address format: " << mem_name << " ==============\n| ";
    for (int64_t i=NUM_FIELDS-1; i>=0; i--)
    {
        for (uint64_t j=0; j<NUM_FIELDS; j++)
        {
            if (orders[j]!=(uint64_t)i) continue;

            switch(j)
            {
                case FLD_CH:
                    std::cout << "CH";
                    break;
                case FLD_RANK:
                    std::cout << "RANK";
                    break;
                case FLD_BANKGRP:
                    std::cout << "BANK-GROUP";
                    break;
                case FLD_BANK:
                    std::cout << "BANK";
                    break;
                case FLD_HALF:
                    std::cout << "HALF-BANK";
                    break;
                case FLD_PART:
                    std::cout << "PARTITION";
                    break;
                case FLD_ROW:
                    std::cout << "ROW";
                    break;
                case FLD_COL:
                    std::cout << "COL";
                    break;
                default:
                    assert(0);
                    break;
            }
            std::cout << "=" << widths[j] << "-bit | ";
        }
    }
    std::cout << "\n===================================================" << std::endl;
}

uint64_t AddressDecoder::decode_addr(uint64_t addr, addr_field_t field)
{
    uint64_t rv = addr>>PAGE_OFFSET; // truncate offset bits
    uint64_t ref_bit1 = 1;
    uint64_t shift_mod = 0;
    uint64_t shift_div = 0;
    for (uint64_t i=0; i<NUM_FIELDS; i++)
    {
        if (orders[i]<orders[field])
        {
            shift_div+=widths[i];
            shift_mod+=widths[i];
        }
        else if (orders[i]==orders[field])
            shift_mod+=widths[i];
    }
    rv = rv % (ref_bit1<<shift_mod);
    rv = rv / (ref_bit1<<shift_div);
    return rv;
}

uint64_t AddressDecoder::encode_addr(std::vector<uint64_t>& partial_bits)
{
    if (partial_bits.size( )!=NUM_FIELDS)
    {
        std::cerr << "[Error] Partial bits are not enough!" << std::endl;
        assert(0);
        exit(1);
    }

    uint64_t shift_amt = 0;
    uint64_t prev_field_width = 0;
    uint64_t rv = 0;
    for (uint64_t i=0; i<NUM_FIELDS; i++)
    {
        for (uint64_t j=0; j<NUM_FIELDS; j++)
        {
            if (i!=orders[j]) continue;

            if (i>0)
                shift_amt+=prev_field_width;
            prev_field_width = widths[j];

            rv = rv | (partial_bits[j]<<shift_amt);
        }
    }

    rv = rv<<PAGE_OFFSET;
    return rv;
}

uint64_t AddressDecoder::mask_host_offset(uint64_t addr)
{
    uint64_t mask = 0;
    uint64_t ref_bit1 = 1;
    for (uint64_t i=0; i<HOST_TX_OFFSET; i++)
        mask = mask | (ref_bit1<<i);
    mask = ~mask;

    return (addr&mask); 
}

uint64_t AddressDecoder::mask_page_offset(uint64_t addr)
{
    uint64_t mask = 0;
    uint64_t ref_bit1 = 1;
    for (uint64_t i=0; i<PAGE_OFFSET; i++)
        mask = mask | (ref_bit1<<i);
    mask = ~mask;

    return (addr&mask); 
}

uint64_t AddressDecoder::mask_block_offset(uint64_t addr)
{
    uint64_t mask = 0;
    uint64_t ref_bit1 = 1;
    for (uint64_t i=0; i<PAGE_OFFSET; i++)
        mask = mask | (ref_bit1<<i);
    mask = ~mask;

    return (addr&mask); 
}

MemInfo::MemInfo(MemoryControlSystem* memsys, std::string cfg_header, AddressDecoder* adec)
{
    mem_name = cfg_header;

    if (adec==NULL)
    {
        std::cerr << "Error! AddressDecoder is not defined yet!" << std::endl;
        assert(0);
        exit(1);
    }

    HOST_TX_SIZE = 1<<(adec->HOST_TX_OFFSET);
    PAGE_SIZE = 1<<(adec->PAGE_OFFSET);
    BLOCK_SIZE = 1<<(adec->BLOCK_OFFSET);
    META_SIZE = memsys->getParamUINT64(cfg_header+".meta_bytes", 0);
    if (PAGE_SIZE>BLOCK_SIZE ||
        HOST_TX_SIZE>PAGE_SIZE)
    {
        std::cerr << "Error! Requirement of unit: HOST<PAGE<BLOCK!" << std::endl;
        assert(0);
        exit(1);
    }

    ctrl_freq = (uint64_t)1e12/memsys->getParamUINT64(cfg_header+".ticks_per_cycle", 1);
    if (ctrl_freq==0)
    {
        std::cerr << "Error! Controller frequency is 0!" << std::endl;
        assert(0);
        exit(1);
    }

    prefetch_len = 2*memsys->getParamUINT64(cfg_header+".tBURST", 4);
    num_DQs = memsys->getParamUINT64(cfg_header+".DQs", 16);
    data_window = prefetch_len*num_DQs;
    num_devs = PAGE_SIZE*8/data_window;
    if ((PAGE_SIZE*8)%data_window!=0)
    {
        std::cerr << "Error! Obtained device# is not integer! " 
            << "Please let page size be divisible by 2*tBURST*DQs!"
            << std::endl;
        assert(0);
        exit(1);
    }

    uint64_t ch_bits, rank_bits, bg_bits, 
             bank_bits, part_bits, col_bits, row_bits, half_bits;

    ch_bits = adec->get_width(FLD_CH);
    rank_bits = adec->get_width(FLD_RANK);
    bg_bits = adec->get_width(FLD_BANKGRP);
    bank_bits = adec->get_width(FLD_BANK);
    half_bits = adec->get_width(FLD_HALF);
    part_bits = adec->get_width(FLD_PART);
    row_bits = adec->get_width(FLD_ROW);
    col_bits = adec->get_width(FLD_COL);

    num_ch = 1<<ch_bits;
    num_ranks = 1<<rank_bits;
    num_bgs = 1<<bg_bits;
    num_banks = 1<<bank_bits;
    lr_bank = (half_bits==1)? true : false;
    num_parts = 1<<part_bits;
    num_cols = 1<<col_bits;
    WLs_per_MAT = 1<<row_bits;
    num_MATs = memsys->getParamUINT64(cfg_header+".MATs", 8);
    BLs_per_row = num_cols*data_window;
    BLs_per_MAT = BLs_per_row / num_MATs;
}

void MemInfo::print( )
{
    /* Get capacity in human readable */  
    uint64_t cap = WLs_per_MAT*BLs_per_row*
        num_parts*((lr_bank)? 2:1)*
        num_banks*num_bgs*num_ranks*
        num_ch*num_devs/8;

    if (cap==0)
    {
        std::cerr << "[Error] Capcaity is too small -> 0 BYTE" << std::endl;
        assert(0);
        exit(1);
    }
    else
        cap_bits = cap*8;

    std::string unit_cap = "B";
    uint64_t mult = 1;
    uint64_t order = 0;
    while (cap/(mult*1024)>0)
    {
        mult*=1024;
        order+=1;
    }
    cap = (order>4)? cap : cap/mult;

    if (order==1)
        unit_cap = "KB";
    else if (order==2)
        unit_cap = "MB";
    else if (order==3)
        unit_cap = "GB";
    else if (order==4)
        unit_cap = "TB";

    /* Get density in human readable */
    uint64_t density = cap_bits / (num_ch*num_devs*num_ranks);
    mult = 1; order = 0;
    std::string unit_density = "b";
    while (density/(mult*1024)>0)
    {
        mult*=1024;
        order+=1;
    }
    density = (order>3)? density : density/mult;

    if (order==1)
        unit_density = "Kb";
    else if (order==2)
        unit_density = "Mb";
    else if (order==3)
        unit_density = "Gb";

    /* Get bandwidth in human readable */
    max_bw = 2*ctrl_freq*num_DQs*num_devs*num_ch/8;
    std::string unit_bw = "B/s";
    if (max_bw>=((uint64_t)1<<30))
    {
        max_bw /= ((uint64_t)1<<30);
        unit_bw = "GB/s";
    }
    else if (max_bw>=((uint64_t)1<<20))
    {
        max_bw /= ((uint64_t)1<<20);
        unit_bw = "MB/s";
    }

    std::cout << "\n========== Constructed memory: " << mem_name << " ===========\n"
        << "Controller frequency: " << ctrl_freq << "Hz\n"
        << "Maximum bandwidth of the module: " << max_bw << unit_bw << "\n"
        << "Channels: " << num_ch << "\n"
        << "Channel width: " << num_DQs*num_devs << "b\n"
        << "Ranks/channel: " << num_ranks << "\n"
        << "> Devices/rank: " << num_devs << "\n"
        << "  > Device density: " << density << unit_density << "\n"
        << "  > Prefetch length: " << prefetch_len << " bits\n"
        << "  > DQs: " << num_DQs << "\n" 
        << "  > Data window: " << data_window << " bits\n"
        << "  > Bank groups: " << num_bgs << "\n"
        << "  > Banks/bank-group: " << num_banks << "\n"
        << "  > Half-bank (left&right): " << ((lr_bank)? "enabled\n" : "disabled\n")
        << "    > Partitions: " << num_parts << "\n"
        << "    > MAT#/partition: " << num_MATs << "\n"
        << "      > MAT-size: " << WLs_per_MAT << "x" << BLs_per_MAT << "\n"
        << "    > Columns: " << num_cols << "\n"
        << "\nTotal module capacity: " << cap << unit_cap << "\n"
        << "==========Information of transaction size==========\n"
        << "Transaction data size from the host: " << HOST_TX_SIZE << "B\n"
        << "Page size of module (transaction unit): " << PAGE_SIZE << "B\n"
        << "Metadata size of module per Tx: " << META_SIZE << "B\n"
        << "===================================================\n"
        << std::endl;
}

