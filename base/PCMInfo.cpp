#include "base/PCMInfo.h"
#include "base/MemInfo.h"
#include "base/MemoryControlSystem.h"

using namespace PCMCsim;

MetaDecoder::MetaDecoder(MemoryControlSystem* memsys, std::string cfg_header, MemInfo* info)
{
    if (info==NULL)
    {
        std::cerr << "Error! MemInfo is not defined yet!" << std::endl;
        assert(0);
        exit(1);
    }

    META_SIZE = info->META_SIZE;

    widths[META_PWCNT] = memsys->getParamUINT64(cfg_header+".meta.page_wcnt", 22);
    widths[META_WLVP] = 1;
    widths[META_WDTP] = 1;
    widths[META_ECC] = memsys->getParamUINT64(cfg_header+".meta.ECC", 336);
    widths[META_FW] = memsys->getParamUINT64(cfg_header+".meta.FW", 3);

    uint64_t sum = 0;
    for (uint64_t i=0; i<NUM_META_FLD; i++)
    {
        sum+=widths[i];
        if (sum>META_SIZE*8)
        {
            std::cerr << "Error! Width of metadata should be shorter than "
                << META_SIZE << "B!" << std::endl;
            assert(0);
            exit(1);
        }
    }
}

DataBlock MetaDecoder::get_meta(const DataBlock& meta, pcm_meta_field_t field)
{
    assert(meta.getSize( )==META_SIZE);

    /* Organize width information */
    uint64_t prev_bits = 0;
    for (uint64_t i=0; i<NUM_META_FLD; i++)
    {
        if (i<field)
            prev_bits+=widths[i];
        else if (i==field)
            break;
    }

    return meta.extract(widths[field], prev_bits);
}

void MetaDecoder::set_meta(DataBlock& meta, pcm_meta_field_t field, const DataBlock& new_data)
{
    assert(meta.getSize( )==META_SIZE);

    /* Organize width information */
    uint64_t prev_bits = 0;
    for (uint64_t i=0; i<NUM_META_FLD; i++)
    {
        if (i<field)
            prev_bits+=widths[i];
        else if (i==field)
            break;
    }

    meta.partial_set(new_data, widths[field], prev_bits);
}

AITDecoder::AITDecoder(MemoryControlSystem* memsys, std::string cfg_header, AddressDecoder* adec)
{
    if (adec==NULL)
    {
        std::cerr << "Error! AddressDecoder is not defined yet!" << std::endl;
        assert(0);
        exit(1);
    }

    widths[AIT_PBA] = memsys->getParamUINT64(cfg_header+".pba_bits", 28);
    widths[AIT_KEY] = adec->BLOCK_OFFSET - adec->PAGE_OFFSET;
    widths[AIT_BWCNT] = memsys->getParamUINT64(cfg_header+".bwcnt_bits", 10);

    uint64_t sum = 0;
    for (uint64_t i=0; i<NUM_AIT_FLD; i++)
    {
        sum+=widths[i];
        if (sum>64)
        {
            std::cerr << "[Error] Width of AIT entry should be shorter than 64b!" << std::endl;
            assert(0);
            exit(1);
        }
    }
}

uint64_t AITDecoder::get_ait(uint64_t e_ait, ait_field_t field)
{
    uint64_t rv = e_ait;
    uint64_t ref_bit1 = 1;
    uint64_t shift_mod = 0;
    uint64_t shift_div = 0;
    for (uint64_t i=0; i<NUM_AIT_FLD; i++)
    {
        if (i<field)
        {
            shift_div+=widths[i];
            shift_mod+=widths[i];
        }
        else if (i==field)
        {
            shift_mod+=widths[i];
            break;
        }
    }
    rv = rv % (ref_bit1<<shift_mod);
    rv = rv / (ref_bit1<<shift_div);
    return rv;
}

void AITDecoder::set_ait(uint64_t& e_ait, ait_field_t field, uint64_t new_data)
{
    /* Make mask for the modified field */
    uint64_t ref_bit1 = 1;
    uint64_t mask = (widths[field]==0)? 0:1;
    for (uint64_t i=0; i<widths[field]; i++)
        mask |= (ref_bit1<<i);
    mask = ~mask; 

    /* Place new data on the modified field */
    uint64_t shift_amt = 0;
    uint64_t prev_field_width = 0;
    for (uint64_t i=0; i<NUM_AIT_FLD; i++)
    {
        shift_amt += prev_field_width;
        prev_field_width = widths[i];
    
        if (i==field)
        {
            uint64_t prev_mask = 0;
            for (uint64_t j=0; j<shift_amt; j++)
                prev_mask |= (ref_bit1<<j);
            mask = (mask<<shift_amt) | prev_mask;
            new_data <<= shift_amt;
            break;
        }
    }
    e_ait = (e_ait&mask)|new_data;
}
