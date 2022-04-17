#include "base/DataBlock.h"

using namespace PCMCsim;

DataBlock::DataBlock( )
:rawData(NULL), num_bytes(0) 
{
}

DataBlock::~DataBlock( )
{
    delete[] rawData;
    rawData = NULL;
    num_bytes = 0;
}

DataBlock::DataBlock( const DataBlock& rhs )
:rawData(NULL), num_bytes(0) 
{
    if (rhs.rawData)
    {
        assert(rhs.num_bytes>0);
        num_bytes = rhs.num_bytes;
        rawData = new nbyte_t[num_bytes];
        memcpy(rawData, rhs.rawData, num_bytes);
    }
}

void DataBlock::printData( )
{
    uint64_t byte = num_bytes-1; 
    while (1)
    {
        if (byte==num_bytes-1)
            std::cout << "0x";
        if (rawData[byte] <= 0xf)
            std::cout << "0";
        std::cout << std::hex << (uint32_t)rawData[byte] << std::dec;

        if (byte==0)
        {
            std::cout << std::endl;
            break;
        }
        else
            byte-=1;
    }
}

void DataBlock::setSize(uint64_t num_bytes)
{
    delete[] rawData;
    if (num_bytes==0) 
    {
        rawData = NULL;
        this->num_bytes = 0;
        return;
    }

    rawData = new nbyte_t[num_bytes];
    assert(rawData);
    
    this->num_bytes = num_bytes;
    memset(rawData, 0x0, num_bytes);
}

uint64_t DataBlock::getSize( ) const
{
    return num_bytes;
}

void DataBlock::setByte(uint64_t byte, nbyte_t value)
{
    if (byte < num_bytes)
        rawData[byte] = value;
    else
    {
        std::cerr << "[DataBlock] Error! Input byte is out of range while setting " << 
                     "(byte, num_bytes): " << byte << ", " << num_bytes << std::endl;
        assert(0);
        exit(1); 
    }
}

nbyte_t DataBlock::getByte(uint64_t byte) const
{
    nbyte_t rv = 0;
    if (byte < num_bytes)
        rv = rawData[byte];
    else
    {
        std::cerr << "[DataBlock] Error! Input byte is out of range while reading " << 
                     "(byte, num_bytes): " << byte << ", " << num_bytes << std::endl;
        assert(0);
        exit(1); 
    }
    
    return rv;
}

/*
 *       obj_ed        obj_st
 *   obj_res valid  valid prev_res
 * |...|xxxxoooo|....|ooooxxxx|...|
 */
DataBlock DataBlock::extract(uint64_t obj_bits, uint64_t prev_bits) const
{
    assert(obj_bits+prev_bits<=num_bytes*8);

    /* Organize width information */
    uint64_t ref_bit1 = 1;
    uint64_t acc_bits = obj_bits+prev_bits;
    uint64_t prev_bytes = prev_bits/8;
    uint64_t prev_res_bits = prev_bits%8; // residual of preceding bits
    uint64_t obj_st_byte = prev_bytes;
    uint64_t obj_ed_byte = (acc_bits/8>0 && acc_bits%8==0)? 
        (acc_bits/8-1) : (acc_bits/8);
    uint64_t obj_res_bits = acc_bits%8;   // residual of obj within last byte

    /* Acquire objective data */
    DataBlock obj_data;
    obj_data.setSize(obj_ed_byte-obj_st_byte+1);
    uint64_t b=0;
    for (uint64_t i=obj_st_byte; i<=obj_ed_byte; i++)
    {
        /* Mask residual bits on start byte */
        uint8_t byte_data = this->getByte(i);
        if (i==obj_st_byte)
        {
            uint8_t mask = 0;
            uint64_t res_bits = prev_res_bits;
            for (uint64_t j=0; j<res_bits; j++)
                mask |= (ref_bit1<<j);
            mask = ~mask;
            byte_data &= mask;
        }

        /* Mask bits in last byte except for residual bits */
        if (i==obj_ed_byte && obj_res_bits>0)
        {
            uint8_t mask = 0;
            uint64_t res_bits = obj_res_bits;
            for (uint64_t j=0; j<res_bits; j++)
                mask |= (ref_bit1<<j);
            byte_data &= mask;
        }

        obj_data.setByte(b, byte_data);
        b+=1;
    }

    /* Align data */
    DataBlock rv; 
    if (prev_res_bits>0)
    {
        uint64_t obj_bytes = (obj_bits%8==0)? (obj_bits/8):(obj_bits/8+1);
        rv.setSize(obj_bytes);
        for (uint64_t i=0; i<obj_data.getSize( ); i++)
        {
            uint8_t curr_byte = obj_data.getByte(i);
            uint8_t next_byte = (i==obj_data.getSize( )-1)? 0 : obj_data.getByte(i+1);
            uint8_t aligned = (next_byte<<(8-prev_res_bits)) | (curr_byte>>prev_res_bits);

            rv.setByte(i, aligned);
            if (i==rv.getSize( )-1)
                break;
        }
    }
    else
        rv = obj_data;

    return rv;
}

void DataBlock::partial_set(const DataBlock& new_data, uint64_t obj_bits, uint64_t prev_bits)
{
    assert(obj_bits+prev_bits<=num_bytes*8);

    /* Organize width information */
    uint64_t ref_bit1 = 1;
    uint64_t acc_bits = obj_bits+prev_bits;
    uint64_t prev_bytes = prev_bits/8;
    uint64_t prev_res_bits = prev_bits%8; // residual of preceding bits
    uint64_t obj_st_byte = prev_bytes;
    uint64_t obj_ed_byte = (acc_bits/8>0 && acc_bits%8==0)? 
        (acc_bits/8-1) : (acc_bits/8);
    uint64_t obj_res_bits = acc_bits%8;   // residual of obj within last byte

    /* Shift aligned new data for update */
    DataBlock obj_field;
    uint64_t obj_bytes = (obj_bits%8==0)? (obj_bits/8):(obj_bits/8+1);
    if (prev_res_bits>0)
    {
        obj_field.setSize(obj_ed_byte-obj_st_byte+1);
        for (uint64_t i=0; i<obj_field.getSize( ); i++)
        {
            uint8_t prev_byte = (i==0)? 0 : new_data.getByte(i-1);
            uint8_t curr_byte = (i>=obj_bytes)? 0 : new_data.getByte(i);
            uint8_t form = (curr_byte<<prev_res_bits) | (prev_byte>>(8-prev_res_bits));
            obj_field.setByte(i, form);
        }
    }
    else if (new_data.getSize( )>obj_bytes)
    {
        obj_field.setSize(obj_bytes);
        for (uint64_t i=0; i<obj_bytes; i++)
            obj_field.setByte(i, new_data.getByte(i));
    }
    else
        obj_field = new_data;

    /* Mask out redundant bits on last byte */
    uint64_t obj_full_bits = obj_bits+prev_res_bits;
    if (obj_full_bits%8>0)
    {
        uint8_t mask = 0;
        uint8_t last_byte = obj_field.getByte(obj_field.getSize( )-1);
        for (uint64_t i=0; i<obj_full_bits; i++)
            mask |= (ref_bit1<<i);
        last_byte &= mask;
        obj_field.setByte(obj_field.getSize( )-1, last_byte);
    }

    /* Update target field */
    uint64_t b=0;
    for (uint64_t i=obj_st_byte; i<=obj_ed_byte; i++)
    {
        /* Mask target field */
        uint8_t mask = 0;
        uint8_t byte_data = this->getByte(i);
        if (i==obj_st_byte)
        {
            uint8_t tmp_mask = 0;
            uint64_t res_bits = prev_res_bits;
            for (uint64_t j=0; j<res_bits; j++)
                tmp_mask |= (ref_bit1<<j);
            mask |= tmp_mask;
        }

        if (i==obj_ed_byte && obj_res_bits>0)
        {
            uint8_t tmp_mask = 0;
            uint64_t res_bits = obj_res_bits;
            for (uint64_t j=0; j<res_bits; j++)
                tmp_mask |= (ref_bit1<<j);
            tmp_mask = ~tmp_mask;
            mask |= tmp_mask;
        }

        byte_data &= mask;
        
        uint8_t new_byte = byte_data | obj_field.getByte(b);
        this->setByte(i, new_byte);
        b+=1;
    }
}

void DataBlock::wrap_u64(uint64_t value)
{
    this->setSize(8);
    for(uint64_t i = 0; i < 8; i++)
        this->setByte(i, uint8_t((value>>(8*i))&0xFF));
}

uint64_t DataBlock::unwrap_u64( )
{
    /* Get LS64b */
    uint64_t value = 0;
    for (uint64_t i=0; i<this->getSize( ); i++)
    {
        if (i==8)
            break;
        
        value |= (this->getByte(i)<<(8*i));
    }
    return value;
}

DataBlock& DataBlock::operator=(const DataBlock& rhs)
{
    if (rawData!=NULL)
    {
        delete[] rawData;
        rawData = NULL;
    }

    if (rhs.rawData)
    {
        assert(rhs.num_bytes>0);
        num_bytes = rhs.num_bytes;
        rawData = new nbyte_t[num_bytes];
        memcpy(rawData, rhs.rawData, num_bytes);
    }

    return (*this);
}

bool DataBlock::operator==(const DataBlock rhs)
{
    bool rv = true;

    if (num_bytes != rhs.num_bytes)
        rv = false;
    else
    {
        for (uint64_t byte = 0; byte < num_bytes; byte++)
        {
            if (rawData[byte] != rhs.rawData[byte])
                rv = false;
        }
    }

    return rv;
}

