/*
 * Copyright (c) 2019 Computer Architecture and Paralllel Processing Lab, 
 * Seoul National University, Republic of Korea. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistribution of source code must retain the above copyright 
 *        notice, this list of conditions and the follwoing disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright 
 *        notice, this list conditions and the following disclaimer in the 
 *        documentation and/or other materials provided with the distirubtion.
 *     3. Neither the name of the copyright holders nor the name of its 
 *        contributors may be used to endorse or promote products derived from 
 *        this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Hyokeun Lee (hklee@capp.snu.ac.kr)
 */

#ifndef __PCMCSIM_TYPES_H_
#define __PCMCSIM_TYPES_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <cassert>
#include <stdint.h>
#include <algorithm>
#include <limits>
#include <bitset>
#include <cmath>
#include <string>
#include <cstring>
#include <queue>
#include <list>
#include <vector>
#include <deque>
#include <map>
#include <set>

#define PCMC_DBG(FLAG, msg, ...) \
    do { if (FLAG) std::fprintf(stdout, msg, ##__VA_ARGS__); } while (0)

namespace PCMCsim 
{
extern uint64_t INVALID_ADDR;

typedef uint64_t ncycle_t;
typedef uint8_t nbyte_t;
typedef int64_t id_t;

/* Commmand types used in Packet */
typedef enum _cmd_t
{
    CMD_UNDEF = 0,
    
    /* 
     * Macro commands 
     * NOTE: these can work as micro cmds for JEDEC engine
     */
    CMD_READ,
    CMD_WRITE,

    /* Micro commands: Hynix format */
    CMD_BWT,        // buffer write
    CMD_WTC,        // core write
    CMD_BRD,        // buffer read
    CMD_RDC,        // core read

    /* Micro commands: JEDEC format */
    CMD_PRE,        // precharge
    CMD_PRE_AB,     // precharge all banks TODO
    CMD_PRE_SB,     // precharge same banks TODO
    CMD_ACT,        // activation
    CMD_REFRESH,    // refresh
    CMD_IMPLICIT,   // temporal cmd for obtaining implicit PRE
    CMD_READ_PRE,   // implicit (auto) precharge after read
    CMD_WRITE_PRE,  // implicit precharge after write
    CMD_APDE,       // active PD (powerdown)
    CMD_FPPDE,      // precharge PD: fast exit mode
    CMD_SPPDE,      // precharge PD: slow exit mode
    CMD_PDX,        // PD exit
    CMD_SRE,        // SR (self-refresh) enter
    CMD_SRX,        // SR exit

    /* MISC */
    CMD_WACK_ID,    // Retire-ID signal
    CMD_PKT_DEL,    // Packet deletion on owner-side (no physical meaning)
    CMD_IRQ,
    CMD_WLV,

    NUM_CMDS
} cmd_t;

/* Address field enumeration */
typedef enum _addr_field_t
{
    FLD_CH=0,       // channel, including sub-channel
    FLD_RANK,
    FLD_BANKGRP,    // bank group
    FLD_BANK,
    FLD_HALF,       // left-part and right-part of a bank (0-bit/1-bit possible)
    FLD_PART,       // partitions within a half-bank
    FLD_ROW,
    FLD_COL,

    NUM_FIELDS
} addr_field_t;

typedef enum _pcm_meta_field_t
{
    META_PWCNT=0,   // page write count
    META_WLVP,      // wear-leveling (WLV) pended
    META_WDTP,      // WDT pending
    META_FW,        // firmware meta
    META_ECC,       // reed solomon parity

    NUM_META_FLD
} pcm_meta_field_t;

typedef enum _ait_field_t
{
    AIT_PBA=0,
    AIT_KEY,
    AIT_BWCNT,

    NUM_AIT_FLD
} ait_field_t;

/* Source ID of packet */
enum _src_ids
{
    SRC_HOST=0,         // request receiver-address remapper
    SRC_BLKMGR,         // block manager. Generates page requests constituting a block unit
    SRC_WDTP,           // write disturbance preventer
    SRC_RDTP,           // read disturbance preventer
    SRC_PTRSCB,         // patrol scrubber
    SRC_MCU,

    NUM_SRC
};

};

#endif
