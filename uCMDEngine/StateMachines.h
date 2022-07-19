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
 * Authors: Hyokeun Lee (hklee@capp.snu.ac.kr)
 *
 * Decription: Rank/Bank Machines that monitors Jedec-defined timings
 * which are modified from NVMain (https://github.com/SEAL-UCSB/NVmain.git)
 * Units of stats:
 *   Current (IDD*): mA
 *   Voltage (VDD): V
 *   Energy (E_*): nJ
 *   Bandwidth: MB/s
 */

#ifndef __PCMCSIM_STATE_MACHINES_H_
#define __PCMCSIM_STATE_MACHINES_H_

#include "base/PCMCTypes.h"

namespace PCMCsim
{
    class JedecEngine;
    class StateMachine;
    class Stats;

    class BankMachine
    {
      public:
        BankMachine( ) = delete;
        BankMachine(StateMachine* sm_, uint64_t id_, uint64_t rank_id);
        BankMachine(const BankMachine& obj);
        ~BankMachine( ) { delete stats; }

        bool is_idle( );
        bool is_issuable(Packet* pkt);
        ncycle_t timely_issuable(Packet* pkt);
        ncycle_t update_states(Packet* pkt);

        void postupdate_states(Packet* pkt);
        void notify(Packet* pkt, bool is_same_bankgroup);

        void update_stats(ncycle_t cycles);
        void register_stats( );
        void calculate_stats( );
        void print_stats(std::ostream& os);
        double get_dynamic_energy( ) { return (E_act+E_rd+E_wr); }

      private:
        uint64_t id;
        std::string name;

        StateMachine* sm;
        uint64_t num_rows;
        uint64_t num_rows_per_part;
        uint64_t num_parts;
        uint64_t half_banks;
        
        uint64_t open_row;  // row that is currently activated
        int state;

        /* Machine states */
        ncycle_t issuable_ACT;
        ncycle_t issuable_PRE;
        ncycle_t issuable_READ;
        ncycle_t issuable_WRITE;
        ncycle_t issuable_REFRESH;
        ncycle_t issuable_PDE;
        ncycle_t issuable_PDX;
        ncycle_t issuable_SRE;
        ncycle_t issuable_SRX;

        ncycle_t activate(Packet* pkt);
        ncycle_t precharge(Packet* pkt);
        ncycle_t read(Packet* pkt);
        ncycle_t write(Packet* pkt);
        ncycle_t powerdown(Packet* pkt);
        ncycle_t powerup(Packet* pkt);
        ncycle_t refresh(Packet* pkt);
        ncycle_t sre(Packet* pkt);
        ncycle_t srx(Packet* pkt);

        uint64_t get_row_idx(uint64_t half, uint64_t part, uint64_t row)
        {
            return ((half*num_parts+part)*num_rows_per_part+row);
        }

        /* Stats (BM only records dynamic energy) */
        Stats* stats;
        ncycle_t cycles_act;
        ncycle_t cycles_bckgnd;
        ncycle_t cycles_io;

      public:
        double E_act;
        double E_rd;
        double E_wr;
        double E_refresh;

        double bandwidth;
        double util_bw;

        uint64_t num_ACT;
        uint64_t num_PRE;
        uint64_t num_REFRESH;
        uint64_t num_READ;
        uint64_t num_WRITE;
        uint64_t num_PD;
        uint64_t num_SREF;
    };

    class RankMachine
    {
      public:
        RankMachine( ) = delete;
        RankMachine(StateMachine* sm_, uint64_t id_);
        RankMachine(const RankMachine& obj);
        ~RankMachine( ) { delete stats; }

        bool is_idle( );
        bool is_issuable(Packet* pkt);

        ncycle_t timely_issuable(Packet* pkt);
        ncycle_t update_states(Packet* pkt);
        void postupdate_states(Packet* pkt);

        void notify(Packet* pkt);
        void update_stats(ncycle_t cycles);
        void register_stats( );
        void calculate_stats( );
        void print_stats(std::ostream& os);
        
      private:
        uint64_t id;
        std::string name;

        std::vector<BankMachine> bm;
        StateMachine* sm;
        uint64_t num_bgs;
        uint64_t num_banks;
        uint64_t num_all_banks;
        int state;

        /* Machine states */
        ncycle_t issuable_READ;
        ncycle_t issuable_WRITE;
        std::vector<uint64_t> last_ACTs;  // time window of last X ACTs
        uint64_t XAW_ptr;

        ncycle_t activate(Packet* pkt);
        ncycle_t rdwr(Packet* pkt);
        ncycle_t precharge(Packet* pkt);
        ncycle_t powerdown(Packet* pkt);
        ncycle_t powerup(Packet* pkt);
        ncycle_t refresh(Packet* pkt);
        ncycle_t sre(Packet* pkt);
        ncycle_t srx(Packet* pkt);

        uint64_t get_bm_idx(uint64_t bg, uint64_t bk) { return (bg*num_banks+bk); }

        /* Stats */
        Stats* stats;
        ncycle_t cycles_apd; // units are not ticks
        ncycle_t cycles_fppd;
        ncycle_t cycles_sppd;
        ncycle_t cycles_sref;
        ncycle_t cycles_stnby_act;
        ncycle_t cycles_stnby_pre;

      public:
        double E_total;     // units are 'nJ'
        double E_bckgnd;    // background energy = standby + pdwn(or sref) + refresh
        double E_stnby;
        double E_refresh;
        double E_act;       // activation energy
        double E_rd;
        double E_wr;

        double avg_bandwidth;
        double util_bw;

        uint64_t num_ACT;
        uint64_t num_PRE;
        uint64_t num_REFRESH;
        uint64_t num_READ;
        uint64_t num_WRITE;
        uint64_t num_PD;
        uint64_t num_SREF;
    };

    class StateMachine
    {
      public: 
        StateMachine( ) = delete;
        StateMachine(JedecEngine* je_);
        ~StateMachine( ) { }

        bool is_idle(uint64_t rank);
        bool is_issuable(Packet* pkt);

        ncycle_t timely_issuable(Packet* pkt);
        ncycle_t update_states(Packet* pkt);
        ncycle_t get_postupdate_latency(Packet* pkt);
        void postupdate_states(Packet* pkt);

        void update_stats(ncycle_t cycles);
        void register_stats( );
        void calculate_stats( );
        void print_stats(std::ostream& os);

        /* Timing-related parameters */
        bool timing_set = false;
        ncycle_t tCL;       // RDCMD-to-placement of rdata on data bus
        ncycle_t tRCD;      // RAS-to-CAS delay (RAS: ACT, CAS: RD/WR)
        ncycle_t tRP;       // PRE latency
        ncycle_t tRAS;      // min requirement for RAS, including restoring cells
        ncycle_t tCMD;      // CMD latency from engine to devices
        ncycle_t tBURST;    // burst I/O
        ncycle_t tCWL;      // WRCMD-to-wdata from engine on data bus (=tCWD)
        ncycle_t tPPD;      // PRE-to-PRE
        ncycle_t tRTP;      // RD-to-PRE
        ncycle_t tWR;       // write recovery
        ncycle_t tAL;       // posted CAS latency. For early issuing CAS cmd
        ncycle_t tRRD_S;    // short (S) tRRD of different bank groups (BG)
        ncycle_t tRRD_L;    // long (L) tRRD of same BGs
        ncycle_t tCCD_S;    // tCCD (CAS-to-CAS delay) of different BGs
        ncycle_t tCCD_L;    // tCCD of same BGs
        ncycle_t tCCD_L_WR; // tCCD of same BGs but WR incurs RMW before ODE
        ncycle_t tCCD_L_WR2;// tCCD of same BGs but JW(just-write) after ODE
        ncycle_t tWTR_S;    // tWTR of different BGs
        ncycle_t tWTR_L;    // tWTR of same BGs
        ncycle_t tRTRS;     // rank-to-rank switch

        ncycle_t tRDPDEN;   // RD/RDA-to-PD entry (A stands for auto-precharge)
        ncycle_t tWRPDEN;   // WR-to-PD entry
        ncycle_t tWRAPDEN;  // WRA-to-PD entry
        ncycle_t tPD;       // PD entry-to-PD exit
        ncycle_t tXP;       // PD exit-to-no locked DLL CMD
        ncycle_t tXPDLL;    // PD exit-to-locked DLL CMD

        ncycle_t tCKESR;    // SR entry minimum duration
        ncycle_t tXS;       // SR exit-to-no locked DLL CMD
        ncycle_t tXSDLL;    // SR exit-to-locked DLL CMD

        ncycle_t tREFW;     // AR window (e.g., 64ms)
        ncycle_t tREFI;     // AR distrubuted refresh interval (NOT USER-DEFINED)
        ncycle_t tRFC;      // AR cycle within tREFI (determined by pb/ab, FGR)

        ncycle_t tXAW;      // min latency to accomodate ACT window
        uint64_t XAW;       // ACT window width

        /* On-die ECC (ODE) parameters */
        bool ode;
        uint64_t data_window;
        uint64_t ode_databits;
        uint64_t get_tCCD_L(cmd_t type);

        /* AR-related parameters */
        uint64_t order_fgr;         // order of fine-grained refresh (FGR)
        uint64_t num_bundles;       // # of refresh bundles (NOT USER-DEFINED)
        uint64_t rows_per_bundle;   // rows to refresh per cmd (NOT USER-DEFINED)
        uint64_t banks_per_refresh; // # of banks to refresh at once (pb/ab)
        bool sb_refresh;            // same bank refresh (for DDR5)

        /* Energy-related parameters */
        double VDD;     // core voltage
        double IDD0;    // active-precharge current
        double IDD2P0;  // precharge-to-PD current for fast-exit
        double IDD2P1;  // precharge-to-PD current for slow-exit
        double IDD2N;   // standby precharge current
        double IDD3P;   // active-to-PD current
        double IDD3N;   // standby active current
        double IDD4R;   // read current
        double IDD4W;   // write current
        double IDD5;    // auto refresh (AR) current
        double IDD6;    // self refresh (SR) current

        double E_act_inc;
        double E_rd_inc;
        double E_wr_inc;
        double E_refresh_inc;
        double E_sref_inc;
        double E_apd_inc;
        double E_sppd_inc;
        double E_fppd_inc;
        double E_stnby_act_inc;
        double E_stnby_pre_inc;

      private:
        JedecEngine* je;
        std::vector<RankMachine> rm;
        uint64_t num_ranks;
        
        std::string print_state(int i=0);
        ncycle_t get_tick_after(ncycle_t LAT);

        /* Device parameter setup */
        void setup_timings(const uint64_t num_all_banks);
        void setup_currents( );
        
        friend class RankMachine;
        friend class BankMachine;
    };
};

#endif
