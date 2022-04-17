PCMCsim (Phase-Change Memory Controller Simulator)
==================================================
It is an event-driven simulator that accurately simulates a modern 
phase-chanage memory controller (PCMC). We made every effort to make
this architecture resemble the PCM product. Furthermore, the accuracy 
of the confidential version is verified against RTL trace in SK Hynix,
where functional error=3.16% and cycle error=13.6%

Descriptions of important classes in this source code:
+ `Component`: base class for all modules
+ `MemoryControlSystem`: top module of the controller
+ `RequestReceiver`: an interface that receives requests from the host
+ `DataCache`: serves as a data cache for RequestReceiver
+ `AITManager`: generates commands to read AIT entries from DRAM subsystem
+ `MicroControlUnit`: parent class for firmware algorithms (e.g., wear leveling)
+ `ReadModifyWrite`: performs RMW because PCM access granularity > 64B
+ `uCMDEngine`: decomposes requests as microcommands (e.g., ACT, PRE, etc.)
+ `DataPathUnit`: encode/decode data to/from PCM media

Build and run the PCMCsim
-------------------------
Followings are requirements for the simulator:
+ Git
+ g++ (version>=4.8, which supports C++11)
+ Scons

Steps for executing the simulator:
1. Download PCMCsim

        $ git clone https://gitlab.com/harrylee365/pcmcsim_public.git

2. Build PCMCsim in O3 mode with 4 cores (confer `debug` option if O0 is desired)

        $ cd pcmcsim_public
        $ scons --build-type=fast -j4

3. Run PCMCsim (for more information, use --help):

        $ ./pcmcsim.fast -i ./test_trace/test.input -c ./configs/pcmcsim_base_public.cfg

About the configuration
-----------------------
The simplest configuration example is listed in `pcmcsim_public/configs/pcmcsim_base_public.cfg`

Two configurations should be noted:
+ `global.system`: determines the system configuration. It can be `DRAM` or `PCM`. The former simply builds a memory subsystem only instantiating `uCMDEngine`; the latter builds a memory subsystem that incorporates all necessary features for a PCM controller
+ `global.ticks_per_cycle`: determines the system frequency. The value '1' means 1 THz frequency. Also, each module can have its on frequency by configuring `*.ticks_per_cycle`

Contributors of PCMCsim
-----------------------
+ Hyokeun Lee      hklee@capp.snu.ac.kr
+ Seokbo Shim      sbshim@capp.snu.ac.kr
+ Seungyong Lee    sylee@capp.snu.ac.kr
+ Hyungsuk Kim     kimhs@capp.snu.ac.kr

Citation
--------
```
@inproceedings{PCMCSIM, 
author = {Lee, Hyokeun and Kim, Hyungsuk and Lee, Seungyong and Hong, Dosun and Lee, Hyuk-Jae and Kim, Hyun},
title = {PCMCsim: An Accurate Phase-Change Memory Controller Simulator and its Performance Analysis},
booktitle = {IEEE International Symposium on Performance Analysis of Systems and Software (ISPASS)},
year = {2022}
}
```

Project LICENSE description
---------------------------
Copyright (c) 2019 Computer Architecture and Paralllel Processing Lab, 
Seoul National University, Republic of Korea. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistribution of source code must retain the above copyright 
   notice, this list of conditions and the follwoing disclaimer.
2. Redistributions in binary form must reproduce the above copyright 
   notice, this list conditions and the following disclaimer in the 
   documentation and/or other materials provided with the distirubtion.
3. Neither the name of the copyright holders nor the name of its 
   contributors may be used to endorse or promote products derived from 
   this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This project is supported by SK Hynix Inc. (2019-2021)

CONFIDENTIAL codes are REMOVED in this version
