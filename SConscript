# -*- mode:python -*-

Import('env')

if not 'PCMCSIM_STANDALONE' in env:
    pass
else:
# Build according to environment
    Import('AppendSourceList')

# Standalone-related sources
    AppendSourceList('base/main.cpp')
    AppendSourceList('TraceExec/TraceExec.cpp')
    AppendSourceList('TraceGen/TraceGen.cpp')

# Base sources
    AppendSourceList('base/PCMCTypes.cpp')
    AppendSourceList('base/Component.cpp')
    AppendSourceList('base/DataBlock.cpp')
    AppendSourceList('base/EventQueue.cpp')
    AppendSourceList('base/Packet.cpp')
    AppendSourceList('base/MemoryControlSystem.cpp')
    AppendSourceList('base/PipeBufferv2.cpp')
    AppendSourceList('base/Stats.cpp')
    AppendSourceList('base/MemInfo.cpp')
    AppendSourceList('base/PCMInfo.cpp')

    AppendSourceList('Parsers/Parser.cpp')

# RequestReceiver/
    AppendSourceList('RequestReceiver/RequestReceiver.cpp')

# AITManager/
    AppendSourceList('AITManager/AITManager.cpp')

# DataCache/
    AppendSourceList('DataCache/DataCache.cpp')

# ReadModifyWrite/
    AppendSourceList('ReadModifyWrite/ReadModifyWrite.cpp')

# uCMDEngine/
    AppendSourceList('uCMDEngine/uCMDEngine.cpp')
    AppendSourceList('uCMDEngine/HynixEngine.cpp')
    AppendSourceList('uCMDEngine/JedecEngine.cpp')
    AppendSourceList('uCMDEngine/StateMachines.cpp')
    AppendSourceList('uCMDEngine/JedecPolicyFactory.cpp')
    AppendSourceList('uCMDEngine/FCFS.cpp')
    AppendSourceList('uCMDEngine/FRFCFS.cpp')

# DataPathUnit/
    AppendSourceList('DataPathUnit/DataPathUnit.cpp')

# MemoryModules/
    AppendSourceList('MemoryModules/DummyMemory/DummyMemory.cpp')
    AppendSourceList('MemoryModules/DummyMemory/DummyJedecMEM.cpp')
    AppendSourceList('MemoryModules/DummyMemory/DummyHynixPMEM.cpp')
    AppendSourceList('MemoryModules/DummyMemory/DummyAIT.cpp')
    AppendSourceList('MemoryModules/GPCache/GPCache.cpp')

# ReplacePolicy/
    AppendSourceList('ReplacePolicy/ReplacePolicy.cpp')
    AppendSourceList('ReplacePolicy/TrueLRU/TrueLRU.cpp')
    AppendSourceList('ReplacePolicy/PseudoLRU/PseudoLRU.cpp')
    AppendSourceList('ReplacePolicy/RoundRobin/RoundRobin.cpp')

# Subsystems/
    AppendSourceList('Subsystems/XBar/XBar.cpp')
    AppendSourceList('Subsystems/MCU/MicroControlUnit.cpp')
    AppendSourceList('Subsystems/MCU/BucketWLV.cpp')
    AppendSourceList('Subsystems/MCU/MicroControlUnitFactory.cpp')
