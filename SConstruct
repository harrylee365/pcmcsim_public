# -*- mode:python -*-

import os
import sys
import shutil
import SCons

from os.path import basename, dirname, exists, isdir, isfile, join as joinpath

#
# Setup command line options
#
AddOption('--verbose', dest='verbose', action='store_true',
          help='Show full compiler command line', default=False)
AddOption('--build-type', dest='build_type', type='choice',
          choices=["debug", "fast"], 
          help='Type of build [debug, fast]')

#
# Clean if option is provided
#
clean_opt = GetOption("clean")
if clean_opt == True:
    print("\033[96mClean the project\033[0m")


#
# List of source files
#
src_list = []
def AppendSourceList(src):
    src_list.append(File(src))
Export('AppendSourceList')


#
# Setup the default environment variables
#
env = Environment(ENV = os.environ, CXX = 'g++')
env['PCMCSIM_STANDALONE'] = ''
env.Append(CXXFLAGS='-Wno-unknown-pragmas')
env.Append(CXXFLAGS='-Werror')
env.Append(CXXFLAGS='-Wall')
env.Append(CXXFLAGS='-Woverloaded-virtual')


#
# Setup the environment variables according to build option
#
build_type = GetOption("build_type")
if build_type == None or build_type == "fast":
    build_type = "fast"
    env.Append(CXXFLAGS='-O3')
    env.Append(CXXFLAGS='-g')
    env.Append(CXXFLAGS='-fPIC')
    env.Append(CXXFLAGS='-std=c++11')
    env['OBJSUFFIX'] = '.fo'
    env['BUILDROOT'] = 'obj_fast'
elif build_type == "debug":
    env.Append(CXXFLAGS='-O0')
    env.Append(CXXFLAGS='-ggdb')
    env.Append(CXXFLAGS='-std=c++11')
    env['OBJSUFFIX'] = '.do'
    env['BUILDROOT'] = 'obj_debug'

prog_name = 'pcmcsim_run'

# Define paths
env.AppendUnique(CPPPATH=Dir('.')) # for getting headers


#
# Construct the structure of build-directory
# 
env.srcdir = Dir(".") 
env.SetOption("duplicate", "soft-copy")
base_dir = env.srcdir.abspath
Export('env')


#
# Walk current directory to find SConscripts
#
obj_dir = joinpath(base_dir, env['BUILDROOT'])
for root, dirs, files in os.walk(base_dir, topdown=True):
    if root.startswith(obj_dir):
        continue

    if 'SConscript' in files:
        build_dir = joinpath(env['BUILDROOT'], root[len(base_dir) + 1:]) # format: build/...
        SConscript(joinpath(root, 'SConscript'), variant_dir=build_dir, duplicate=0) 

#
# Definitions for getting customized output message formats
#
msg_colors = {}
msg_colors['cyan']   = '\033[96m'
msg_colors['purple'] = '\033[95m'
msg_colors['blue']   = '\033[94m'
msg_colors['green']  = '\033[92m'
msg_colors['yellow'] = '\033[93m'
msg_colors['red']    = '\033[91m'
msg_colors['normal'] = '\033[0m'

if not sys.stdout.isatty():
    for key, value in msg_colors.iteritems():
        msg_colors[key] = ''

class PrettyPrint(object):
    tool_color  = msg_colors['green']
    verb_color  = msg_colors['normal']
    type_color  = msg_colors['normal']
    src_color   = msg_colors['blue']
    arrow_color = msg_colors['red']
    out_color   = msg_colors['blue']

    def __init__(cls, tool, verb, show_src=True):
        cls.output_str = cls.tool_color + ("[%s] " % tool) \
                       + cls.verb_color + ("%s " % verb) \
                       + cls.type_color + "%s " \
                       + cls.src_color + "\"%s\"" \
                       + cls.arrow_color + " ==> " \
                       + cls.out_color + "%s" \
                       + msg_colors['normal']

        cls.show_src = show_src

    def __call__(cls, target, source, env, for_signature=None):
        if not cls.show_src:
            source = []

        def strip(f):
            return strip_build_path(str(f), env)

        if len(source) > 0:
            srcs = map(strip, source)
            tgts = map(strip, target)
            split_src = srcs[0].split("/")

            src_basename = os.path.splitext(split_src[-1])[0]
            src_name = src_basename
            suffix = env['OBJSUFFIX']
        else:
            tgts = map(strip, target)
            split_src = tgts[0].split("/")
            src_basename = os.path.splitext(split_src[-1])[0]
            src_name = tgts[0]
            suffix = ''

        tmp_split = split_src[0].split(".")
        if tmp_split[0]==prog_name:
            src_explain = "binary file"
        else:
            src_explain = "CXX sources"

        return cls.output_str % (src_explain, src_basename, ("%s%s" % (src_name, suffix)))
Export('PrettyPrint')

def strip_build_path(path, env):
    path = str(path)
    variant_base = env['BUILDROOT'] + os.path.sep
    if path.startswith(variant_base):
        path = path[len(variant_base):]
    elif path.startswith('build/'):
        path = path[6:]
    return path

if GetOption("verbose") != True:
    MakeAction = Action
    env['CCCOMSTR']        = PrettyPrint("CC", "Compiling")
    env['CXXCOMSTR']       = PrettyPrint("CXX", "Compiling")
    env['ASCOMSTR']        = PrettyPrint("AS", "Assembling")
    env['ARCOMSTR']        = PrettyPrint("AR", "Archiving", False)
    env['LINKCOMSTR']      = PrettyPrint("LINK", "Linking", False)
    env['RANLIBCOMSTR']    = PrettyPrint("RANLIB", "Indexing Archive", False)
    Export('MakeAction')


# 
# Build according to the name of the final output binary
#
final_bin = "%s.%s" % (prog_name, build_type)
env.Program(final_bin, src_list)

