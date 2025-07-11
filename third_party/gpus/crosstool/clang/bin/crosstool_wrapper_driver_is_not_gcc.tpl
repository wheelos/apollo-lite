#!/usr/bin/env python3
# Copyright 2015 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""Crosstool wrapper for compiling CUDA programs.

SYNOPSIS:
  crosstool_wrapper_is_not_gcc [options passed in by cc_library()
                                or cc_binary() rule]

DESCRIPTION:
  This script is expected to be called by the cc_library() or cc_binary() bazel
  rules. When the option "-x cuda" is present in the list of arguments passed
  to this script, it invokes the nvcc CUDA compiler. Most arguments are passed
  as is as a string to --compiler-options of nvcc. When "-x cuda" is not
  present, this wrapper invokes hybrid_driver_is_not_gcc with the input
  arguments as is.
"""

from __future__ import print_function

__author__ = 'keveman@google.com (Manjunath Kudlur)'

from argparse import ArgumentParser
import os
import subprocess
import re
import sys
import pipes

# Template values set by cuda_autoconf.
CPU_COMPILER = ('%{cpu_compiler}')
GCC_HOST_COMPILER_PATH = ('%{gcc_host_compiler_path}')

NVCC_PATH = '%{nvcc_path}'
PREFIX_DIR = os.path.dirname(GCC_HOST_COMPILER_PATH)
NVCC_VERSION = '%{cuda_version}'

def Log(s):
  print('gpus/crosstool: {0}'.format(s))


def GetOptionValue(argv, option, unquote=False):
  """Extract the list of values for option from the argv list.

  Args:
    argv: A list of strings, possibly the argv passed to main().
    option: The option whose value to extract, with the leading '-'.

  Returns:
    A list of values, either directly following the option,
    (eg., -opt val1 val2) or values collected from multiple occurrences of
    the option (eg., -opt val1 -opt val2).
  """

  parser = ArgumentParser()
  parser.add_argument(option, nargs='*', action='append')
  option = option.lstrip('-').replace('-', '_')
  args, _ = parser.parse_known_args(argv)
  if unquote:
    args, _ = parser.parse_known_args(map(lambda x: x.strip("'"), argv))

  if not args or not vars(args)[option]:
    return []
  else:
    if unquote:
      return list(map(repr, sum(vars(args)[option], [])))
    return sum(vars(args)[option], [])


def remove_prefix(text, prefix):
  if text.startswith(prefix):
    return text[len(prefix):]
  return text


def GetHostCompilerOptions(argv):
  """Collect the -isystem, -iquote, and --sysroot option values from argv.

  Args:
    argv: A list of strings, possibly the argv passed to main().

  Returns:
    The string that can be used as the --compiler-options to nvcc.
  """

  parser = ArgumentParser()
  parser.add_argument('-isystem', nargs='*', action='append')
  parser.add_argument('-iquote', nargs='*', action='append')
  parser.add_argument('--sysroot', nargs=1)
  parser.add_argument('-g', nargs='*', action='append')
  parser.add_argument('-fno-canonical-system-headers', action='store_true')
  parser.add_argument('-no-canonical-prefixes', action='store_true')

  args, _ = parser.parse_known_args(argv)

  opts = ''

  if args.isystem:
    isystem_opts = map(
        lambda x: repr(remove_prefix(x.strip("'\""), '-I')),
        sum(args.isystem, []),
    )
    opts += ' -isystem ' + ' -isystem '.join(isystem_opts)
  if args.iquote:
    iquote_opts = map(
        lambda x: repr(remove_prefix(x.strip("'\""), '-I')),
        sum(args.iquote, []),
    )
    opts += ' -iquote ' + ' -iquote '.join(iquote_opts)
  if args.g:
    opts += ' -g' + ' -g'.join(sum(args.g, []))
  if args.fno_canonical_system_headers:
    opts += ' -fno-canonical-system-headers'
  if args.no_canonical_prefixes:
    opts += ' -no-canonical-prefixes'
  if args.sysroot:
    opts += ' --sysroot ' + args.sysroot[0]

  return opts

def _update_options(nvcc_options):
  if NVCC_VERSION in ("7.0",):
    return nvcc_options

  update_options = { "relaxed-constexpr" : "expt-relaxed-constexpr" }
  return [ update_options[opt] if opt in update_options else opt
                    for opt in nvcc_options ]

def GetNvccOptions(argv):
  """Collect the -nvcc_options values from argv.

  Args:
    argv: A list of strings, possibly the argv passed to main().

  Returns:
    The string that can be passed directly to nvcc.
  """

  parser = ArgumentParser()
  parser.add_argument('-nvcc_options', nargs='*', action='append')

  args, _ = parser.parse_known_args(argv)

  if args.nvcc_options:
    options = _update_options(sum(args.nvcc_options, []))
    return ' '.join(['--'+a for a in options])
  return ''

def system(cmd):
  """Invokes cmd with os.system().

  Args:
    cmd: The command.

  Returns:
    The exit code if the process exited with exit() or -signal
    if the process was terminated by a signal.
  """
  retv = os.system(cmd)
  if os.WIFEXITED(retv):
    return os.WEXITSTATUS(retv)
  else:
    return -os.WTERMSIG(retv)

def InvokeNvcc(argv, log=False):
  """Call nvcc with arguments assembled from argv.

  Args:
    argv: A list of strings, possibly the argv passed to main().
    log: True if logging is requested.

  Returns:
    The return value of calling system('nvcc ' + args)
  """

  host_compiler_options = GetHostCompilerOptions(argv)
  nvcc_compiler_options = GetNvccOptions(argv)
  opt_option = GetOptionValue(argv, '-O')
  m_options = GetOptionValue(argv, '-m')
  m_options = ''.join([' -m' + m for m in m_options if m in ['32', '64']])
  include_options = GetOptionValue(argv, '-I', True)
  out_file = GetOptionValue(argv, '-o')
  depfiles = GetOptionValue(argv, '-MF')
  defines = GetOptionValue(argv, '-D', True)
  defines = ''.join([' -D' + define for define in defines])
  undefines = GetOptionValue(argv, '-U')
  undefines = ''.join([' -U' + define for define in undefines])
  std_options = GetOptionValue(argv, '-std')
  # Supported -std flags as of CUDA 9.0. Only keep last to mimic gcc/clang.
  nvcc_allowed_std_options = ["c++03", "c++11", "c++14", "c++17"]
  std_options = ''.join([' -std=' + define
      for define in std_options if define in nvcc_allowed_std_options][-1:])
  fatbin_options = ''.join([' --fatbin-options=' + option
      for option in GetOptionValue(argv, '-Xcuda-fatbinary')])

  # The list of source files get passed after the -c option. I don't know of
  # any other reliable way to just get the list of source files to be compiled.
  src_files = GetOptionValue(argv, '-c')

  # Pass -w through from host to nvcc, but don't do anything fancier with
  # warnings-related flags, since they're not necessarily the same across
  # compilers.
  warning_options = ' -w' if '-w' in argv else ''

  if len(src_files) == 0:
    return 1
  if len(out_file) != 1:
    return 1

  opt = (' -O2' if (len(opt_option) > 0 and int(opt_option[0]) > 0)
         else ' -g')

  includes = (' -I ' + ' -I '.join(include_options)
              if len(include_options) > 0
              else '')

  # Unfortunately, there are other options that have -c prefix too.
  # So allowing only those look like C/C++ files.
  src_files = [f for f in src_files if
               re.search('\.cpp$|\.cc$|\.c$|\.cxx$|\.cu$|\.C$', f)]
  srcs = ' '.join(src_files)
  out = ' -o ' + out_file[0]

  nvccopts = '-D_FORCE_INLINES '
  for capability in GetOptionValue(argv, "--cuda-gpu-arch"):
    capability = capability[len('sm_'):]
    nvccopts += r'-gencode=arch=compute_%s,\"code=sm_%s\" ' % (capability,
                                                               capability)
  for capability in GetOptionValue(argv, '--cuda-include-ptx'):
    capability = capability[len('sm_'):]
    nvccopts += r'-gencode=arch=compute_%s,\"code=compute_%s\" ' % (capability,
                                                                    capability)
  nvccopts += nvcc_compiler_options
  nvccopts += undefines
  nvccopts += defines
  nvccopts += std_options
  nvccopts += m_options
  nvccopts += warning_options
  nvccopts += fatbin_options

  if depfiles:
    # Generate the dependency file
    depfile = depfiles[0]
    cmd = (NVCC_PATH + ' ' + nvccopts +
           ' --compiler-options "' + host_compiler_options + '"' +
           ' --compiler-bindir=' + GCC_HOST_COMPILER_PATH +
           ' -I .' +
           ' -x cu ' + opt + includes + ' ' + srcs + ' -M -o ' + depfile)
    if log: Log(cmd)
    exit_status = system(cmd)
    if exit_status != 0:
      return exit_status

  cmd = (NVCC_PATH + ' ' + nvccopts +
         ' --compiler-options "' + host_compiler_options + ' -fPIC"' +
         ' --compiler-bindir=' + GCC_HOST_COMPILER_PATH +
         ' -I .' +
         ' -x cu ' + opt + includes + ' -c ' + srcs + out)

  # TODO(zhengxq): for some reason, 'gcc' needs this help to find 'as'.
  # Need to investigate and fix.
  cmd = 'PATH=' + PREFIX_DIR + ':$PATH ' + cmd
  if log: Log(cmd)
  return system(cmd)


def main():
  parser = ArgumentParser()
  parser.add_argument('-x', nargs=1)
  parser.add_argument('--cuda_log', action='store_true')
  args, leftover = parser.parse_known_args(sys.argv[1:])

  if args.x and args.x[0] == 'cuda':
    if args.cuda_log: Log('-x cuda')
    leftover = [pipes.quote(s) for s in leftover]
    if args.cuda_log: Log('using nvcc')
    return InvokeNvcc(leftover, log=args.cuda_log)

  # Strip our flags before passing through to the CPU compiler for files which
  # are not -x cuda. We can't just pass 'leftover' because it also strips -x.
  # We not only want to pass -x to the CPU compiler, but also keep it in its
  # relative location in the argv list (the compiler is actually sensitive to
  # this).
  cpu_compiler_flags = [flag for flag in sys.argv[1:]
                             if not flag.startswith(('--cuda_log'))]

  if args.cuda_log:
    Log(' '.join([CPU_COMPILER] + cpu_compiler_flags))
  return subprocess.call([CPU_COMPILER] + cpu_compiler_flags)

if __name__ == '__main__':
  sys.exit(main())
