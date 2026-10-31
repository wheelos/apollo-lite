/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __CHECK_HPP__
#define __CHECK_HPP__

#include <assert.h>
#include <cuda_runtime.h>
#include <stdarg.h>
#include <stdio.h>

#include <stdexcept>
#include <string>

namespace nv {

inline std::runtime_error assertion_error(const char *condition,
                                          const char *message,
                                          const char *file, int line) {
  char diagnostic[1024];
  snprintf(diagnostic, sizeof(diagnostic),
           "BEVFusion assertion failed: %s in %s:%d: %s", condition, file,
           line, message);
  return std::runtime_error(diagnostic);
}

#define NVUNUSED2(a, b) \
  {                     \
    (void)(a);          \
    (void)(b);          \
  }
#define NVUNUSED(a) \
  { (void)(a); }

#if DEBUG
#define checkRuntime(call) nv::check_runtime(call, #call, __LINE__, __FILE__)
#define checkKernel(...)                                                            \
  [&] {                                                                             \
    __VA_ARGS__;                                                                    \
    checkRuntime(cudaStreamSynchronize(nullptr));                                   \
    return nv::check_runtime(cudaGetLastError(), #__VA_ARGS__, __LINE__, __FILE__); \
  }()
#define dprintf printf
#else
#define checkRuntime(call) nv::check_runtime(call, #call, __LINE__, __FILE__)
#define checkKernel(...)                                                        \
  do {                                                                          \
    __VA_ARGS__;                                                                \
    nv::check_runtime(cudaPeekAtLastError(), #__VA_ARGS__, __LINE__, __FILE__); \
  } while (false)
#define dprintf(...)
#endif

#define Assertf(cond, fmt, ...)                                                                                         \
  do {                                                                                                                  \
    if (!(cond)) {                                                                                                      \
      char diagnostic[1024];                                                                                            \
      snprintf(diagnostic, sizeof(diagnostic), fmt, __VA_ARGS__);                                                       \
      throw nv::assertion_error(#cond, diagnostic, __FILE__, __LINE__);                                                 \
    }                                                                                                                   \
  } while (false)

#define Asserts(cond, s)                                                                                 \
  do {                                                                                                   \
    if (!(cond)) {                                                                                       \
      throw nv::assertion_error(#cond, s, __FILE__, __LINE__);                                          \
    }                                                                                                    \
  } while (false)

#define Assert(cond)                                                                     \
  do {                                                                                   \
    if (!(cond)) {                                                                       \
      throw nv::assertion_error(#cond, "condition was false", __FILE__, __LINE__);        \
    }                                                                                    \
  } while (false)

static inline bool check_runtime(cudaError_t e, const char *call, int line, const char *file) {
  if (e != cudaSuccess) {
    fprintf(stderr,
            "CUDA Runtime error %s # %s, code = %s [ %d ] in file "
            "%s:%d\n",
            call, cudaGetErrorString(e), cudaGetErrorName(e), e, file, line);
    char diagnostic[1024];
    snprintf(diagnostic, sizeof(diagnostic),
             "CUDA Runtime error %s: %s [%d] in %s:%d", call,
             cudaGetErrorString(e), static_cast<int>(e), file, line);
    throw std::runtime_error(diagnostic);
  }
  return true;
}

};  // namespace nv

#endif  // #ifndef __CHECK_HPP__