/***********************************************************************
* Software License Agreement (Ruler License)
*
* Copyright 2008-2011  Li YunQiang (liyunqiang@91ruler.com). All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/

#ifndef _SCENERENDER_DEFINES_H_
#define _SCENERENDER_DEFINES_H_

#include "config.h"

#ifdef SCENERENDER_EXPORT
#undef SCENERENDER_EXPORT
#endif
#ifdef WIN32
/* win32 dll export/import directives */
#ifdef SCENERENDER_EXPORTS
#define SCENERENDER_EXPORT __declspec(dllexport)
#elif defined(SCENERENDER_STATIC)
#define SCENERENDER_EXPORT
#else
#define SCENERENDER_EXPORT __declspec(dllimport)
#endif
#else
/* unix needs nothing */
#define SCENERENDER_EXPORT
#endif

#ifdef SCENERENDER_DEPRECATED
#undef SCENERENDER_DEPRECATED
#endif
#ifdef __GNUC__
#define SCENERENDER_DEPRECATED __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define SCENERENDER_DEPRECATED __declspec(deprecated)
#else
#pragma message("WARNING: You need to implement SCENERENDER_DEPRECATED for this compiler")
#define SCENERENDER_DEPRECATED
#endif

#undef SCENERENDER_PLATFORM_64_BIT
#undef SCENERENDER_PLATFORM_32_BIT
#if __amd64__ || __x86_64__ || _WIN64 || _M_X64
#define SCENERENDER_PLATFORM_64_BIT
#else
#define SCENERENDER_PLATFORM_32_BIT
#endif

#undef SCENERENDER_ARRAY_LEN
#define SCENERENDER_ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))

#ifdef __cplusplus
namespace Ruler
{
#endif

enum SCENERENDER_log_level_t
{
    SCENERENDER_LOG_NONE = 0,
    SCENERENDER_LOG_FATAL = 1,
    SCENERENDER_LOG_ERROR = 2,
    SCENERENDER_LOG_WARN = 3,
    SCENERENDER_LOG_INFO = 4,
    SCENERENDER_LOG_DEBUG = 5
};

enum SCENERENDER_datatype_t
{
    SCENERENDER_NONE = -1,
    SCENERENDER_INT8 = 0,
    SCENERENDER_INT16 = 1,
    SCENERENDER_INT32 = 2,
    SCENERENDER_INT64 = 3,
    SCENERENDER_UINT8 = 4,
    SCENERENDER_UINT16 = 5,
    SCENERENDER_UINT32 = 6,
    SCENERENDER_UINT64 = 7,
    SCENERENDER_FLOAT32 = 8,
    SCENERENDER_FLOAT64 = 9
};

#ifdef __cplusplus
}
#endif

#include <vector>
#include <string>
#include <stdio.h>
#include <stdarg.h>

#include <vld.h>
#include "opencv2/opencv.hpp"

#endif /* SCENERENDER_DEFINES_H_ */
