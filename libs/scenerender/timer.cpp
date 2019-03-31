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
#include "timer.h"
#include "Windows.h"
#include "logger.h"

LONGLONG g_freq;
LARGE_INTEGER g_begin;
LARGE_INTEGER g_end;

Ruler::Timer::Timer() 
{
    LARGE_INTEGER tmp;
    QueryPerformanceFrequency(&tmp);
    g_freq = tmp.QuadPart;
}

Ruler::Timer::~Timer() {}

Ruler::Timer& Ruler::Timer::instance()
{
    static Ruler::Timer timer_;
    return timer_;
}

void Ruler::Timer::tic()
{
    Ruler::Timer::instance().start();
}

float Ruler::Timer::toc()
{
    Ruler::Timer::instance().end();
    return ((g_end.QuadPart - g_begin.QuadPart) * 1000000 / g_freq) / 1000000.0f;
}

void Ruler::Timer::start()
{
    QueryPerformanceCounter(&g_begin);
}

void Ruler::Timer::end()
{
    QueryPerformanceCounter(&g_end);
}