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
#ifndef _SCENERENDER_SCENERENDER_H
#define _SCENERENDER_SCENERENDER_H

#include "defines.h"
#include "camera.h"

namespace Ruler
{

class SceneRenderImpl;

class SCENERENDER_EXPORT SceneRender
{
public:
    SceneRender(const CameraD& param, int boxwidth, int panowidth, int panoheight);
    ~SceneRender();

    void renderPanorama(const char* panopath);
    void renderTrimesh(const char* objpath, const char* imgpath = "", int record_label = 0, bool is_rotate_axis = false);
    void renderRectangle(const char* imgpath, const CameraD& param, float rectw, float recth, int record_label = 1, bool is_rotate_axis = false);

    void savePanoDepthImage(const char* imgpath);
    void savePanoRecordImage(const char* imgpath);
    void savePanoSimulateImage(const char* imgpath);

    void saveSixBoxDepthImage(const char* imgpath);
    void saveSixBoxRecordImage(const char* imgpath);
    void saveSixBoxSimulateImage(const char* imgpath);

    void showPanoSimulateWithOpenGL();

private:
    SceneRenderImpl* const impl_ptr_;
};

} // namespace Ruler

#endif // _SCENERENDER_SCENERENDER_H