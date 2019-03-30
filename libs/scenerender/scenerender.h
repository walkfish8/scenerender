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
#include "trimesh.h"
#include "raster.h"
#include "sixbox.h"

namespace Ruler
{

enum class ObjectState
{
    OBJECTSTATE_TRIMESH,
    OBJECTSTATE_RECTANGLE,
    OBJECTSTATE_PANORAMA
};

class SCENERENDER_EXPORT RenderObject
{
public:
    TriMesh mesh;
    virtual ObjectState label() = 0;
};

class SCENERENDER_EXPORT RenderTrimesh : public RenderObject
{
public:
    RenderTrimesh(const std::string& objpath, const std::string& imgpath = "", bool is_rotate_axis = false);

    virtual ObjectState label() { return ObjectState::OBJECTSTATE_TRIMESH; }
};

class SCENERENDER_EXPORT RenderRectangle :public RenderObject
{
public:
    RenderRectangle(const std::string& imgpath, const CameraD& param, float rectw, float recth, bool is_rotate_axis = false);

    virtual ObjectState label() { return ObjectState::OBJECTSTATE_RECTANGLE; }
};

class SCENERENDER_EXPORT RenderPanorama :public RenderObject
{
public:
    RenderPanorama(const std::string& panopath);

    virtual ObjectState label() { return ObjectState::OBJECTSTATE_PANORAMA; }
};

class SCENERENDER_EXPORT SceneRender
{
public:
    SceneRender(const CameraD& param, int boxwidth, int panowidth, int panoheight);
    ~SceneRender();

    void render(const RenderPanorama& obj);
    void render(const RenderTrimesh& obj, int recordLabel = -1);
    void render(const RenderRectangle& obj, int recordLabel = -1);

    void renderPano(const cv::Mat& panoimage);
    void renderMesh(const TriMesh& mesh, int recordLabel = 0);

    cv::Mat getPanoDepth();
    cv::Mat getPanoRecord();
    cv::Mat getPanoSimulate();
    cv::Mat getSixBoxDepth();
    cv::Mat getSixBoxRecord();
    cv::Mat getSixBoxSimulate();

private:
    int boxwidth_, panowidth_, panoheight_;
    MeshRasterResult result_array_[6];

    cv::Mat K_; // 相机内参
    cv::Mat RT_; // 相机外参数
    SixBox sixbox_;
};

} // namespace Ruler

#endif // _SCENERENDER_SCENERENDER_H