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
#ifndef _SCENERENDER_RASTER_H_
#define _SCENERENDER_RASTER_H_

#include "defines.h"
#include "camera.h"
#include "trimesh.h"

#include "opencv2/opencv.hpp"

namespace Ruler
{

enum class RasterType
{
    RasterType_Depth,
    RasterType_Record,
    RasterType_Simulate
};

struct MeshRasterResult
{
    cv::Mat depth;       // 深度图
    cv::Mat simulate;    // 仿真图
    cv::Mat record;      // 索引图
};

class SCENERENDER_EXPORT MeshRaster
{
public:
    static MeshRaster& instance(); // 单例，用于后续全局参数配置

    static MeshRasterResult raster(const TriMesh& mesh, const cv::Mat& K, const cv::Mat& P, int image_width, int image_height);
    static void raster(const TriMesh& mesh, const cv::Mat& K, const cv::Mat& P, int image_width, int image_height, MeshRasterResult& result, int label = -1);

private:
    MeshRaster();
    ~MeshRaster();
    MeshRaster(MeshRaster&) = delete;
};

} // namespace Ruler


#endif // _SCENERENDER_RASTER_H_