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

#ifndef _SCENERENDER_MESH_H_
#define _SCENERENDER_MESH_H_

#include "defines.h"
#include <vector>
#include <string>

#include "opencv2/opencv.hpp"

namespace Ruler
{

struct Face
{
    unsigned vertices[3], texcoords[3], normals[3];
};

class SCENERENDER_EXPORT TriMesh
{
public:
    typedef unsigned Index;
    typedef std::vector<std::pair<Index, float>> FaceDatas;

    // 改造成公共访问变量
    cv::Mat teximage;
    std::vector<Face> faces;
    std::vector<cv::Point2f> texcoords;
    std::vector<cv::Point3f> vertices, normals;

    TriMesh();
    ~TriMesh();

    /*
    加载OBJ数据，暂不读取材质文件
    */
    bool loadOBJ(const std::string& obj_path, bool is_rotate_axis = false, float scale = 1.0f);
    bool saveOBJ(const std::string& obj_path);
    bool loadTexture(const std::string& tex_path);

    /*
    列出顶点对应的三角面索引
    */
    void listIncidenteFaces();

    /*
    获取邻近面片索引
    */
    void getAdjacencyFaces(Index face_index, std::vector<Index>& adjacency_faces);

    /*
    清空当前加载数据
    */
    void clear();

private:
    TriMesh(TriMesh&) = delete;
    std::vector<std::vector<Index>> vertex_faces_;

}; // class TriMesh


SCENERENDER_EXPORT inline void clearImageOnlyOnes();

SCENERENDER_EXPORT inline void setReadImageOnlyOnes(bool _on);

SCENERENDER_EXPORT inline cv::Mat readImageOnlyOnes(const std::string& _file_name);

} // namespace Ruler

#endif // _RULER_SCENERENDER_MESH_H_