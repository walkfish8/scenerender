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
#include "raster.h"
#include "omp.h"

const float g_max_value = std::numeric_limits<float>::max();

Ruler::MeshRaster::MeshRaster() {}

Ruler::MeshRaster::~MeshRaster() {}

Ruler::MeshRaster& Ruler::MeshRaster::instance()
{
    static Ruler::MeshRaster mesh_raster;
    return mesh_raster;
}

void Ruler::MeshRaster::raster(const Ruler::TriMesh& mesh, const cv::Mat& K, const cv::Mat& P, int image_width, int image_height, MeshRasterResult& result, int label)
{
    cv::Mat R = P.rowRange(0, 3).colRange(0, 3);
    cv::Mat tvec = P.rowRange(0, 3).colRange(3, 4);
    cv::Mat C = -R.t()*tvec;
    cv::Point3f P0(C); // 中心点

    bool is_overlapped = true;
    if (result.record.empty() || result.depth.empty() || result.simulate.empty())
    {
        is_overlapped = false;
        result.depth = cv::Mat::zeros(cv::Size(image_width, image_height), CV_32F);
        result.record = -cv::Mat::ones(cv::Size(image_width, image_height), CV_32S);
        result.simulate = cv::Mat::zeros(cv::Size(image_width, image_height), CV_8UC3);
    }

    // 用于映射纹理
    cv::Mat map_x(cv::Size(image_width, image_height), CV_32F, -1);
    cv::Mat map_y(cv::Size(image_width, image_height), CV_32F, -1);

    float fx = static_cast<float>(K.at<double>(0, 0));
    float fy = static_cast<float>(K.at<double>(1, 1));
    float cx = static_cast<float>(K.at<double>(0, 2));
    float cy = static_cast<float>(K.at<double>(1, 2));
    float a0 = static_cast<float>(R.at<double>(0, 0));
    float a1 = static_cast<float>(R.at<double>(0, 1));
    float a2 = static_cast<float>(R.at<double>(0, 2));
    float b0 = static_cast<float>(R.at<double>(1, 0)); 
    float b1 = static_cast<float>(R.at<double>(1, 1)); 
    float b2 = static_cast<float>(R.at<double>(1, 2));
    float c0 = static_cast<float>(R.at<double>(2, 0));
    float c1 = static_cast<float>(R.at<double>(2, 1));
    float c2 = static_cast<float>(R.at<double>(2, 2));
    float tx = static_cast<float>(tvec.at<double>(0, 0));
    float ty = static_cast<float>(tvec.at<double>(1, 0));
    float tz = static_cast<float>(tvec.at<double>(2, 0));

    std::vector<cv::Point3f> xyz_points(mesh.vertices.size());
    std::vector<bool> image_points_flags(mesh.vertices.size());
    std::vector<cv::Point2f> image_points(mesh.vertices.size());
    for (int i = 0; i < mesh.vertices.size(); i++)
    {
        const cv::Point3f& pt = mesh.vertices[i];
        float x = a0*pt.x + a1*pt.y + a2*pt.z + tx;
        float y = b0*pt.x + b1*pt.y + b2*pt.z + ty;
        float z = c0*pt.x + c1*pt.y + c2*pt.z + tz;

        xyz_points[i].x = x;
        xyz_points[i].y = y;
        xyz_points[i].z = z;
        image_points[i].x = x*fx / z + cx;
        image_points[i].y = y*fy / z + cy;
        image_points_flags[i] = z > 0;
    }

#pragma omp parallel for
    for (int face_index = 0; face_index < mesh.faces.size(); face_index++)
    {
        const unsigned& id1 = mesh.faces[face_index].vertices[0];
        const unsigned& id2 = mesh.faces[face_index].vertices[1];
        const unsigned& id3 = mesh.faces[face_index].vertices[2];
        const unsigned& uv_id1 = mesh.faces[face_index].texcoords[0];
        const unsigned& uv_id2 = mesh.faces[face_index].texcoords[1];
        const unsigned& uv_id3 = mesh.faces[face_index].texcoords[2];

        const cv::Point3f& mp0 = mesh.vertices[id1];
        const cv::Point3f& mp1 = mesh.vertices[id2];
        const cv::Point3f& mp2 = mesh.vertices[id3];
        const cv::Point2f& imgp1 = image_points[id1];
        const cv::Point2f& imgp2 = image_points[id2];
        const cv::Point2f& imgp3 = image_points[id3];

        // 判断跨相平面三角面
        int minx(-1), miny(-1), maxx(-1), maxy(-1);
        int flag_number = image_points_flags[id1] + image_points_flags[id2] + image_points_flags[id3];

        if (flag_number == 3)
        {
            minx = static_cast<int>(std::max(std::min(std::min(imgp1.x, imgp2.x), imgp3.x), 0.0f));
            miny = static_cast<int>(std::max(std::min(std::min(imgp1.y, imgp2.y), imgp3.y), 0.0f));
            maxx = static_cast<int>(std::min(std::max(std::max(imgp1.x, imgp2.x), imgp3.x), image_width - 1.0f));
            maxy = static_cast<int>(std::min(std::max(std::max(imgp1.y, imgp2.y), imgp3.y), image_height - 1.0f));
        }
        else if (flag_number == 2)
        {
            // 这里粗暴的采用全局搜索，后续改造成判断交叉点进行处理
            unsigned int ids[3] = { id1, id2, id3 };
            int invalid_ind = !image_points_flags[id1] ? 0 : !image_points_flags[id2] ? 1 : 2;
            int valid_ind0 = (invalid_ind + 1) % 3;
            int valid_ind1 = (invalid_ind + 2) % 3;

            const cv::Point3f& invalid_pt = xyz_points[ids[invalid_ind]];
            const cv::Point3f& valid_pt0 = xyz_points[ids[valid_ind0]];
            const cv::Point3f& valid_pt1 = xyz_points[ids[valid_ind1]];

            cv::Point3f v0 = invalid_pt - valid_pt0;
            cv::Point3f v1 = invalid_pt - valid_pt1;

            cv::Point3f intersect_pt0 = (-valid_pt0.z / v0.z)*v0 + valid_pt0;
            cv::Point3f intersect_pt1 = (-valid_pt1.z / v1.z)*v1 + valid_pt1;

            cv::Point2f intersect_uv0 = cv::Point2f(intersect_pt0.x > 0 ? g_max_value : -g_max_value, intersect_pt0.y > 0 ? g_max_value : -g_max_value);
            cv::Point2f intersect_uv1 = cv::Point2f(intersect_pt1.x > 0 ? g_max_value : -g_max_value, intersect_pt1.y > 0 ? g_max_value : -g_max_value);
            const cv::Point2f& intersect_uv2 = image_points[ids[valid_ind0]];
            const cv::Point2f& intersect_uv3 = image_points[ids[valid_ind1]];

            minx = static_cast<int>(std::max(std::min(std::min(std::min(intersect_uv0.x, intersect_uv1.x), intersect_uv2.x), intersect_uv3.x), 0.0f));
            miny = static_cast<int>(std::max(std::min(std::min(std::min(intersect_uv0.y, intersect_uv1.y), intersect_uv2.y), intersect_uv3.y), 0.0f));
            maxx = static_cast<int>(std::min(std::max(std::max(std::max(intersect_uv0.x, intersect_uv1.x), intersect_uv2.x), intersect_uv3.x), image_width - 1.0f));
            maxy = static_cast<int>(std::min(std::max(std::max(std::max(intersect_uv0.y, intersect_uv1.y), intersect_uv2.y), intersect_uv3.y), image_height - 1.0f));

        }
        else if (flag_number == 1)
        {
            // 这里粗暴的采用全局搜索，后续改造成判断交叉点进行处理
            unsigned int ids[3] = { id1, id2, id3 };
            int valid_ind = image_points_flags[id1] ? 0 : image_points_flags[id2] ? 1 : 2;
            int invalid_ind0 = (valid_ind + 1) % 3;
            int invalid_ind1 = (valid_ind + 2) % 3;

            const cv::Point3f& valid_pt = xyz_points[ids[valid_ind]];
            const cv::Point3f& invalid_pt0 = xyz_points[ids[invalid_ind0]];
            const cv::Point3f& invalid_pt1 = xyz_points[ids[invalid_ind1]];

            cv::Point3f v0 = invalid_pt0 - valid_pt;
            cv::Point3f v1 = invalid_pt1 - valid_pt;

            cv::Point3f intersect_pt0 = (-valid_pt.z / v0.z)*v0 + valid_pt;
            cv::Point3f intersect_pt1 = (-valid_pt.z / v1.z)*v1 + valid_pt;

            cv::Point2f intersect_uv0 = cv::Point2f(intersect_pt0.x > 0 ? g_max_value : -g_max_value, intersect_pt0.y > 0 ? g_max_value : -g_max_value);
            cv::Point2f intersect_uv1 = cv::Point2f(intersect_pt1.x > 0 ? g_max_value : -g_max_value, intersect_pt1.y > 0 ? g_max_value : -g_max_value);
            const cv::Point2f& intersect_uv2 = image_points[ids[valid_ind]];

            minx = static_cast<int>(std::max(std::min(std::min(intersect_uv0.x, intersect_uv1.x), intersect_uv2.x), 0.0f));
            miny = static_cast<int>(std::max(std::min(std::min(intersect_uv0.y, intersect_uv1.y), intersect_uv2.y), 0.0f));
            maxx = static_cast<int>(std::min(std::max(std::max(intersect_uv0.x, intersect_uv1.x), intersect_uv2.x), image_width - 1.0f));
            maxy = static_cast<int>(std::min(std::max(std::max(intersect_uv0.y, intersect_uv1.y), intersect_uv2.y), image_height - 1.0f));
        }
        else
        {
            // do nothing!
        }

        cv::Point3f mv0 = mp2 - mp0;
        cv::Point3f mv1 = mp1 - mp0;
        cv::Point3f mnormal = mv0.cross(mv1);

        float dot00 = mv0.dot(mv0);
        float dot01 = mv0.dot(mv1);
        float dot11 = mv1.dot(mv1);
        for (int i = minx; i <= maxx; i++)
        {
            for (int j = miny; j <= maxy; j++)
            {
                cv::Point3f ray0 = cv::Point3f((i - cx) / fx, (j - cy) / fy, 1);
                cv::Point3f ray = cv::Point3f(a0*ray0.x + b0*ray0.y + c0*ray0.z, a1*ray0.x + b1*ray0.y + c1*ray0.z, a2*ray0.x + b2*ray0.y + c2*ray0.z);

                float s = mnormal.dot(mp0 - P0) / mnormal.dot(ray);
                cv::Point3f pt = s*ray + cv::Point3f(P0);
                cv::Point3f mv2 = pt - cv::Point3f(mp0);

                float dot02 = mv0.dot(mv2);
                float dot12 = mv1.dot(mv2);

                float invDenom = 1.0f / (dot00*dot11 - dot01*dot01);
                float u = (dot11*dot02 - dot01*dot12) * invDenom;
                float v = (dot00*dot12 - dot01*dot02) * invDenom;
                // 判断此点是否在三角形中
                if (u >= 0 && v >= 0 && u + v <= 1)
                {
                    float d = c0*pt.x + c1*pt.y + c2*pt.z + tz;
                    if (d > 0 && (d < result.depth.at<float>(j, i) || result.depth.at<float>(j, i) <= 0))
                    {
                        result.depth.at<float>(j, i) = d;
                        result.record.at<int>(j, i) = label == -1 ? face_index : label;

                        auto pointUV = (mesh.texcoords[uv_id3] - mesh.texcoords[uv_id1])*u
                            + (mesh.texcoords[uv_id2] - mesh.texcoords[uv_id1])*v + mesh.texcoords[uv_id1];
                        map_x.at<float>(j, i) = pointUV.x*mesh.teximage.cols;
                        map_y.at<float>(j, i) = (1 - pointUV.y)*mesh.teximage.rows;
                    }
                }

            }
        }
    }

    if (!mesh.teximage.empty())
    {
        if (is_overlapped)
        {
            cv::Mat remapimage;
            cv::remap(mesh.teximage, remapimage, map_x, map_y, cv::INTER_LINEAR);
            for (int i = 0; i < image_height; i++)
            {
                const auto& map_x_ptr = map_x.ptr<float>(i);
                const auto& map_y_ptr = map_y.ptr<float>(i);
                const auto& remapimage_ptr = remapimage.ptr<cv::Vec3b>(i);
                const auto& simulate_ptr = result.simulate.ptr<cv::Vec3b>(i);
                for (int j = 0; j < image_width; j++)
                {
                    if (map_x_ptr[j] >= 0 && map_y_ptr[j] >= 0 && map_x_ptr[j] <= mesh.teximage.cols && map_y_ptr[j] <= mesh.teximage.rows)
                    {
                        simulate_ptr[j] = remapimage_ptr[j];
                    }
                }
            }
        }
        else
        {
            cv::remap(mesh.teximage, result.simulate, map_x, map_y, cv::INTER_LINEAR);
        }
    }
}

Ruler::MeshRasterResult Ruler::MeshRaster::raster(const Ruler::TriMesh& mesh, const cv::Mat& K, const cv::Mat& P, int image_width, int image_height)
{
    MeshRasterResult result;
    raster(mesh, K, P, image_width, image_height, result);

    return std::move(result);
}