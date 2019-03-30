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
#include "scenerender.h"
#include "sixbox.h"
#include "raster.h"
#include "logger.h"

Ruler::RenderTrimesh::RenderTrimesh(const std::string& objpath, const std::string& imgpath, bool is_rotate_axis)
{
    mesh.loadOBJ(objpath, is_rotate_axis);
    mesh.loadTexture(imgpath);
}

Ruler::RenderRectangle::RenderRectangle(const std::string& imgpath, const Ruler::CameraD& param, float rectwx, float recthy, bool is_rotate_axis)
{
    cv::Mat R = param.GetRotationMatrix();
    cv::Mat tvec = param.GetTranslationMatrix();

    const auto& a0 = R.at<double>(0, 0); const auto& a1 = R.at<double>(0, 1); const auto& a2 = R.at<double>(0, 2);
    const auto& b0 = R.at<double>(1, 0); const auto& b1 = R.at<double>(1, 1); const auto& b2 = R.at<double>(1, 2);
    const auto& c0 = R.at<double>(2, 0); const auto& c1 = R.at<double>(2, 1); const auto& c2 = R.at<double>(2, 2);
    const auto& tx = tvec.at<double>(0, 0); const auto& ty = tvec.at<double>(1, 0); const auto& tz = tvec.at<double>(2, 0);

    double x(0), y(0), z(0);
    cv::Point3f point_lt = cv::Point3f(-rectwx / 2.0, recthy / 2.0, 0);
    cv::Point3f point_rt = cv::Point3f(rectwx / 2.0, recthy / 2.0, 0);
    cv::Point3f point_rb = cv::Point3f(rectwx / 2.0, -recthy / 2.0, 0);
    cv::Point3f point_lb = cv::Point3f(-rectwx / 2.0, -recthy / 2.0, 0);

    x = a0*point_lt.x + a1*point_lt.y + a2*point_lt.z + tx;
    y = b0*point_lt.x + b1*point_lt.y + b2*point_lt.z + ty;
    z = c0*point_lt.x + c1*point_lt.y + c2*point_lt.z + tz;
    cv::Point3f point_lt_new = is_rotate_axis ? cv::Point3f(x, -z, y) : cv::Point3f(x, y, z);

    x = a0*point_rt.x + a1*point_rt.y + a2*point_rt.z + tx;
    y = b0*point_rt.x + b1*point_rt.y + b2*point_rt.z + ty;
    z = c0*point_rt.x + c1*point_rt.y + c2*point_rt.z + tz;
    cv::Point3f point_rt_new = is_rotate_axis ? cv::Point3f(x, -z, y) : cv::Point3f(x, y, z);

    x = a0*point_rb.x + a1*point_rb.y + a2*point_rb.z + tx;
    y = b0*point_rb.x + b1*point_rb.y + b2*point_rb.z + ty;
    z = c0*point_rb.x + c1*point_rb.y + c2*point_rb.z + tz;
    cv::Point3f point_rb_new = is_rotate_axis ? cv::Point3f(x, -z, y) : cv::Point3f(x, y, z);

    x = a0*point_lb.x + a1*point_lb.y + a2*point_lb.z + tx;
    y = b0*point_lb.x + b1*point_lb.y + b2*point_lb.z + ty;
    z = c0*point_lb.x + c1*point_lb.y + c2*point_lb.z + tz;
    cv::Point3f point_lb_new = is_rotate_axis ? cv::Point3f(x, -z, y) : cv::Point3f(x, y, z);

    mesh.clear();
    mesh.vertices = std::vector<cv::Point3f>{point_lt_new, point_rt_new, point_rb_new, point_lb_new};
    mesh.texcoords = std::vector<cv::Point2f>{cv::Point2f(1.0, 0.0), cv::Point2f(0.0, 0.0), cv::Point2f(0.0, 1.0), cv::Point2f(1.0, 1.0)};
    mesh.faces.resize(2);
    mesh.faces[0].vertices[0] = mesh.faces[0].texcoords[0] = 0;
    mesh.faces[0].vertices[1] = mesh.faces[0].texcoords[1] = 1;
    mesh.faces[0].vertices[2] = mesh.faces[0].texcoords[2] = 3;
    mesh.faces[1].vertices[0] = mesh.faces[1].texcoords[0] = 1;
    mesh.faces[1].vertices[1] = mesh.faces[1].texcoords[1] = 2;
    mesh.faces[1].vertices[2] = mesh.faces[1].texcoords[2] = 3;

    mesh.loadTexture(imgpath);
}

Ruler::RenderPanorama::RenderPanorama(const std::string& panopath)
{
    mesh.loadTexture(panopath);
}

Ruler::SceneRender::SceneRender(const CameraD& param, int boxwidth, int panowidth, int panoheight)
{
    Ruler::Logger::setLevel(Ruler::SCENERENDER_LOG_INFO);
    Ruler::Logger::info("initialize...\n");
    boxwidth_ = boxwidth;
    panowidth_ = panowidth;
    panoheight_ = panoheight;
    for (int k = 0; k < 6; k++)
    {
        result_array_[k].depth = cv::Mat::zeros(boxwidth_, boxwidth_, CV_32F);
        result_array_[k].simulate = cv::Mat::zeros(boxwidth_, boxwidth_, CV_8UC3);
        result_array_[k].record = -cv::Mat::ones(boxwidth_, boxwidth_, CV_32S);
    }

    float half_boxwidth = (boxwidth - 1.0f) / 2.0f;
    K_ = (cv::Mat_<double>(3, 3) << half_boxwidth, 0, half_boxwidth, 0, half_boxwidth, half_boxwidth, 0, 0, 1);
    RT_ = param.GetRotationAndTranslationMatrix();
    sixbox_ = Ruler::SixBox();
    sixbox_.init(boxwidth_, panowidth_, panoheight_);
    Ruler::Logger::info("initialize finished...\n");
    
}

Ruler::SceneRender::~SceneRender()
{}

cv::Mat Ruler::SceneRender::getPanoDepth()
{
    return std::move(sixbox_.convertSixBoxToPanorama(getSixBoxDepth()));
}

cv::Mat Ruler::SceneRender::getPanoRecord()
{
    return std::move(sixbox_.convertSixBoxLabelToPanorama(getSixBoxRecord()));
}

cv::Mat Ruler::SceneRender::getPanoSimulate()
{
    return std::move(sixbox_.convertSixBoxToPanorama(getSixBoxSimulate()));
}

cv::Mat Ruler::SceneRender::getSixBoxDepth()
{
    cv::Mat sixdepth(boxwidth_, 6 * boxwidth_, CV_16U);
    for (int i = 0; i < 6; i++)
    {
        result_array_[i].depth.convertTo(sixdepth.colRange(i*boxwidth_, (i + 1)*boxwidth_), CV_16U);
    }
    return std::move(sixdepth);
}

cv::Mat Ruler::SceneRender::getSixBoxRecord()
{
    cv::Mat sixrecord(boxwidth_, 6 * boxwidth_, CV_32S);
    for (int i = 0; i < 6; i++)
    {
        result_array_[i].record.convertTo(sixrecord.colRange(i*boxwidth_, (i + 1)*boxwidth_), CV_32S);
    }
    return std::move(sixrecord);
}

cv::Mat Ruler::SceneRender::getSixBoxSimulate()
{
    cv::Mat siximage(boxwidth_, 6 * boxwidth_, CV_8UC3);
    for (int i = 0; i < 6; i++)
    {
        result_array_[i].simulate.convertTo(siximage.colRange(i*boxwidth_, (i + 1)*boxwidth_), CV_8UC3);
    }
    return std::move(siximage);
}

void Ruler::SceneRender::render(const Ruler::RenderTrimesh& obj, int recordLabel)
{
    renderMesh(obj.mesh, obj.mesh.teximage.empty() ? 0 : recordLabel);
}

void Ruler::SceneRender::render(const Ruler::RenderPanorama& obj)
{
    renderPano(obj.mesh.teximage);
}

void Ruler::SceneRender::render(const Ruler::RenderRectangle& obj, int recordLabel)
{
    renderMesh(obj.mesh, obj.mesh.teximage.empty() ? 1 : recordLabel);
}

void Ruler::SceneRender::renderMesh(const Ruler::TriMesh& mesh, int label)
{
    cv::Mat R = RT_.rowRange(0, 3).colRange(0, 3);
    cv::Mat T = RT_.rowRange(0, 3).colRange(3, 4);
    cv::Mat C = -R.t()*T;
    std::vector<cv::Mat> rvecs = {
        (cv::Mat_<double>(3, 1) << 0, 0, 0), (cv::Mat_<double>(3, 1) << 0, CV_PI / 2, 0), (cv::Mat_<double>(3, 1) << 0, CV_PI, 0),
        (cv::Mat_<double>(3, 1) << 0, -CV_PI / 2.0, 0), (cv::Mat_<double>(3, 1) << CV_PI / 2.0, 0, 0), (cv::Mat_<double>(3, 1) << -CV_PI / 2.0, 0, 0) };

    cv::Mat Rr;
    cv::Mat rvec_r = (cv::Mat_<double>(3, 1) << 0, 0, CV_PI);
    cv::Rodrigues(rvec_r, Rr);
    for (int k = 0; k < 6; k++)
    {
        cv::Mat R0;
        cv::Rodrigues(rvecs[k], R0);

        cv::Mat Rc = R0.t()*Rr*R;
        cv::Mat Tc = -Rc*C;
        cv::Mat P = cv::Mat::eye(4, 4, CV_64F);
        Rc.copyTo(P.rowRange(0, 3).colRange(0, 3));
        Tc.copyTo(P.rowRange(0, 3).colRange(3, 4));

        Ruler::MeshRaster::raster(mesh, K_, P, boxwidth_, boxwidth_, result_array_[k], label);
        Ruler::Logger::info("finished %d...\n", k);
    }
}

void Ruler::SceneRender::renderPano(const cv::Mat& panoimage)
{
    assert(panoimage.cols == panowidth_ && panoimage.rows == panoheight_);

    cv::Mat siximage = sixbox_.convertPanoramaToSixBox(panoimage);
    for (int k = 0; k < 6; k++)
    {
        cv::Mat subimage = siximage.colRange(k*boxwidth_, (k + 1)*boxwidth_);
        for (int i = 0; i < boxwidth_; i++)
        {
            const auto& subimage_ptr = subimage.ptr<cv::Vec3b>(i);
            const auto& record_ptr = result_array_[k].record.ptr<int>(i);
            const auto& simulate_ptr = result_array_[k].simulate.ptr<cv::Vec3b>(i);
            for (int j = 0; j < boxwidth_; j++)
            {
                if (record_ptr[j] <= 0)
                    simulate_ptr[j] = subimage_ptr[j];
            }
        }
    }
}