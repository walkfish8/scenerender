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
#include "timer.h"
#include "logger.h"
#include "camera.h"
#include "trimesh.h"
#include "raster.h"
#include "sixbox.h"

namespace Ruler
{

class SceneRenderImpl
{
public:
    SceneRenderImpl(const CameraD& param, int boxwidth, int panowidth, int panoheight);
    ~SceneRenderImpl();

    void clearDepthAndImage();

    // 重设相机参数
    void setCameraParam(const CameraD& param);

    void renderSix(const cv::Mat& siximage);
	void renderSix(const cv::Mat siximage[6]);
    void renderPano(const cv::Mat& panoimage);
    void renderMesh(const TriMesh& mesh, int recordLabel = 0);

    cv::Mat getPanoDepth(float scale = 1.0f);
    cv::Mat getPanoRecord();
    cv::Mat getPanoSimulate();
    cv::Mat getSixBoxDepth(float scale = 1.0f);
    cv::Mat getSixBoxDistance(float scale = 1.0f);
    cv::Mat getSixBoxRecord();
    cv::Mat getSixBoxSimulate();

private:
    int boxwidth_, panowidth_, panoheight_;
    MeshRasterResult result_array_[6];

    cv::Mat K_; // 相机内参
    cv::Mat RT_; // 相机外参数
    cv::Mat rotate_mats_[6];

    SixBox sixbox_;
};


SceneRender::SceneRender(const CameraD& param, int boxwidth, int panowidth, int panoheight)
    : impl_ptr_(new SceneRenderImpl(param, boxwidth, panowidth, panoheight))
{}

SceneRender::~SceneRender() { delete impl_ptr_; }

void SceneRender::clearDepthAndImage()
{
    impl_ptr_->clearDepthAndImage();
}

void SceneRender::setCameraParam(const CameraD& param)
{
    impl_ptr_->setCameraParam(param);
}

void SceneRender::renderSixBox(const char* sixpath)
{
    impl_ptr_->renderSix(cv::imread(sixpath));
}

void SceneRender::renderSixBox(const char* sixpath[6])
{
	cv::Mat siximage[6];
	for (int i = 0; i < 6; ++i)
		siximage[i] = cv::imread(sixpath[i]);

	impl_ptr_->renderSix(siximage);
}

void SceneRender::renderPanorama(const char* panopath)
{
    impl_ptr_->renderPano(cv::imread(panopath));
}

void SceneRender::renderTrimesh(const char* objpath, const char* imgpath, int record_label, bool is_rotate_axis, float scale)
{
    Ruler::TriMesh mesh;
    mesh.loadOBJ(objpath, is_rotate_axis, scale);
    mesh.loadTexture(imgpath);
    impl_ptr_->renderMesh(mesh, record_label);
}

void SceneRender::renderRectangle(const char* imgpath, const CameraD& param, float rectw, float recth, int record_label, bool is_rotate_axis)
{
    Ruler::TriMesh mesh;

    float a0 = param.m[0][0]; float a1 = param.m[0][1]; float a2 = param.m[0][2];
    float b0 = param.m[1][0]; float b1 = param.m[1][1]; float b2 = param.m[1][2];
    float c0 = param.m[2][0]; float c1 = param.m[2][1]; float c2 = param.m[2][2];
    float tx = param.t[0]; float ty = param.t[1]; float tz = param.t[2];

    float x(0), y(0), z(0);
    cv::Point3f point_lt = cv::Point3f(-rectw / 2.0f, recth / 2.0f, 1e-5);
    cv::Point3f point_rt = cv::Point3f(rectw / 2.0f, recth / 2.0f, 1e-5);
    cv::Point3f point_rb = cv::Point3f(rectw / 2.0f, -recth / 2.0f, 1e-5);
    cv::Point3f point_lb = cv::Point3f(-rectw / 2.0f, -recth / 2.0f, 1e-5);

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
    mesh.texcoords = std::vector<cv::Point2f>{cv::Point2f(0.0, 1.0), cv::Point2f(1.0, 1.0), cv::Point2f(1.0, 0.0), cv::Point2f(0.0, 0.0)};
    mesh.faces.resize(2);
    mesh.faces[0].vertices[0] = mesh.faces[0].texcoords[0] = 0;
    mesh.faces[0].vertices[1] = mesh.faces[0].texcoords[1] = 1;
    mesh.faces[0].vertices[2] = mesh.faces[0].texcoords[2] = 3;
    mesh.faces[1].vertices[0] = mesh.faces[1].texcoords[0] = 1;
    mesh.faces[1].vertices[1] = mesh.faces[1].texcoords[1] = 2;
    mesh.faces[1].vertices[2] = mesh.faces[1].texcoords[2] = 3;

    if(mesh.loadTexture(imgpath))
        impl_ptr_->renderMesh(mesh, record_label);
}

void SceneRender::savePanoDepthImage(const char* imgpath, float scale)
{
    cv::imwrite(imgpath, impl_ptr_->getPanoDepth(scale));
}

void SceneRender::savePanoRecordImage(const char* imgpath)
{
    cv::imwrite(imgpath, impl_ptr_->getPanoRecord());
}

void SceneRender::savePanoSimulateImage(const char* imgpath)
{
    cv::imwrite(imgpath, impl_ptr_->getPanoSimulate());
}

void SceneRender::saveSixBoxDepthImage(const char* imgpath, float scale)
{
    cv::imwrite(imgpath, impl_ptr_->getSixBoxDepth(scale));
}

void SceneRender::saveSixBoxRecordImage(const char* imgpath)
{
    cv::imwrite(imgpath, impl_ptr_->getSixBoxRecord());
}

void SceneRender::saveSixBoxSimulateImage(const char* imgpath)
{
    cv::imwrite(imgpath, impl_ptr_->getSixBoxSimulate());
}

//void SceneRender::showPanoSimulateWithOpenGL()
//{
//    Ruler::PanoViewer::instance().show(impl_ptr_->getSixBoxSimulate(), "Panorama Viewer");
//}

} // namespace Ruler

Ruler::SceneRenderImpl::SceneRenderImpl(const CameraD& param, int boxwidth, int panowidth, int panoheight) 
    : boxwidth_(boxwidth), panowidth_(panowidth), panoheight_(panoheight), sixbox_(boxwidth, panowidth, panoheight)
{
    Ruler::Timer::tic();
    Ruler::Logger::setLevel(Ruler::SCENERENDER_LOG_DEBUG);
    Ruler::Logger::info("initialize...\n");

    std::vector<cv::Mat> rvecs = {
        (cv::Mat_<double>(3, 1) << 0, 0, 0), (cv::Mat_<double>(3, 1) << 0, CV_PI / 2, 0), (cv::Mat_<double>(3, 1) << 0, CV_PI, 0),
        (cv::Mat_<double>(3, 1) << 0, -CV_PI / 2.0, 0), (cv::Mat_<double>(3, 1) << CV_PI / 2.0, 0, 0), (cv::Mat_<double>(3, 1) << -CV_PI / 2.0, 0, 0) };

    cv::Mat Rr = (cv::Mat_<double>(3, 3) << -1, 0, 0, 0, -1, 0, 0, 0, 1); //相平面坐标系定义引起的旋转
    cv::Mat Ro = (cv::Mat_<double>(3, 3) << 0, 1, 0, 0, 0, 1, 1, 0, 0); // 全景坐标系定义引起的旋转
    for (int k = 0; k < 6; k++)
    {
        cv::Mat R0;
        cv::Rodrigues(rvecs[k], R0); 
        rotate_mats_[k] = Rr*R0*Ro;

        result_array_[k].depth = cv::Mat::zeros(boxwidth, boxwidth, CV_32F);
        result_array_[k].record = -cv::Mat::ones(boxwidth, boxwidth, CV_32S);
        result_array_[k].simulate = cv::Mat::zeros(boxwidth, boxwidth, CV_8UC3);
    }

    float half_boxwidth = (boxwidth - 1.0f) / 2.0f;
    K_ = (cv::Mat_<double>(3, 3) << half_boxwidth, 0, half_boxwidth, 0, half_boxwidth, half_boxwidth, 0, 0, 1);

    setCameraParam(param);
    Ruler::Logger::info("initialize finished..., elapsed %fs\n", Ruler::Timer::toc());
}

Ruler::SceneRenderImpl::~SceneRenderImpl()
{}

void Ruler::SceneRenderImpl::clearDepthAndImage()
{
    for (int k = 0; k < 6; k++)
    {
        result_array_[k].depth = cv::Mat::zeros(boxwidth_, boxwidth_, CV_32F);
        result_array_[k].record = -cv::Mat::ones(boxwidth_, boxwidth_, CV_32S);
        result_array_[k].simulate = cv::Mat::zeros(boxwidth_, boxwidth_, CV_8UC3);
    }
}

void Ruler::SceneRenderImpl::setCameraParam(const CameraD& param)
{
    RT_ = cv::Mat::eye(4, 4, CV_64F);
    RT_.at<double>(0, 0) = param.m[0][0]; RT_.at<double>(0, 1) = param.m[0][1]; RT_.at<double>(0, 2) = param.m[0][2]; RT_.at<double>(0, 3) = param.t[0];
    RT_.at<double>(1, 0) = param.m[1][0]; RT_.at<double>(1, 1) = param.m[1][1]; RT_.at<double>(1, 2) = param.m[1][2]; RT_.at<double>(1, 3) = param.t[1];
    RT_.at<double>(2, 0) = param.m[2][0]; RT_.at<double>(2, 1) = param.m[2][1]; RT_.at<double>(2, 2) = param.m[2][2]; RT_.at<double>(2, 3) = param.t[2];
    RT_ = RT_.inv();
}

cv::Mat Ruler::SceneRenderImpl::getPanoDepth(float scale)
{
    return std::move(sixbox_.convertSixBoxToPanorama(getSixBoxDistance(scale)));
}

cv::Mat Ruler::SceneRenderImpl::getPanoRecord()
{
    return std::move(sixbox_.convertSixBoxLabelToPanorama(getSixBoxRecord()));
}

cv::Mat Ruler::SceneRenderImpl::getPanoSimulate()
{
    return std::move(sixbox_.convertSixBoxToPanorama(getSixBoxSimulate()));
}

cv::Mat Ruler::SceneRenderImpl::getSixBoxDepth(float scale)
{
    cv::Mat sixdepth(boxwidth_, 6 * boxwidth_, CV_16U);
    for (int i = 0; i < 6; i++)
    {
        result_array_[i].depth.convertTo(sixdepth.colRange(i*boxwidth_, (i + 1)*boxwidth_), CV_16U, scale);
    }
    return std::move(sixdepth);
}

cv::Mat Ruler::SceneRenderImpl::getSixBoxDistance(float scale)
{
    cv::Mat sixdepth(boxwidth_, 6 * boxwidth_, CV_16U);
    for (int i = 0; i < 6; i++)
    {
        sixbox_.convertBoxDepthToDistance(result_array_[i].depth, i).convertTo(sixdepth.colRange(i*boxwidth_, (i + 1)*boxwidth_), CV_16U, scale);
    }
    return std::move(sixdepth);
}

cv::Mat Ruler::SceneRenderImpl::getSixBoxRecord()
{
    cv::Mat sixrecord(boxwidth_, 6 * boxwidth_, CV_32S);
    for (int i = 0; i < 6; i++)
    {
        result_array_[i].record.convertTo(sixrecord.colRange(i*boxwidth_, (i + 1)*boxwidth_), CV_32S);
    }
    return std::move(sixrecord);
}

cv::Mat Ruler::SceneRenderImpl::getSixBoxSimulate()
{
    cv::Mat siximage(boxwidth_, 6 * boxwidth_, CV_8UC3);
    for (int i = 0; i < 6; i++)
    {
        result_array_[i].simulate.convertTo(siximage.colRange(i*boxwidth_, (i + 1)*boxwidth_), CV_8UC3);
    }
    return std::move(siximage);
}

void Ruler::SceneRenderImpl::renderMesh(const Ruler::TriMesh& mesh, int label)
{
    Ruler::Timer::tic();
    cv::Mat R = RT_.rowRange(0, 3).colRange(0, 3);
    cv::Mat T = RT_.rowRange(0, 3).colRange(3, 4);
    cv::Mat C = -R.t()*T;

#pragma omp parallel for
    for (int k = 0; k < 6; k++)
    {
        cv::Mat Rc = rotate_mats_[k]*R;
        cv::Mat Tc = -Rc*C;
        cv::Mat P = cv::Mat::eye(4, 4, CV_64F);
        Rc.copyTo(P.rowRange(0, 3).colRange(0, 3));
        Tc.copyTo(P.rowRange(0, 3).colRange(3, 4));

        Ruler::MeshRaster::raster(mesh, K_, P, boxwidth_, boxwidth_, result_array_[k], label);
    }
    Ruler::Logger::info("renderMesh finished..., elapsed %fs\n", Ruler::Timer::toc());
}

void Ruler::SceneRenderImpl::renderPano(const cv::Mat& panoimage)
{
    assert(panoimage.cols == panowidth_ && panoimage.rows == panoheight_);
    cv::Mat siximage = sixbox_.convertPanoramaToSixBox(panoimage);

#pragma omp parallel for
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

void Ruler::SceneRenderImpl::renderSix(const cv::Mat& siximage)
{
    assert(siximage.cols == 6 * boxwidth_ && siximage.rows == boxwidth_);

#pragma omp parallel for
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

void Ruler::SceneRenderImpl::renderSix(const cv::Mat siximage[6])
{
#pragma omp parallel for
	for (int k = 0; k < 6; k++)
	{
		if (siximage[k].cols == boxwidth_ && siximage[k].rows == boxwidth_)
		{
			const cv::Mat& subimage = siximage[k];
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
		else
		{
			Ruler::Logger::error("size of image for index %d wrong!\n", k);
		}
	}
}