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
#include "sixbox.h"
#include "logger.h"
#include "omp.h"

Ruler::SixBox::SixBox()
{}

Ruler::SixBox::~SixBox()
{}

Ruler::SixBox::SixBox(int boxwidth, int panowidth, int panoheight, const cv::Mat& R_offset)
{
    init(boxwidth, panowidth, panoheight, R_offset);
}

// 右手坐标系
bool Ruler::SixBox::init(int boxwidth, int panowidth, int panoheight, const cv::Mat& R_offset)
{
    assert(panoheight * 2 == panowidth && panoheight > 0);

    boxwidth_ = boxwidth;
    panowidth_ = panowidth;
    panoheight_ = panoheight;

    // 计算六面体到全景的映射
    map_sixbox_to_pano_x_.create(boxwidth, 6 * boxwidth, CV_32F);
    map_sixbox_to_pano_y_.create(boxwidth, 6 * boxwidth, CV_32F);

    // 匿名函数，将XYZ向量转换成角度表示
    auto convertXYZtoAngle = [](float x0, float y0, float z0, const cv::Mat& R0)->cv::Point2f
    {
        float x = R0.at<double>(0, 0)*x0 + R0.at<double>(0, 1)*y0 + R0.at<double>(0, 2)*z0;
        float y = R0.at<double>(1, 0)*x0 + R0.at<double>(1, 1)*y0 + R0.at<double>(1, 2)*z0;
        float z = R0.at<double>(2, 0)*x0 + R0.at<double>(2, 1)*y0 + R0.at<double>(2, 2)*z0;

        float r = sqrt(x*x + y*y + z*z);
        return cv::Point2f(atan2f(x, z) + CV_PI, acosf(y / r));
    };

    //std::vector<cv::Mat> rvecs = {
    //    (cv::Mat_<double>(3, 1) << 0, 0, 0), (cv::Mat_<double>(3, 1) << 0, CV_PI / 2.0, 0), (cv::Mat_<double>(3, 1) << 0, CV_PI, 0),
    //    (cv::Mat_<double>(3, 1) << 0, -CV_PI / 2.0, 0), (cv::Mat_<double>(3, 1) << -CV_PI / 2.0, 0, 0), (cv::Mat_<double>(3, 1) << CV_PI / 2.0, 0, 0) };
    std::vector<cv::Mat> rotate_mats = 
    {
        (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1),
        (cv::Mat_<double>(3, 3) << 0, 0, 1, 0, 1, 0, -1, 0, 0),
        (cv::Mat_<double>(3, 3) << -1, 0, 0, 0, 1, 0, 0, 0, -1),
        (cv::Mat_<double>(3, 3) << 0, 0, -1, 0, 1, 0, 1, 0, 0),
        (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 0, 1, 0, -1, 0),
        (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 0, -1, 0, 1, 0) 
    };

    // 冗余一个像素，保持边界过度的平滑
    float boxwidth_cxy = (boxwidth - 1) / 2.0, boxwidth_fxy = (boxwidth - 1) / 2.0;;
    float pixel_per_angle = panoheight / CV_PI, angle_per_pixel = CV_PI / panoheight;

#pragma omp parallel for
    for (int k = 0; k < 6; k++)
    {
        cv::Mat R = R_offset*rotate_mats[k];
        for (int i = 0; i < boxwidth; i++)
        {
            for (int j = 0; j < boxwidth; j++)
            {
                auto angle = convertXYZtoAngle(i - boxwidth_cxy, boxwidth_cxy - j, boxwidth_fxy, R);
                float panox = angle.x*pixel_per_angle;
                float panoy = angle.y*pixel_per_angle;

                map_sixbox_to_pano_x_.at<float>(j, i + k*boxwidth) = panox;
                map_sixbox_to_pano_y_.at<float>(j, i + k*boxwidth) = panoy;
            }
        }
    }

    // 计算全景到六面体的映射
    map_pano_to_sixbox_x_ = -cv::Mat::ones(panoheight, panowidth, CV_32F);
    map_pano_to_sixbox_y_ = -cv::Mat::ones(panoheight, panowidth, CV_32F);

    float pano_cx = (panowidth - 1) / 2.0;
    float pano_cy = (panoheight - 1) / 2.0;

#pragma omp parallel for
    for (int k = 0; k < 6; k++)
    {
        cv::Mat R = (R_offset*rotate_mats[k]).t();
        const auto& a0 = R.at<double>(0, 0); const auto& a1 = R.at<double>(0, 1); const auto& a2 = R.at<double>(0, 2);
        const auto& b0 = R.at<double>(1, 0); const auto& b1 = R.at<double>(1, 1); const auto& b2 = R.at<double>(1, 2);
        const auto& c0 = R.at<double>(2, 0); const auto& c1 = R.at<double>(2, 1); const auto& c2 = R.at<double>(2, 2);

        for (int j = 0; j < panoheight; j++)
        {
            for (int i = 0; i < panowidth; i++)
            {
                float alpha = (i - pano_cx)*angle_per_pixel;
                float beta = (pano_cy - j)*angle_per_pixel;

                float x0 = cos(beta)*sin(alpha);
                float y0 = sin(beta);
                float z0 = cos(beta)*cos(alpha);

                float x = a0*x0 + a1*y0 + a2*z0;
                float y = b0*x0 + b1*y0 + b2*z0;
                float z = c0*x0 + c1*y0 + c2*z0;

                if (z > 0 && abs(x / z) < 1.0 && abs(y / z) < 1.0) // 表明同向，可能存在交点
                {
                    float u = x*boxwidth_fxy / z + boxwidth_cxy;
                    float v = boxwidth_cxy - y*boxwidth_fxy;
                    if (u >= 0 && u < boxwidth && v >= 0 && v < boxwidth)
                    {
                        map_pano_to_sixbox_x_.at<float>(j, i) = x*boxwidth_fxy / z + boxwidth_cxy + k*boxwidth;
                        map_pano_to_sixbox_y_.at<float>(j, i) = boxwidth_cxy - y*boxwidth_fxy / z;
                    }
                }
            }
        }
    }

    return true;
}

// 将六面体转换成全景
cv::Mat Ruler::SixBox::convertSixBoxToPanorama(const cv::Mat& boximage)
{
    assert(boximage.cols == 6 * boxwidth_ && boximage.rows == boxwidth_);

    cv::Mat panoimage;
    cv::remap(boximage, panoimage, map_pano_to_sixbox_x_, map_pano_to_sixbox_y_, CV_INTER_LINEAR);
    return std::move(panoimage);
}

cv::Mat Ruler::SixBox::convertSixBoxLabelToPanorama(const cv::Mat& boximage)
{
    assert(boximage.cols == 6 * boxwidth_ && boximage.rows == boxwidth_);

    int sixboxlength = boxwidth_ * 6;
    cv::Mat panoimage = -cv::Mat::ones(map_pano_to_sixbox_x_.size(), CV_32S);
    for (int j = 0; j < map_pano_to_sixbox_x_.rows; j++)
    {
        for (int i = 0; i < map_pano_to_sixbox_x_.cols; i++)
        {
            float x = map_pano_to_sixbox_x_.at<float>(j, i);
            float y = map_pano_to_sixbox_y_.at<float>(j, i);

            int u = int(x + 0.5);
            int v = int(y + 0.5);
            if (u >= 0 && u < sixboxlength && v >= 0 && v < boxwidth_)
            {
                panoimage.at<int>(j, i) = boximage.at<int>(v, u);
            }
            else
            {
                int u = int(x);
                int v = int(y);
                panoimage.at<int>(j, i) = boximage.at<int>(v, u);
            }
        }
    }
    //cv::remap(boximage, panoimage, map_pano_to_sixbox_x_, map_pano_to_sixbox_y_, CV_INTER_LINEAR);
    return std::move(panoimage);
}

// 将全景转换成六面体
cv::Mat Ruler::SixBox::convertPanoramaToSixBox(const cv::Mat& panoimage)
{
    assert(panoimage.cols == panowidth_ && panoimage.rows == panoheight_);

    cv::Mat boximage;
    cv::remap(panoimage, boximage, map_sixbox_to_pano_x_, map_sixbox_to_pano_y_, CV_INTER_LINEAR);
    return std::move(boximage);
}