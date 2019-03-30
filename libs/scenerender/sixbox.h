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
#ifndef _SCENERENDER_SIXBOX_H_
#define _SCENERENDER_SIXBOX_H_

#include "defines.h"

namespace Ruler
{

class SCENERENDER_EXPORT SixBox
{
public:
    SixBox();
    ~SixBox();

    // 初始化用于生成六面体到全景的映射关系，计算一次就可以了,其中R_offset用于指出中心偏移
    bool init(int boxwidth, int panowidth, int panoheight, const cv::Mat& R_offset = cv::Mat::eye(3,3,CV_64F));

    // 将六面体转换成全景
    cv::Mat convertSixBoxToPanorama(const cv::Mat& boximage);
    cv::Mat convertSixBoxLabelToPanorama(const cv::Mat& boximage);

    // 将全景转换成六面体,依次为后左前右上下
    cv::Mat convertPanoramaToSixBox(const cv::Mat& panoimage);

private:
    cv::Mat map_sixbox_to_pano_x_, map_sixbox_to_pano_y_; // 六面体到全景的映射
    cv::Mat map_pano_to_sixbox_x_, map_pano_to_sixbox_y_; // 全景到六面体的映射

    int boxwidth_, panowidth_, panoheight_;
};

} // namspace Ruler


#endif // _SCENERENDER_SIXBOX_H_