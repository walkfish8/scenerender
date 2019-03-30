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

    // ��ʼ���������������嵽ȫ����ӳ���ϵ������һ�ξͿ�����,����R_offset����ָ������ƫ��
    bool init(int boxwidth, int panowidth, int panoheight, const cv::Mat& R_offset = cv::Mat::eye(3,3,CV_64F));

    // ��������ת����ȫ��
    cv::Mat convertSixBoxToPanorama(const cv::Mat& boximage);
    cv::Mat convertSixBoxLabelToPanorama(const cv::Mat& boximage);

    // ��ȫ��ת����������,����Ϊ����ǰ������
    cv::Mat convertPanoramaToSixBox(const cv::Mat& panoimage);

private:
    cv::Mat map_sixbox_to_pano_x_, map_sixbox_to_pano_y_; // �����嵽ȫ����ӳ��
    cv::Mat map_pano_to_sixbox_x_, map_pano_to_sixbox_y_; // ȫ�����������ӳ��

    int boxwidth_, panowidth_, panoheight_;
};

} // namspace Ruler


#endif // _SCENERENDER_SIXBOX_H_