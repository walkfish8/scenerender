#include "sixbox.h"

void main()
{
	Ruler::SixBox sixbox;
    //cv::Mat R, rvec = (cv::Mat_<double>(3, 1) << 0, CV_PI / 4, 0);
    //cv::Rodrigues(rvec, R);
	sixbox.init(2048, 6000, 3000);
	cv::Mat panoimage = cv::imread("..\\..\\..\\datas\\scene01\\È«¾°Í¼.jpg");

	cv::Mat siximage = sixbox.convertPanoramaToSixBox(panoimage);
	cv::imwrite("..\\..\\..\\datas\\scene01\\out\\six.bmp", siximage);

    cv::Mat panoimage_new = sixbox.convertSixBoxToPanorama(siximage);
    cv::imwrite("..\\..\\..\\datas\\scene01\\out\\pano.bmp", panoimage_new);
}