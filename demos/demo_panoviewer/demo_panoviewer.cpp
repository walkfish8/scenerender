#include "sixbox.h"
#include "panoview.h"

void main(int argc, char* argv[])
{
    //Ruler::SixBox sixbox(4096, 9000, 4500);
    //cv::Mat panoimage = cv::imread("..\\..\\..\\datas\\scene02\\ȫ��ͼ.jpg");
    //Ruler::PanoViewer::instance().show(sixbox.convertPanoramaToSixBox(panoimage), "ȫ��");

    cv::Mat siximage = cv::imread("..\\..\\..\\datas\\scene02\\out\\siximage.bmp");
    Ruler::PanoViewer::instance().show(siximage, "������");
}