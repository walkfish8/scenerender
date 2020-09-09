#include "sixbox.h"
#include "panoview.h"

void main(int argc, char* argv[])
{
    //Ruler::SixBox sixbox(4096, 9000, 4500);
    //cv::Mat panoimage = cv::imread("..\\..\\..\\datas\\scene02\\全景图.jpg");
    //Ruler::PanoViewer::instance().show(sixbox.convertPanoramaToSixBox(panoimage), "全景");

    cv::Mat siximage = cv::imread("..\\..\\..\\datas\\scene02\\out\\siximage.bmp");
    Ruler::PanoViewer::instance().show(siximage, "六面体");
}