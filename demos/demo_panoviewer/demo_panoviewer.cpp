#include "sixbox.h"
#include "panoview.h"

void main(int argc, char* argv[])
{
    Ruler::SixBox sixbox(2048, 6000, 3000);
    cv::Mat panoimage = cv::imread("..\\..\\..\\datas\\scene01\\È«¾°Í¼.jpg");
    Ruler::PanoViewer::instance().show(sixbox.convertPanoramaToSixBox(panoimage), "È«¾°");
}