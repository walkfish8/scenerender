#include "opencv2/opencv.hpp"


template <typename T>
inline T clamp(const T& val, const T& min, const T& max)
{
    return std::max(min, std::min(val, max));
    //(val < min) ? val : ((val>max) ? val : max);
}

void RotatePanoImage(cv::InputArray inarr, cv::InputArray Rarr, cv::OutputArray& outarr)
{
    cv::Mat R = Rarr.getMat();
    cv::Mat panoimg = inarr.getMat();
    cv::Mat& outimg = outarr.getMatRef();

    //if (panoimg.size() != outimg.size()) 
    //    outimg = -cv::Mat::ones(panoimg.size(), panoimg.type());

    const auto& a0 = R.at<double>(0, 0); const auto& a1 = R.at<double>(0, 1); const auto& a2 = R.at<double>(0, 2);
    const auto& b0 = R.at<double>(1, 0); const auto& b1 = R.at<double>(1, 1); const auto& b2 = R.at<double>(1, 2);
    const auto& c0 = R.at<double>(2, 0); const auto& c1 = R.at<double>(2, 1); const auto& c2 = R.at<double>(2, 2);


    //// 匿名函数，将XYZ向量转换成角度表示
    //auto rotatePanoUV = [](float u, float v, const cv::Mat& R0)->cv::Point2f
    //{
    //    float alpha = 
    //    float x0 = 

    //    float x = R0.at<double>(0, 0)*x0 + R0.at<double>(0, 1)*y0 + R0.at<double>(0, 2)*z0;
    //    float y = R0.at<double>(1, 0)*x0 + R0.at<double>(1, 1)*y0 + R0.at<double>(1, 2)*z0;
    //    float z = R0.at<double>(2, 0)*x0 + R0.at<double>(2, 1)*y0 + R0.at<double>(2, 2)*z0;

    //    float r = sqrt(x*x + y*y + z*z);
    //    return cv::Point2f(atan2f(x, z) + CV_PI, acosf(y / r));
    //};

    double two_pi = 2 * CV_PI;
    double pixel_per_rad = (panoimg.rows-1) / CV_PI;

    cv::Mat map_pano_to_pano_x = -cv::Mat::ones(panoimg.rows, panoimg.cols, CV_32F);
    cv::Mat map_pano_to_pano_y = -cv::Mat::ones(panoimg.rows, panoimg.cols, CV_32F);
    for (int v = 0; v < panoimg.rows; ++v)
    {
        auto map_pano_x_ptr = map_pano_to_pano_x.ptr<float>(v);
        auto map_pano_y_ptr = map_pano_to_pano_y.ptr<float>(v);
        for (int u = 0; u < panoimg.cols; ++u)
        {
            float theta = v / pixel_per_rad;
            float alpha = u / pixel_per_rad;

            float x = sin(theta)*sin(alpha);
            float y = cos(theta);
            float z = sin(theta)*cos(alpha);

            float x1 = a0*x + a1*y + a2*z;
            float y1 = b0*x + b1*y + b2*z;
            float z1 = c0*x + c1*y + c2*z;

            float r = sqrt(x1*x1 + y1*y1 + z1*z1);
            float theta1 = acos(y1 / r);
            float alpha1 = atan2(x1, z1);
            if (alpha1 < 0.0f) alpha1 += two_pi;

            float v1 = theta1*pixel_per_rad;
            float u1 = alpha1*pixel_per_rad;

            map_pano_x_ptr[u] = u1;
            map_pano_y_ptr[u] = v1;
        }
    }
    cv::remap(panoimg, outimg, map_pano_to_pano_x, map_pano_to_pano_y, CV_INTER_LANCZOS4);
}

void main()
{
    cv::Mat Rmat = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.3, 0.2, 0.5);
    cv::Rodrigues(rvec, Rmat);
    cv::Mat panoimg = cv::imread(R"(D:\scenerender\datas\scene01\全景图.jpg)");


    cv::Mat outimg;
    RotatePanoImage(panoimg, Rmat, outimg);
    //cv::imshow("", outimg);
    //cv::waitKey();
    cv::imwrite(R"(d:\1.bmp)", outimg);
}