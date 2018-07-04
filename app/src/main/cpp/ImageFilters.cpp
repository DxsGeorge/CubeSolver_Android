#include "ImageFilters.h"

using namespace cv;
using namespace std;


Mat FilterImage(Mat src) {
    //flip(src, src, 1);
    Mat src1;
    src.copyTo(src1);
    pyrDown(src1, src1, Size(src.cols / 2, src.rows / 2));
    pyrDown(src, src, Size(src.cols / 2, src.rows / 2));
    cvtColor(src, src1, CV_BGR2GRAY);
    Mat dst(src.cols, src.rows, CV_8UC1);
    Mat dst1(src.cols, src.rows, CV_8UC1);
    GaussianBlur(src1, src1, Size(3, 3), 0, 0);
    Laplacian(src1, dst, CV_16SC1, 3);
    compare(dst, 8, dst, CV_CMP_GT);
    compare(src1, 100, dst1, CV_CMP_LT);
    bitwise_and(dst, dst1, dst);
    convertScaleAbs(dst, dst, 1, 0);
    return dst;
    }
