#ifndef VIEWER_UTILS_HPP
#define VIEWER_UTILS_HPP
#include <opencv2/opencv.hpp>
#include <random>
using namespace cv;
namespace viewer{

    class DRAW{
        public:
            void draw_rectangle(Mat& img,int lpos,int rpos,int offset=0);
            
            void draw_lines(Mat& img,std::vector<Vec4i>& lines,int Offset);

            void draw_point(Mat& img, Point pt, Scalar color);
    };
}
#endif