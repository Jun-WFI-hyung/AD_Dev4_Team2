#ifndef TRACKING_HPP
#define TRACKING_HPP
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <iostream>
#include <tuple>
#include "../viewer/draw.h"
#include "../data/frame.h"
#define _USE_MATH_DEFINES
#include <cmath>

using namespace cv;
using namespace viewer;
using namespace std;
namespace lane_detection{
class tracking_module{
    private:

        int lpos_last=0;
        int rpos_last=640;

    public:
        tracking_module(){
            draw = make_shared<viewer::DRAW>();
        };
        
        tracking_module(shared_ptr<data::frame> cur_frame){
            
            cur_frame_ = cur_frame;
        }
        ~tracking_module(){};

        void set_cur_frame(shared_ptr<data::frame> cur_frame){
            cur_frame_ = cur_frame;
        }

        cv::Mat get_roi(cv::Mat& img,int width_start,int width_gap);

        vector<Point2f> find_edges(Mat& img,bool left,bool right);
        
        void binary_3part();

        void calculate_poses();

        vector<shared_ptr<data::frame>> video_start(std::string videoPath);

        shared_ptr<data::frame> cur_frame_;
        shared_ptr<viewer::DRAW> draw;

        int kernel_size = 5;
        int low_threshold = 30;
        int high_threshold = 90;

        unsigned int offset = 350;
        unsigned int gap = 45;

// ===================================================================================================
        void birdeye(Mat& src,Mat& dst);

        cv::Mat canny_image();

        std::tuple<float, float> get_line_params(std::vector<Vec4i>& lines);
        
        int get_line_pos(std::vector<Vec4i>& lines,bool left=false,bool right=false);

        std::tuple<std::vector<Vec4i>,std::vector<Vec4i>> divide_left_right(std::vector<Vec4i>& lines);

};
}
#endif