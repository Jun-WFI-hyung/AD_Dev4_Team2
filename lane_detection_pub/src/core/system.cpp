#include "system.h"

std::pair<int,int> lane_detection::system::feed_frame(cv::Mat& image){
    shared_ptr<data::frame> cur_frame = make_shared<data::frame>(image);
    tracking_->set_cur_frame(cur_frame);
    tracking_->binary_3part();
    return {0,640};
}

