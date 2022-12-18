#ifndef LANE_DETECTION_HPP
#define LANE_DETECTION_HPP
#include <opencv2/opencv.hpp>
#include "../data/frame.h"
#include "tracking_module.h"

namespace lane_detection{
    class system{
        public:
            system(){
                tracking_ = make_shared<tracking_module>();
            };
            std::pair<int,int> feed_frame(cv::Mat& image);
            std::pair<int,int> run();

            shared_ptr<tracking_module> tracking_;

    };
}

#endif