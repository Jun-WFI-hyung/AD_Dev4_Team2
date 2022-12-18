#include "lane_detection_ros.h"


void lane_detection_ros::system::image_callback(const sensor_msgs::ImageConstPtr& msg){
    auto angle_speed=lane_detection_->feed_frame(cv_bridge::toCvCopy(msg,"bgr8")->image);
};

