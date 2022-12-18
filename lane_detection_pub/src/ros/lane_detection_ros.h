#ifndef LANE_DETECTION_ROS_HPP
#define LANE_DETECTION_ROS_HPP
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <tuple>
#include "../core/system.h"


namespace lane_detection_ros{
    class system{
        public:
            system() = default;
            ~system() = default;
            system(std::shared_ptr<lane_detection::system>& lane_detection)
                :lane_detection_(lane_detection){
                    nh_ = ros::NodeHandle();
                    sub_ = nh_.subscribe<sensor_msgs::Image>(camera_topic_,1,&system::image_callback,this);
                // pub_ = nh_.advertise<xycar_motor::xycar_motor>(xycar_motor_,1);
            };

            std::shared_ptr<lane_detection::system> lane_detection_;
            
            ros::NodeHandle nh_;
            ros::Publisher pub_;
            ros::Subscriber sub_;

            std::string camera_topic_ = "/usb_cam/image_raw/";

        private:
            void image_callback(const sensor_msgs::ImageConstPtr& msg);
    };
}



#endif