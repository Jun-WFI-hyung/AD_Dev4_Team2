#ifndef DATA_FRAME_H
#define DATA_FRAME_H
#include <opencv2/opencv.hpp>
#include <atomic>
namespace data{
    class frame{
        public:
            frame()=default;
            frame(cv::Mat& image);
            void set_poses(unsigned int lposl,unsigned int lposr,unsigned int rposr,unsigned int rposl);

            static std::atomic<unsigned int> next_id_;
            
            unsigned int id_{};
            
            cv::Mat image_;
            
            unsigned int lposl_;
            unsigned int lposr_;

            unsigned int rposl_;
            unsigned int rposr_;

            unsigned int Width = 640;
            unsigned int Height = 480;
            
            // unsigned int lpos_last=0;
            // unsigned int rpos_last=640;
    };
}
#endif