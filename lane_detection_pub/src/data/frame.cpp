#include "frame.h"
namespace data{

    std::atomic<unsigned int> frame::next_id_{0};

    void frame::set_poses(unsigned int lposl,unsigned int lposr,unsigned int rposl,unsigned int rposr){
            lposl_ = lposl;
            lposr_ = lposr;
            rposl_ = rposl;
            rposr_ = rposr;
    };
    frame::frame(cv::Mat& image):image_(image),id_(next_id_++){
    };
}
