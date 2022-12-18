#include "lane_detection.h"

int main(int argc, char **argv) 
{
    string path = "/home/junhyung/xycar_ws/src/simul/src/Sub_project.avi";
    int cam_width = 640;
    int cam_height = 480;
    int fps = 60;

    int roi_offset = 380;
    int roi_height = 40;
    int window_width = 120;
    int thres = 100;
    int sobel_diff_thres = 110;

    return 0;
}
