#ifndef SIMUL
#define SIMUL

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using std::vector;
using namespace std;
using namespace cv;
using namespace cv_bridge;
using namespace ros;

#define LOG_VECTOR vector<vector<pair<int,int>>>


class Simulator {
protected:
    int cam_width, cam_height, cam_fps;

public:
    Simulator(int width, int height, int fps);
    ~Simulator();
};



class SimulCV : public Simulator {
private:
    string img_path;
    Mat image;
    int delay;
    // lane detecting class

public:
    LOG_VECTOR start(int roi_o, int roi_h, int win_w, int thres, int s_thres);
    
    SimulCV(string path, int width, int height, int fps);
    ~SimulCV();
};



class LD {
private:
    int roi_offset, roi_height;
    int width, height;
    int threshold_thres;
    int sobel_thres;
    int win_width;
    Mat image;

public:
    vector<pair<int,int>> detect(Mat, const int thres, int s_thres);
    pair<int,int> img_retouch(const Rect sector, const bool is_left);
    void hist_stretch(Mat& img);

    void calib();
    Mat birdeye();

    LD(int roi_o, int roi_h, int w, int h, int win_w);
    ~LD();
};

#endif