#include "simul/Simulator.h"

LOG_VECTOR SimulCV::start(int roi_o, int roi_h, int win_w, int thres, int s_thres)
{
    TickMeter tm;
    VideoCapture cap(img_path);

    LOG_VECTOR log;
    LOG_VECTOR::reverse_iterator r_itr;    

    LD ld(roi_o, roi_h, this->cam_width, this->cam_height, win_w);

    int counter = 0;
    while (1) {
        cap >> image;
        if (image.empty()) {
            cout << "empty!" << endl;
            break;
        }

        tm.start(); // ----------------------------- tickmeter start
        vector<pair<int,int>> temp = ld.detect(image, thres, s_thres);
        tm.stop(); // ------------------------------ tickmeter stop

        circle(image, Point(temp[0].first, roi_o + roi_h / 2), 3, Scalar(255,0,0), -1);
        circle(image, Point(temp[0].second, roi_o + roi_h / 2), 3, Scalar(0,0,255), -1);
        circle(image, Point(temp[1].first, roi_o + roi_h / 2), 3, Scalar(255,0,0), -1);
        circle(image, Point(temp[1].second, roi_o + roi_h / 2), 3, Scalar(0,0,255), -1);

        if (!(counter % 30)) {
            log.push_back(temp);
            r_itr = log.rbegin();
            counter = 0;
        }

        putText(
            image, 
            format("elapsed : %f ms.", tm.getTimeMilli()),
            Point(10, 20),
            FONT_HERSHEY_SIMPLEX, 
            0.5, 
            Scalar(255, 0, 0), 
            1.2, 
            LINE_AA
            );

        putText(
            image, 
            format("left_l : %d  /  left_r : %d", r_itr[0][0].first, r_itr[0][0].second),
            Point(10, 40),
            FONT_HERSHEY_SIMPLEX, 
            0.5, 
            Scalar(0, 0, 255), 
            1.2, 
            LINE_AA
            );

        putText(
            image, 
            format("right_l : %d  /  right_r : %d", r_itr[0][1].first, r_itr[0][1].second),
            Point(10, 60),
            FONT_HERSHEY_SIMPLEX, 
            0.5, 
            Scalar(0, 255, 0), 
            1.2, 
            LINE_AA
            );

        tm.reset();
        counter++;

        imshow("img", image);
        if (waitKey(delay) == 27) break;        
    }

    return log;
}

SimulCV::SimulCV(
    string path, 
    int width, int height, int fps
    ) : Simulator(width, height, fps)
{
    img_path = path;
    delay = 1000 / fps;
}

SimulCV::~SimulCV()
{
}