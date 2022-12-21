#include "simul/Simulator.h"

int main(int argc, char **argv) 
{
    string path = "/home/junhyung/xycar_ws/src/simul/src/Sub_project.avi";
    int cam_width = 640;
    int cam_height = 480;
    int fps = 90;

    int roi_offset = 380;
    int roi_height = 40;
    int window_width = 110;
    int thres = 100;
    int sobel_diff_thres = 100;

    LOG_VECTOR log;

    SimulCV simul(path, cam_width, cam_height, fps);
    log = simul.start(roi_offset, roi_height, window_width, thres, sobel_diff_thres);    
    
    ofstream outfile;
    outfile.open("test.csv", ios::out);

    for (int i = 0; i < log.size(); i++)
    {
        outfile << log[i][0].first <<","<< log[i][0].second <<","<< log[i][1].first <<","<< log[i][1].second << endl;
        cout << "saving.." << endl;

    }
    outfile.close();
    return 0;
}
