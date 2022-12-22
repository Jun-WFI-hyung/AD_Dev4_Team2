#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "xycar_msgs/xycar_motor.h"

using namespace std;
using namespace ros;
using namespace cv;
using namespace cv_bridge;

void calib(Mat& image, Mat& map1, Mat& map2); // undistort
void img_warp(Mat& img, Mat& flat_img, Mat& pers_mtx, Mat& flat_mtx, vector<Point> pts_draw, vector<Point2f> pts_pers, Size flat_size); // warpPerspective
void img_filter(Mat& img, Mat& bin_img, Size k_size, double t_thres, int r_thres);
void hist_stretch(Mat& img);
void window_detect(Mat& img, int w_height, int w_width, int w_num);
Mat img_retouch(Mat& img, int thres1, int thres2);
void sobel_test(Mat& img);


class Drive
{
private:
    float max_speed;
    float speed;
    int angle;
    int stop_counter;
    float kp_, ki_, kd_;
    float p_error, i_error, d_error;

public:
    pair<int,int> calc_order(float theta, float l_pos, float r_pos, float roi_x_value, float roi_y_value);
    float pid_control(float err);
    Drive(float init_speed, float max_spd, float kp, float ki, float kd);
    ~Drive();
};


pair<int,int> Drive::calc_order(float theta, float l_pos, float r_pos, float roi_x_value, float roi_y_value)
{
    float x_center = roi_x_value / 2;
    float y_center = roi_y_value / 2;
    float lane_center = (l_pos + r_pos) / 2;

    // first = speed, second = angle
    pair<int,int> order(speed, 0);
    int theta_;

    if (theta < 0) {
        theta_ = angle;
    } else {        
        float center_diff = 50 * (lane_center - x_center) / 320;
        theta_ = cvRound((90 - theta) * 2.5f + center_diff);

        // steering
        angle = abs(theta_) < 50 ? theta_ : (theta_ < 0) ? -50 : 50;
        order.second = angle;
    }    

    // accel - break
    float break_press = 0;
    if (abs(order.second) > 15) {
        break_press = (order.second - 15) / 35;
    } else {
        break_press = -1;
    }

    speed = (speed - break_press < 1) ? 1 : (speed - break_press > max_speed) ? max_speed : speed - break_press;

    order.first = cvRound(speed);
    //printf("\r speed : %f    //    angle : %f", speed, theta_);
    return order;
}

float Drive::pid_control(float err)
{
    d_error = err - p_error;
    p_error = err;
    i_error += err;

    return kp_ * p_error + ki_ * i_error + kd_ * d_error;
}

// 0.4 / 0.00012 / 0.19
Drive::Drive(float init_speed, float max_spd, float kp, float ki, float kd)
{
    max_speed = max_spd;
    speed = init_speed;
    angle = 0;
    stop_counter = 0;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

Drive::~Drive()
{
}





# if 0

#define LIDAR_TOPIC "/scan"
#define CAM_TOPIC "/usb_cam/image_raw"


CvImagePtr bridge;

void callback(const sensor_msgs::Image& img) 
{
    bridge = toCvCopy(img, "bgr8");
}

int main(int argc, char **argv) 
{
    
    init(argc, argv, "driver");
    NodeHandle nh;
    //Rate rate(30);
    Subscriber sub = nh.subscribe(CAM_TOPIC, 1, callback);

    while (ok()) {   
        Mat image = bridge -> image;
        while (image.empty()) { break; }

        spinOnce();

    return 0;
}

# else

int main(int argc, char **argv) 
{
    TickMeter tm;
    string path = "/home/junhyung/xycar_ws/src/simul/src/Sub_project.avi";
    VideoCapture cap(path);
    Mat image, map1, map2;

    int cam_width = 640;
    int cam_height = 480;
    double fps = cap.get(CAP_PROP_FPS);
    
    int speed = 4;
    int delay = 1000 / (fps * speed);

    // draw polygon - editable -
    int poly_top = 300;
    int poly_bottom = 400;
    int poly_margin = 10;
    // draw polygon ------------
    int poly_left = 0 + poly_margin;
    int poly_right = cam_width - poly_margin - 1;
    int poly_side_diff = (poly_right - poly_left) / 3.5f - 5;
    Point2f pt_t_l(poly_left + poly_side_diff, poly_top);
    Point2f pt_t_r(poly_right - poly_side_diff, poly_top);
    Point2f pt_b_r(poly_right, poly_bottom);
    Point2f pt_b_l(poly_left, poly_bottom);
    vector<Point2i> pts_draw = {pt_t_l, pt_t_r, pt_b_r, pt_b_l};
    vector<Point2f> pts_pers = {pt_t_l, pt_t_r, pt_b_r, pt_b_l};

    // warp perspective
    int flat_width = 260;
    int flat_height = 300;
    Size flat_size(flat_width, flat_height);
    Mat filt_img, flat_img, pers_mtx, flat_mtx;

    // filter
    int kernel_num = 5;
    Mat bin_img;
    Size k_size(kernel_num, kernel_num);
    double t_thres = 180;
    // window - max_height = flat_height
    int window_height = 40;
    int window_width = 40;
    int window_num = 5;
    int r_thres = 7;

    while (1) {
        cap >> image;
        if (image.empty()) break;

        tm.start(); // ----------------------------- tickmeter start
        calib(image, map1, map2);
        hist_stretch(image);
        //imshow("img", image);
        GaussianBlur(image, image, Size(5, 5), 2);
        filt_img = img_retouch(image, 155, 50);

        img_warp(filt_img, flat_img, pers_mtx, flat_mtx, pts_draw, pts_pers, flat_size);
        img_filter(flat_img, bin_img, k_size, t_thres, r_thres);
        window_detect(bin_img, window_height, window_width, window_num);

        //Mat canny_img;
        //Canny(image, canny_img, 150, 160);
        tm.stop(); // ------------------------------ tickmeter stop

        putText(
            image, 
            format("elapsed : %f", tm.getTimeMilli()),
            Point(10, 20),
            FONT_HERSHEY_SIMPLEX, 
            0.5, 
            Scalar(255, 0, 0), 
            1.2, 
            LINE_AA
            );
        
        tm.reset();
        /* imshow("img", image);
        cvtColor(bin_img, bin_img, CV_GRAY2BGR);
        imshow("flat", flat_img);
        imshow("bin", bin_img); */
        //cvtColor(filt_img, filt_img, CV_GRAY2BGR);
        imshow("filt_img", filt_img);
        if (waitKey(10) == 27) break;        
    }

    return 0;
}

# endif





void calib(Mat& img, Mat& map1, Mat& map2)
{
    Mat mtx(3, 3, CV_64FC1);
    Mat dist(1, 5, CV_64FC1);

    mtx = (Mat1d(3, 3) <<  368.827865, 0.000000, 333.683460, 0.000000, 368.680750, 233.586572, 0.000000, 0.000000, 1.000000);
    dist = (Mat1d(1, 5) << -0.301885, 0.066512, -0.000027, -0.001219, 0.000000);
    Size sz(img.cols, img.rows);

    
    if (map1.empty()) {initUndistortRectifyMap(mtx, dist, Mat(), mtx, sz, CV_32FC1, map1, map2);} // test
    remap(img, img, map1, map2, INTER_LINEAR);
}


void img_warp(Mat& img, Mat& flat_img, Mat& pers_mtx, Mat& flat_mtx, vector<Point> pts_draw, vector<Point2f> pts_pers, Size flat_size)
{
    Point2f top_l(0, 0);
    Point2f top_r(flat_size.width - 1, 0);
    Point2f bottom_r(flat_size.width - 1, flat_size.height - 1);
    Point2f bottom_l(0, flat_size.height - 1);
    vector<Point2f> pts_flat = {top_l, top_r, bottom_r, bottom_l};

    flat_mtx = getPerspectiveTransform(pts_pers, pts_flat);
    pers_mtx = getPerspectiveTransform(pts_flat, pts_pers);
    warpPerspective(img, flat_img, flat_mtx, flat_size);

    polylines(img, pts_draw, 1, Scalar(255, 255, 0), 1);
}


void img_filter(Mat& img, Mat& bin_img, Size k_size, double t_thres, int r_thres)
{
    cvtColor(img, bin_img, CV_BGR2GRAY);
    GaussianBlur(bin_img, bin_img, k_size, 1);
    
    hist_stretch(bin_img);
    
}


void hist_stretch(Mat& img)
{
	int hist[256] = {0, };
	for (int r = 0; r < img.rows; r++) {
		for (int c = 0; c < img.cols; c++) {
			hist[img.at<uchar>(r,c)]++;
		}
	}

	int gmin, gmax;
	int ratio = img.cols * img.rows * 0.02;

	for (int i = 0, s = 0; i <= 255; i++) {
		s += hist[i];
		if (s >= ratio) {gmin = i; break;}
	}

	for (int i = 255, s = 0; i >= 0; i--) {
		s += hist[i];
		if (s >= ratio) {gmax = i; break;}
	}

	img = (img - gmin) * 255 / (gmax - gmin);
}

void window_detect(Mat& img, int w_height, int w_width, int w_num)
{
    // flat image //
    Mat flat_img = img.clone();
    Rect win_rect(Point(0, 0), Point(flat_img.cols -1, flat_img.rows - 1));
    Mat win = flat_img(win_rect).clone();
    threshold(win, win, 65, 255, THRESH_BINARY);
    cvtColor(win, win, CV_GRAY2BGR);
    //imshow("flat", win);
}

Mat img_retouch(Mat& img, int thres1, int thres2)
{
    vector<Mat> img_vec;
    split(img, img_vec);

    Mat b = img_vec[0];
    Mat g = img_vec[1];
    Mat r = img_vec[2];

    Mat dst;
    for (int r = 0; r < img_vec[0].rows; r++) {
        uchar* ptr_b = img_vec[0].ptr<uchar>(r);
        uchar* ptr_g = img_vec[1].ptr<uchar>(r);
        uchar* ptr_r = img_vec[2].ptr<uchar>(r);
        uchar* ptr_dst = dst.ptr<uchar>(r);

        for (int c = 0; c < img_vec[0].cols; c++) {
            if (ptr_b[c] > thres1 || ptr_g[c] > thres1 || ptr_r[c] > thres1) {
                ptr_b[c] = 255;
                ptr_g[c] = 255;
                ptr_r[c] = 255;
            } else if (abs(ptr_b[c] - ptr_g[c]) > thres2 || abs(ptr_b[c] - ptr_r[c]) > thres2 || abs(ptr_g[c] - ptr_r[c]) > thres2) {
                ptr_b[c] = 255;
                ptr_g[c] = 255;
                ptr_r[c] = 255;
            }
        }
    }

    merge(img_vec, dst);
    //sobel_test(dst);
    return dst;
}

void sobel_test(Mat& img) 
{
    Mat image;
    cvtColor(img, image, CV_BGR2GRAY);
    Mat dx;
    Sobel(image, dx, CV_32F, 1, 0);

    double minv, maxv;
    Point minloc, maxloc;

    Mat roi = dx.row(400);
    float* ptr_roi = roi.ptr<float>(0);

    minMaxLoc(roi, &minv, &maxv);
    int row_num = maxv - minv;

    Mat graph(200, roi.cols, CV_8UC3, Scalar(0));
    for (int i = 0; i < roi.cols; i++) {
        Point pt = Point(i, cvRound(ptr_roi[i] / row_num * 200 + 100));
        Scalar cr = (pt.y-100 < 0) ? Scalar(0,0,abs((pt.y-100)*5)) : (pt.y-100 > 0) ? Scalar((pt.y-100)*5,0,0) : Scalar(50,50,50);
        circle(graph, pt, 2, cr, -1);
    }
    
    cvtColor(dx, dx, CV_GRAY2BGR);
    line(dx, Point(0, 400), Point(roi.cols - 1, 400), Scalar(0,0,255), 2);
    imshow("dx", dx);
    imshow("graph", graph);
}