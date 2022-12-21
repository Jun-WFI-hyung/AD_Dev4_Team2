#include "simul/Simulator.h"

vector<pair<int,int>> LD::detect(Mat img, const int thres, int s_thres)
{
    image = img;
    threshold_thres = thres;
    sobel_thres = s_thres;
    vector<pair<int,int>> pos;
    
    Rect roi_rect(Point(0, roi_offset), Point(width - 1, roi_offset + roi_height - 1));
    Mat roi = img(roi_rect).clone();

    Rect roi_l_rect(Point(0, roi_offset), Point(width/2-40-1, roi_offset + roi_height - 1));
    Rect roi_r_rect(Point(width/2 + 40, roi_offset), Point(width - 1, roi_offset + roi_height - 1));

    pos.push_back(img_retouch(roi_l_rect, 1));
    pos.push_back(img_retouch(roi_r_rect, 0));

    return pos;
}



pair<int,int> LD::img_retouch(const Rect sector, const bool is_left) 
{  
    Mat roi = image(sector).clone();   
    GaussianBlur(roi, roi, Size(5, 5), 0);
    cvtColor(roi, roi, CV_BGR2GRAY);

    Mat hist = roi.clone();
    hist_stretch(roi);
    threshold(roi, hist, threshold_thres, 255, THRESH_BINARY_INV);

    Mat hist_sum(1, hist.cols, CV_8U);
    uchar* hs_p = hist_sum.ptr<uchar>(0);

    for (int i = 0; i < hist.rows; i++) {
        uchar* p = hist.ptr<uchar>(i);

        for (int j = 0; j < hist.cols; j++) {
            if (p[j]) hs_p[j]++;
        }
    }

    int area_loc = 0; 
    int area_sum = 0;
    int area_min = win_width * roi_height * 0.8;

    int i_ = is_left ? 0 : 40;
    int end_v = is_left ? hist_sum.cols - 40 : hist_sum.cols;

    for (i_; i_ <= end_v - win_width; i_++) {
        Mat hist_area = hist_sum(Rect(i_, 0, win_width, hist_sum.rows)).clone();
        
        int s = 0;
        uchar* p = hist_area.ptr<uchar>(0);
        for (int j = 0; j < hist_area.cols; j++) {
            if (p[j]) s++;
        }

        if (s > area_sum) {
            area_loc = i_;
            area_sum = s;
        }
    }
    
    Rect roi_window = Rect(area_loc, 0, win_width, roi_height-1);
    rectangle(hist, roi_window, Scalar(0,255,0), 1);    

    Mat roi_win = roi(roi_window).clone();    
    Mat dx;
    Sobel(roi_win, dx, CV_32F, 1, 0);

    double minv, maxv;
    Point minloc, maxloc;
    pair<int,int> x_pos;

    for (int i = 0; i < dx.rows; i++) {
        float* p = dx.ptr<float>(i);
        for (int j = 0; j < dx.cols; j++) {
            p[j] = (abs(p[j]) > sobel_thres) ? p[j] : 0;
        }
    }

    minMaxLoc(dx.row(roi_height / 2), &minv, &maxv, &minloc, &maxloc);

    if (!minloc.x || !maxloc.x) {
        minloc.x = is_left ? 0 : this->width-1;
        maxloc.x = is_left ? 0 : this->width-1;
    } else {
        minloc.x = is_left ? minloc.x + area_loc : minloc.x + area_loc + width / 2 + 40 - 1;
        maxloc.x = is_left ? maxloc.x + area_loc : maxloc.x + area_loc + width / 2 + 40 - 1;
    }

    x_pos = make_pair(minloc.x, maxloc.x);
    return x_pos;
}



void LD::hist_stretch(Mat& img)
{
	int hist[256] = {0, };
	for (int r = 0; r < img.rows; r++) {
		for (int c = 0; c < img.cols; c++) {
			hist[img.at<uchar>(r,c)]++;
		}
	}

	int gmin, gmax;
	int ratio = img.cols * img.rows * 0.01;

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



LD::LD(int roi_o, int roi_h, int w, int h, int win_w)
{
    roi_offset = roi_o;
    roi_height = roi_h;
    width = w;
    height = h;
    win_width = win_w;
}

LD::~LD()
{
}
