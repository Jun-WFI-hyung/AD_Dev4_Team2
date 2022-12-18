#include "tracking_module.h"


using namespace lane_detection;

cv::Mat tracking_module::get_roi(cv::Mat& img,int y,int y_g){
    return img(cv::Range(offset, offset+gap), cv::Range(y, y_g));
}

vector<Point2f> tracking_module::find_edges(Mat& img,bool left,bool right)
{
    cvtColor(img,img,COLOR_BGR2GRAY);
    
	Mat fimg, blr, dx;
	img.convertTo(fimg, CV_32F);
	GaussianBlur(fimg, blr, Size(), 1.);
	Sobel(blr, dx, CV_32F, 1, 0);
	double minv, maxv;
	Point minloc, maxloc;

	int y2 = img.rows / 2;
	Mat roi = dx.row(y2);
	minMaxLoc(roi, &minv, &maxv, &minloc, &maxloc);
    
	vector<Point2f> pts;
    if(left){
        if(minv>-100 && maxv<100){
            minloc = Point2f(0,y2);
            maxloc = Point2f(0,y2);
        }
        if(minloc.x>=maxloc.x){
            minloc = Point2f(0,y2);
            maxloc = Point2f(0,y2);
        }
    }
    else{
        if(minv>-100 && maxv<100){
            minloc = Point2f(640,y2);
            maxloc = Point2f(640,y2);
        }
        if(minloc.x>=maxloc.x){
            minloc = Point2f(640,y2);
            maxloc = Point2f(640,y2);
        }
    }
	pts.push_back(Point2f(maxloc.x, y2));
	pts.push_back(Point2f(minloc.x, y2));
	return pts;
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
void tracking_module::binary_3part(){
    Mat dx;
    Mat divide_img = cur_frame_->image_.clone();

    Mat show = divide_img.clone();
    imshow("original",divide_img);
    cvtColor(divide_img,divide_img,COLOR_BGR2GRAY);
    GaussianBlur(divide_img, divide_img, Size(), 2.);
    hist_stretch(divide_img);
    imshow("hist_stretch", divide_img);
    Mat dst_IC;
    double alpha = 1.0f;
    int m = (int)mean(divide_img)[0];
    divide_img = divide_img + (divide_img - m) * alpha;
    imshow("Increase Contrast", divide_img);

    threshold(divide_img, divide_img, 110, 255, THRESH_BINARY);
    Sobel(divide_img, dx, CV_32F, 1, 0);
    Mat left_roi = get_roi(dx,0,300);
    Mat right_roi = get_roi(dx,320,600);
    
    double minv, maxv;
    Point minloc, maxloc;
    vector<Point> min_pts, max_pts;
    int half_Width = (cur_frame_->Width)/2;
    for(int i = 0;i<gap;i+=5){
        // float* left_p = left_roi.ptr<float>(i*15);
        // float* right_p = right_roi.ptr<float>(i*15);

        minMaxLoc(left_roi.row(i), &minv, &maxv, &minloc, &maxloc);
        
        if(abs(minv)<300){
            std::cout <<"Left Min: " << std::endl;
            std::cout << minv << std::endl;
        }else{
            min_pts.push_back(Point(minloc.x, i+offset));
        }
        if(abs(maxv)<300 || maxloc.x <= minloc.x){
            std::cout <<"Left Max: " << std::endl;
            std::cout << maxv << std::endl;
        }else{
            max_pts.push_back(Point(maxloc.x, i+offset));
        }

        minMaxLoc(right_roi.row(i), &minv, &maxv, &minloc, &maxloc);

        if(abs(minv)<300){
            std::cout <<"Right Min: " << std::endl;
            std::cout << minv << std::endl;
        }else{
            min_pts.push_back(Point(minloc.x+half_Width, i+offset));
        }
        if(abs(maxv)<300 || maxloc.x <= minloc.x){
            std::cout <<"Right Max: " << std::endl;
            std::cout << maxv << std::endl;
        }else{
            max_pts.push_back(Point(maxloc.x+half_Width, i+offset));
        }
    }
// BGR
    for (Point pt : min_pts) {
        circle(show, pt, 2, Scalar(255,0,0), -1);
    }
    for (Point pt : max_pts) {
        circle(show, pt, 2, Scalar(0,0,255), -1);
    }

    imshow("show",show);
    imshow("sobel",dx);
    imshow("Left",left_roi);
    imshow("Right",right_roi);
    imshow("roi",divide_img);
    cv::waitKey(1);
    return;
}

void tracking_module::calculate_poses(){
    Mat left_roi = get_roi(cur_frame_->image_,10,270);
    auto left_pts = find_edges(left_roi,true,false);
    int lposl = static_cast<int>(left_pts[1].x);
    int lposr = static_cast<int>(left_pts[0].x);
    
    if(lposl==0 || lposr ==0){
        lposl = 0;
        lposr = 0;
        draw->draw_point(cur_frame_->image_, Point(cvRound(lposl), 380), Scalar(255, 0, 0));
        draw->draw_point(cur_frame_->image_, Point(cvRound(lposr), 380), Scalar(0, 0, 255));
    }
    else{
        lposl += 10 ;
        lposr += 10 ;
        draw->draw_point(cur_frame_->image_, Point(cvRound(lposl), 380), Scalar(255, 0, 0));
        draw->draw_point(cur_frame_->image_, Point(cvRound(lposr), 380), Scalar(0, 0, 255));
    }
    Mat right_roi = get_roi(cur_frame_->image_,380,640);
    auto right_pts = find_edges(right_roi,false,true);
    int rposl = static_cast<int>(right_pts[1].x);
    int rposr  = static_cast<int>(right_pts[0].x);
    
    if(rposl==640 || rposr ==640){
        rposl = 640;
        rposr = 640;
        draw->draw_point(cur_frame_->image_, Point(cvRound(lposl), 380), Scalar(255, 0, 0));
        draw->draw_point(cur_frame_->image_, Point(cvRound(lposr), 380), Scalar(0, 0, 255));
    }
    else{
        rposl += 380;
        rposr += 380;
        draw->draw_point(cur_frame_->image_, Point(cvRound(rposl), 380), Scalar(255, 0, 0));
        draw->draw_point(cur_frame_->image_, Point(cvRound(rposr), 380), Scalar(0, 0, 255));
    }
    cur_frame_->set_poses(lposl,lposr,rposl,rposr);
}

vector<shared_ptr<data::frame>> tracking_module::video_start(std::string videoPath){
    VideoCapture cap(videoPath);
    vector<shared_ptr<data::frame>> frames;

    Mat image;
    if (!cap.isOpened()){
        std::cerr << "no Video" << std::endl;
        return frames;
    }
    while (1){
        cap >> image;
        if (image.empty()){
            std::cerr <<"empty image" <<std::endl;
            return frames;
        }
        cur_frame_ = make_shared<data::frame>(image);

        binary_3part();

        // imshow("image",cur_frame_->image_);
        
        if (cur_frame_->id_ % 30 == 0) {
            frames.push_back(cur_frame_);
        }
        if (waitKey(25) == 27){
            break;
        }
    }

    return frames;
}

// ===================================================================================================
void tracking_module::birdeye(Mat& src,Mat& dst){
    int w = 640, h = 100;

    vector<Point2f> src_pts(4);
    vector<Point2f> dst_pts(4);

    src_pts[0] = Point2f(0, 350);	src_pts[1] = Point2f(640, 350);
    src_pts[2] = Point2f(640, 390); src_pts[3] = Point2f(0, 390);

    dst_pts[0] = Point2f(0, 0);		dst_pts[1] = Point2f(w - 1, 0);
    dst_pts[2] = Point2f(w - 1, h - 1);	dst_pts[3] = Point2f(0, h - 1);

    Mat per_mat = getPerspectiveTransform(src_pts, dst_pts);

    warpPerspective(src, dst, per_mat, Size(w, h));
}

cv::Mat tracking_module::canny_image(){
    cv::Mat image = cur_frame_->image_.clone();

    Mat gray,blur_gray,edge_img;

    cvtColor(image,gray,COLOR_BGR2GRAY);

    GaussianBlur(gray,blur_gray,Size(kernel_size, kernel_size), 0);
    Canny(blur_gray,edge_img,low_threshold, high_threshold);
    return edge_img;
};