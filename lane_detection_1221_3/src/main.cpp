#include "../include/Simulator.h"

cv::Mat frame_;
std::tuple<float,float,float> poses;

unsigned int gap = 30;
float multi = 180/M_PI;
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
        theta_ = cvRound(((90 - theta) * 2.5f + center_diff) / 2);

        // steering
        angle = abs(theta_) < 50 ? theta_ : (theta_ < 0) ? -50 : 50;
        order.second = angle;
    }    
	//std::cout << "abs(order.second)" << abs(order.second) << std::endl;
    // accel - break
    float break_press = 0;
    if (abs(order.second) > 15) {
        break_press = (abs(order.second) - 15) / 35.F;
        
        
    } else {
        break_press = -1;
        
    }
    //std::cout << "Break press" << break_press << std::endl;
    if (speed - break_press < 5) {
    	speed = 5;
    } else if (speed - break_press > max_speed) {
    	speed = max_speed;
    } else {
    	speed -= break_press;
    }
    //std::cout <<"Befor Speed " <<speed << std::endl;
    order.first = (int)cvRound(speed);
    printf("\r speed : %d    //    angle : %d", order.first, order.second);
    //std::cout << std::endl;
    return order;
}

float Drive::pid_control(float err)
{
    d_error = err - p_error;
    p_error = err;
    i_error += err;

    return kp_ * p_error + ki_ * i_error + kd_ * d_error;
}

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



cv::Mat get_roi(cv::Mat& img,int x,int x_g,int y,int y_g){
    return img(cv::Range(x, x_g), cv::Range(y, y_g));
}

void hist_stretch(Mat& img){
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


struct SLine{
    SLine():
        nums_valid_points(0),
        params(-1.0F, -1.0F, -1.0F){};
    cv::Vec3f params;
    int nums_valid_points;
};

cv::Vec3f total_least_squares(std::vector<cv::Point>& points,std::vector<int> pt_on_Line){
    float x = 0, y = 0, x2 = 0, y2 = 0, xy = 0, w = 0;
    float dx2, dy2, dxy;
    float t;
    for( size_t i = 0; i < points.size(); ++i ){
        x += pt_on_Line[i] * points[i].x;
        y += pt_on_Line[i] * points[i].y;
        x2 += pt_on_Line[i] * points[i].x * points[i].x;
        y2 += pt_on_Line[i] * points[i].y * points[i].y;
        xy += pt_on_Line[i] * points[i].x * points[i].y;
        w += pt_on_Line[i];
    }

    x /= w;
    y /= w;
    x2 /= w;
    y2 /= w;
    xy /= w;

    //Covariance matrix
    dx2 = x2 - x * x;
    dy2 = y2 - y * y;
    dxy = xy - x * y;

    t = (float) atan2(2*dxy,dx2-dy2) / 2;
    cv::Vec3f line;
    line[0] = (float)t;
    line[1] = (float)x;
    line[2] = (float)y;
    
    return line;
}

SLine RANSAC_Line(float t,float p,float e,int T,std::vector<cv::Point>& points){
    //int s = 2;
    //int N = (int)ceilf(log(1-p)/log(1 - pow(1-e, s)));
    int N = 5;
    std::vector<SLine> line_candidates;
    std::vector<int> pt_on_line(points.size());
    RNG rng((uint64)-1);
    SLine line;
    for (int i = 0; i < N; i++){
        
        int idx1 = (int)rng.uniform(0, (int)points.size());
        int idx2 = (int)rng.uniform(0, (int)points.size());
        cv::Point p1 = points[idx1];
        cv::Point p2 = points[idx2];

        
        if (cv::norm(p1- p2) < t){
            continue;
        }

        //(y1 - y2)X + (x2 - x1)Y + x1y2 - x2y1 = 0 
        float a = static_cast<float>(p1.y - p2.y);
        float b = static_cast<float>(p2.x - p1.x);
        float c = static_cast<float>(p1.x*p2.y - p2.x*p1.y);
        
        float scale = 1.f/sqrt(a*a + b*b);
        a *= scale;
        b *= scale;
        c *= scale;

        
        int nums_Inliers = 0;
        for (size_t i = 0; i < points.size(); ++i){
            cv::Point& p0 = points[i];
            float rho      = abs(a*p0.x + b*p0.y + c);
            bool is_Inlier  = rho  < t;
            if ( is_Inlier ) nums_Inliers++;
            pt_on_line[i]    = is_Inlier;
        }

        if ( nums_Inliers < T){
            continue;
        }

        line.params = total_least_squares( points, pt_on_line);
        line.nums_valid_points = nums_Inliers;
        line_candidates.push_back(line);
    }

    int best_line_id = 0;
    int best_line_score = 0;
    for (size_t i = 0; i < line_candidates.size(); i++){
        if (line_candidates[i].nums_valid_points > best_line_score)
        {
            best_line_id = i;
            best_line_score = line_candidates[i].nums_valid_points;
        }
    }

    if (line_candidates.empty()){
        return SLine();
    }
    else{
        return line_candidates[best_line_id];
    }
}

std::tuple<float,float,float> binary_3part(){
	TickMeter tm;
    tm.start();
    Mat dx;
    Mat sobel_img = frame_.clone();
    Mat show = frame_.clone();
    
    cvtColor(sobel_img,sobel_img,COLOR_BGR2GRAY);
    hist_stretch(sobel_img);
    
    //double alpha = 1.0f;
    //int m = (int)mean(sobel_img)[0];
    //sobel_img = sobel_img + (sobel_img - m) * alpha;
    
    GaussianBlur(sobel_img, sobel_img, Size(), 3.);
	
    vector<Point> left,right;
    Point minloc, maxloc;
    double minv, maxv;
    //threshold(sobel_img,sobel_img,200,255,THRESH_BINARY);
    Sobel(sobel_img, dx, CV_32F, 1, 0);
    erode(dx,dx,Mat());
    Mat left_roi = get_roi(dx,360,390,0,300);
    Mat right_roi = get_roi(dx,360,390,320,600);
    
    
    for(int i = 0;i<gap;i++){
    	minMaxLoc(left_roi.row(i), &minv, &maxv, &minloc, &maxloc);
    	//if(abs(maxv)>100){
        	left.push_back(Point(maxloc.x, i+360));
        //}
        minMaxLoc(right_roi.row(i), &minv, &maxv, &minloc, &maxloc);
        std::cout << minv << std::endl;
        if(abs(minv)>20){
        	right.push_back(Point(minloc.x+320, i+360));
        }
    }

    for (Point pt : left) {
        circle(show, pt, 2, Scalar(255,0,0), -1);
    }
    for (Point pt : right) {
        circle(show, pt, 2, Scalar(0,0,255), -1);
    }
    float m1,m2;
    float x1,x2;
    float y1,y2; 
    float x0_l,x0_r; 
    float x480_l,x480_r;
    float lpos,rpos;
    float theta;
    float theta_l,theta_r;
    
    SLine result_l, result_r; 
    if(!left.empty()){
        result_l = RANSAC_Line(1.,0.5,0.5,1,left);
        theta_l = result_l.params[0];
        m1 = tan(result_l.params[0]);
        x1 = result_l.params[1];
        y1 = result_l.params[2];
        x0_l =(0-y1)/m1 +x1;
        x480_l = (480-y1)/m1 +x1;
        lpos = (380-y1)/m1 +x1;
        line(show,Point(x0_l,0),Point(x480_l,480),Scalar(0,0,255));
    }else{
    	
        result_l = SLine();
        lpos = 0;
    }
    //std::cout << result_l.nums_valid_points << std::endl;
    if(!right.empty()){
        result_r = RANSAC_Line(1.,0.5,0.5,1,right);
        theta_r = result_r.params[0];
        m2 = tan(result_r.params[0]);
        x2 = result_r.params[1];
        y2 = result_r.params[2];
        x0_r =(0-y2)/m2 +x2;
        x480_r = (480-y2)/m2 +x2;
        rpos = (380-y2)/m2 +x2;
        line(show,Point(x0_r,0),Point(x480_r,480),Scalar(0,0,255));
    }else{
    	result_r = SLine();
        rpos = 640;
    }
    float x_center;
    float y_center;
    
    if(result_l.nums_valid_points!=0 && result_r.nums_valid_points!=0){
    	x_center = (m1*x1-m2*x2-y1+y2)/(m1-m2);
    	y_center = ((m1*m2*(x2-x1))+m2*y1-m1*y2)/(m2-m1);
    	if((x_center==320.F)){
    		theta = 90.F;
    		//std::cout << theta << std::endl;
    		//std::cout << theta*180/M_PI << std::endl;
    	}else{
    		theta  =(float) atan2(320-x_center,380-y_center )*multi+90;
    		
    		//std::cout << theta*180/M_PI+90 << std::endl;
    	}
    	
    }else if(result_l.nums_valid_points!=0 && result_r.nums_valid_points==0){
    	rpos = 640;
    	theta = theta_l*multi+90;
    	//std::cout << theta*180/M_PI+90 << std::endl;
    }else if(result_l.nums_valid_points==0 && result_r.nums_valid_points!=0){
    	lpos = 0;
    	theta = theta_r*multi+90;
    	//std::cout << theta*180/M_PI+90 << std::endl;
    }else{
    	theta = -1;
    }
    //circle(show, Point(x_center,y_center), 2, Scalar(255,0,0));
    
    //imshow("left_roi",left_roi);
    //imshow("right_roi",right_roi);
    imshow("show",show);
    waitKey(1);
    //std::cout << lpos << " " << rpos<< " " <<theta << std::endl;
    tm.stop();
    //cout << "TIME: " << tm.getTimeMilli() << endl;
    return {lpos,rpos,theta};
    
}
void imageCallback(const sensor_msgs::Image& msg){
    
    cv::Mat src = cv::Mat(480, 640, CV_8UC3, const_cast<uchar *>(&msg.data[0]), msg.step);
    cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
    
    
}

void controll(ros::Publisher &pub, int angle,int speed ){
    xycar_msgs::xycar_motor msg;
    
    msg.angle = angle;
    msg.speed = speed;
    
    pub.publish(msg);
}

int main(int argc, char **argv) {
    float steer_angle = 1;
    ros::init(argc, argv, "test");      
    ros::NodeHandle nh;                  
    ros::Publisher pub = nh.advertise<xycar_msgs::xycar_motor>("xycar_motor", 10);
    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw/", 1, imageCallback);
    Drive drive(10,10,0.5,0.00012,0.21);
    ros::Rate rate(50);
    while(ros::ok()){
    	
    	//if(sub.getNumPublishers() > 0){
    	ros::spinOnce();
    	if (frame_.empty()) {
      		continue;
    	}
        	poses = binary_3part();
        	
        	//auto speed_angle = drive.calc_order(get<2>(poses),get<0>(poses),get<1>(poses),640,380);
        	//std::cout << "angle" << speed_angle.second << "speed" <<speed_angle.first << endl;
        	//controll(pub, speed_angle.second,speed_angle.first);
        	
        
    	rate.sleep();
    }
    return 0;
}


