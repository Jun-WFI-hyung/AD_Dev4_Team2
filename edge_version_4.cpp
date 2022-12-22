#include <iostream>
#include <algorithm>
#include "opencv2/opencv.hpp"


using namespace cv;
using namespace std;

class Edge {
private:

	Mat edge_mat;
	Mat blr;
	Mat blr2;
	Mat roi;
	Mat roi2;
	Mat roi3;
	Mat show;
	vector<Vec4i> lines;
	vector<Vec4i> lines2;
	vector<Vec4i> lines3;
	vector<float> Lpoints;
	vector<float> Rpoints;
	vector<float> leftpoints;
	vector<float> rightpoints;
	Vec4i L, R;
	float height;
	float mid_val;
	float Offset;
	float theta;
	float crossX = 320;
	float crossY = 240;
	float rightp;
	float leftp;
	float rposl;
	float lposr;
	float theta_l;
	float theta_r;
	float x1, x2, x3, x4, y1, y2, y3, y4;
	float Lbuff_index;
	float Rbuff_index;
	const float pi = 3.14159265358979;
	const float multi = 180 / pi;

public:
	void Img2Canny(Mat frame);
	void PointMake();
	void PointCheck();
	void ThetaMake();
};

void Edge::Img2Canny(Mat frame)
{
	Offset = 355;
	height = 40;

	Mat gray;
	Mat edge;
	show = frame.clone();

	roi = frame(Rect(0, Offset, 640, height));
	cvtColor(roi, gray, COLOR_BGR2GRAY);
	bilateralFilter(gray, blr, -1, 20, 20);
	Canny(blr, edge, 80, 200);
	imshow("blr", edge);
	HoughLinesP(edge, lines, 1, CV_PI / 180, 18, 18, 10);
	cout << "detected lines number:" << ' ' << lines.size() << endl;

}

void Edge::PointMake()
{
	float Lbuff = 0;
	Lbuff_index = -1;
	float Rbuff = 641;
	Rbuff_index = -1;




	for (size_t i = 0; i < lines.size(); i++) {

		Vec4i l = lines[i];

		//if ((-0.2 < (float(l[0] - l[2]) / float(l[1] - l[3])) && (float(l[0] - l[2]) / float(l[1] - l[3])) < 0) || (0 < (float(l[0] - l[2]) / float(l[1] - l[3]) > 0.2)) && (float(l[0] - l[2]) / float(l[1] - l[3])) < 0.2) continue;
		line(show, Point(l[0], l[1] + Offset), Point(l[2], l[3] + Offset), Scalar(0, 255, 0), 3);
		if (l[1] - l[3] == 0) mid_val = (height / 2 - l[1]) / (1) * (l[0] - l[2]) + l[0];
			
		else mid_val = (height / 2 - l[1]) / (l[1] - l[3]) * (l[0] - l[2]) + l[0];

		if (mid_val < 340) {
			if (mid_val > Lbuff) {
				Lbuff = mid_val;
				Lbuff_index = i;
				Lpoints.push_back(mid_val);
			}
		}

		else {
			if (mid_val < Rbuff) {
				Rbuff = mid_val;
				Rbuff_index = i;
				Rpoints.push_back(mid_val);
			}
		}
	}

	if (!Lpoints.empty() && !Rpoints.empty()) {

		L = lines[Lbuff_index];

		x1 = L[0];
		y1 = L[1] + Offset;
		x2 = L[2];
		y2 = L[3] + Offset;

		R = lines[Rbuff_index];

		x3 = R[0];
		y3 = R[1] + Offset;
		x4 = R[2];
		y4 = R[3] + Offset;

		theta_l = atan2(y2 - y1, x2 - x1);
		theta_r = atan2(y4 - y3, x4 - x3);

		crossX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
		crossY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
	}


	else if (!Lpoints.empty() && Rpoints.empty()) {

		L = lines[Lbuff_index];

		x1 = L[0];
		y1 = L[1] + Offset;
		x2 = L[2];
		y2 = L[3] + Offset;

		x3 = 640;
		y3 = Offset;
		x4 = 640;
		y4 = Offset + height;

		theta_l = atan2(y2 - y1, x2 - x1);

		crossX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
		crossY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
	}

	else if (Lpoints.empty() && !Rpoints.empty()) {

		x1 = 0;
		y1 = Offset;
		x2 = 0;
		y2 = Offset + height;

		R = lines[Rbuff_index];

		x3 = R[0];
		y3 = R[1] + Offset;
		x4 = R[2];
		y4 = R[3] + Offset;

		theta_r = atan2(y4 - y3, x4 - x3);

		crossX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
		crossY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

	}

	else {
		crossX = 320;
		crossY = 240;
	}

	cout << "crossX:" << ' ' << crossX << ' ' << "crossY:" << ' ' << crossY << endl;

}


void Edge::PointCheck()
{

	if (Lpoints.size() != 0) {
		leftp = (height / 2 - L[1]) / (L[1] - L[3]) * (L[0] - L[2]) + L[0];
		circle(show, Point(x1, y1), 1, Scalar(255, 0, 0), 5);
		circle(show, Point(x2, y2), 1, Scalar(255, 0, 0), 5);
		circle(show, Point(leftp, Offset + height * 0.5), 1, Scalar(0, 0, 255), 5);
	}
	else leftp = 0;

	if (Rpoints.size() != 0) {
		rightp = (height / 2 - R[1]) / (R[1] - R[3]) * (R[0] - R[2]) + R[0];
		circle(show, Point(x3, y3), 1, Scalar(255, 0, 0), 5);
		circle(show, Point(x4, y4), 1, Scalar(255, 0, 0), 5);
		circle(show, Point(rightp, Offset + height * 0.5), 1, Scalar(0, 0, 255), 5);
	}
	else rightp = 640;

	if (leftp < 0) leftp = 0;
	if (rightp > 640) rightp = 640;
	cout << "leftp:" << ' ' << leftp << ' ' << "rightp:" << ' ' << rightp << endl;

}

void Edge::ThetaMake()
{

	if (Lpoints.size() != 0 && Rpoints.size() != 0) {
		line(show, Point(leftp, Offset + height * 0.5), Point(rightp, Offset + height * 0.5), Scalar(0, 0, 255));
		line(show, Point(leftp, Offset + height * 0.5), Point(crossX, crossY), Scalar(0, 0, 255), 2);
		line(show, Point(rightp, Offset + height * 0.5), Point(crossX, crossY), Scalar(0, 0, 255), 2);
		if (crossX == 320) theta = 90;
		else theta = atan2(320 - crossX, Offset + (height / 2) - crossY) * multi + 90;
	}

	else if (Lpoints.size() != 0 && Rpoints.size() == 0) {
		line(show, Point(leftp, Offset + height * 0.5), Point(crossX, crossY), Scalar(0, 0, 255), 2);
		theta = theta_l * multi + 90;

	}

	else if (Lpoints.size() == 0 && Rpoints.size() != 0) {
		line(show, Point(rightp, Offset + height * 0.5), Point(crossX, crossY), Scalar(0, 0, 255), 2);
		theta = theta_r * multi + 90;

	}
	else theta = -1;
	cout << "theta:" << ' ' << theta << endl;
	imshow("show", show);

}

int main() {

	Mat src = imread("C:\\OpenCVV\\frames\\950_frames.png");

	if (src.empty()) {
		cerr << "ERROR" << endl;
		return -1;
	}

	Edge sim;

	sim.Img2Canny(src);
	sim.PointMake();
	sim.PointCheck();
	sim.ThetaMake();

	waitKey(0);


}