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
	vector<Vec4i> lines;
	vector<Vec4i> lines2;
	vector<Vec4i> lines3;
	vector<float> Lpoints;
	vector<float> Rpoints;
	vector<float> leftpoints;
	vector<float> rightpoints;
	int height;
	int mid_val;
	float rightp;
	float leftp;
	float rposl;
	float lposr;

public:
	void Img2Canny(Mat frame);
	void HoughMake();
	void PointMake();
	void PointCheck();
};

void Edge::Img2Canny(Mat frame)
{
	Mat gray;
	Mat edge;
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	bilateralFilter(gray, blr, -1, 20, 20);
	Canny(blr, edge, 80, 200);
	edge_mat = edge;

}

void Edge::HoughMake()
{
	int Offset = 390;
	height = 20;
	roi = edge_mat(Rect(0, Offset - 15, 640, height));
	HoughLinesP(roi, lines, 1, CV_PI / 180, 18, 18, 10);
	Mat dst2;
	cvtColor(roi, dst2, COLOR_GRAY2BGR);
	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l = lines[i];
		line(dst2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
	}
}

void Edge::PointMake()
{
	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l = lines[i];
		if ((-0.2 < int(float(l[0] - l[2]) / float(l[1] - l[3])) && int(float(l[0] - l[2]) / float(l[1] - l[3])) < 0) || (0 < int(float(l[0] - l[2]) / float(l[1] - l[3]) > 0.2)) && int(float(l[0] - l[2]) / float(l[1] - l[3])) < 0.2) continue;
		mid_val = (height/2 - l[1]) / (l[1] - l[3]) * (l[0] - l[2]) + l[0];
		if (mid_val < 340) Lpoints.push_back(mid_val);
		else Rpoints.push_back((mid_val));
	}
}


void Edge::PointCheck()
{

	if (Lpoints.size() != 0) leftp = *max_element(Lpoints.begin(), Lpoints.end());
	else leftp = 0;
	 
	if (Rpoints.size() != 0) rightp = *min_element(Rpoints.begin(), Rpoints.end());
	else rightp = 640;
		
	if (leftp < 0) leftp = 0;
	if (rightp > 640) rightp = 640;

	cout << leftp << ' ' << rightp << endl;
}



int main() {

	Mat src = imread("C:\\OpenCVV\\frames\\2467_frames.png");

	if (src.empty()) {
		cerr << "ERROR" << endl;
		return -1;
	}

	Edge sim;

	sim.Img2Canny(src);
	sim.HoughMake();
	sim.PointMake();
	sim.PointCheck();

	waitKey(0);


}