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
	int Offset;
	float crossX = 9999;
	float crossY = 9999;
	float rightp;
	float leftp;
	float rposl;
	float lposr;

public:
	void Img2Canny(Mat frame);
	void PointMake();
	void PointCheck();
};

void Edge::Img2Canny(Mat frame)
{
	Offset = 355;
	height = 40;

	Mat gray;
	Mat edge;
	roi = frame(Rect(0, Offset, 640, height));
	cvtColor(roi, gray, COLOR_BGR2GRAY);
	bilateralFilter(gray, blr, -1, 20, 20);
	Canny(blr, edge, 80, 200);
	imshow("blr", edge);
	HoughLinesP(edge, lines, 1, CV_PI / 180, 18, 18, 10);

}

void Edge::PointMake()
{
	int Lbuff = 0;
	int Lbuff_index = -1;
	int Rbuff = 641;
	int Rbuff_index = -1;
	int x1, x2, x3, x4, y1, y2, y3, y4;

	Vec4i L, R;

	for (size_t i = 0; i < lines.size(); i++) {

		Vec4i l = lines[i];

		if ((-0.2 < int(float(l[0] - l[2]) / float(l[1] - l[3])) && int(float(l[0] - l[2]) / float(l[1] - l[3])) < 0) || (0 < int(float(l[0] - l[2]) / float(l[1] - l[3]) > 0.2)) && int(float(l[0] - l[2]) / float(l[1] - l[3])) < 0.2) continue;
		cout << l << endl;

		mid_val = (height / 2 - l[1]) / (l[1] - l[3]) * (l[0] - l[2]) + l[0];

		if (mid_val < 340) {
			if (mid_val > Lbuff) {
				Lbuff_index = i;
				Lpoints.push_back(mid_val);
			}
		}

		else {
			if (mid_val < Rbuff) {
				Rbuff_index = i;
				Rpoints.push_back((mid_val));
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


		crossX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
		crossY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
	}

	cout << "crossX" << ' ' << crossX << ' ' << "crossY" << ' ' << crossY << endl;


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

	Mat src = imread("C:\\OpenCVV\\frames\\999_frames.png");

	if (src.empty()) {
		cerr << "ERROR" << endl;
		return -1;
	}

	Edge sim;

	sim.Img2Canny(src);
	sim.PointMake();
	sim.PointCheck();

	waitKey(0);


}