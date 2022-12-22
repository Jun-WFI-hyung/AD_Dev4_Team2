#include <iostream>
#include <algorithm>
#include "opencv2/opencv.hpp"


using namespace cv;
using namespace std;

class Edge {
private:

	Mat edge_mat;
	//Mat gray;
	Mat blr;
	Mat blr2;
	//Mat edge;
	Mat roi;
	Mat roi2;
	Mat roi3;
	//int Offset = 400;
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
	//Mat edge_mat;
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	bilateralFilter(gray, blr, -1, 20, 20);
	//bilateralFilter(blr, blr2, -1, 20, 20);
	Canny(blr, edge, 80, 200);
	edge_mat = edge;
	
	//vector<Vec4i> liness;
	//HoughLinesP(edge_mat, liness, 1, CV_PI / 180, 20, 10, 5);

	/*
	Mat dst2;
	cvtColor(edge_mat, dst2, COLOR_GRAY2BGR);
	for (size_t i = 0; i < liness.size(); i++) {
		Vec4i l = liness[i];
		line(dst2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
	}
	*/

	imshow("edge", edge_mat);

}

void Edge::HoughMake()
{
	int Offset = 390;
	height = 20;
	roi = edge_mat(Rect(0, Offset - 15, 640, height));
	//roi2 = edge_mat(Rect(0, Offset - 20, 640, 40));
	//roi3 = edge_mat(Rect(0, Offset - 10, 640, 20));

	//vector<Vec4i> lines, lines2, lines3; // int 자료형 4개
	HoughLinesP(roi, lines, 1, CV_PI / 180, 18, 18, 10);
	//HoughLinesP(roi2, lines2, 1, CV_PI / 180, 30, 30, 10);
	//HoughLinesP(roi3, lines3, 1, CV_PI / 180, 20, 20, 10);
	
	Mat dst2;
	cvtColor(roi, dst2, COLOR_GRAY2BGR);
	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l = lines[i];
		line(dst2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
	}

	imshow("roi", dst2);
	//imshow("roi2", roi2);
	//imshow("roi3", roi3);
}

void Edge::PointMake()
{
	for (size_t i = 0; i < lines.size(); i++) {

		Vec4i l = lines[i];
		
		if ((-0.2 < int(float(l[0] - l[2]) / float(l[1] - l[3])) && int(float(l[0] - l[2]) / float(l[1] - l[3])) < 0) || (0 < int(float(l[0] - l[2]) / float(l[1] - l[3]) > 0.2)) && int(float(l[0] - l[2]) / float(l[1] - l[3])) < 0.2) continue;
		cout << l << endl;

		mid_val = (height/2 - l[1]) / (l[1] - l[3]) * (l[0] - l[2]) + l[0];

		if (mid_val < 340) Lpoints.push_back(mid_val);
		else Rpoints.push_back((mid_val));

	/*
	for (auto i : Lpoints) {
		cout << i << " ";
	}
	std::cout << "\n";

	for (auto i : Rpoints) {
		cout << i << " ";
	}
	std::cout << "\n";
	*/


	}

		/*
		if (l[1] <= 0 || l[3] <= 0) {
			if (float(l[1] - l[3]) == 0) {
				offpoint = l[1];
			}
			else {
				offpoint = (float(l[0] - l[2]) / float(l[1] - l[3])) * float(15 - l[3]) + l[2];
			}
			points.push_back(int(offpoint));
		}
		*/

	
	/*
	for (size_t i = 0; i < HoughLine2.size(); i++) {
		Vec4i l = HoughLine2[i];
		if (l[1] <= 3 || l[3] <= 3) {
			if (float(l[1] - l[3]) == 0) {
				offpoint2 = l[1];
			}
			else {
				offpoint2 = (float(l[0] - l[2]) / float(l[1] - l[3])) * float(20 - l[3]) + l[2];
			}
			points.push_back(int(offpoint2));
		}

	}

	for (size_t i = 0; i < HoughLine3.size(); i++) {
		Vec4i l = HoughLine3[i];
		if (l[1] <= 3 || l[3] <= 3) {
			if (float(l[1] - l[3]) == 0) {
				offpoint3 = l[1];
			}
			else {
				offpoint3 = (float(l[0] - l[2]) / float(l[1] - l[3])) * float(10 - l[3]) + l[2];
			}
			points.push_back(int(offpoint3));
		}

	}
	*/

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
	
	//imshow("src", src);

	waitKey(0);


}