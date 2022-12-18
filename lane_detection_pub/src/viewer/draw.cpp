#include "draw.h"

namespace viewer{
void DRAW::draw_rectangle(cv::Mat& img,int lpos,int rpos,int offset){
        int center = (lpos + rpos) / 2;

        rectangle(img, Rect(Point(lpos - 5, 15 + offset),
                        Point(lpos + 5, 25 + offset)),
                        Scalar(0, 255, 0), 2);
        rectangle(img, Rect(Point(rpos - 5, 15 + offset),
                        Point(rpos + 5, 25 + offset)),
                        Scalar(0, 255, 0), 2);
        rectangle(img, Rect(Point(center-5, 15 + offset),
                        Point(center+5, 25 + offset)),
                        Scalar(0, 255, 0), 2);
        rectangle(img, Point(315, 15 + offset),
                        Point(325, 25 + offset),
                        Scalar(0, 0, 255), 2);
    }

void DRAW::draw_lines(Mat& img,std::vector<Vec4i>& lines,int Offset){
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> uni(0,255);

    for (size_t i = 0; i < lines.size(); i++){
        Vec4i l = lines[i];
        Scalar color = Scalar(uni(rng), uni(rng), uni(rng));
        line(img, Point(l[0], l[1]+Offset), Point(l[2], l[3]+Offset), color, 2);
    }
};

void DRAW::draw_point(Mat& img, Point pt, Scalar color)
{
	line(img, pt, pt, color, 4, LINE_AA);
};
}