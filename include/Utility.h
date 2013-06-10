#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
class Utility
{
public:
	Utility(void);
	~Utility(void);
	void saveAsMatlab3D(Mat img, string path);
	void drawSegmentBorder(InputOutputArray imgInputOutput, Mat_<int> segment, Scalar color = Scalar(255, 255, 255));
};

