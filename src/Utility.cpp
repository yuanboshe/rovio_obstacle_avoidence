#include "Utility.h"
#include <stdio.h>

Utility::Utility(void)
{
}


Utility::~Utility(void)
{
}


void Utility::saveAsMatlab3D(Mat img, string path)
{
	Mat rv;
	FILE* fp;
	fp = fopen(path.c_str(), "w");
	cvtColor(img, rv, CV_RGB2GRAY);
	assert(rv.type() == CV_8UC1);
	string content = "";
	int rows = rv.rows, cols = rv.cols;
	for (int i = 0; i < rows; i++)
	{
		uchar* pRow = rv.ptr<uchar>(i);
		for (int j = 0; j < cols; j++)
		{
			stringstream ss;
			ss << pRow[j];
			string tmp = ss.str();
			content.append(tmp).append(" ");
		}
		content.append("\n");
	}
	fwrite(content.c_str(), content.size(), 1, fp);
	fclose(fp);
}

void Utility::drawSegmentBorder(InputOutputArray imgInputOutput, Mat_<int> segment, Scalar color /*= Scalar(255, 255, 255)*/)
{
	Mat img = imgInputOutput.getMat();
	Mat_<uchar> mask(img.rows, img.cols); // ��־SuperPixel�߽�ľ���
	mask.setTo(0);
	int rows = img.rows;
	int cols = img.cols;
	for (int i = 1; i < rows; i++)
	{
		int* pCurrentPoint = segment.ptr<int>(i)+1; // ָ��ǰ��
		int* pUpperPoint = segment.ptr<int>(i-1)+1; // ָ������ĵ�
		for (int j = 1; j < cols; j++)
		{
			int cPoint = *pCurrentPoint; // ��ǰ���SuperPixelID
			int lPoint = *(pCurrentPoint-1); // ��ߵ��SuperPixelID
			int uPoint = *pUpperPoint; // ������SuperPixelID
			if (cPoint != lPoint || cPoint != uPoint)
			{
				mask(i, j) = 1;
			}
			pCurrentPoint++;
			pUpperPoint++;
		}
	}
	add(img, color, img, mask);
}
