#pragma once

#include <opencv2/opencv.hpp>

using namespace std;

using namespace cv;
using namespace dnn;

class CRobot
{
public:
	CRobot();
	~CRobot();
private:
	Mat _canvas;
	Size _image_size;

	Mat _cam_virtual_intrinsic;
	Mat _cam_virtual_extrinsic;

	std::vector<Mat> createBox(float w, float h, float d);
	void transformBox(std::vector<Mat>& box, Mat T);
	void drawBox(Mat& im, std::vector<Mat> box, Scalar colour);
	void drawPose(Mat& im, Mat T);

	bool save_camparam(string filename, Mat& cam, Mat& dist);
	bool load_camparam(string filename, Mat& cam, Mat& dist);

public:

	Mat createHT(float tx, float ty, float tz, float rx, float ry, float rz);

	void calibrate_board(int cam_id);

	void draw();
};

