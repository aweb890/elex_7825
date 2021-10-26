#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include <opencv2/aruco/charuco.hpp>

CRobot::CRobot()
{
	_image_size = Size(1280, 768);
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);

	//////////////////////////////////////
	// Virtual Camera Setup (set up virtual camera
	float focal_length = 0.003;
	float pixel_size = 0.0000046;
	Point2f principal_point = _image_size / 2;

	/// TODO Setup of the virtual camera transformation matrix
	//_cam_virtual_intrinsic = (Mat1f(3, 4) << ...;
	//_cam_virtual_extrinsic = createHT(0, 0, 0, 0, 0, 0);

	// Create and move window if desired
	cv::namedWindow("7825 Project");
	cv::moveWindow("7825 Project", 150, 0);
}

CRobot::~CRobot()
{
}

// Create Homogeneous Transformation Matrix
Mat CRobot::createHT(float tx, float ty, float tz, float rx, float ry, float rz)
{
	/// TODO Setup of the transformation matrix

	return (Mat1f(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
}


std::vector<Mat> CRobot::createBox(float w, float h, float d)
{
	std::vector <Mat> box;

	// The 8 vertexes with the
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

	// Shift box so origin is center of the leftmost side
	Mat T = createHT(w / 2, 0, 0, 0, 0, 0);
	for (int i = 0; i < box.size(); i++)
	{
		box.at(i) = T * box.at(i);
	}

	return box;
}

// Apply transform to each of the box points
void CRobot::transformBox(std::vector<Mat>& box, Mat T)
{
	for (int i = 0; i < box.size(); i++)
	{
		box.at(i) = T * box.at(i);
	}
}

// Draw box with lines connecting each vertex, shift origin to center of image (from top left corner)
void CRobot::drawBox(Mat& im, std::vector<Mat> box, Scalar colour)
{
	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	// Project 3D box vertexes to 2D image points
	transformBox(box, _cam_virtual_intrinsic * _cam_virtual_extrinsic);

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = Point2f(box.at(draw_box1[i]).at<float>(0, 0) / box.at(draw_box1[i]).at<float>(2, 0), _image_size.height - box.at(draw_box1[i]).at<float>(1, 0) / box.at(draw_box1[i]).at<float>(2, 0));
		Point pt2 = Point2f(box.at(draw_box2[i]).at<float>(0, 0) / box.at(draw_box2[i]).at<float>(2, 0), _image_size.height - box.at(draw_box2[i]).at<float>(1, 0) / box.at(draw_box2[i]).at<float>(2, 0));

		line(im, pt1, pt2, colour, 1);
	}
}

// Draw 3 coordinate axes X/Y/Z 
void CRobot::drawPose(Mat& im, Mat T)
{
	float axis_length = 0.05;

	Mat O = (Mat1f(4, 1) << 0, 0, 0, 1);
	Mat X = (Mat1f(4, 1) << axis_length, 0, 0, 1);
	Mat Y = (Mat1f(4, 1) << 0, axis_length, 0, 1);
	Mat Z = (Mat1f(4, 1) << 0, 0, axis_length, 1);


	// Camera projection transformation matrix
	Mat cT = _cam_virtual_intrinsic * _cam_virtual_extrinsic;

	O = cT * T * O;
	X = cT * T * X;
	Y = cT * T * Y;
	Z = cT * T * Z;

	// Draw lines from O to axis_length along each axis
	Point pt1, pt2;
	pt1 = Point2f(O.at<float>(0, 0) / O.at<float>(2, 0), _image_size.height - O.at<float>(1, 0) / O.at<float>(2, 0));
	pt2 = Point2f(X.at<float>(0, 0) / X.at<float>(2, 0), _image_size.height - X.at<float>(1, 0) / X.at<float>(2, 0));
	line(im, pt1, pt2, CV_RGB(255, 0, 0), 1); // X
	pt2 = Point2f(Y.at<float>(0, 0) / Y.at<float>(2, 0), _image_size.height - Y.at<float>(1, 0) / Y.at<float>(2, 0));
	line(im, pt1, pt2, CV_RGB(0, 255, 0), 1); // Y 
	pt2 = Point2f(Z.at<float>(0, 0) / Z.at<float>(2, 0), _image_size.height - Z.at<float>(1, 0) / Z.at<float>(2, 0));
	line(im, pt1, pt2, CV_RGB(0, 0, 255), 1); // Z
}

void CRobot::draw()
{
	// Create a pose at origin
	Mat O = createHT(0, 0, 0, 0, 0, 0);

	// Create a box
	std::vector <Mat> obj = createBox(0.2, 0.2, 0.2);

	// Rotate camera about box
	for (float theta = 0; theta < 2 * CV_PI; theta = theta + 0.01)
	{
		// Create canvas image
		_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

		_cam_virtual_extrinsic = createHT(0, 0, 0.5, 0, theta, 0);

		drawBox(_canvas, obj, CV_RGB(255, 0, 0));
		drawPose(_canvas, O);

		// Note that createHT must be implemented to transform (move) objects in space
		imshow("7825 Project", _canvas);
		cv::waitKey(10);
	} 
}

bool CRobot::save_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::WRITE);

	if (!fs.isOpened())
	{
		return false;
	}

	fs << "camera_matrix" << cam;
	fs << "distortion_coefficients" << dist;

	return true;
}

bool CRobot::load_camparam(string filename, Mat& cam, Mat& dist)
{
	FileStorage fs(filename, FileStorage::READ);

	if (!fs.isOpened())
	{
		return false;
	}

	fs["camera_matrix"] >> cam;
	fs["distortion_coefficients"] >> dist;

	return true;
}

void CRobot::calibrate_board(int cam_id)
{
	// Calib data
	vector<vector<vector<Point2f>>> calib_corner;
	vector<vector<int>> calib_id;
	vector<Mat> calib_im;
	Size calib_im_size;

	// Board settings (measure your printout)
	float size_square = 0.04;
	float size_mark = 0.017;
	Size board_size = Size(5, 7);
	int dictionary_id = aruco::DICT_6X6_250;

	string cam_file = "camparam.xml";

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id));

	Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(board_size.width, board_size.height, size_square, size_mark, dictionary);
	Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

	VideoCapture inputVideo;
	inputVideo.open(cam_id);

	// Collect data from live video 
	while (inputVideo.grab())
	{
		Mat im, draw_im;
		vector<int> corner_ids;
		vector<vector<Point2f>> corners, rejected_corners;
		Mat corner_Charuco, id_Charuco;

		// Get image
		inputVideo.retrieve(im);
		im.copyTo(draw_im);

		// First pass detect markers
		aruco::detectMarkers(im, dictionary, corners, corner_ids, detectorParams, rejected_corners);
		// Second pass detect markers
		aruco::refineDetectedMarkers(im, board, corners, corner_ids, rejected_corners);

		// Refine charuco corners
		if (corner_ids.size() > 0)
		{
			aruco::interpolateCornersCharuco(corners, corner_ids, im, charucoboard, corner_Charuco, id_Charuco);
		}

		// Draw detected corners 
		if (corner_ids.size() > 0)
		{
			aruco::drawDetectedMarkers(draw_im, corners);
		}

		// Draw detected ChArUco corners
		if (corner_Charuco.total() > 0)
		{
			aruco::drawDetectedCornersCharuco(draw_im, corner_Charuco, id_Charuco);
		}

		putText(draw_im, "Press 'c' to add current frame. 'ESC' to finish and calibrate", Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
		imshow("out", draw_im);

		char key = (char)waitKey(10);
		if (key == 27) break;
		if (key == 'c' && corner_ids.size() > 0)
		{
			cout << "Frame captured" << endl;
			calib_corner.push_back(corners);
			calib_id.push_back(corner_ids);
			calib_im.push_back(im);
			calib_im_size = im.size();
		}
	}

	if (calib_id.size() < 1) {
		cerr << "Not enough captures for calibration" << endl;
		return;
	}

	Mat cameraMatrix, distCoeffs;
	vector< Mat > rvecs, tvecs;
	double repError;

	int calibrationFlags = 0;
	double aspectRatio = 1;

	if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
		cameraMatrix = Mat::eye(3, 3, CV_64F);
		cameraMatrix.at< double >(0, 0) = aspectRatio;
	}

	// prepare data for calibration
	vector< vector< Point2f > > allCornersConcatenated;
	vector< int > allIdsConcatenated;
	vector< int > markerCounterPerFrame;
	markerCounterPerFrame.reserve(calib_corner.size());
	for (unsigned int i = 0; i < calib_corner.size(); i++) {
		markerCounterPerFrame.push_back((int)calib_corner[i].size());
		for (unsigned int j = 0; j < calib_corner[i].size(); j++) {
			allCornersConcatenated.push_back(calib_corner[i][j]);
			allIdsConcatenated.push_back(calib_id[i][j]);
		}
	}

	// calibrate camera using aruco markers
	double arucoRepErr;
	arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
		markerCounterPerFrame, board, calib_im_size, cameraMatrix, distCoeffs, noArray(), noArray(), calibrationFlags);

	// prepare data for charuco calibration
	int nFrames = (int)calib_corner.size();
	vector< Mat > allCharucoCorners;
	vector< Mat > allCharucoIds;
	vector< Mat > filteredImages;
	allCharucoCorners.reserve(nFrames);
	allCharucoIds.reserve(nFrames);

	for (int i = 0; i < nFrames; i++) {
		// interpolate using camera parameters
		Mat currentCharucoCorners, currentCharucoIds;
		aruco::interpolateCornersCharuco(calib_corner[i], calib_id[i], calib_im[i], charucoboard,
			currentCharucoCorners, currentCharucoIds, cameraMatrix,
			distCoeffs);

		allCharucoCorners.push_back(currentCharucoCorners);
		allCharucoIds.push_back(currentCharucoIds);
		filteredImages.push_back(calib_im[i]);
	}

	if (allCharucoCorners.size() < 4) {
		cerr << "Not enough corners for calibration" << endl;
		return;
	}

	// calibrate camera using charuco
	repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, calib_im_size, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

	bool saveOk = save_camparam("cam_param.xml", cameraMatrix, distCoeffs);
	if (!saveOk) {
		cerr << "Cannot save output file" << endl;
		return;
	}

	cout << "Rep Error: " << repError << endl;
	cout << "Rep Error Aruco: " << arucoRepErr << endl;
	cout << "Calibration saved to " << "cam_param.xml" << endl;

	// show interpolated charuco corners for debugging
	for (unsigned int frame = 0; frame < filteredImages.size(); frame++)
	{
		Mat imageCopy = filteredImages[frame].clone();

		if (calib_id[frame].size() > 0) {

			if (allCharucoCorners[frame].total() > 0)
			{
				aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame], allCharucoIds[frame]);
			}
		}

		imshow("out", imageCopy);
		char key = (char)waitKey(0);
		if (key == 27) break;
	}
}