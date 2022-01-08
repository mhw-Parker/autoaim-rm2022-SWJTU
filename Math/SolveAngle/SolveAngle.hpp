#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <iostream>


using namespace std;
using namespace cv;
using namespace Eigen;

class SolveAngle
{
public:
	SolveAngle();

    float scale = 0.99f;
	float f_ = 1500;

    float yaw,pitch,dist;
	vector<Point2f> rectPoint2D;

	bool shoot;
    Mat tvecs;

	void Generate2DPoints(Rect rect);
	void GetPose(const Rect& rect, float ballet_speed, bool small);
    void GetPoseV(const vector<Point2f>& pts, bool armor_mode); //Pnp模型
    void GetPoseSH(const Point2f p); //小孔成像模型
	void Generate3DPoints(bool mode);

    void camXYZ2YPD(Mat tvecs);
    void backProjection(Mat tvecs, Mat rvecs, Vector3d obj_p_ypd, vector<Point2f> &img_p);


private:
    void compensator();
	Mat_<double> cameraMatrix;
	Mat_<double> distortionCoefficients;
	Mat rvecs;
	vector<Point3f> targetPoints3D;
	float targetWidth3D{};
	float targetHeight3D{};

    Matrix3f camMat_E;
	int shootPriority = 0;
	float averageX;
	float averageY;

    float yaw_static, pitch_static;

	int value;
    Mat waveBG = Mat(480,640,CV_8UC3,Scalar(0,0,0));
    Vector3d p_cam_xyz; //相机坐标系下的x,y,z
    Vector3d p_wld; //世界坐标系下的x,y,z
};
