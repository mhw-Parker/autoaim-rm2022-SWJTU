#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include "utility.hpp"


using namespace std;
using namespace cv;
using namespace Eigen;

class SolveAngle
{
public:
    SolveAngle();

    void Generate2DPoints(Rect rect);

    void GetPoseV(const vector<Point2f>& pts, bool armor_mode, Vector3f gimbal_ypd); //Pnp模型

    float CalPitch(Vector3f target_xyz, float v, float &t) const;
    float iteratePitch(Vector3f target_xyz, float v, float &t);

    void backProject2D(Mat &src, Vector3f target_xyz);
    Point2f getBackProject2DPoint(Vector3f target_xyz);

    float scale = 0.99f;
    float f_ = 1500;

    float yaw{};
    float pitch{};
    float dist{};
    vector<Point2f> rectPoint2D;
    Vector3f p_cam_xyz; //相机坐标系下的x,y,z
    Vector3f world_xyz;
    Matrix3f r_mat;
    bool shoot;
    float yaw_;

private:
    void Generate3DPoints(bool mode);
    void camXYZ2YPD();
    void GunXYZ2YPD(Vector3f cam_xyz);

    Vector3f Cam2World(Vector3f cam_xyz);
    Vector3f World2Cam(Vector3f world_xyz);
    Vector3f Cam2Pixel(Vector3f cam_xyz);

    Matrix3f Ry, Rp, cam2world_mat;

    float degree2rad = CV_PI / 180;
    Matrix3f cam_mat;

    Mat tvecs;
    Mat rvecs;
    Mat R;

    Vector3f r_vec;

    Mat_<double> cameraMatrix;
    Mat_<double> distortionCoefficients;
    vector<Point3f> targetPoints3D;
    float targetWidth3D{};
    float targetHeight3D{};

    Matrix3f camMat_E;
    int shootPriority = 0;
    float averageX;
    float averageY;

    float yaw_static{};
    float pitch_static{};
    float x_static, y_static;

    Vector3f fit_xyz;
    Vector3f gun_xyz; //枪口坐标系下的x,y,z
    float g = 9.8; //
    float g2 = 9.8*9.8;
    float fit_gun_error = 0;// m
    // 阻力系数
    float coeff = 0.025;
};
