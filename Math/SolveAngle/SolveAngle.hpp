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

    void GetPoseV(const vector<Point2f>& pts, int armor_mode, const Vector3f& gimbal_ypd); //Pnp模型

    float iteratePitch(Vector3f target_xyz, float v, float &t);

    Point2f getBackProject2DPoint(Vector3f target_xyz);
    /** get rotate matrix **/
    Matrix3f GetRotateMat(const Vector3f &gimbal_ypd);
    /** from camera to world **/
    Vector3f Cam2World(const Vector3f &gimbal_ypd, const Vector3f &cam_xyz);
    Vector3f Cam2Gim(const Vector3f &cam_xyz);
    Vector3f Gim2World(const Vector3f &gim_xyz);
    /** from world to camera **/
    Vector3f World2Cam(const Vector3f &world_xyz);
    Vector3f World2Gim(const Vector3f &world_xyz);
    Vector3f Gim2Cam(const Vector3f &gim_xyz);
    /** from cam to pixel **/
    Vector3f Cam2Pixel(const Vector3f &cam_xyz);
    /** from relative xyz to delta yaw, pitch, distance **/
    Vector3f xyz2ypd(const Vector3f &_xyz);

    float yaw{};
    float pitch{};
    float dist{};
    vector<Point2f> rectPoint2D;

    Vector3f p_cam_xyz; //相机坐标系下的x,y,z
    Vector3f gim_xyz; //平移到云台中心后的目标坐标
    Vector3f world_xyz; //目标世界坐标
    Matrix3f r_mat;

    float yaw_;

private:
    void Generate3DPoints(const int targetSize);
    void camXYZ2YPD();

    [[maybe_unused]] void GunXYZ2YPD(Vector3f cam_xyz);

    Vector3f gim_xyz_error{0,0,0};
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

    float x_static, y_static;

    Vector3f fit_xyz;
    Vector3f gun_xyz; //枪口坐标系下的x,y,z
    float g = 9.8; //
    float g2 = 9.8*9.8;
    float fit_gun_error = 0;// m
    // 阻力系数
    float coeff = 0.025;
};
