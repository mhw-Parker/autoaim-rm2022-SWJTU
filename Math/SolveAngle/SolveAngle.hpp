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
    /**
     * @brief 用 pnp 解算目标相对相机坐标系的 yaw pitch
     * @param armor_mode 大装甲板还是小装甲
     * @param pts 装甲板 4 点坐标
     * @param v_ 弹速
     * */
    void GetPoseV(const vector<Point2f>& pts, bool armor_mode, Vector3f gimbal_ypd); //Pnp模型

    /**
     * @brief 弹道补偿函数
     * @param fitXYZ 相机相对枪口 左 上 前 为正 单位为：mm
     * @param v 当前弹速
     * */
    void Compensator(Vector3f cam_xyz, float v);
    float CalPitch(Vector3f target_xyz, float v, float &t) const;
    float pitchCompensate(Vector3f target_xyz, float v);

    /**
     * @brief 将预测点反投影到图像上
     * @param src 预测点的（ yaw, pitch, dist ）坐标
     * @param target_xyz 目标的陀螺仪绝对坐标
     * @param gimbal_ypd 旋转矩阵
     * */
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
    bool shoot;

private:
    void Generate3DPoints(bool mode);
    void camXYZ2YPD();
    void GunXYZ2YPD(Vector3f cam_xyz);

    Vector3f Cam2World(Vector3f cam_xyz);
    Vector3f World2Cam(Vector3f world_xyz);
    Vector3f Cam2Pixel(Vector3f cam_xyz);

    Matrix3f Ry, Rp, cam2world_mat;

    float g = 9.8; //
    float g2 = 9.8*9.8;

    float degree2rad = CV_PI / 180;
    Matrix3f cam_mat;

    Mat tvecs;
    Mat rvecs;

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
};
