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
	void GetPose(const Rect& rect, float ballet_speed, bool small);
    /**
     * @brief 用 pnp 解算目标相对相机坐标系的 yaw pitch
     * @param armor_mode 大装甲板还是小装甲
     * @param pts 装甲板 4 点坐标
     * @param v_ 弹速
     * */
    void GetPoseV(const vector<Point2f>& pts, bool armor_mode, const float v_); //Pnp模型

	void Generate3DPoints(bool mode);
    /**
     * @brief 弹道补偿函数
     * @param fitXYZ 相机相对枪口 左 上 前 为正 单位为：mm
     * @param v 当前弹速
     * */
    void Compensator(Vector3f cam_xyz, float v);

    void pitchCompensate();
    /**
     * @brief 将预测点反投影到图像上
     * @param src 预测点的（ yaw, pitch, dist ）坐标
     * @param target_xyz 目标的陀螺仪绝对坐标
     * @param gimbal_ypd 旋转矩阵
     * */
    void backProject2D(Mat &src, Vector3f target_xyz, Vector3f gimbal_ypd, int direct_y = 1,int direct_p = 1);

    float scale = 0.99f;
    float f_ = 1500;

    float yaw = 0,pitch = 0,dist = 0;
    vector<Point2f> rectPoint2D;
    Vector3f p_cam_xyz; //相机坐标系下的x,y,z
    Vector3f ypd;
    bool shoot;

private:
    void camXYZ2YPD(Mat tvecs);
    int direct = 1;
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

    float yaw_static, pitch_static;
    float x_static, y_static;

    Vector3f fit_xyz;
    Vector3f gun_xyz; //枪口坐标系下的x,y,z
};
