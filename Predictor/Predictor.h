//
// Created by tyy on 2022/1/19.
//

//#ifndef MASTER_PREDICTOR_H
#define MASTER_PREDICTOR_H

#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "utility.hpp"
#include "Kalman.hpp"
#include "RMKF.hpp"
#include "SolveAngle.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;

class Predictor{
public:
    Predictor();
    ~Predictor();

    void armorPredictor(const Vector3f& target_ypd, float v_);

    Vector3f kalmanPredict(Vector3f target_xyz, float v_, float t);

    Vector3f getGyroXYZ(Vector3f target_ypd);

    void Refresh();

    vector<Point2f>predict_point;

    float yaw, pitch;

    Vector3f predict_xyz;
    Vector3f predict_ypd;

    Vector3f target_xyz{};
    Vector3f target_v_xyz{};
    Vector3f target_a_xyz{};

private:
    /**
     * @brief RMKF更新，包括预测部分和更正部分
     * @param z_k 观测量 默认为三维坐标 x y z
     */
    void UpdateKF(const Vector3f& z_k);
    /**
     * @brief 迭代更新卡尔曼滤波器一定次数达到预测
     * @param KF 当前卡尔曼滤波器
     * @param iterate_times 迭代次数
     * @return 预测目标xyz坐标
     */
    Vector3f PredictKF(EigenKalmanFilter KF, const int& iterate_times);
    /**
     * @brief 初始化转移矩阵
     * */
    void InitTransMat(const float dt);
    /**
     * @brief 初始化匀加速kalman模型
     * */
    void InitKfAcceleration(const float dt);

    vector<Vector3f> abs_pyd;

    float degree2rad = CV_PI / 180;

    vector<float> abs_yaw;
    vector<float> frame_list;
    vector<float> time_list;

    float frame = 30.0; //预测帧数，根据情况预测可能从帧改变为时间
    int step = 10;

    RMTools::DisPlayWaveCLASS waveClass;

    EigenKalmanFilter RMKF = EigenKalmanFilter(9, 3);
    bool RMKF_flag = false;
    float pre_t = 0.5;
    float delta_t = 0.015; // s

    SolveAngle solveAngle;
};

//#endif //MASTER_PREDICTOR_H
