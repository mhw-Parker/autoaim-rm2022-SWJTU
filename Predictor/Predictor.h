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

using namespace std;
using namespace cv;
using namespace Eigen;

class Predictor{
public:
    Predictor();
    ~Predictor();

    void armorPredictor(Vector3f target_ypd, Vector3f gimbal_ypd, int direct = 1);

    void InitKfAcceleration(const float dt);

    float kalmanPredict(Vector3f target_xyz, int direct);

    Vector3f getGyroXYZ(Vector3f target_ypd, int direct);

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
    void UpdateKF(Vector3f z_k);
    /**
     * @brief 迭代更新卡尔曼滤波器一定次数达到预测
     * @param KF 当前卡尔曼滤波器
     * @param iterate_times 迭代次数
     * @return 预测目标xyz坐标
     */
    Vector3f PredictKF(EigenKalmanFilter KF, const int& iterate_times);

    vector<Vector3f> abs_pyd;
    Vector3f gim_d_ypd;
    float degree2rad = CV_PI / 180;

    vector<float> abs_yaw;
    vector<float> frame_list;
    vector<float> time_list;
    vector<float> target_x;
    vector<float> target_z;
    float frame = 30.0; //预测帧数，根据情况预测可能从帧改变为时间
    Eigen::MatrixXd k;

    double rate[4] = {0,0,0,0}, yaw_arr[4] = {0,0,0,0};

    RMTools::DisPlayWaveCLASS waveClass;

    Kalman kf;
    bool kf_flag = false;
    EigenKalmanFilter RMKF = EigenKalmanFilter(9, 3);
    bool RMKF_flag = false;
    Vector3f z_k; //观测量



};

//#endif //MASTER_PREDICTOR_H
