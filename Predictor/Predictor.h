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
#include <ceres/ceres.h>

#include "utility.hpp"
#include "Kalman.hpp"
#include "RMKF.hpp"
#include "SolveAngle.hpp"


#ifndef STANDBY
#define STANDBY 1
#endif

#ifndef BEGIN
#define BEGIN 2
#endif

#ifndef ESTIMATE
#define ESTIMATE 3
#endif

#ifndef PREDICT
#define PREDICT 4
#endif

using namespace std;
using namespace cv;
using namespace Eigen;

class Predictor{
public:
    Predictor();
    ~Predictor();
    cv::Point2f predict_point;
    SolveAngle solveAngle;

public:
    void armorPredictor(const Vector3f& target_ypd, float v_);

    Vector3f kalmanPredict(Vector3f target_xyz, float v_, float t);

    Vector3f getGyroXYZ(Vector3f target_ypd);

    void Refresh();

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
    float latency = 0.5;
    float fly_t = 0.2;
    float delta_t = 0.015; // s

public:
    void BigEnergyPredictor(vector<Point2f> target_pts, Point2f center, float latency, float dt);
    vector<Point2f> predict_pts;

private:
    /** energy machine common function **/
    bool JudgeFanRotation();
    float calOmega(int step, float &total_theta);
    bool EnergyStateSwitch();

    float total_t = 0;
    float total_theta = 0;
    float current_theta = 0;
    float current_omega = 0;
    vector<float> time_series, angle;
    vector<float> omega;

    int energy_rotation_direction = 1;//风车旋转方向 1:顺时针 -1：逆时针
    u_int8_t ctrl_mode = STANDBY;

    RMTools::DisPlayWaveCLASS omegaWave;

    /**---- energy machine kalman filter ----**/
    void FilterOmega(const float dt);
    void FilterRad(const float latency);
    void initFanRotateKalman();
    void initFanRadKalman();
    EigenKalmanFilter omega_kf = EigenKalmanFilter(3, 2, 1);
    EigenKalmanFilter rad_kf = EigenKalmanFilter(3,1);
    vector<float> filter_omega;

    /**---- estimate rotation speed parameter ----**/
    void estimateParam(vector<float> &omega_, vector<float> &t_);
    ceres::Problem problem;
    double a_ = 0.780, w_ = 1.9, phi_ = 0; //参数初值
    int st = 0;
    struct SinResidual{
        SinResidual(double t, double omega): omega_(omega), t_(t) {}

        template<class T>
        bool operator()(const T* const a,const T* const w, const T* const phi, T* residual) const{
            residual[0] = omega_ - (a[0] * sin(w[0] * t_ + phi[0]) + 2.09 - a[0]); // spd = a*sin(w*t + phi) + b
            return true;
        }
    private:
        const double omega_;
        const double t_;
    };

    /**---- energy machine predictor function ----**/
    float spdInt(float t);
    float spdPhi(float omega, int flag);
    Point2f calPredict(Point2f &p, Point2f &center, float theta) const;
    void getPredictRect(Point2f &center, vector<Point2f> &pts, float theta);

    int change_cnt;
    bool last_flag, flag = 1;
    float react_t = 0.2;
    float predict_rad;

};

//#endif //MASTER_PREDICTOR_H
