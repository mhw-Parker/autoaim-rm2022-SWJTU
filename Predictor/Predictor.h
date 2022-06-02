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
#include "mydefine.h"


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
    void Refresh();
    cv::Point2f predict_point;
    float latency = 0.5, fly_t = 0.2, react_t = 0.3;
    Vector3f target_ypd, delta_ypd{0,0,0}, predict_ypd{};
    Vector3f target_xyz{}, predict_xyz{};
    Vector3f last_xyz{};
    // 弹速相关
    float average_v_bullet;
    float v_vec[4]{};
    int v_vec_pointer = 1;
    // 丢失目标
    short max_lost = showArmorBox ? 7 : 14;

private:
    void UpdateTimeStamp(float &dt);
    void TimeRefresh();
    SolveAngle solveAngle;
    vector<float> time_series;
    float total_t = 0;
    float degree2rad = CV_PI / 180;

public:
    void ArmorPredictor(vector<Point2f> &target_pts, const int& armor_type,
                        const Vector3f &gimbal_ypd, float v_, float dt,
                        int lost_cnt);
    Vector3f target_v_xyz{};
    Vector3f target_a_xyz{};

private:
    Vector3f KalmanPredict(float v_, float t);
    Vector3f GetGyroXYZ();
    Vector3f PredictKF(EigenKalmanFilter KF, const int& iterate_times);
    inline void KalmanRefresh();
    inline void KalmanShallowRefresh();
    void UpdateKF(const Vector3f& z_k);
    void InitKfAcceleration(const float dt);

    RMTools::DisPlayWaveCLASS waveClass;

    // 自瞄装甲板Kalman
    EigenKalmanFilter RMKF = EigenKalmanFilter(9, 3);
    // flag为false则初始化Kalman，否则更新Kalman
    bool RMKF_flag = false;
    float delta_t = 0.016; // s
    float predict_dt = 0.05; // 预测时拉长步长节省计算，sentry:0.03

public:
    void EnergyPredictor(uint8_t mode, vector<Point2f> &target_pts, Point2f &center, const Vector3f &gimbal_ypd, float v_, float dt);
    void EnergyRefresh();
    vector<Point2f> predict_pts;

private:
    /** energy machine common function **/
    bool JudgeFanRotation();
    float CalOmegaNStep(int step, float &total_theta);
    bool EnergyStateSwitch();

    float total_theta = 0;
    float current_theta = 0;
    float current_omega = 0;
    vector<float> angle;
    vector<float> omega;

    Point2f last_point;
    float dt_ = 0.01;

    int energy_rotation_direction = 1;//风车旋转方向 1:顺时针 -1：逆时针
    u_int8_t ctrl_mode = STANDBY;

    RMTools::DisPlayWaveCLASS omegaWave;

    /**---- energy machine kalman filter ----**/
    void FilterOmega(const float &dt);
    void FilterRad(const float &latency);
    void initFanRotateKalman();
    void initFanRadKalman();
    EigenKalmanFilter omega_kf = EigenKalmanFilter(3, 2, 1);
    EigenKalmanFilter rad_kf = EigenKalmanFilter(3,1);
    vector<float> filter_omega;
    bool energy_flag = false;

    /**---- estimate big fan rotation speed parameters ----**/
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
    float predict_rad;

};

//#endif //MASTER_PREDICTOR_H
