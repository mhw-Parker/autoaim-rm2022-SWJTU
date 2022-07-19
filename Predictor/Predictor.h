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


#ifndef INIT
#define INIT 1
#endif

#ifndef STANDBY
#define STANDBY 2
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
    // 预测时长
    float latency = 0.5;
    // 子弹飞行时长
    float fly_t = 0.2;
    // 电机响应时间
    float react_t{};
    Vector3f target_ypd, delta_ypd{0,0,0}, predict_ypd{};
    // 各车偏置补偿
    Vector3f offset{};
    // 打符专用补偿
    Vector3f energy_offset{};
    // 给电控发数据补偿
    Vector3f back_ypd{};

    Vector3f target_xyz{}, predict_xyz{};
    Vector3f last_xyz{};
    Vector3f target_v_xyz{};
    Vector3f target_a_xyz{};
    // 弹速相关
    float average_v_bullet;
    float v_vec[4]{};
    int v_vec_pointer = 1;

private:
    void InitParams();
    void UpdateTimeStamp(float &dt);
    void TimeRefresh();
    SolveAngle solveAngle;
    vector<float> t_list, time_series;
    float total_t = 0;
    float degree2rad = CV_PI / 180;

public:
    void ArmorPredictor(vector<Point2f> &target_pts, const int& armor_type,
                        const Vector3f &gimbal_ypd, float v_, float dt,
                        int lost_cnt);
    // 哨兵自动射击
    uint8_t CheckShoot(const Vector3f& gimbal_ypd, const int& armor_type);
    uint8_t shootCmd{};

private:
    Vector3f KalmanPredict(float v_, float t);
    Vector3f GetGyroXYZ();
    Vector3f PredictKF(EigenKalmanFilter KF, const int& iterate_times);
    inline void KalmanRefresh();
    inline void KalmanShallowRefresh();
    void UpdateKF(const Vector3f& z_k);
    void InitKfAcceleration(const float dt);

    bool JudgeSpinning();
    void AgainstSpinning();
    float last_pose_yaw = - CV_PI / 2, last_t;
    float middle_t;
    bool spin_flag = false;
    int spin_cnt = 0;

    RMTools::DisPlayWaveCLASS waveClass, poseAngle;

    // 自瞄装甲板Kalman
    EigenKalmanFilter RMKF = EigenKalmanFilter(9, 3);
    // flag为false则初始化Kalman，否则更新Kalman
    bool RMKF_flag = false;
    // 状态转移方程中的dt
    float delta_t = 0.016;
    float predict_dt = 0.05; // 预测时拉长步长节省计算，sentry:0.03

public:
    void EnergyPredictor(uint8_t mode, vector<Point2f> &target_pts, Point2f &center, const Vector3f &gimbal_ypd, float v_, float t_stamp);
    void EnergyRefresh();
    vector<Point2f> predict_pts;

private:
    /** energy machine common function **/
    void JudgeFanRotation();
    float CalOmegaNStep(int step, float &total_theta);
    bool EnergyStateSwitch();

    float IdealOmega(float &t_);
    float IdealRad(float t1, float t2);

    float total_theta = 0;
    float current_theta = 0;
    float current_omega = 0;
    vector<float> angle, omega;

    Point2f last_point;
    float dt_ = 0.012;
    float iterate_pitch ;

    int differ_step = 4;
    bool peak_flag = false;

    int energy_rotation_direction = 1;//风车旋转方向 1:顺时针 -1：逆时针
    u_int8_t ctrl_mode = INIT;

    RMTools::DisPlayWaveCLASS omegaWave;

    int clockwise_cnt = 0;

    /**---- energy machine kalman filter ----**/
    void FilterOmega(const float &dt);
    void FilterRad(const float &latency);
    void initFanRotateKalman();
    void initFanRadKalman();
    EigenKalmanFilter omega_kf = EigenKalmanFilter(3, 2, 1);
    EigenKalmanFilter rad_kf = EigenKalmanFilter(3,1);
    vector<float> filter_omega;
    bool omega_kf_flag = false;

    /**---- estimate big fan rotation speed parameters ----**/
    bool FindWavePeak();
    float max_omega = 0, min_omega = 5;
    void estimateParam(vector<float> &omega_, vector<float> &t_);
    int fit_cnt = 0;
    ceres::Problem problem;
    double a_ = 0.9125; // 0.78 ~ 1.045       middle value: 0.9125
    double w_ = 1.942;  // 1.884 ~ 2.0      middle: 1.942
    double phi_ = CV_PI/2; //参数初值
    int st_ = 15;
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
    float predict_rad = 0;

};

//#endif //MASTER_PREDICTOR_H
