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

using namespace std;
using namespace cv;
using namespace Eigen;

class Predictor{
public:
    Predictor();
    ~Predictor();

    void test1Predict(Vector3f pyd, const float deltaT);

    void armorPredictor(Vector3f ypd, Vector3f gimbal_ypd, const float deltaT);
    void testPredictLineSpeed(Vector3f ypd, Vector3f yp_speed, const float deltaT);
    void kalmanPredict(Vector3f target_ypd, Vector3f gimbal_ypd);

    void Refresh();

    Vector3f predict_xyz;
    Vector3f predict_ypd;
    vector<Point2f>predict_point;

    float yaw, pitch;

    float x_,y_,z_;

private:

    void showData(vector<float> data, string *str);
    /**
     * @brief 从陀螺仪 yaw pitch dist 解算目标点的世界坐标
     * @param target_ypd 目标的绝对 yaw pitch dist
     * @param target_xyz 目标的世界坐标系 x y z
     * */
    void calWorldPoint(Vector3f target_ypd, Vector3d &target_xyz);

    void backProject2D(Vector3f delta_ypd);

    vector<Vector3f> abs_pyd;

    vector<float> abs_yaw;
    vector<float> frame_list;
    vector<float> time_list;
    vector<float> predict_yaw;
    vector<float> target_x;
    vector<float> target_z;
    float frame = 70.0; //预测帧数，根据情况预测可能从帧改变为时间
    Eigen::MatrixXd k;

    double rate[4] = {0,0,0,0}, yaw_arr[4] = {0,0,0,0};

    RMTools::DisPlayWaveCLASS waveClass;

    Kalman kf;
    Vector3d z_k;
    bool kf_flag = false;

    MatrixXd A,Q,R,H;
};

//#endif //MASTER_PREDICTOR_H
