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

    void test0(Vector3f p_cam_xyz, const float deltaT);
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

    vector<Point3_<float>> target_xyz;
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

    MatrixXd A,Q,R,H;
};

//#endif //MASTER_PREDICTOR_H
