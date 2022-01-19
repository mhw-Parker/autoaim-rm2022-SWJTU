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

using namespace std;
using namespace cv;
using namespace Eigen;

class Predictor{
public:
    Predictor();
    ~Predictor();

    void armorPredictor(Vector3f p_cam_xyz, const float deltaT);
    void test1Predict(Vector3f pyd, const float deltaT);
    void refresh();

    Vector3f predict_xyz;
    Vector3f predict_ypd;
    vector<Point2f>predict_point;

    float yaw, pitch;

private:
    vector<Point3_<float>> target_xyz;
    vector<Vector3f> abs_pyd;

};

//#endif //MASTER_PREDICTOR_H
