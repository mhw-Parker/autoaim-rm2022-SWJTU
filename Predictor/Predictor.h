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
    void armorPredictor(Vector3f p_cam_xyz, const float deltaT);
    Point3f predict_xyz;
    Point2f predict_point = Point2f(0,0);

    float yaw, pitch;
private:
    vector<Point3_<float>> target_xyz;
};

//#endif //MASTER_PREDICTOR_H
