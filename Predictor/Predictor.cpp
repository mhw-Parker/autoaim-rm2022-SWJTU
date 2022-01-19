//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

void Predictor::armorPredictor(Vector3f p_cam_xyz, const float deltaT) {
    predict_point.clear();
    target_xyz.push_back(Point3_<float>(p_cam_xyz[0], p_cam_xyz[1], p_cam_xyz[2]));
    if(target_xyz.size()){
        Point3_<float> v_xyz = (target_xyz.back() - target_xyz[target_xyz.size()-4]) / (deltaT * 3);
        Point3f out_xyz = target_xyz.back() + v_xyz * 200;
        predict_xyz[0] = out_xyz.x;
        predict_xyz[1] = out_xyz.y;
        predict_xyz[2] = out_xyz.z;
        //cout << p_cam_xyz << " " << predict_xyz << endl;
    }
}

void Predictor::test1Predict(Vector3f pyd, const float deltaT) {


}
