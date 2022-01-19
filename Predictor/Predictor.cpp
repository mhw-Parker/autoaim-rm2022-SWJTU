//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

void Predictor::armorPredictor(Vector3f p_cam_xyz, const float deltaT) {
    target_xyz.push_back(Point3_<float>(p_cam_xyz[0], p_cam_xyz[1], p_cam_xyz[2]));
    if(target_xyz.size()){
        Point3_<float> v_xyz = (target_xyz.back() - target_xyz[target_xyz.size()-4]) / (deltaT * 3);
        predict_xyz = target_xyz.back() + v_xyz * 200;

        cout << p_cam_xyz << " " << predict_xyz << endl;
    }

}
