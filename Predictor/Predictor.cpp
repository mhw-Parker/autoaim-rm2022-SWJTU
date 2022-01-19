//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

Predictor::Predictor(){}

Predictor::~Predictor() = default;

void Predictor::armorPredictor(Vector3f p_cam_xyz, const float deltaT) {
    predict_point.clear();
    target_xyz.push_back(Point3_<float>(p_cam_xyz[0], p_cam_xyz[1], p_cam_xyz[2]));
    if(target_xyz.size()>4){
        Point3_<float> v_xyz = (target_xyz.back() - target_xyz[target_xyz.size()-4]) / (deltaT * 3);
        Point3f out_xyz = target_xyz.back() + v_xyz * 200;
        predict_xyz[0] = out_xyz.x;
        predict_xyz[1] = out_xyz.y;
        predict_xyz[2] = out_xyz.z;
        //cout << p_cam_xyz << " " << predict_xyz << endl;
    }
}

/**
 * @brief
 * @param pyd 绝对 yaw pitch dist
 * @param deltaT 3 帧平均时间
 * */
void Predictor::test1Predict(Vector3f ypd, const float deltaT) {
    abs_pyd.push_back(ypd);
    if(abs_pyd.size()>4){
        Vector3f v_ypd = (abs_pyd.back() - abs_pyd[abs_pyd.size()-4]) / (deltaT*3/1000);
        predict_ypd = abs_pyd.back() + v_ypd * 0.2;
        //predict_yp = tmp.col(1);
    }
}

/**
 * @brief 变更目标时更新预测器
 * */
void Predictor::refresh() {
    target_xyz.clear();
    abs_pyd.clear();
}
