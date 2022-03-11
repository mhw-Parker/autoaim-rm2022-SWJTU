//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

Predictor::Predictor() : waveClass(4,600,1000){
    for (int i = 0; i < 7; i++) {
        frame_list.push_back(i);
    }
}

Predictor::~Predictor() = default;

void Predictor::test0(Vector3f p_cam_xyz, const float deltaT) {
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
 * @brief nuc上测试效果还行的绝对角度预测
 * */
 void Predictor::armorPredictor(Vector3f ypd, Vector3f gimbal_ypd, const float deltaT) {
     abs_yaw.push_back(ypd(0,0));
     if(abs_yaw.size() > 8){
         vector<float> cut_yaw (abs_yaw.end() - 7, abs_yaw.end());
         k = RMTools::LeastSquare(frame_list, cut_yaw, 1); //目前用最小二乘法做一次函数拟合，二次有无必要？
         for(int i = 0; i < sizeof (rate)/sizeof (rate[i]) - 1; i++){
             rate[i] = rate[i+1];
             yaw_arr[i] = yaw_arr[i+1];
         }
         rate[3] = k(0,0);
         float avg_k =RMTools::average(rate,4); //移除速度数据突变的可能性
         yaw_arr[3] = abs_yaw.back() + avg_k * frame;
         if(abs_yaw.size() > 13){
             yaw = RMTools::average(yaw_arr,4); //预测后 60 帧 yaw 角度
         }else
             yaw = yaw_arr[3];
         waveClass.displayWave(yaw_arr[2]-gimbal_ypd[0], 0);
     }
 }
 /**
  * @brief 利用云台绝对 yaw 角反解目标相对云台坐标系的 x
  * @param target_ypd 电控的绝对 yaw pitch 角和距离 distance
  * @param yp_speed 电控6轴陀螺仪获得的
  * @param lastT 上一帧时间
  * */
void Predictor::testPredictLineSpeed(Vector3f target_ypd, Vector3f yp_speed, const float lastT) {
    time_list.push_back(lastT);
    float tan_yaw = tan(target_ypd[0] / 360 * 2 * CV_PI);
    float tan_pitch = tan(target_ypd[1] / 360 * 2 * CV_PI);
    float dist2 = pow(target_ypd[2],2); //
    float z_ = sqrt( dist2 / (1+pow(tan_yaw,2)) / (1+pow(tan_pitch,2)) );
    float x_ = sqrt(pow(z_,2) * pow(tan_yaw,2));
    target_x.push_back(x_); //由yaw pitch distance解出的
}

/**
 * @brief
 * */
void Predictor::kalmanPredict(Vector3f target_ypd, Vector3f gimbal_ypd) {
    pair<float, float> quadrant[4] = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
    float yaw_ = target_ypd[0] - (int)(target_ypd[0] / 360) * 360;
    float tan_yaw = tan(yaw_ / 360 * 2 * CV_PI);
    float tan_pitch = tan(target_ypd[1] / 360 * 2 * CV_PI);
    float dist2 = target_ypd[2] * target_ypd[2]; //
    z_ = sqrt( dist2 / (1 + tan_yaw * tan_yaw) / (1 + tan_pitch * tan_pitch) );
    x_ = z_ * fabs(tan_yaw);
    y_ = tan_pitch * sqrt(x_ * x_ + z_ * z_);
    //算x,z符号
    int t = yaw / 90;
    x_ *= quadrant[t].first;z_ *= quadrant[t].second;

    if(tan_yaw > 0)
    target_x.push_back(x_);
    target_z.push_back(z_);
    if(target_x.size()){

    }
}

/**
 * @brief 变更目标时更新预测器
 * */
void Predictor::Refresh() {
    target_xyz.clear();
    abs_pyd.clear();
    abs_yaw.clear();
}
