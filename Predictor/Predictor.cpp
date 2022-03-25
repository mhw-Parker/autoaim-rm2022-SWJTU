//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

Predictor::Predictor() : waveClass(1000,600,1000){
    for (int i = 0; i < 7; i++) {
        frame_list.push_back(i);
    }
}

Predictor::~Predictor() = default;


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
 * @brief
 * */
void Predictor::kalmanPredict(Vector3f target_ypd, Vector3f gimbal_ypd, int direct) {
    pair<float, float> quadrant[4] = {{direct, direct},
                                      {-direct, direct},
                                      {-direct, -direct},
                                      {direct, -direct}};

    float yaw_ = RMTools::total2circle(target_ypd[0]);

    float tan_yaw = tan(yaw_ * degree2rad);
    float tan_pitch = tan(target_ypd[1] * degree2rad);
    float dist2 = target_ypd[2] * target_ypd[2]; //
    z_ = sqrt( dist2 / (1 + tan_yaw * tan_yaw) / (1 + tan_pitch * tan_pitch) );
    x_ = z_ * fabs(tan_yaw);
    y_ = tan_pitch * sqrt(x_ * x_ + z_ * z_);
    //算x,z符号
    int t = yaw_ / 90;
    x_ *= quadrant[t].first; z_ *= quadrant[t].second;
    target_xyz << x_, y_, z_;

    vector<float> show_data;
    for(int len = 0;len<target_xyz.size();len++)
        show_data.push_back(target_xyz[len]);

    if(RMKF_flag){
        UpdateKF(target_xyz);

        for(int len = 0;len<RMKF.state_post_.rows();len++)
            show_data.push_back(RMKF.state_post_[len]);
        //predict_xyz << kf.x_k[0] + frame*kf.x_k[3], kf.x_k[1]+frame*kf.x_k[4], kf.x_k[2] + frame*kf.x_k[5];

        if(target_x.size()>8)
            waveClass.displayWave(x_, predict_xyz[0]);

    }else{
        RMKF_flag = true;
        InitKfAcceleration(0.025);
    }
    predict_xyz = PredictKF(RMKF, 30);
    float target_theta = RMTools::XZ2RhoTheta({x_,z_}).y();
    float predict_theta = RMTools::XZ2RhoTheta({predict_xyz[0], predict_xyz[2]}).y();
    float delta_yaw = predict_theta - target_theta;
    if (delta_yaw > CV_PI)
        delta_yaw -= CV_PI;
    if (delta_yaw < -CV_PI)
        delta_yaw += CV_PI;
    predict_ypd = {target_ypd[0] + direct * delta_yaw / degree2rad, gimbal_ypd[1], gimbal_ypd[2]};

    for(int i=0;i<3;i++)
        show_data.push_back(predict_ypd[i]);
    string str[] = {"m_x","m_y","m_z",
                    "kf_x","kf_y","kf_z",
                    "kf_vx","kf_vy","kf_vz",
                    "kf_ax","kf_ay","kf_az",
                    "pre_yaw","pre_pitch","pre_dist"};
    RMTools::showData(show_data, str, "data window");
}

/**
 * @brief 用匀加速模型初始化RMKF
 * @param dt 两帧间隔
 */
void Predictor::InitKfAcceleration(const float dt) {
    // 1/2 * a * t^2
    float t0 = 0.5f * dt * dt;
    // 转移矩阵
    RMKF.trans_mat_ <<  1, 0, 0, dt, 0, 0, t0, 0, 0,
                        0, 1, 0, 0, dt, 0, 0, t0, 0,
                        0, 0, 1, 0, 0, dt, 0, 0, t0,
                        0, 0, 0, 1, 0, 0, dt, 0, 0,
                        0, 0, 0, 0, 1, 0, 0, dt, 0,
                        0, 0, 0, 0, 0, 1, 0, 0, dt,
                        0, 0, 0, 0, 0, 0, 1, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 1, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 1;
    // 测量值矩阵
    RMKF.measure_mat_.setIdentity();
    // 过程噪声协方差矩阵Q
    RMKF.process_noise_.setIdentity();
    // 测量噪声协方差矩阵R
    RMKF.measure_noise_.setIdentity();
    RMKF.measure_noise_ *= 25;
    // 误差估计协方差矩阵P
    RMKF.error_post_.setIdentity();
    // 后验估计
    RMKF.state_post_ << target_xyz[0],
            target_xyz[1],
            target_xyz[2],
            target_v_xyz[0],
            target_v_xyz[1],
            target_v_xyz[2],
            0,
            0,
            0;
}

/**
 * @brief RMKF更新，包括预测部分和更正部分
 */
void Predictor::UpdateKF(Vector3f z_k) {
    // 预测
    RMKF.predict();
    // 更正
    RMKF.correct(target_xyz);
}

/**
 * @brief 迭代更新卡尔曼滤波器一定次数达到预测
 * @param KF 当前卡尔曼滤波器
 * @param iterate_times 迭代次数
 * @return 预测目标xyz坐标
 */
Vector3f Predictor::PredictKF(EigenKalmanFilter KF, const int &iterate_times) {
    Vector3f temp_target_xyz;
    temp_target_xyz << KF.state_post_(0),
            KF.state_post_(1),
            KF.state_post_(2);

    for (int i = 0; i < iterate_times; ++i) {
        KF.predict();
        KF.correct(temp_target_xyz);
        temp_target_xyz << KF.state_post_(0),
                KF.state_post_(1),
                KF.state_post_(2);
    }
    return temp_target_xyz;
}

/**
 * @brief 变更目标时更新预测器
 * */
void Predictor::Refresh() {
    kf_flag = false;
    abs_pyd.clear();
    abs_yaw.clear();
}


