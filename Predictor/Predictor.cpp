//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

Predictor::Predictor() : waveClass(90,300,500){
    for (int i = 0; i < 7; i++) {
        frame_list.push_back(i);
    }
}

Predictor::~Predictor() = default;


/**
 * @brief 装甲板预测
 * */
void Predictor::armorPredictor(const Vector3f& target_ypd, float v_) {
    // 通过目标xyz坐标计算yaw pitch distance
    target_xyz = getGyroXYZ(target_ypd);

    // kalman预测要击打位置的xyz
    predict_xyz = kalmanPredict(target_xyz, v_, pre_t);

    // 计算要转过的角度
    Vector3f delta_ypd = RMTools::GetDeltaYPD(predict_xyz,target_xyz);
    predict_ypd = target_ypd + delta_ypd;

    // 计算这一次的预测时间pre_t
    float fly_t = 0;
    predict_ypd[1] = solveAngle.CalPitch(predict_xyz,v_,fly_t);
    pre_t = 0.2 + fly_t; //预测时长组成为  响应时延+飞弹时延

    // 以下为debug显示数据
    vector<float> show_data;
    for(int len = 0;len<target_xyz.size();len++)
        show_data.push_back(target_xyz[len]);
    for(int len = 0;len<RMKF.state_post_.rows();len++)
        show_data.push_back(RMKF.state_post_[len]);
    for(int i=0;i<3;i++)
        show_data.push_back(predict_ypd[i]);
    string str[] = {"m_x","m_y","m_z",
                    "kf_x","kf_y","kf_z",
                    "kf_vx","kf_vy","kf_vz",
                    "kf_ax","kf_ay","kf_az",
                    "pre_yaw","pre_pitch","pre_dist"};
    RMTools::showData(show_data, str, "data window");
    //waveClass.displayWave(gimbal_ypd[0],target_ypd[0],"yaw&pitch");
}

/**
 * @brief 获得陀螺仪坐标系下的 x y z
 * @param target_ypd 目标的 yaw pitch dist
 * */
Vector3f Predictor::getGyroXYZ(Vector3f target_ypd) {
    pair<float, float> quadrant[4] = {{-1, 1},
                                      {-1, -1},
                                      {1, -1},
                                      {1, 1}};

    float yaw_ = RMTools::total2circle(target_ypd[0]);

    float tan_yaw = tan(yaw_ * degree2rad);
    float tan_pitch = tan(target_ypd[1] * degree2rad);
    float dist2 = target_ypd[2] * target_ypd[2];
    float z_ = sqrt( dist2 / (1 + tan_yaw * tan_yaw) / (1 + tan_pitch * tan_pitch) );
    float x_ = z_ * fabs(tan_yaw);
    float y_ = -tan_pitch * sqrt(x_ * x_ + z_ * z_);
    //算x,z符号
    int t = yaw_ / 90;
    x_ *= quadrant[t].first; z_ *= quadrant[t].second;
    return {x_,y_,z_};
}

/**
 * @brief kalman 迭代预测
 * @param target_xyz 目标的绝对坐标 xyz
 * @param v_ 裁判系统读取的弹速
 * @param t 预测时间
 * */
Vector3f Predictor::kalmanPredict(Vector3f target_xyz, float v_, float t) {
    int step = t / delta_t + 15;
    cout << "step: " << step << endl;
    if (RMKF_flag) {
        UpdateKF(target_xyz);
        target_v_xyz << RMKF.state_post_[3],
                        RMKF.state_post_[4],
                        RMKF.state_post_[5];
        target_a_xyz << RMKF.state_post_[6],
                        RMKF.state_post_[7],
                        RMKF.state_post_[8];
    } else {
        RMKF_flag = true;
        InitKfAcceleration(delta_t);
    }
    Vector3f pre_xyz;
    pre_xyz = PredictKF(RMKF, step);
    //pre_xyz = target_xyz + target_v_xyz*t + 0.5*target_a_xyz*t*t;
    return pre_xyz;
}

/**
 * @brief 用匀加速模型初始化RMKF
 * @param dt 两帧间隔
 */
void Predictor::InitKfAcceleration(const float dt) {
    // 转移矩阵
    InitTransMat(dt);
    // 测量值矩阵
    RMKF.measure_mat_.setIdentity();
    // 过程噪声协方差矩阵Q
    RMKF.process_noise_.setIdentity();
    // 测量噪声协方差矩阵R
    RMKF.measure_noise_.setIdentity();
    RMKF.measure_noise_ *= 5;
    // 误差估计协方差矩阵P
    RMKF.error_post_.setIdentity();
    // 后验估计
    RMKF.state_post_ << target_xyz[0],
                        target_xyz[1],
                        target_xyz[2],
                        0,
                        0,
                        0,
                        0,
                        0,
                        0;
}

void Predictor::InitTransMat(const float dt) {
    float t0 = 0.5f * dt * dt;
    RMKF.trans_mat_ <<  1, 0, 0, dt, 0, 0, t0, 0, 0,
                        0, 1, 0, 0, dt, 0, 0, t0, 0,
                        0, 0, 1, 0, 0, dt, 0, 0, t0,
                        0, 0, 0, 1, 0, 0, dt, 0, 0,
                        0, 0, 0, 0, 1, 0, 0, dt, 0,
                        0, 0, 0, 0, 0, 1, 0, 0, dt,
                        0, 0, 0, 0, 0, 0, 1, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 1, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 1;
}

/**
 * @brief RMKF更新，包括预测部分和更正部分
 */
void Predictor::UpdateKF(const Vector3f& z_k) {
    // 预测
    RMKF.predict();
    // 更正
    RMKF.correct(z_k);
}

/**
 * @brief 迭代更新卡尔曼滤波器一定次数达到预测
 * @param KF 当前卡尔曼滤波器
 * @param iterate_times 迭代次数
 * @return 预测目标xyz坐标
 */
Vector3f Predictor::PredictKF(EigenKalmanFilter KF, const int &iterate_times) {
    Vector3f temp_target_xyz;
    temp_target_xyz <<  KF.state_post_(0),
                        KF.state_post_(1),
                        KF.state_post_(2);

    for (int i = 0; i < iterate_times; ++i) {
        KF.predict();
        KF.correct(temp_target_xyz);
        temp_target_xyz <<  KF.state_post_(0),
                            KF.state_post_(1),
                            KF.state_post_(2);
    }
    return temp_target_xyz;
}

/**
 * @brief 变更目标时更新预测器
 * */
void Predictor::Refresh() {
    RMKF_flag = false;
    target_a_xyz << 0, 0, 0;
    target_v_xyz << 0, 0, 0;
    pre_t = 0.5;
}


