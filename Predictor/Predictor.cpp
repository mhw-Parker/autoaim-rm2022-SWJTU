//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

<<<<<<< HEAD
Predictor::Predictor() : waveClass(1000,600,1000){
=======
Predictor::Predictor() : waveClass(30,300,500){
>>>>>>> c2c726514c54921773c1399a06c837a862c6155b
    for (int i = 0; i < 7; i++) {
        frame_list.push_back(i);
    }
}

Predictor::~Predictor() = default;


/**
 * @brief 装甲板预测
 * */
<<<<<<< HEAD
 void Predictor::armorPredictor(Vector3f target_ypd, Vector3f gimbal_ypd, int direct) {
    target_xyz = getGyroXYZ(target_ypd,direct);
=======
void Predictor::armorPredictor(Vector3f target_ypd, const float v_) {
    target_xyz = getGyroXYZ(target_ypd);
>>>>>>> c2c726514c54921773c1399a06c837a862c6155b
    vector<float> show_data;
    for(int len = 0;len<target_xyz.size();len++)
        show_data.push_back(target_xyz[len]);

<<<<<<< HEAD
    float delta_yaw = kalmanPredict(target_xyz,direct);
=======
    float delta_yaw = kalmanPredict(target_xyz,target_ypd[2], v_);
>>>>>>> c2c726514c54921773c1399a06c837a862c6155b
    predict_ypd = {target_ypd[0] + delta_yaw, target_ypd[1], target_ypd[2]};

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
<<<<<<< HEAD
 }

/**
 * @brief
 * */
float Predictor::kalmanPredict(Vector3f target_xyz, int direct) {

=======
    waveClass.displayWave(target_ypd[0],predict_ypd[0]);
}

/**
 * @brief kalman
 * */
float Predictor::kalmanPredict(Vector3f target_xyz, float dist, float v_) {
    int step = dist / 1000 / v_ / delta_t + 1;
    cout << step << endl;
>>>>>>> c2c726514c54921773c1399a06c837a862c6155b
    if(RMKF_flag){
        UpdateKF(target_xyz);
        target_v_xyz << RMKF.state_post_[3],
                        RMKF.state_post_[4],
                        RMKF.state_post_[5];
        target_a_xyz << RMKF.state_post_[6],
                        RMKF.state_post_[7],
                        RMKF.state_post_[8];


    }else{
        RMKF_flag = true;
<<<<<<< HEAD
        InitKfAcceleration(0.025);
    }
    predict_xyz = PredictKF(RMKF, 30);
    return RMTools::GetDeltaTheta(target_xyz,predict_xyz,direct);
=======
        InitKfAcceleration(delta_t);
    }
    predict_xyz = PredictKF(RMKF, 10);
    return RMTools::GetDeltaTheta(target_xyz,predict_xyz);
>>>>>>> c2c726514c54921773c1399a06c837a862c6155b
}
/**
 * @brief 获得陀螺仪坐标系下的 x y z
 * @param direct 方向，极坐标正向取向右
 * @param target_ypd 目标的 yaw pitch dist
 * */
<<<<<<< HEAD
Vector3f Predictor::getGyroXYZ(Vector3f target_ypd, int direct) {
    pair<float, float> quadrant[4] = {{direct, direct},
                                      {-direct, direct},
                                      {-direct, -direct},
                                      {direct, -direct}};
=======
Vector3f Predictor::getGyroXYZ(Vector3f target_ypd) {
    pair<float, float> quadrant[4] = {{-1, 1},
                                      {-1, -1},
                                      {1, -1},
                                      {1, 1}};
>>>>>>> c2c726514c54921773c1399a06c837a862c6155b

    float yaw_ = RMTools::total2circle(target_ypd[0]);

    float tan_yaw = tan(yaw_ * degree2rad);
    float tan_pitch = tan(target_ypd[1] * degree2rad);
    float dist2 = target_ypd[2] * target_ypd[2]; //
    float z_ = sqrt( dist2 / (1 + tan_yaw * tan_yaw) / (1 + tan_pitch * tan_pitch) );
    float x_ = z_ * fabs(tan_yaw);
    float y_ = tan_pitch * sqrt(x_ * x_ + z_ * z_);
    //算x,z符号
    int t = yaw_ / 90;
    x_ *= quadrant[t].first; z_ *= quadrant[t].second;
    return {x_,y_,z_};
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
<<<<<<< HEAD
            target_xyz[1],
            target_xyz[2],
            0,
            0,
            0,
            0,
            0,
            0;
=======
                        target_xyz[1],
                        target_xyz[2],
                        0,
                        0,
                        0,
                        0,
                        0,
                        0;
>>>>>>> c2c726514c54921773c1399a06c837a862c6155b
}

/**
 * @brief RMKF更新，包括预测部分和更正部分
 */
void Predictor::UpdateKF(Vector3f z_k) {
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
<<<<<<< HEAD
    kf_flag = false;
=======
    RMKF_flag = false;
>>>>>>> c2c726514c54921773c1399a06c837a862c6155b
    abs_pyd.clear();
    abs_yaw.clear();
}


