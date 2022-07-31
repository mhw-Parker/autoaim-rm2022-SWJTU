//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

Predictor::Predictor() : waveClass(30,300,1000),
                         omegaWave(3,600,1000),
                         poseAngle(CV_PI,600,1000) {
    predict_pts.assign(4,Point2f(0,0));
    // 初始化参数：初始弹速，响应时间
    InitParams();

    Refresh();
}

/**
 * 初始化参数
 */
void Predictor::InitParams() {
    string whose_params;
    switch (carName) {
        case HERO:
            whose_params = "Hero";
            break;
        case INFANTRY3:
            whose_params = "Infantry3";
            break;
        case INFANTRY4:
            whose_params = "Infantry4";
            break;
        case INFANTRY5:
            whose_params = "Infantry5";
            break;
        case INFANTRY_TRACK:
            break;
        case SENTRYTOP:
            whose_params = "SentryTop";
            break;
        case SENTRYDOWN:
            whose_params = "SentryDown";
            break;
        case UAV:
            break;
        case VIDEO:
        case IMAGE:
            whose_params = "Video";
            break;
        case NOTDEFINED:
            break;
    }
    FileStorage fs("../Predictor/Predictor.yaml", FileStorage::READ);
    // 找到车名对应的参数
    FileNode param_set = fs[whose_params];
    param_set["average_v_bullet"] >> average_v_bullet;
    param_set["react_t"] >> react_t;
    param_set["yaw_offset"] >> offset[0];
    param_set["pitch_offset"] >> offset[1];
    v_vec[0] = average_v_bullet;
    FileNode energy_params = fs["Energy"];
    energy_params["yaw_offset"] >> energy_offset[0];
    energy_params["pitch_offset"] >> energy_offset[1];
}

Predictor::~Predictor() = default;

/**
 * @brief 变更目标时更新预测器
 * */
void Predictor::Refresh() {
    /**--------- 公共享有部分清空 ---------**/
    TimeRefresh();
    /**--------- 装甲板预测部分清空 ---------**/
    KalmanRefresh();
    /**--------- 能量机关预测部分清空 ---------**/
    EnergyRefresh();
}

void Predictor::TimeRefresh() {
    total_t = 0;
    time_series.clear();
    latency = 0.5;
}

/**
 * @brief 深重置，Kalman滤波器整个重置，目标速度加速度置零
 */
inline void Predictor::KalmanRefresh() {
    RMKF_flag = false;
    target_a_xyz << 0, 0, 0;
    target_v_xyz << 0, 0, 0;
}

/**
 * @brief 浅重置，只重置目标位置，不重置速度加速度，用于小陀螺
 */
inline void Predictor::KalmanShallowRefresh() {
    for (int i = 0; i < 3; i++) {
        RMKF.state_pre_[i] = target_xyz[i];
        RMKF.state_post_[i] = target_xyz[i];
    }
}

void Predictor::UpdateTimeStamp(float &dt) {
    total_t += dt;  ///时序更新方式最好变成全局变量
    time_series.push_back(total_t);    //存放时间序列
}

/**
 * @brief 装甲板平动预测
 * @param target_pts 目标装甲板四点
 * @param armor_type 装甲板类型
 * @param gimbal_ypd 云台陀螺仪角度
 * @param v_ 弹速
 * @param dt 两次处理源图像时间间隔
 * @param lost_cnt
 * */
void Predictor::ArmorPredictor(vector<Point2f> &target_pts, const int& armor_type,
                               const Vector3f &gimbal_ypd, float v_, float dt,
                               int lost_cnt) {
    UpdateTimeStamp(dt);
    // 检查异常弹速数据
    bool check = RMTools::CheckBulletVelocity(carName, v_);
    // 如果弹速不等于上一次插入的值，说明接收到新弹速，应当插入数组取平均；数组满则覆盖头部
    if (check && v_vec[(v_vec_pointer + 3) % 4] != v_) {
        v_vec[v_vec_pointer++ % 4] = v_;
    }
    // 取弹速平均值
    average_v_bullet = RMTools::average(v_vec, 4);
    // 识别到
    if (!lost_cnt) {
        // 解算YPD
        //云台参考为：左+ 右- 上+ 下-    解算参考为：左- 右+ 上+ 下-
        solveAngle.GetPoseV(target_pts,armor_type,gimbal_ypd);
        /** test against spinning **/
        //AgainstSpinning();
        /****/
        delta_ypd << -solveAngle.yaw, solveAngle.pitch, solveAngle.dist;
        target_ypd = gimbal_ypd + delta_ypd;
        // 通过目标xyz坐标计算yaw pitch distance
        target_xyz = solveAngle.world_xyz;
        // 和上一个目标的距离
        float distance = RMTools::GetDistance(last_xyz, target_xyz);
        // 小范围更换目标认为是小陀螺场景
        ///TODO 同时考虑数字识别的结果更合理，距离参数待修改
        if (distance > 250 && distance < 550) {
            // 进行浅重置，保留速度
            KalmanShallowRefresh();
        }
        // kalman预测要击打位置的xyz
        predict_xyz = KalmanPredict(dt, latency);
        //
        predict_point = solveAngle.getBackProject2DPoint(predict_xyz);
        // 计算要转过的角度
        predict_ypd = target_ypd + RMTools::GetDeltaYPD(predict_xyz,target_xyz);
        // 计算抬枪和子弹飞行时间
        predict_ypd[1] = solveAngle.iteratePitch(predict_xyz, average_v_bullet, fly_t);
        //预测时长为：响应时延+飞弹时延
        latency = react_t + fly_t;
    }
    // 闪烁导致丢失目标时的处理策略为匀加速运动模型插值
    else if (lost_cnt <= max_lost) {
        // 预测目标当前位置
        target_xyz += target_v_xyz*dt + 0.5*target_a_xyz*dt*dt;
        // 预测目标要击打位置
        predict_xyz = KalmanPredict(dt, latency);
        // 计算要击打位置的YPD
        predict_ypd = target_ypd + RMTools::GetDeltaYPD(predict_xyz, last_xyz);
        // 计算抬枪和子弹飞行时间
        predict_ypd[1] = solveAngle.iteratePitch(predict_xyz,average_v_bullet,fly_t);
        //预测时长为：响应时延+飞弹时延
        latency = react_t + fly_t;
    }
    else {
        // 自动射击命令清零
        shootCmd = 0;
        KalmanRefresh();
    }
    // 哨兵射击命令
    if (carName == SENTRYTOP || carName == SENTRYDOWN) {
        shootCmd = CheckShoot(gimbal_ypd, armor_type);
    }
    // 更新last值
    last_xyz = target_xyz;
    // 发回电控值加偏置
    back_ypd = offset + predict_ypd;
    //back_ypd = offset + target_ypd;

    // 显示数据，一般关掉
    if (DEBUG) {
        vector<float> data2;
        for (int len = 0; len < target_xyz.size(); len++)
            data2.push_back(target_xyz[len]);
        for (int len = 0; len < RMKF.state_post_.rows(); len++)
            data2.push_back(RMKF.state_post_[len]);
        for (int i = 0; i < 3; i++)
            data2.push_back(predict_ypd[i]);
        vector<string> str2 = {"m_x","m_y","m_z",
                               "kf_x","kf_y","kf_z",
                               "kf_vx","kf_vy","kf_vz",
                               "kf_ax","kf_ay","kf_az",
                               "pre_yaw","pre_pitch","pre_dist"};
        RMTools::showData(data2, str2, "data window");
        vector<string> str1 = {"re-yaw:","pre-yaw","re-pitch:","pre-pit",
                               "v bullet","Average v","latency"};
        vector<float> data1 = {gimbal_ypd[0],predict_ypd[0] + offset[0],
                               gimbal_ypd[1],predict_ypd[1] + offset[1],
                               v_,average_v_bullet,latency};
        RMTools::showData(data1,str1,"abs degree");
    }
//    waveClass.displayWave(target_ypd[0], target_ypd[1], "target_yp");
}

/**
 * @brief 获得陀螺仪坐标系下的 x y z
 * @param target_ypd 目标的 yaw pitch dist
 * */
__attribute__((unused)) Vector3f Predictor::GetGyroXYZ() {
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
 * @param latency 预测时间
 * @param dt 两针时间
 * */
Vector3f Predictor::KalmanPredict(float dt, float latency) {
    InitKFATransMat(dt);
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
    //pre_xyz = PredictKF(RMKF, step);
    pre_xyz = target_xyz + target_v_xyz*latency + 0.5*target_a_xyz*latency*latency;
    return pre_xyz;
}

/**
 * @brief 用匀加速模型初始化RMKF
 * @param dt 两帧间隔
 */
void Predictor::InitKfAcceleration(const float dt) {
    // 转移矩阵
    InitKFATransMat(dt);
    // 测量值矩阵
    RMKF.measure_mat_.setIdentity();
    // 过程噪声协方差矩阵Q
    float temp[9] = {1, 1, 1, 20, 5, 20, 100, 25, 100};
    RMKF.process_noise_.setIdentity();
    for (int i = 0; i < 9; i++) {
        RMKF.process_noise_(i, i) *= temp[i];
    }
    // 测量噪声协方差矩阵R
    RMKF.measure_noise_.setIdentity();
    RMKF.measure_noise_ *= 1;
    // 误差估计协方差矩阵P
    RMKF.error_post_.setIdentity();
    for (int i = 3; i < 9; ++i) {
        RMKF.error_post_(i, i) *= 1000;
    }
    // 后验估计
    RMKF.state_post_ << target_xyz[0],target_xyz[1],target_xyz[2],
                        0,0,0,
                        0,0,0;
}
void Predictor::InitKFATransMat(const float dt) {
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
 * @param z_k 观测量 默认为三维坐标 x y z
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
    for (int i = 0; i < 6; ++i) {
        KF.trans_mat_(i, i + 3) = predict_dt;
    }
    for (int i = 0; i < 3; ++i) {
        KF.trans_mat_(i, i + 6) = 0.5f * predict_dt * predict_dt;
    }
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
 * 哨兵射击命令
 * @brief 直接用装甲板长度的1/1.4倍作为垂直于相机的长度分量
 */
uint8_t Predictor::CheckShoot(const Vector3f& gimbal_ypd, const int& armor_type) {
    float distance = predict_ypd[2];
    float armor_width = (armor_type == 1 ? 230.0f : 135.0f) * 4;
    float armor_height = 126.0f * 4;
    float yaw_error_threshold = atan(armor_width / 2 / distance) / degree2rad;
    float pitch_error_threshold = atan(armor_height / 2 / distance) / degree2rad;
    //printf("yaw_error_threshold: %.2f\npitch_error_threshold: %.2f\n", yaw_error_threshold, pitch_error_threshold);
    if (fabs(gimbal_ypd[0] - predict_ypd[0] - offset[0]) < yaw_error_threshold &&
        fabs(gimbal_ypd[1] - predict_ypd[1] - offset[1]) < pitch_error_threshold) {
        return 1;
    }
    return 0;
}

/**
 * @brief used to judge whether the enemy car is in spinning mode
 * */
bool Predictor::JudgeSpinning() {
    // delta_yaw_ pose between last stamp and now stamp
    float d_yaw = fabs(solveAngle.yaw_ - last_pose_yaw);
    //poseAngle.displayWave(d_yaw, solveAngle.yaw_, "pose-yaw_");
    last_pose_yaw = solveAngle.yaw_;
    // while changing target armor the delta pose angle will meet a peak
    if (d_yaw > CV_PI / 3) {
        cout << d_yaw << endl;
        if (!spin_flag) {
            last_t = time_series.back();
            spin_flag = true;
        } else {
            float spin_cycle_t = time_series.back() - last_t;
            cout << "a quarter of spin cycle time: " << spin_cycle_t << endl;
            if (spin_cycle_t < 1) {
                middle_t = (time_series.back() + last_t) / 2; // the probably time while the armor locates in the middle
                last_t = time_series.back();
                spin_cnt++; // spin mode counter
            } else {
                spin_flag = false;
                spin_cnt = 0;
            }
        }
    } else {
        // quit spin mode
        if(time_series.back() - last_t > 1) {
            spin_cnt = 0;
            spin_flag = false;
        }
    }
    if (spin_cnt > 1) { // detect more than twice
        return true;
    } else {
        return false;
    }
}
void Predictor::AgainstSpinning() {
    if(JudgeSpinning()) {
        cout << "spinning top mode" << endl;
    } else {
        return;
    }
}

