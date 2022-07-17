//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

Predictor::Predictor() : waveClass(2000,300,1000),
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

void Predictor::EnergyRefresh(){
    angle.clear();
    omega.clear();
    filter_omega.clear();
    time_series.clear();
    t_list.clear();
    ctrl_mode = STANDBY;
    total_theta = 0;
    omega_kf_flag = false;
    clockwise_cnt = 0;
    fit_cnt = 0;    // curve fitting times
    initFanRadKalman();
    initFanRotateKalman();
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
    // z不置零，初始化为一个大概的距离
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
    // 解算YPD
    //云台参考为：左+ 右- 上+ 下-    解算参考为：左- 右+ 上+ 下-
    solveAngle.GetPoseV(target_pts,armor_type,gimbal_ypd);
//    /** test against spinning **/
//    AgainstSpinning();
//    /****/
    delta_ypd << -solveAngle.yaw, solveAngle.pitch, solveAngle.dist;
    target_ypd = gimbal_ypd + delta_ypd;
    // 通过目标xyz坐标计算yaw pitch distance
    target_xyz = GetGyroXYZ();
    // 识别到
    if (last_xyz != target_xyz) {
        float distance = RMTools::GetDistance(last_xyz, target_xyz);
        // 大范围更换目标认为是切换机器人
        if (distance > 1000) {
            // 深重置
            KalmanRefresh();
        }
        // 小范围更换目标认为是小陀螺场景
        ///TODO 同时考虑数字识别的结果更合理，距离参数待修改
        else if (distance > 250 && distance < 550) {
            // 进行浅重置，保留速度
            KalmanShallowRefresh();
        }
        // kalman预测要击打位置的xyz
        predict_xyz = KalmanPredict(average_v_bullet, latency);
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
        predict_xyz = KalmanPredict(average_v_bullet, latency);
        // 计算要击打位置的YPD
        predict_ypd = target_ypd + RMTools::GetDeltaYPD(predict_xyz, last_xyz);
        // 计算抬枪和子弹飞行时间
        predict_ypd[1] = solveAngle.iteratePitch(predict_xyz,average_v_bullet,fly_t);
        //预测时长为：响应时延+飞弹时延
        latency = react_t + fly_t;
    } else {
        target_xyz << 0, 0, 3000;
        // 自动射击命令
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
    back_ypd = offset + back_ypd;

    // 显示数据，会耗时17ms左右，一般关掉
//    if (showArmorBox) {
//        vector<float> data2;
//        for (int len = 0; len < target_xyz.size(); len++)
//            data2.push_back(target_xyz[len]);
//        for (int len = 0; len < RMKF.state_post_.rows(); len++)
//            data2.push_back(RMKF.state_post_[len]);
//        for (int i = 0; i < 3; i++)
//            data2.push_back(predict_ypd[i]);
//        vector<string> str2 = {"m_x","m_y","m_z",
//                               "kf_x","kf_y","kf_z",
//                               "kf_vx","kf_vy","kf_vz",
//                               "kf_ax","kf_ay","kf_az",
//                               "pre_yaw","pre_pitch","pre_dist"};
//        RMTools::showData(data2, str2, "data window");
//        vector<string> str1 = {"re-yaw:","pre-yaw","re-pitch:","pre-pit",
//                               "v bullet","Average v","latency"};
//        vector<float> data1 = {gimbal_ypd[0],predict_ypd[0] + offset[0],
//                               gimbal_ypd[1],predict_ypd[1] + offset[1],
//                               v_,average_v_bullet,latency};
//        RMTools::showData(data1,str1,"abs degree");
//    }
    //waveClass.displayWave(gimbal_ypd[1], predict_ypd[1] + offset[1], "y");
}

/**
 * @brief 获得陀螺仪坐标系下的 x y z
 * @param target_ypd 目标的 yaw pitch dist
 * */
Vector3f Predictor::GetGyroXYZ() {
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
Vector3f Predictor::KalmanPredict(float v_, float t) {
    int step = t / predict_dt;
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
    pre_xyz = target_xyz + target_v_xyz*t + 0.5*target_a_xyz*t*t;
    return pre_xyz;
}

/**
 * @brief 用匀加速模型初始化RMKF
 * @param dt 两帧间隔
 */
void Predictor::InitKfAcceleration(const float dt) {
    // 转移矩阵
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
    // 测量值矩阵
    RMKF.measure_mat_.setIdentity();
    // 过程噪声协方差矩阵Q
    float temp[9] = {1, 1, 1, 1, 1, 1, 5, 5, 5};
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

/******************************************************************************************/

/**----------------------- Helios 2022赛季能量机关预测部分 ---------------------------------**/

/******************************************************************************************/
/**
 * @brief 大能量机关预测
 * @param mode 大幅或小幅模式
 * @param target_pts 目标装甲板四点
 * @param center 旋转中心
 * @param gimbal_ypd 云台陀螺仪角度
 * @param v_ 弹速
 * @param t_stamp 当次时间戳
 * */
void Predictor::EnergyPredictor(uint8_t mode, vector<Point2f> &target_pts, Point2f &center, const Vector3f &gimbal_ypd,
                                float v_, float t_stamp) {
    t_list.push_back(t_stamp); // 更新时间戳 单位：s
    bool check = RMTools::CheckBulletVelocity(carName, v_);
    // 如果弹速不等于上一次插入的值，说明接收到新弹速，应当插入数组取平均；数组满则覆盖头部
    if (check && v_vec[(v_vec_pointer + 3) % 4] != v_) {
        v_vec[v_vec_pointer++ % 4] = v_;
    }
    /// average bullet speed
    average_v_bullet = RMTools::average(v_vec, 4);
    //average_v_bullet = v_;
    Point2f target_point;
    if(target_pts.size() == 4)
        target_point = Point2f((target_pts[0].x+target_pts[2].x)/2, (target_pts[0].y+target_pts[2].y)/2);
    else
        return;
    if(last_point == target_point) return; // if the target didn't refresh
    last_point = target_point;
    /// calculate current fan angle rad
    Point2f p = target_point - center;  // vector from center to target point
    current_theta = atan2(p.y,p.x);
    angle.push_back(current_theta);     // a condition used to store angle
    /// estimate and predict
    if(angle.size() <= differ_step) {   // if differ_step = n, angle condition minimum size is n+1, so that we can get 4 time gap
        predict_pts = target_pts;
    }
    else {
        current_omega = CalOmegaNStep(differ_step, total_theta);
        time_series.push_back(t_list[t_list.size()-1-differ_step/2]);
        omega.push_back(current_omega);
        JudgeFanRotation();
        if(mode == BIG_ENERGY_STATE) {
            FilterOmega(dt_);
            if(EnergyStateSwitch()) {
                float wt = IdealOmega(time_series.back());
                float d_w = wt - filter_omega.back();
                if(fabs(d_w) > 0.5 && fit_cnt/30 < 3) {
                    fit_cnt++;
                    if(fit_cnt%30 == 0){
                        ctrl_mode = STANDBY; // if the difference value between ideal omega and filter omega is too big
                        st_ = omega.size();// use 200 points refit the ideal omega
                        predict_rad = filter_omega.back() * 0.5;
                    }
                }
                else {
                    predict_rad = energy_rotation_direction * IdealRad(t_list.back(), t_list.back() + 0.4);
                    omegaWave.displayWave(wt, energy_rotation_direction*current_omega, "omega");
                    //FilterRad(latency);
                }
            }
            else {
                predict_rad = (omega_kf_flag) ? filter_omega.back()*0.4 : 0;
            }
        }
        else
            predict_rad = energy_rotation_direction * 1.05 * latency;
    }
    predict_point = calPredict(target_point,center,predict_rad); // 逆时针为负的预测弧度，顺时针为正 的预测弧度
    getPredictRect(center, target_pts, predict_rad); // 获得预测矩形
    solveAngle.GetPoseV(predict_pts, ENERGY_ARMOR, gimbal_ypd); /// 测试弹道 predict_pts -> target_pts
    delta_ypd << -solveAngle.yaw, solveAngle.pitch, solveAngle.dist;
    predict_ypd = gimbal_ypd + delta_ypd;
    predict_xyz = solveAngle.world_xyz;
    //predict_xyz = GetGyroXYZ();
    iterate_pitch = solveAngle.iteratePitch(predict_xyz, average_v_bullet, fly_t);
    back_ypd = predict_ypd + energy_offset;
    latency = react_t + fly_t;
}

bool Predictor::EnergyStateSwitch() {
    switch(ctrl_mode){
        case STANDBY:
            if(!fit_cnt) // first time estimate
            {
                if(fabs(current_omega) > 2.08) {
                    peak_flag = true; // which means we get a wave peak
                    phi_ = CV_PI/4; // initial phi
                }
                if(time_series.back() - time_series.front() > 2 && peak_flag && omega.size() - st_ >= 300) {
                    //st_ = (st_ < 0) ? 0 : st_;
                    ctrl_mode = ESTIMATE;
                }
            }
            else
            {
                if(omega.size() - st_ >= 300)
                    ctrl_mode = ESTIMATE;
            }

            return false;
        case ESTIMATE:
            estimateParam(filter_omega,time_series);
            ctrl_mode = PREDICT;
            return true;
        case PREDICT:
            return true;
        default: return true;
    }
}
void Predictor::JudgeFanRotation() {
    clockwise_cnt = (current_omega > 0) ? clockwise_cnt+1 : clockwise_cnt-1;// clockwise counter
    energy_rotation_direction = (clockwise_cnt > 0) ? 1 : -1;// anticlockwise counter
}
/**
 * @brief 计算能量机关旋转角速度
 * @param step 逐差步长
 * @param total_theta 累积角度
 * */
float Predictor::CalOmegaNStep(int step, float &total_theta) {
    int step_ = step + 1;
    if(angle.size() < step_) {
        return 0;
    } else {
        float d_theta;
        float dt = t_list.back() - t_list[t_list.size()-step_];
        /** new logic in order to solve the changing fan problem **/
        float last_d_theta = angle.back() - angle[angle.size()-2]; // delta value between last time
        if(last_d_theta > 6.1) // angle pass through the zero reference
            last_d_theta -= CV_2PI;
        else if(last_d_theta < -6.1)
            last_d_theta += CV_2PI;
        int d_fan = RMTools::get4Left5int(last_d_theta / 1.2566); // 1.2566 = 2*pi/5  四舍五入
        if(abs(d_fan) >= 1) {
            //printf("delta fan : %d\tdelta the: %f\tangle_2: %f\tangle_1: %f\n",d_fan,last_d_theta,angle.back()*180/CV_PI,angle[angle.size()-2]*180/CV_PI);
            for(int i = 2;i <= step_;i++) {
                angle[angle.size()-i] += d_fan * 1.2566;
                if(angle[angle.size()-i] < CV_2PI) angle[angle.size()-i] += CV_2PI;
                if(angle[angle.size()-i] > CV_2PI) angle[angle.size()-i] += -CV_2PI;
            }
        }
        /**------------------------------------------------------**/
        d_theta = angle.back() - angle[angle.size()-step_];
        if(d_theta > 6) {
            d_theta -= CV_2PI;
        }
        if(d_theta < -6) {
            d_theta += CV_2PI;
        }
        //omegaWave.displayWave(d_theta,0,"d the");
        total_theta += d_theta;
        float tmp_omega = d_theta / dt;
        //printf("omega: %f\tdt: %f\n",tmp_omega,dt);
        if(fabs(tmp_omega)>2.1) { //如果观测到的omega太离谱
            if(omega_kf_flag) //该次omega用kalman插值
            {
                total_theta = omega_kf.state_post_[0];
                tmp_omega = omega_kf.state_post_[1] + omega_kf.state_post_[2] * (dt/step);
            }
            else //kalman未初始化则直接限幅
                tmp_omega = (tmp_omega > 0) ? 2.09 : -2.09;
        }
        //omegaWave.displayWave(total_theta,0,"current angle");
        //omegaWave.displayWave(d_theta,0,"d_theta");
        return tmp_omega;
    }
}
float Predictor::IdealOmega(float &t_) {
    return a_ * sin( w_ * t_ + phi_) + 2.09 - a_;
}
/**
 * @brief integral from t1 to t2
 * */
float Predictor::IdealRad(float t1, float t2) {
    return -a_/w_ * (cos(w_*t2+phi_) - cos(w_*t1+phi_)) + (2.09-a_)*(t2-t1);
}
/**
 * @brief 利用 kalman 平滑量测的角速度获得滤波后的角速度
 * */
void Predictor::FilterOmega(const float& dt) {
    omega_kf.trans_mat_ <<  1, dt,0.5*dt*dt,
                            0, 1, dt,
                            0, 0, 1;
    VectorXf measure_vec(2,1);
    measure_vec <<  total_theta,
                    current_omega;
    omega_kf.predict();
    omega_kf.correct(measure_vec);
    omega_kf_flag = true;
    filter_omega.push_back(energy_rotation_direction*omega_kf.state_post_[1]);
    if(showEnergy){
        vector<string> str = {"flat-dist","height","v-bullet","cal-pitch","send-pitch","latency"};
        vector<float> data = {sqrt(predict_xyz[0]*predict_xyz[0]+predict_xyz[2]*predict_xyz[2]),-predict_xyz[1],
                              average_v_bullet,iterate_pitch,predict_ypd[1],latency};
        RMTools::showData(data,str,"energy param");
        //omegaWave.displayWave(predict_rad,filter_omega.back(),"omega");
        //omegaWave.displayWave(total_theta,filter_omega.back(),"total_theta");
    }
}
void Predictor::FilterRad(const float& latency) {
    vector<float> cut_filter_omega(filter_omega.end()-6,filter_omega.end()); //取 av_omega 的后 6 个数
    vector<float> cut_time_series(time_series.end()-6,time_series.end());
    Eigen::MatrixXd rate = RMTools::LeastSquare(cut_time_series,cut_filter_omega,1); //一元函数最小二乘
    // Five consecutive same judge can change the current state.
    int cur_flag = rate(0,0) > 0 ? 1 : 0; //最小二乘判断角速度增减性
    change_cnt = (cur_flag != last_flag) ? 0 : (change_cnt+1);

    if (change_cnt == 4) {
        flag = cur_flag;
        change_cnt = 0;
    }
    last_flag = cur_flag;
    float cur_phi;
    if (filter_omega.back() > 2.09-2*a_ && filter_omega.back() < 2.09)
        cur_phi = spdPhi(filter_omega.back(), flag);
    else if(filter_omega.back() > 2.09)
        cur_phi = CV_PI / 2;
    else
        cur_phi = - CV_PI / 2;
    double t = cur_phi / w_;
    predict_rad = energy_rotation_direction * (spdInt(t + latency) - spdInt(t));
    VectorXf rad_vec(1,1);
    rad_vec << predict_rad;
    rad_kf.Update(rad_vec);
    predict_rad = rad_kf.state_post_[0];
}

/**
 * @brief 初始化角速度 omega 的 kalman 滤波器
 * */
void Predictor::initFanRotateKalman() {
    omega_kf.measure_mat_.setIdentity();
    omega_kf.process_noise_.setIdentity();
    // 测量噪声协方差矩阵R
    omega_kf.measure_noise_.setIdentity();
    omega_kf.measure_noise_ <<  20, 0,      // TODO improve rotate kalman noise parameters
                                0, 50;
    // 误差估计协方差矩阵P
    omega_kf.error_post_.setIdentity();
    omega_kf.state_post_ << current_theta,
                            current_omega,
                            0;
}
/**
 * @brief 初始化预测弧度 rad 的 kalman 滤波器  ---有无必要？
 * */
void Predictor::initFanRadKalman() {
    rad_kf.measure_mat_.setIdentity();
    rad_kf.process_noise_.setIdentity();
    // 测量噪声协方差矩阵R
    rad_kf.measure_noise_.setIdentity();
    rad_kf.measure_noise_ << 10;
    // 误差估计协方差矩阵P
    rad_kf.error_post_.setIdentity();
    rad_kf.state_post_ <<   1,
                            0,
                            0;
}
/**
 * @brief 利用 ceres-solver 对目标 sin 函数进行参数估计
 * @param omega_ 用于参数估计的 omega 数组
 * @param t_ 与 omega 相对应的时间戳
 * @param times 用于曲线拟合的数据点数量
 * */
void Predictor::estimateParam(vector<float> &omega_, vector<float> &t_) {
    for(int i = st_; i < omega_.size(); i++){
        ceres::CostFunction* cost_func =
                new ceres::AutoDiffCostFunction<SinResidual,1,1,1,1>(
                        new SinResidual(t_[i], omega_[i])); //确定拟合问题是横坐标问题，需要初始化第一个坐标为 0
        problem.AddResidualBlock(cost_func, NULL, &a_, &w_,&phi_ );
    }
    problem.SetParameterLowerBound(&a_,0,0.780);
    problem.SetParameterUpperBound(&a_,0,1.045);
    problem.SetParameterLowerBound(&w_,0,1.884);
    problem.SetParameterUpperBound(&w_,0,2.0);
    problem.SetParameterLowerBound(&phi_,0,-CV_PI);
    problem.SetParameterUpperBound(&phi_,0,CV_PI);

    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   a: " << a_ << " w: " << w_ << " phi: " << phi_ <<"\n";
    //cout << "拟合数据下标起点：" << st_ << " " << omega_[st_] << " 拟合数据点数 ： " << omega.size() - st_ << " 函数中值：" << (2.09-min_w)/2 << endl;

    float sim_omega;
    for(int i=st_; i < omega_.size(); i++){
        sim_omega = IdealOmega(t_[i]);
        omegaWave.displayWave(omega_[i],sim_omega,"curve fitting");
    }

    if(a_ < 0.780) a_ = 0.785;
    else if(a_ > 1.045 ) a_ = 1.045;
    if(w_ < 0) w_ = abs(w_);
    if(w_ < 1.884) w_ = 1.884;
    else if(w_ > 2) w_ = 2;
}
/**
 * @brief 通过预测的弧度增量值获得目标点的对应预测点
 * @param p 待预测点
 * @param center 圆心
 * @param theta 弧度增量值
 * @return Point2f 的预测点
 * */
Point2f Predictor::calPredict(Point2f &p, Point2f &center, float theta) const {
    Eigen::Matrix2f rotate_matrix;
    Eigen::Vector2f cur_vec, pre_vec;
    float c = cos(theta), s = sin(theta);
    rotate_matrix << c, -s,
            s,  c; //旋转矩阵
    cur_vec << (p.x-center.x), (p.y-center.y);
    pre_vec = rotate_matrix * cur_vec;
    return Point2f (center.x + pre_vec[0], center.y + pre_vec[1]);
}
/**
 * @brief 反解角速度
 * */
float Predictor::spdInt(float t) {
    return -(a_ * cos(w_ * t) / w_) - (a_ - 2.09) * t;
}
/**
 * @brief 获取当前相位
 * */
float Predictor::spdPhi(float omega, int flag) {
    float sin_phi = (omega - (2.09-a_))/a_;
    float  phi = asin(sin_phi);
    return flag ? phi : CV_PI - phi;
}
void Predictor::getPredictRect(Point2f &center, vector<Point2f> &pts, float theta) {
    for(int i = 0;i<4;i++)
        predict_pts[i] = calPredict(pts[i],center, theta);
}
