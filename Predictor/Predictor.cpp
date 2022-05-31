//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

Predictor::Predictor() : waveClass(400,300,1000),
                         omegaWave(3,600,1000)
{
    predict_pts.assign(4,Point2f(0,0));
    // TODO 通过各种优先模式设置初始弹速
    switch (carName) {
        case HERO:
            average_v_bullet = v_vec[0] = 14;
            break;
        case INFANTRY_MELEE0:
        case INFANTRY_MELEE1:
            average_v_bullet = v_vec[0] = 14;
            break;
        case INFANTRY_TRACK:
            break;
        case SENTRY:
            average_v_bullet = v_vec[0] = 28;
            react_t = 0.6;
            break;
        case UAV:
            break;
        case VIDEO:
            average_v_bullet = v_vec[0] = 14;
            break;
        case NOTDEFINED:
            break;
    }
}

Predictor::~Predictor() = default;

/**
 * @brief 变更目标时更新预测器
 * */
void Predictor::Refresh() {
    /**--------- 公共享有部分清空 ---------**/
    time_series.clear();
    total_t = 0;
    /**--------- 装甲板预测部分清空 ---------**/
    KalmanRefresh();
    latency = 0.5;
    /**--------- 能量机关预测部分清空 ---------**/
    angle.clear();
    omega.clear();
    filter_omega.clear();
    ctrl_mode = STANDBY;
    total_theta = 0;

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
 * */
void Predictor::ArmorPredictor(vector<Point2f> &target_pts, bool armor_type,
                               const Vector3f &gimbal_ypd, float v_, float dt) {
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
    delta_ypd << -solveAngle.yaw, solveAngle.pitch, solveAngle.dist;
    target_ypd = gimbal_ypd + delta_ypd;
    // 通过目标xyz坐标计算yaw pitch distance
    target_xyz = GetGyroXYZ(target_ypd);
    //
    if (target_xyz != last_xyz) {
        // 和上一次target的距离大于某个阈值，则认为更换目标
        if (RMTools::GetDistance(last_xyz, target_xyz) > 250) {
            KalmanShallowRefresh();
        }
        // kalman预测要击打位置的xyz
        predict_xyz = KalmanPredict(average_v_bullet, latency);
        predict_point = solveAngle.getBackProject2DPoint(predict_xyz);
        // 计算要转过的角度
        predict_ypd = target_ypd + RMTools::GetDeltaYPD(predict_xyz,target_xyz);
        // 计算抬枪
        predict_ypd[1] = solveAngle.iteratePitch(predict_xyz, average_v_bullet, fly_t);
        //预测时长为：响应时延+飞弹时延
        latency = react_t + fly_t;
    }
    // 闪烁导致丢失目标时的处理策略为匀加速运动模型插值
    else {
        // 预测目标当前位置
        target_xyz += target_v_xyz*dt + 0.5*target_a_xyz*dt*dt;
        // 预测目标要击打位置
        predict_xyz = KalmanPredict(average_v_bullet, latency);
        // 计算要击打位置的YPD
        predict_ypd = target_ypd + RMTools::GetDeltaYPD(predict_xyz, last_xyz);
        // 计算抬枪
        predict_ypd[1] = solveAngle.iteratePitch(predict_xyz,average_v_bullet,fly_t);
        //预测时长为：响应时延+飞弹时延
        latency = react_t + fly_t;
    }
    // 更新last值
    last_xyz = target_xyz;

    // 以下为debug显示数据
    if (showArmorBox) {
        vector<float> data2;
        for (int len = 0; len < target_xyz.size(); len++)
            data2.push_back(target_xyz[len]);
        for (int len = 0; len < RMKF.state_post_.rows(); len++)
            data2.push_back(RMKF.state_post_[len]);
        for (int i = 0; i < 3; i++)
            data2.push_back(predict_ypd[i]);
        string str2[] = {"m_x","m_y","m_z",
                        "kf_x","kf_y","kf_z",
                        "kf_vx","kf_vy","kf_vz",
                        "kf_ax","kf_ay","kf_az",
                        "pre_yaw","pre_pitch","pre_dist"};
        RMTools::showData(data2, str2, "data window");

        string str1[] = {"re-yaw:","re-pitch:","tar-yaw:",
                        "tar-pit:","pre-yaw","pre-pit",
                        "v bullet","Average v","latency"};
        vector<float> data1(8);
        Vector2f offset = RMTools::GetOffset(carName);
        data1 = {gimbal_ypd[0],gimbal_ypd[1],
                target_ypd[0],target_ypd[1],
                predict_ypd[0] + offset[0],predict_ypd[1] + offset[1],
                v_,average_v_bullet,latency};
        RMTools::showData(data1,str1,"abs degree");
    }
    waveClass.displayWave(target_xyz[0], predict_xyz[0], "x");
}

/**
 * @brief 获得陀螺仪坐标系下的 x y z
 * @param target_ypd 目标的 yaw pitch dist
 * */
Vector3f Predictor::GetGyroXYZ(Vector3f target_ypd) {
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
    RMKF.process_noise_.setIdentity();
    RMKF.process_noise_ *= 0.5;
    // 测量噪声协方差矩阵R
    RMKF.measure_noise_.setIdentity();
    RMKF.measure_noise_ *= 1;
    // 误差估计协方差矩阵P
    RMKF.error_post_.setIdentity();
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
        KF.trans_mat_(i, i + 6) = 0.5 * predict_dt * predict_dt;
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
 * @param dt 两次处理源图像时间间隔
 * */
void Predictor::EnergyPredictor(uint8_t mode, vector<Point2f> &target_pts, Point2f &center, const Vector3f &gimbal_ypd, float v_, float dt) {
    UpdateTimeStamp(dt);
    Point2f target_point;
    if(target_pts.size() == 4)
        target_point = Point2f((target_pts[0].x+target_pts[2].x)/2, (target_pts[0].y+target_pts[2].y)/2);
    else
        return;
    Point2f p = target_point - center; //圆心指向目标的向量
    current_theta = atan2(p.y,p.x);     //当前角度 弧度制
    angle.push_back(current_theta);    //存放对应的角度序列
    if(JudgeFanRotation()) {
        if(mode == BIG_ENERGY_STATE) {
            FilterOmega(dt);
            if(EnergyStateSwitch())
                FilterRad(latency);
            else
                predict_rad = 0;
        }
        else
            predict_rad = energy_rotation_direction * 1.4 * latency;
        predict_point = calPredict(target_point,center,predict_rad); //逆时针为负的预测弧度，顺时针为正 的预测弧度
        getPredictRect(center, target_pts, predict_rad); //获得预测矩形
    }
    else
        predict_pts = target_pts;
    solveAngle.GetPoseV(predict_pts, false, gimbal_ypd);
    delta_ypd << -solveAngle.yaw, solveAngle.pitch, solveAngle.dist;
    predict_ypd = gimbal_ypd + delta_ypd;
    predict_xyz = GetGyroXYZ(predict_ypd);
    predict_ypd[1] = solveAngle.CalPitch(predict_xyz,v_,fly_t);
    latency = 0.2 + fly_t;
}
bool Predictor::EnergyStateSwitch() {
    switch(ctrl_mode){
        case STANDBY:
            if(fabs(filter_omega.back()) > 2.05) {
                st = filter_omega.size() - 1;
                phi_ = CV_PI / 2;
                ctrl_mode = BEGIN;
            }
            return false;
        case BEGIN:
            if(time_series.back() - time_series[st] > 2)
                ctrl_mode = ESTIMATE;
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
bool Predictor::JudgeFanRotation() {
    if(angle.size() > 3 && angle.size() < 19){
        current_omega = CalOmega(3, total_theta);
        omega.push_back(current_omega);
        if(angle.size() == 18) {
            int clockwise_cnt = 0; //顺时针旋转计数器
            for(auto &i : omega)
                if(i>0) clockwise_cnt++;
            energy_rotation_direction = clockwise_cnt > omega.size()/2 ? 1 : -1;
            total_theta = current_theta; //将当前累积角度更新为当前角度
            initFanRotateKalman();
            initFanRadKalman();
            return true;
        }
        else return false;
    }
    else return true;
}
/**
 * @brief 计算能量机关旋转角速度
 * @param step 逐差步长
 * @param total_theta 累积角度
 * */
float Predictor::CalOmega(int step, float &total_theta) {
    int step_ = step + 1;
    float d_theta = angle.back() - angle[angle.size()-step_];
    float dt = time_series.back() - time_series[time_series.size()-step_];
    if(d_theta > 6)
        d_theta -= 2*CV_PI;
    if(d_theta < -6)
        d_theta += 2*CV_PI;
    total_theta += d_theta;
    return d_theta / dt;
}
/**
 * @brief 利用 kalman 平滑量测的角速度获得滤波后的角速度
 * */
void Predictor::FilterOmega(const float dt) {
    omega_kf.trans_mat_ <<  1, dt,0.5*dt*dt,
                            0, 1, dt,
                            0, 0, 1;
    current_omega = CalOmega(3, total_theta);
    VectorXf measure_vec(2,1);
    measure_vec <<  total_theta,
                    current_omega;

    omega_kf.predict();
    omega_kf.correct(measure_vec);
    filter_omega.push_back(energy_rotation_direction*omega_kf.state_post_[1]);
    omegaWave.displayWave(energy_rotation_direction*current_omega,filter_omega.back(),"omega");
}
void Predictor::FilterRad(const float latency) {
    vector<float> cut_filter_omega(filter_omega.end()-6,filter_omega.end()); //取 av_omega 的后 6 个数
    vector<float> cut_time_series(time_series.end()-6,time_series.end());
    Eigen::MatrixXd rate = RMTools::LeastSquare(cut_time_series,cut_filter_omega,1); //一元函数最小二乘
    // Five consecutive same judge can change the current state.
    int cur_flag = rate(0,0) > 0 ? 1 : 0; //最小二乘判断角速度增减性
    change_cnt = (cur_flag != last_flag) ? 0 : (change_cnt+1);

    if (change_cnt == 3) {
        flag = cur_flag;
        change_cnt = 0;
    }
    last_flag = cur_flag;
    float cur_phi;
    if (filter_omega.back() > 0.52 && filter_omega.back() < 2.09)
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
    omega_kf.measure_noise_ <<  10, 0,
                                0, 110;
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
    for(int i = st; i < omega_.size(); i++){
        ceres::CostFunction* cost_func =
                new ceres::AutoDiffCostFunction<SinResidual,1,1,1,1>(
                        new SinResidual(t_[i]-t_[st],omega_[i])); //确定拟合问题是横坐标问题，需要初始化第一个坐标为 0
        problem.AddResidualBlock(cost_func, NULL, &a_, &w_,&phi_ );
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   a: " << a_ << " w: " << w_ << " phi: " << phi_ <<"\n";
    //cout << "拟合数据下标起点：" << st << " " << omega_[st] << " 拟合数据点数 ： " << omega.size() - st << " 函数中值：" << (2.09-min_w)/2 << endl;

    float sim_omega;
    for(int i=st;i < omega_.size(); i++){
        sim_omega = a_ * sin(w_*(t_[i]-t_[st])+phi_) + 2.09-a_;
        omegaWave.displayWave(omega_[i],sim_omega,"curve fitting");
    }

    if(a_ < 0.780) a_ = 0.785;
    else if(a_ > 1.045 ) a_ = 1.045;
    if(w_ < 0) w_ = abs(w_);
    if(w_ < 1.884) w_ = 1.884;
    else if(w_ > 2) w_ = 2;
    //waitKey(0);
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
