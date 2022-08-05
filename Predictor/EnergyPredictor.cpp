//
// Created by tyy on 22-7-20.
//
#include "Predictor.h"

/******************************************************************************************/
/**                                                                                      **/
/**----------------------- Helios 2022赛季能量机关预测部分 ---------------------------------**/
/**                                                                                      **/
/******************************************************************************************/

/**
 * @brief clear all container
 * */
void Predictor::EnergyRefresh(){
    /** clear vector container **/
    angle.clear();
    omega.clear();
    filter_omega.clear();
    time_series.clear();
    t_list.clear();
    total_theta = 0;
    /** init state **/
    ctrl_mode = INIT;
    omega_kf_flag = false;
    est_flag = false;
    clockwise_cnt = 0;
    fit_cnt = 0;    // curve fitting times

    initFanRadKalman();
    initFanRotateKalman();
}
/**
 * @brief 大能量机关预测算法顶层接口
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
    //average_v_bullet = 28;
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
        predict_rad = 0;
    }
    else {
        current_omega = CalOmegaNStep(differ_step, total_theta);
        time_series.push_back(t_list[t_list.size()-1-differ_step/2]);
        omega.push_back(current_omega);
        JudgeFanRotation();
        if(mode == BIG_ENERGY_STATE) {
            FilterOmega(dt_);
            //ctrl_mode = PREDICT; // TODO: turn it off
            if(EnergyStateSwitch()) {
                est_flag = true;
                float wt = IdealOmega(time_series.back());
                float d_w = wt - filter_omega.back();
                if(fabs(d_w) > 0.5 && fit_cnt/30 < 3) {
                    fit_cnt++;
                    if(fit_cnt%30 == 0){
                        ctrl_mode = STANDBY; // if the difference value between ideal omega and filter omega is too big
                        st_ = filter_omega.size() - 70;// use 200 points refit the ideal omega
                    }
                }
                else {
                    predict_rad = IdealRad(t_list.back(), t_list.back() + latency);
                    if(DEBUG)
                        imshow("omega", omegaWave.Oscilloscope(filter_omega.back(),energy_rotation_direction*current_omega));

                }
            }
            else {
                predict_rad = (omega_kf_flag) ? filter_omega.back()*latency : 0;
            }
        }
        else {
            predict_rad = 1.05 * latency;
            est_flag = true;
        }
    }
    predict_rad *= energy_rotation_direction;
    predict_point = calPredict(target_point,center,predict_rad); // 逆时针为负的预测弧度，顺时针为正 预测弧度
    getPredictRect(center, target_pts, predict_rad); // 获得预测矩形
    solveAngle.GetPoseV(target_pts, ENERGY_ARMOR, gimbal_ypd); /// 测试弹道 predict_pts -> target_pts
    delta_ypd << -solveAngle.yaw, solveAngle.pitch, solveAngle.dist;
    predict_ypd = gimbal_ypd + delta_ypd;
    predict_xyz = solveAngle.world_xyz;
    if(average_v_bullet > 22) {
        predict_xyz[1] += 130;
    }else {
        predict_xyz[1] += 280;
    }
    //cout << predict_xyz << endl << endl;
    predict_ypd[1] = solveAngle.iteratePitch(predict_xyz, average_v_bullet, fly_t);
    back_ypd = predict_ypd + energy_offset;
    latency = react_t + fly_t;
    if(DEBUG){
        vector<string> str = {"flat-dist","height","v-bullet","cal-pitch","gim_pitch","yaw","latency","rad"};
        vector<float> data = {sqrt(predict_xyz[0]*predict_xyz[0]+predict_xyz[2]*predict_xyz[2]),-predict_xyz[1],
                              average_v_bullet,predict_ypd[1],gimbal_ypd[1],predict_ypd[0],latency,predict_rad};
        RMTools::showData(data,str,"energy param");
        //omegaWave.displayWave(predict_rad,filter_omega.back(),"omega");
        //omegaWave.displayWave(total_theta,filter_omega.back(),"total_theta");
    }

}

bool Predictor::EnergyStateSwitch() {
    switch(ctrl_mode){
        case INIT:
            if(FindWavePeak()) {
                ctrl_mode = STANDBY;
                fit_cnt = 0;    // reset param
                change_cnt = 0;
                max_omega = 0;
                min_omega = 5;
            }
            return false;
        case STANDBY:
            if(time_series.back() - time_series[st_] > 1.5) {
                ctrl_mode = ESTIMATE;
            }
            return false;
        case ESTIMATE:
            estimateParam(filter_omega,time_series);
            ctrl_mode = PREDICT;
            return true;
        case PREDICT:
            return true;
        default:
            return true;
    }
}
/**
 * @brief find a peak or valley to init phi in order to make estimate much more precise
 * */
bool Predictor::FindWavePeak() {
    if(filter_omega.size() > 15) {
        vector<float> cut_filter_omega(filter_omega.end()-6,filter_omega.end()); // 取 kalman滤波角速度 的后 6 个数
        vector<float> cut_time_series(time_series.end()-6,time_series.end());
        Eigen::MatrixXd rate = RMTools::LeastSquare(cut_time_series,cut_filter_omega,1); // 一元函数最小二乘
        fit_cnt = (rate(0,0) > 0) ? fit_cnt + 1 : fit_cnt - 1;
        if(fit_cnt > 5) {   // rise find peak
            if(fabs(filter_omega.back()) > max_omega) {
                change_cnt = 0;
                max_omega = filter_omega.back(); // reset max omega
                phi_ = CV_PI / 2 - w_ * time_series.back();
                while (phi_ < -CV_PI || phi_ > CV_PI) {
                    phi_ += 2*CV_PI;
                }
            } else {
                change_cnt++;
            }
        } else if(fit_cnt < -5) {
            if(fabs(filter_omega.back()) < min_omega) {
                change_cnt = 0;
                min_omega = filter_omega.back(); // reset max omega
                phi_ = -CV_PI / 2 - w_ * time_series.back();
                while (phi_ < -CV_PI || phi_ > CV_PI) {
                    phi_ += 2*CV_PI;
                }
            } else {
                change_cnt++;
            }
        } else {
            return false;
        }
        if(change_cnt > 5) {
            if(fit_cnt >= 0) {
                cout << "--- get peak omega : " << max_omega << "   init phi :" << phi_ << endl;
            } else {
                cout << "--- get valley omega : " << min_omega << "   init phi :" << phi_ << endl;
            }
            return true;
        } else {
            return false;
        }
    }
    else
        return false;
}


/**
 * @brief detect rotation
 * */
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
        int d_fan = RMTools::round2int(last_d_theta / 1.2566); // 1.2566 = 2*pi/5  四舍五入
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
        total_theta += d_theta;
        float tmp_omega = d_theta / dt;
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
/**
 * @brief get rotate omega from estimate sin function parameters
 * */
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
 * @brief use kalman filter to smoothen omega
 * */
void Predictor::FilterOmega(const float &dt) {
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

}
/**
 * @brief get rad by determining phi with inverse solution
 * @details abandoned
 * */
void Predictor::FilterRad(const float &latency) {
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
    omega_kf.measure_noise_ <<  50, 0,      // TODO improve rotate kalman noise parameters
            0, 100;
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
    for(int i = st_; i < omega_.size(); i=i+2){
        ceres::CostFunction* cost_func =
                new ceres::AutoDiffCostFunction<SinResidual,1,1,1,1>(
                        new SinResidual(t_[i], omega_[i]));
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
    //std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   a: " << a_ << " w: " << w_ << " phi: " << phi_ <<"\n";
    //cout << "拟合数据下标起点：" << st_ << " " << omega_[st_] << " 拟合数据点数 ： " << omega.size() - st_ << " 函数中值：" << (2.09-min_w)/2 << endl;

    float sim_omega;
    string win_name = "fit curve No. " + to_string(fit_cnt);
    for(int i=st_; i < omega_.size(); i++){
        sim_omega = IdealOmega(t_[i]);
        //omegaWave.displayWave(omega_[i],sim_omega,win_name);
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
    for(int i = 0;i<4;i++) {
        predict_pts[i] = calPredict(pts[i],center, theta);
        //cout << predict_pts[i] << " ";
    }
}