//
// Created by 17703 on 2022/3/17.
//

#include "RMKF.hpp"

EigenKalmanFilter::EigenKalmanFilter(int state_param, int measure_param, int control_param) {
    Init(state_param, measure_param, control_param);
}

void EigenKalmanFilter::Init(int state_param, int measure_param, int control_param) {
    state_pre_.setZero(state_param, 1);
    state_post_.setZero(state_param, 1);
    trans_mat_.setIdentity(state_param, state_param);

    process_noise_.setIdentity(state_param, state_param);
    measure_mat_.setZero(measure_param, state_param);
    measure_noise_.setIdentity(measure_param, measure_param);

    error_pre_.setZero(state_param, state_param);
    error_post_.setZero(state_param, state_param);
    gain_.setZero(state_param, measure_param);

    if (control_param > 0)
        control_mat_.setIdentity(state_param, control_param);

    temp1.setZero(state_param, state_param);
    temp2.setZero(measure_param, state_param);
    temp3.setZero(measure_param, measure_param);
    temp4.setZero(measure_param, 1);
}

MatrixXf EigenKalmanFilter::predict(const MatrixXf& control) {
    state_pre_ = trans_mat_ * state_post_;
    if (control.rows() && control.cols())
        state_pre_ += control_mat_ * control;
    temp1 = trans_mat_ * error_post_;
    error_pre_ = temp1 * trans_mat_.transpose() + process_noise_;
    return state_pre_;
}

MatrixXf EigenKalmanFilter::correct(const VectorXf& measurement) {
    temp2 = error_pre_ * measure_mat_.transpose();
    temp3 = measure_mat_ * temp2 + measure_noise_;
    gain_ = temp2 * temp3.inverse();
    temp4 = measurement - measure_mat_ * state_pre_;
    state_post_ = state_pre_ + gain_ * temp4;
    error_post_ = error_pre_ - gain_ * measure_mat_ * error_pre_;
    return state_post_;
}

void EigenKalmanFilter::Update(VectorXf& measurement){
    predict();
    correct(measurement);
}

