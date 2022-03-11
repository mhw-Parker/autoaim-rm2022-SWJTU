#include "Kalman.hpp"

Kalman::Kalman() {}

void Kalman::Init(MatrixXd A_in, MatrixXd Q_in, MatrixXd R_in) {
    A_ = A_in;
    Q_ = Q_in;
    R_ = R_in;
}

void Kalman::Update(VectorXd z_k) {
    //k时刻的先验状态估计
    VectorXd x_k_bar = A_ * x_k;
    //k时刻先验估计协方差
    MatrixXd P_bar = A_ * P_ * A_.transpose() + Q_;
    //更新卡尔曼系数
    K_ = P_bar * (P_bar + R_).inverse();
    //更新状态估计值
    x_k = x_k_bar + K_ * (z_k - x_k_bar);
    //更新估计协方差
    MatrixXd I = MatrixXd::Identity(x_k.size(),x_k.size());
    P_ = (I - K_) * P_bar;
}

void Kalman::Predict(int l) {
    VectorXd z_pre = x_k;
    VectorXd x_pre = x_k;
    MatrixXd K_pre = K_;
    MatrixXd P_pre = P_;
    for(;l>0;l--){
        VectorXd x_pre_bar = A_ * x_pre;
        MatrixXd P_pre_bar = A_ * P_pre * A_.transpose() + Q_;
        K_pre = P_pre_bar * (P_pre_bar + R_).inverse();
        x_pre = x_pre_bar + K_pre * (z_pre - x_pre_bar);
        MatrixXd I = MatrixXd::Identity(x_k.size(),x_k.size());
        P_pre = (I - K_pre) * P_pre_bar;
    }
}

void Kalman::reset() {}