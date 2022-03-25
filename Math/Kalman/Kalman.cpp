#include "Kalman.hpp"

Kalman::Kalman() {}

void Kalman::Init(int z_dim, int x_dim, float step) {
    H_ = MatrixXf::Identity(z_dim,x_dim); //初始化为 z_dim * x_dim 的标准矩阵
    A_ = MatrixXf::Identity(x_dim,x_dim); //初始化为 x_dim * x_dim 的标准矩阵
    /// 速度模型
    if(x_dim / z_dim == 2){
        for(int i = 0; i < z_dim; i++){
            A_(i,z_dim + i) = step;
        }
    }/// 加速度模型
    else if(x_dim / z_dim == 3){
        for(int i = 0; i < x_dim - z_dim; i++){
            A_(i,z_dim + i) = step;
        }
        for(int i = 0; i < z_dim; i++)
            A_(i,2*z_dim + i) = 0.5 * step * step;
    }
    P_ = MatrixXf::Identity(x_dim, x_dim);
    I = MatrixXf::Identity(x_dim,x_dim);
    Q_.resize(x_dim,x_dim);
    R_.resize(z_dim,z_dim);
}

void Kalman::Update(VectorXf z_k) {
    //k时刻的先验状态估计
    VectorXf x_k_bar = A_ * x_k;
    //k时刻先验估计协方差
    MatrixXf P_bar = A_ * P_ * A_.transpose() + Q_;
    //更新卡尔曼系数
    MatrixXf P_H_trans = P_bar * H_.transpose();
    K_ = P_H_trans * (H_ * P_H_trans + R_).inverse();
    //更新状态估计值
    x_k = x_k_bar + K_ * (z_k - H_ * x_k_bar);
    //更新估计协方差
    P_ = (I - K_ * H_) * P_bar;

    //cout << "--- A Matrix ---" << endl << A_ << endl;
    //cout << "--- H Matrix ---" << endl << H_ << endl;
    //cout << "--- P Matrix ---" << endl << P_ << endl;
    //cout << "--- K Matrix ---" << endl << K_ << endl;
    //cout << "--- x estimate Vector ---" << endl << x_k << endl;
}

void Kalman::Predict(int l) {
    VectorXf z_pre;
    VectorXf x_pre = x_k;
    MatrixXf K_pre = K_;
    MatrixXf P_pre = P_;
    for(;l>0;l--){
        z_pre = H_ * x_pre; //将测量值作为第一次迭代的测量最优估计
        VectorXf x_pre_bar = A_ * x_pre;
        MatrixXf P_pre_bar = A_ * P_pre * A_.transpose() + Q_;
        K_pre = P_pre_bar * H_.transpose() * (H_ * P_pre_bar * H_.transpose() + R_).inverse();
        x_pre = x_pre_bar + K_pre * (z_pre - H_ * x_pre_bar);
        P_pre = (I - K_pre * H_) * P_pre_bar;
    }
    x_l_k = x_pre;
    //cout << " x_K : " << x_k.transpose() << endl;
    //cout << " x_l : " << x_l_k.transpose() << endl;
}

void Kalman::reset() {}