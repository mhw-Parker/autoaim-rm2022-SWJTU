//
// Created by 17703 on 2022/3/17.
//

#ifndef KALMAN_RMKF_HPP
#define KALMAN_RMKF_HPP

#endif //KALMAN_RMKF_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

class EigenKalmanFilter{
public:
    EigenKalmanFilter(int state_param, int measure_param, int control_param = 0);
    void Init(int state_param, int measure_param, int control_param = 0);
    MatrixXf predict(const MatrixXf& control = MatrixXf());
    MatrixXf correct(const VectorXf& measurement);

    VectorXf state_pre_;        // x_k_bar  k时刻的先验估计
    VectorXf state_post_;       // x_k  k时刻的后验估计

    MatrixXf trans_mat_;        // A 状态转移矩阵
    MatrixXf control_mat_;      // Buk 控制矩阵
    MatrixXf measure_mat_;      // H 测量转移矩阵
    MatrixXf process_noise_;    // Q 过程激励噪声协方差矩阵
    MatrixXf measure_noise_;    // R 测量噪声协方差矩阵
    MatrixXf error_pre_;
    MatrixXf gain_;             // K 滤波增益系数，卡尔曼系数
    MatrixXf error_post_;

private:
    MatrixXf temp1;
    MatrixXf temp2;
    MatrixXf temp3;
    MatrixXf temp4;
    MatrixXf temp5;
};