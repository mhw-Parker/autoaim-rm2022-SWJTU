/**
 * directed by tyy on 2022-3-11
 * it's a kalman filter
 * */
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;

class Kalman{
public:
    Kalman();
    ~Kalman() = default;


    void Init(MatrixXd A_in,
              MatrixXd Q_in,
              MatrixXd R_in);
    /**
     * @brief 利用存储的最优估计做预测
     * @param l 预测步数
     * */
    void Predict(int l);
    /**
     * @brief 观测更新方程
     * @param z_k k时刻输入的观测值
     * */
    void Update(VectorXd z_k);

    /**
     * @brief 变更目标重置 P_k 与 K_
     * */
    void reset();

    VectorXd x_k; //k时刻的后验估计

private:
    MatrixXd A_;  //状态转移矩阵
    MatrixXd P_;  //k时刻后验估计协方差
    MatrixXd Q_;  //过程激励噪声协方差
    MatrixXd R_;  //测量噪声协方差
    MatrixXd H_;  //状态变量到观测的转移矩阵
    MatrixXd K_;  //滤波增益系数，卡尔曼系数

};

