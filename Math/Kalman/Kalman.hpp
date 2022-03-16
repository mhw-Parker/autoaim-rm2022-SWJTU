/**
 * directed by tyy on 2022-3-11
 * it's a kalman filter
 * */
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

using namespace std;
using namespace cv;
using namespace Eigen;

class Kalman{
public:
    Kalman();
    ~Kalman() = default;

    /**
     * @brief 滤波初始化函数
     * @param z_dim 输入测量参数维度  e.g. x y z   ----- 3 dimension
     * @param x_dim 状态变量维度     e.g. x y z vx vy vz  ----- 6 dimension
     * @param step 预测步长 帧 or ms
     * */
    void Init(int z_dim, int x_dim, float step);
    /**
     * @brief 利用存储的最优估计做预测，迭代卡尔曼
     * @param l 预测步数
     * */
    void Predict(int l);
    /**
     * @brief 观测更新方程
     * @param z_k k时刻输入的观测值
     * */
    void Update(VectorXf z_k);
    /**
     * @brief 变更目标重置 P_k 与 K_
     * */
    void reset();

    VectorXf x_k; //k时刻的后验估计
    MatrixXf A_;  //状态转移矩阵
    MatrixXf P_;  //k时刻后验估计协方差
    MatrixXf Q_;  //过程激励噪声协方差
    MatrixXf R_;  //测量噪声协方差
    MatrixXf H_;  //状态变量到观测的转移矩阵
    MatrixXf K_;  //滤波增益系数，卡尔曼系数
    MatrixXf I;

private:

};

