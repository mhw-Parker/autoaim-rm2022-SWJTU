#ifndef ENERGY_H
#define ENERGY_H

#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <utility.hpp>
#include <ceres/ceres.h>

#include "mydefine.h"
#include "log.h"
#include "RMKF.hpp"

#include"struct_define.h"


using namespace std;
using namespace cv;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


#ifdef DAHUA
#define IMGWIDTH 1024
#define IMGHEIGHT 820
#endif

#ifdef MIND
#define IMGWIDTH 1280
#define IMGHEIGHT 1024
#endif

#ifndef INIT
#define INIT 0
#endif

#ifndef STANDBY
#define STANDBY 1
#endif

#ifndef BEGIN
#define BEGIN 2
#endif

#ifndef ESTIMATE
#define ESTIMATE 3
#endif

#ifndef DEFAULT_MODE
#define DEFAULT_MODE 4
#endif

class EnergyDetector{
public:
    explicit EnergyDetector();//构造函数
    ~EnergyDetector();//析构函数
    void EnergyTask(const Mat &src, int8_t mode, const float dt, const float fly_t);//接口
    void EnergyDetectTask(const Mat &src);
    void Refresh();
    //void getPredictPoint();
    vector<Point2f> pts;
    vector<Point2f> predict_pts;
    cv::Point2f target_point;//目标装甲板中心坐标
    cv::Point2f predict_point;//预测的击打点坐标
    cv::Point2f circle_center_point;//风车圆心坐标
    std::vector<cv::Point2f> target_armor_centers;//get R

	bool detect_flag = false;
    float cur_phi = 0;
    float cur_omega = 0;
    float predict_rad = 0;
    Mat outline;

private:
    const float R = 168;
    const float K = 10; //半径倍数

    cv::Point2f last_circle_center_point;//上一次风车圆心坐标
    cv::Point2f last_target_point;//上一次目标装甲板坐标

    WindmillParamFlow _flow;

private:
    void clearAll();//清空所有容器vector
    void initEnergy();//能量机关初始化
    void initEnergyPartParam();//能量机关参数初始化
    Mat preprocess(Mat& src);

private:
    bool DetectAll(Mat &src);

    bool detectArmor(Mat &src);
    bool detectFlowStripFan(Mat &src);
    bool getTargetPoint(Mat &src);
    bool getCircleCenter(Mat &src);

    Point2f calR1P();
    Point2f calR3P();

    void findROI(Mat &src, Mat &dst);
    void roiPoint2src();
    Point2f roi_sp; //roi的左上角起始点

    bool isValidArmorContour(const vector<cv::Point>& armor_contour) const;//装甲板矩形尺寸要求
    bool isValidCenterRContour(const vector<cv::Point>& center_R_contour);//风车中心选区尺寸要求
    bool isValidFlowStripFanContour(cv::Mat& src, const vector<cv::Point>& flow_strip_fan_contour) const;//流动条扇叶矩形尺寸要求

    static double pointDistance(const cv::Point& point_1, const cv::Point& point_2);//计算两点距离

    //void getPredictPoint(Mat src);
    void getPts(RotatedRect armor);

    //Point2f calPredict(float theta) const;
    Point2f calPredict(Point2f p, Point2f center, float theta) const;

    std::vector<Blade> target_blades;//可能的目标装甲板
    std::vector<cv::RotatedRect> armors;//图像中所有可能装甲板
    std::vector<cv::RotatedRect> valid_fan_strip;//可能的流动扇叶
    std::vector<cv::Point2f> armor_centers;//用来做最小二乘拟合

    Mat roi;
    int misscount = 10;
/*** *** *** *** *** ***/
private:
/*** new predict ***/
    float spdInt(float t);
    float spdPhi(float omega, int flag);
    float startT = 0;
    vector<float> delta_theta;
    vector<float> angle;
    vector<float> omega;

    int predict_cnt = 0;
    int flag = 0;
    int last_flag = 0;
    void getPredictPointSmall(const float fly_t);
    void getPredictPoint();
    void getPredictRect(float theta, vector<Point2f> pts);
    RMTools::DisPlayWaveCLASS waveClass;

/*** *** *** *** *** ***/
private:
    void FilterOmega(const float dt);
    void initRotateKalman();
    void initRadKalman();
    void getPredictPointBig(const float fly_t);
    EigenKalmanFilter energy_kf = EigenKalmanFilter(3, 2, 1);
    EigenKalmanFilter rad_kf = EigenKalmanFilter(3,1);
    double freq;
    float current_theta = 0;
    float current_omega = 0;
    float total_theta = 0;
    vector<float> filter_omega; //kalman filter omega
    u_int8_t ctrl_mode = INIT;  //控制当前打幅模式状态
    int st = 0; //estimate init flag

private:
    bool judgeRotation(int8_t mode);
    void estimateParam(vector<float>omega_, vector<float>t_);
    float total_t = 0;
    vector<float> time_series; //记录每次的时间
    ceres::Problem problem;
    int counter = 0; //拟合数据计数器
    double a_ = 0.780, w_ = 1.9, phi_ = 0; //参数初值

    Blade target_blade;//目标扇叶
    cv::RotatedRect target_armor;//目标装甲板
    cv::RotatedRect last_target_armor;//上一次目标装甲板

    int energy_rotation_direction = 1;//风车旋转方向 1:顺时针 -1：逆时针
    int clockwise_rotation_init_cnt = 0;//装甲板顺时针旋转次数
    void updateLastValues();//更新上一次的各种值
    //预测提前角
    int64 last_frame_time;
    int64 frame_time;
private:
    float calOmega(vector<float> &omega_vec, vector<float> &time_vec, int step);
};

#endif //ENERGY_H

