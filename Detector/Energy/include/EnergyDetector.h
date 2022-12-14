#ifndef ENERGY_H
#define ENERGY_H

#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "utility.hpp"
#include <ceres/ceres.h>

#include "mydefine.h"
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
    void EnergyDetectTask(Mat &src);

    vector<Point2f> output_pts;
    vector<Point2f> pts;
    cv::Point2f target_point;//目标装甲板中心坐标
    cv::Point2f circle_center_point;//风车圆心坐标

	bool detect_flag = false;

    Mat outline;

private:
    std::vector<cv::Point2f> target_armor_centers;//get
    const float K = 8; //半倍数 11

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

    Point2f roi_sp; //roi的左上角起始点

    bool isValidArmorContour(const vector<cv::Point>& armor_contour) const;//装甲板矩形尺寸要求
    bool isValidCenterRContour(const vector<cv::Point>& center_R_contour);//风车中心选区尺寸要求
    bool isValidFlowStripFanContour(cv::Mat& src, const vector<cv::Point>& flow_strip_fan_contour) const;//流动条扇叶矩形尺寸要求

    static double pointDistance(const cv::Point& point_1, const cv::Point& point_2);//计算两点距离

    //void getPredictPoint(Mat src);
    void getPts(RotatedRect armor);

    std::vector<Blade> target_blades;//可能的目标装甲板
    std::vector<cv::RotatedRect> armors;//图像中所有可能装甲板
    std::vector<cv::RotatedRect> valid_fan_strip;//可能的流动扇叶
    std::vector<cv::Point2f> armor_centers;//用来做最小二乘拟合

    Mat roi;
    int misscount = 10;
/*** *** *** *** *** ***/

private:
    Blade target_blade;//目标扇叶
    cv::RotatedRect target_armor;//目标装甲板
    cv::RotatedRect last_target_armor;//上一次目标装甲板

    void updateLastValues();//更新上一次的各种值
    int64 frame_time, last_frame_time;
};

#endif //ENERGY_H

