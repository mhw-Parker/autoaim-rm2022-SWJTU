#ifndef ENERGY_H
#define ENERGY_H

#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <utility.hpp>

#include "mydefine.h"
#include "log.h"

#include"struct_define.h"

using namespace std;
using namespace cv;

#define SHOOT_TIME 1
#define FPS 60
#define OMEGA 1.884
#define MAX_ANGLE 75
#ifdef DAHUA
#define IMGWIDTH 1024
#define IMGHEIGHT 820
#endif
#ifdef MIND
#define IMGWIDTH 1280
#define IMGHEIGHT 1024
#endif

typedef struct {
    int radius;
    float angle;
    double time_stamp;
} polarLocal;

typedef struct Blade_{
    int armor_index;
    int flow_strip_fan_index;
    Blade_(int i_, int j_)
    {
        flow_strip_fan_index = i_;
        armor_index = j_;
    }

    Blade_()
    {
        armor_index = 0;
        flow_strip_fan_index = 0;
    }
}Blade;

class EnergyDetector {
public:
    explicit EnergyDetector();//构造函数
    ~EnergyDetector();//析构函数
    void EnergyTask(const Mat &src, bool mode, const float deltaT);//接口
    Point getPredict();
    Point getOffset();
    vector<Point2f> pts;
    vector<Point2f> predict_pts;
    cv::Point2f target_point;//目标装甲板中心坐标
    cv::Point2f last_target_point;//上一次目标装甲板坐标
    cv::Point2f predict_point;//预测的击打点坐标
    cv::Point2f last_circle_center_point;//上一次风车圆心坐标
    cv::Point2f circle_center_point;//风车圆心坐标
    std::vector<cv::Point2f> target_armor_centers;//get R

	bool detect_flag = false;
    float cur_phi = 0;
    float cur_omega = 0;
    float predict_rad = 0;

private:
    const float R = 168;
    const float PI = 3.14;
    const float K = 9; //半径倍数

    polarLocal polar_t;
    bool show_armors; //是否显示所有装甲
    bool show_target_armor; //是否显示目标装甲
    bool show_strip_fan;//是否显示有流动条的扇叶
    bool show_center_R;//是否显示中心点R
    bool show_target_point;//是否显示目标点
    bool show_predict_point;//是否显示预测点
    bool BIG_MODE = true;//是否为大符模式
    bool inter_flag = false;//是否contour有交集
    bool start_flag = false;//是否开始预测

    WindmillParamFlow _flow;
    McuData mcu_data;

    void clearAll();//清空所有容器vector
    void initEnergy();//能量机关初始化
    void initEnergyPartParam();//能量机关参数初始化
    Mat preprocess(Mat& src);

    bool detectArmor(Mat &src);
    bool detectFlowStripFan(Mat &src);
    bool detectR(Mat &src, Mat &show);
    bool getTargetPoint(Mat &src);

    Point2f calR();
	bool detectCircleCenter(Mat &src);

    void findROI(Mat &src, Mat &dst);
    void roiPoint2src();
    Point2f roi_sp; //roi的左上角起始点

    bool isValidArmorContour(const vector<cv::Point>& armor_contour) const;//装甲板矩形尺寸要求
    bool isValidCenterRContour(const vector<cv::Point>& center_R_contour);//风车中心选区尺寸要求
    bool isValidFlowStripFanContour(cv::Mat& src, const vector<cv::Point>& flow_strip_fan_contour) const;//流动条扇叶矩形尺寸要求

    static double pointDistance(const cv::Point& point_1, const cv::Point& point_2);//计算两点距离
    static double magnitude(const cv::Point& p);
    polarLocal toPolar(Point cart, double time_stamp);
    Point2f toCartesian(polarLocal pol);
    Point rotate(cv::Point target_point) const;
    float calPreAngle(float start_time,float end_time);
    //void getPredictPoint(Mat src);
    void getPts(RotatedRect armor);


    //Point2f calPredict(float theta) const;
    Point2f calPredict(Point2f p, Point2f center, float theta) const;

    Mat roi;

#ifdef DAHUA
    float dpx = 1811.5;
	float dpy = 1682.2;
#endif
#ifdef MIND
    float dpx = 726.52;
    float dpy = 529.27;
#endif

/*** *** *** *** *** ***/

/*** new predict ***/
    float spd_int(float t);
    float spd_phi(float omega, int flag);
    vector<float> delta_theta;
    vector<float> angle;
    vector<float> omega;
    vector<float> av_omega;
    vector<float> x_list;
    vector<float> time_list;
    float filter_rad;
    float predict_arr[6];
    int predict_cnt = 0;
    int vec_length = 4;
    int flag = 0;
    int last_flag = 0;
    void getPredictPointSmall(const Mat& src);
    void getPredictPoint(const Mat& src,float deltaT);
    //void testPredict(const Mat& src, float deltaT); // 2021/12/11
    void getPredictRect(float theta);
    RMTools::DisPlayWaveCLASS waveClass;
/*** *** *** *** *** ***/

    std::vector<Blade> target_blades;//可能的目标装甲板
    std::vector<cv::RotatedRect> armors;//图像中所有可能装甲板
    std::vector<cv::RotatedRect> valid_fan_strip;//可能的流动扇叶
    std::vector<cv::RotatedRect> centerRs;//可能的中心
    std::vector<cv::Point2f> armor_centers;//用来做最小二乘拟合
    std::vector<RotatedRect> valid_armors;//合法的裝甲板
    RMTools::MedianFilter<double> median_filter;//处理中位数

    cv::Rect center_r_area;
    cv::RotatedRect centerR;//风车中心字母R
    cv::RotatedRect pre_centerR;//风车中心字母R

    Blade target_blade;//目标扇叶
    cv::RotatedRect target_armor;//目标装甲板
    cv::RotatedRect last_target_armor;//上一次目标装甲板

    int energy_rotation_direction;//风车旋转方向
    int clockwise_rotation_init_cnt;//装甲板顺时针旋转次数
    int anticlockwise_rotation_init_cnt;//装甲板逆时针旋转次数
    bool energy_rotation_init;//若仍在判断风车旋转方向，则为true
    void initRotation();//对能量机关旋转方向进行初始化
    void updateLastValues();//更新上一次的各种值

    //预测提前角
    float predict_rad_norm;//预测提前角的绝对值

    int misscount = 0;

    float target_polar_angle;//待击打装甲板的极坐标角度
    float last_target_polar_angle_judge_rotation;//上一帧待击打装甲板的极坐标角度（用于判断旋向）

    list<polarLocal> history_target_armor_polar;
    polarLocal predict_polar;
    polarLocal target_polar;
    //todo predict task

    int64 last_frame_time;
    int64 frame_time;


};

#endif //ENERGY_H

