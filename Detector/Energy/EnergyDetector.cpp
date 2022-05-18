/*********************************************************************************
  *Copyright(C),2019-2021,西南交通大学HELIOS战队All Rights Reserved
  *FileName:  EnergyDetector.cpp
  *Author:  黄冬婷 罗俊辉 田翊扬 徐润泽
  *Version: 1.4
  *Date:  2021.08.05
  *Description: 能量机关识别及预测
  *Function List:
     1.EnergyTask   能量机关任务执行接口
     2.getTargetPoint   获得目标点
     3.getPredictPoint    寻找攻击点
**********************************************************************************/

#include <fstream>
#include "EnergyDetector.h"

#if SAVE_LOG == 1
string now_time = getSysTime();
string energyData = string(OUTPUT_PATH + now_time + string("_energyData")).append(".txt") );
ofstream fout(energyData);
ofstream write_energy_data(energyData,ios::out);
#endif

using namespace std;

/**
 * @brief EnergyDetector::EnergyDetector
 * @param null
 * @return null
 * @remark Energy类构造函数，初始化有关参数
 */
EnergyDetector::EnergyDetector() : waveClass(3,600,1000){
    initEnergy();
    initEnergyPartParam();//对能量机关参数进行初始化
    freq = getTickFrequency();
}

/**
 * @brief EnergyDetector::EnergyDetectorDetector
 * @param null
 * @return null
 * @remark Energy类析构函数
 */
EnergyDetector::~EnergyDetector() = default;

/**
 * @brief EnergyDetector::initEnergy
 * @param null
 * @return null
 * @remark 初始化成员变量
 */
void EnergyDetector::initEnergy() {
    predict_rad = 0;//预测提前角
    predict_point = Point(0, 0);//预测打击点初始化
    pts.resize(4);
    predict_pts.resize(4);
    //startT = getTickCount();
}

/**
 * @brief EnergyDetector::initEnergyPartParam
 * @param null
 * @return null
 * @remark 初始化参数
 */
void EnergyDetector::initEnergyPartParam() {
    _flow.BLUE_GRAY_THRESH = 100;//敌方红色时的阈值
    _flow.RED_GRAY_THRESH = 180;//敌方蓝色时的阈值
    // area change to 1/3
///装甲板的相关筛选参数
    _flow.armor_contour_area_max = 2800;//1500
    _flow.armor_contour_area_min = 400;//400
    _flow.armor_contour_length_max = 110;//50
    _flow.armor_contour_length_min = 25;//25
    _flow.armor_contour_width_max = 80;//30
    _flow.armor_contour_width_min = 10;//15
    _flow.armor_contour_hw_ratio_max = 3;//3
    _flow.armor_contour_hw_ratio_min = 1;//1

///流动条所在扇叶的相关筛选参数
    _flow.flow_strip_fan_contour_area_max = 8800;
    _flow.flow_strip_fan_contour_area_min = 2000;
    _flow.flow_strip_fan_contour_length_max = 320;
    _flow.flow_strip_fan_contour_length_min = 90;
    _flow.flow_strip_fan_contour_width_max = 140;
    _flow.flow_strip_fan_contour_width_min = 45;
    _flow.flow_strip_fan_contour_hw_ratio_max = 2.8;
    _flow.flow_strip_fan_contour_hw_ratio_min = 1.2;
    _flow.flow_strip_fan_contour_area_ratio_max = 0.55;
    _flow.flow_strip_fan_contour_area_ratio_min = 0.20;

///流动条到装甲板距离参数
    _flow.Strip_Fan_Distance_max = 56;
    _flow.Strip_Fan_Distance_min = 28;

///流动条相关参数筛选
    _flow.flow_strip_contour_area_max = 1000;
    _flow.flow_strip_contour_area_min = 200;
    _flow.flow_strip_contour_length_max = 55;
    _flow.flow_strip_contour_length_min = 10;//32
    _flow.flow_strip_contour_width_max = 20;
    _flow.flow_strip_contour_width_min = 4;
    _flow.flow_strip_contour_hw_ratio_min = 3;
    _flow.flow_strip_contour_hw_ratio_max = 7;
    _flow.flow_strip_contour_area_ratio_min = 0.6;
    _flow.flow_strip_contour_intersection_area_min = 100;

    _flow.target_intersection_contour_area_min = 40/4;//重合面积
    _flow.twin_point_max = 20;

///中心R标筛选相关参数，中心亮灯面积约为装甲板面积的1/2
    _flow.Center_R_Control_area_max = 800;
    _flow.Center_R_Control_area_min = 150;
    _flow.Center_R_Control_length_max = 70;
    _flow.Center_R_Control_length_min = 6;
    _flow.Center_R_Control_width_max = 70;
    _flow.Center_R_Control_width_min = 6;
    _flow.Center_R_Control_radio_max = 1.2;
    _flow.Center_R_Control_radio_min = 1;
    _flow.Center_R_Control_area_radio_min = 0.6;
    _flow.Center_R_Control_area_intersection_area_min = 10;

///扇叶筛选相关参数
    _flow.flow_area_max = 2000;
    _flow.flow_area_min = 500;
    _flow.flow_length_max = 100;
    _flow.flow_length_min = 30;
    _flow.flow_width_max = 52;
    _flow.flow_width_min = 5;
    _flow.flow_aim_max = 3.5;
    _flow.flow_aim_min = 1.2;
    _flow.flow_area_ratio_min = 0.6;
}

/**
 * @brief EnergyDetector::clearAll
 * @param null
 * @return null
 * @remark 在每帧任务开始前清空容器
 */
void EnergyDetector::clearAll() {
    armors.clear();
    valid_fan_strip.clear();
    target_blades.clear();
    armor_centers.clear();
}

/**
 * @brief EnergyDetector::EnergyTask
 * @param src 摄像头读入的图片
 * @param mode 大小幅模式选择
 * @param dt 两帧时间差  单位：s
 * @param fly_t 子弹计算飞行时间  单位：s
 * @remark 能量机关任务执行接口
 */
void EnergyDetector::EnergyTask(const Mat &src, int8_t mode, const float dt, const float fly_t) {
    clearAll();
    Mat img = src.clone();
    Mat binary;

    double mission_start = getTickCount();
    //roi = preprocess(img); //预处理
    //cout << "--until preprocess : " << RMTools::CalWasteTime(st,getTickFrequency()) << endl;
    binary = preprocess(img);
    //roi = binary;
    //findROI(binary,roi); //设定roi
    //imshow("roi",roi);

    if (detectArmor(binary) && detectFlowStripFan(binary) && getTargetPoint(binary)) {
        getPts(target_armor); //获得的装甲板4点
        getCircleCenter(binary); //识别旋转圆心
        //cout << "--until detect : " << RMTools::CalWasteTime(mission_start,getTickFrequency()) << endl;
        FilterOmega(dt);
        //cout << "--until cal omega : " << RMTools::CalWasteTime(mission_start,getTickFrequency()) << endl;
        if(judgeRotation(mode)) {
            if (mode == SMALL_ENERGY_STATE)
                getPredictPointSmall(fly_t-0.15);
            else if (mode == BIG_ENERGY_STATE) {
                //getPredictPoint();
                getPredictPointBig(fly_t);
                //waveClass.displayWave(av_omega.back(), predict_rad,"energy rad");
            }
        }else {
            predict_pts = pts;
        }
        //waveClass.displayWave(omega.back(), av_omega.back());
        detect_flag = true;
        if(ctrl_mode != DEFAULT_MODE)
            detect_flag = false;
        misscount = 0;
    }else{
        misscount++;
        if(misscount>3){ //连续5帧丢目标
            misscount = 0;
            detect_flag = false;
        }
    }
    if(showEnergy){
        //imshow("binary", roi);
        //circle(outline, Point(IMGWIDTH/2, IMGHEIGHT/2), 2, Scalar(255, 255, 255), 3); //画出图像中心
        //imshow("outline", outline);
        //waitKey(1);
        waveClass.displayWave(energy_kf.state_post_[1], energy_rotation_direction, "theta");
    }
}
void EnergyDetector::EnergyDetectTask(const Mat &src) {
    clearAll();
    Mat img = src.clone();
    Mat binary;
    binary = preprocess(img);
    if(detectArmor(binary) && detectFlowStripFan(binary) && getTargetPoint(binary)){
        getPts(target_armor);
        getCircleCenter(binary);
        detect_flag = true;
    } else {
        misscount++;
        if(misscount>5){ //连续5帧丢目标
            misscount = 0;
            detect_flag = false;
        }
    }
}
/**
 * @brief 每次切换状态清空
 * */
void EnergyDetector::Refresh() {
    clearAll(); //清除容器
    //counter = 0;
    energy_rotation_direction = 1;
    ctrl_mode = INIT;
    //startT = getTickCount();

    angle.clear();       //clear angle vector
    omega.clear();
    filter_omega.clear();//clear filter_omega
    time_series.clear(); //clear time series
    delta_theta.clear();
}

/**
 * @brief 初始化判断大幅旋转旋方向，迭代计算大幅速度正弦变化函数参数 a w phi
 * @param mode 打幅状态
 * */
bool EnergyDetector::judgeRotation(int8_t mode) {
    switch (ctrl_mode) {
        case INIT:
            if(filter_omega.size() > 16){
                clockwise_rotation_init_cnt = 0;
                for(auto &i : filter_omega){
                    if(i>0) clockwise_rotation_init_cnt++;
                }
                energy_rotation_direction = clockwise_rotation_init_cnt>8 ? 1 : -1;
                clockwise_rotation_init_cnt = 0;
                //startT = getTickCount(); //重新初始化起始时间
                total_t = 0;
                if(mode == BIG_ENERGY_STATE)
                    ctrl_mode = STANDBY;
                else
                    ctrl_mode = DEFAULT_MODE;
            }
            return false;
        case STANDBY :
            if(fabs(filter_omega.back())>2.05){
                st = filter_omega.size() - 1;
                phi_ = CV_PI / 2;
                ctrl_mode = BEGIN;
            }
            return false;
        case BEGIN:
            if(time_series.back() - time_series[st] > 3)
                ctrl_mode = ESTIMATE;
            return false;
        case ESTIMATE:
            estimateParam(filter_omega,time_series);
            ctrl_mode = DEFAULT_MODE;
            return true;
        default: return true;
    }
}
/**
 * @brief 利用 ceres-solver 对目标 sin 函数进行参数估计
 * @param omega_ 用于参数估计的 omega 数组
 * @param t_ 与 omega 相对应的时间戳
 * @param times 用于曲线拟合的数据点数量
 * */
void EnergyDetector::estimateParam(vector<float> omega_, vector<float> t_) {
    for(int i = st; i < omega_.size(); i++){
        ceres::CostFunction* cost_func =
                new ceres::AutoDiffCostFunction<SinResidual,1,1,1,1>(
                        new SinResidual(t_[i]-t_[st],omega_[i])); //确定拟合问题是横坐标问题，需要初始化第一个坐标为 0
        problem.AddResidualBlock(cost_func, NULL, &a_, &w_,&phi_ );
#if SAVE_LOG == 1
        write_energy_data << t_[i]-t_[st] << " " << omega_[i] << " " << endl;
#endif
    }
    Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   a: " << a_ << " w: " << w_ << " phi: " << phi_ <<"\n";
    //cout << "拟合数据下标起点：" << st << " " << omega_[st] << " 拟合数据点数 ： " << omega.size() - st << " 函数中值：" << (2.09-min_w)/2 << endl;
#if SAVE_LOG == 1
    write_energy_data << "---Final   a: " << a_ << " w: " << w_ << " phi: " << phi_ << endl;
#endif
    /**  定参数  **/
//    a_ = 0.8655;
//    w_ = 1.9204;
    /**  定参数  **/

    float sim_omega;
    for(int i=st;i < omega_.size(); i++){
        sim_omega = a_ * sin(w_*(t_[i]-t_[st])+phi_) + 2.09-a_;
        waveClass.displayWave(omega_[i],sim_omega,"curve fitting");
    }

    if(a_ < 0.780) a_ = 0.785;
    else if(a_ > 1.045 ) a_ = 1.045;
    if(w_ < 0) w_ = abs(w_);
    if(w_ < 1.884) w_ = 1.884;
    else if(w_ > 2) w_ = 2;
    //waitKey(0);
}
/**
 * @brief EnergyDetector::preprocess
 * @param Mat& src
 * @return Mat& binary
 * @remark 图像预处理，完成二值化
 */
Mat EnergyDetector::preprocess(Mat &src) {
    Mat blue_binary, red_binary, binary;
    //cvtColor(src, dst, COLOR_BGR2GRAY);
    Mat single, blue_c, red_c;
    vector<Mat> channels;

    split(src, channels);

    if (blueTarget) {
        single = channels.at(0);
        threshold(single,binary,120,255,THRESH_BINARY);
    } else {
        single = channels.at(2);
        threshold(single,binary,50,255,THRESH_BINARY);
    }
//    blue_c = channels.at(0);
//    red_c = channels.at(2);
//    threshold(single, binary, 150, 255, THRESH_BINARY);
//    threshold(blue_c,blue_binary,150,255,THRESH_BINARY);
//    threshold(red_c,red_binary,50,255,THRESH_BINARY);
//
//    binary = blueTarget ? blue_binary - red_binary : red_binary - blue_binary; //滤掉白光
//
// //
    Mat element_dilate_1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(binary, binary, element_dilate_1);
    morphologyEx(binary, binary, MORPH_CLOSE, element_dilate_1);
    threshold(binary, binary, 0, 255, THRESH_BINARY);
//
    GaussianBlur(binary,binary,Size(3,3),0);
    if(showBinaryImg)
        imshow("binary",binary);
    return binary;
}

void EnergyDetector::findROI(Mat &src, Mat &dst) {
    if(detect_flag){
        if(circle_center_point.x < 300)
            roi_sp.x = 0;
        else if(circle_center_point.x > IMGWIDTH - 300)
            roi_sp.x = IMGWIDTH - 600;
        else
            roi_sp.x = circle_center_point.x - 300;

        if(circle_center_point.y < 300)
            roi_sp.y = 0;
        else if(circle_center_point.y > IMGHEIGHT - 300)
            roi_sp.y = IMGHEIGHT - 600;
        else
            roi_sp.y = circle_center_point.y - 300;


        dst = src(Rect(roi_sp.x,roi_sp.y,600,600));

    } else {
        dst = src;
        roi_sp = Point2f (0,0);
    }

}
void EnergyDetector::roiPoint2src() {
    target_point += roi_sp;
    circle_center_point += roi_sp;
    predict_point += roi_sp;
    for(int i = 0;i<=pts.size();i++){
        pts[i] += roi_sp;
        predict_pts[i] += roi_sp;
    }
}
bool EnergyDetector::DetectAll(Mat &src) {
    Mat dilate_mat = src.clone();
    vector<vector<Point> > all_contours;
    vector<vector<Point> > external_contours;
    findContours(dilate_mat,all_contours,RETR_LIST,CHAIN_APPROX_NONE);
    findContours(dilate_mat,external_contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);

    std::vector<vector<Point> > armor_contours;
    for (auto &i : external_contours)//去除外轮廓
    {
        auto external_size = i.size();
        for (auto &j : all_contours) {
            auto all_size = j.size();
            if (external_size == all_size) {
                swap(j, armor_contours[armor_contours.size() - 1]);
                armor_contours.pop_back();//清除掉流动条
                break;
            }
        }
    }
}
/**
 * @brief EnergyDetector::detectArmor
 * @param Mat& src
 * @return null
 * @remark 检测所有可能的装甲
 */
bool EnergyDetector::detectArmor(Mat &src) {
    //armor dilate
    Mat armor_dilate = src.clone();
#if DEBUG == 1
    imshow("armor_dilate", armor_dilate);
#endif
    //寻找所有装甲
    std::vector<vector<Point> > armor_contours;
    std::vector<vector<Point> > armor_contours_external;
    findContours(armor_dilate, armor_contours, RETR_LIST, CHAIN_APPROX_NONE);
    findContours(armor_dilate, armor_contours_external, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    //绘制轮廓
    outline = Mat(src.size(), CV_8UC3, Scalar(0, 0, 0));

    for (auto &i : armor_contours_external)//去除外轮廓
    {
        auto external_contour_size = i.size();
        for (auto &j : armor_contours) {
            auto all_size = j.size();
            if (external_contour_size == all_size) {
                swap(j, armor_contours[armor_contours.size() - 1]);
                armor_contours.pop_back();//清除掉流动条
                break;
            }
        }
    }
    for (auto &armor_contour : armor_contours) {
        if (!isValidArmorContour(armor_contour)) {
            continue;
        }
        RotatedRect flow_rect = cv::minAreaRect(armor_contour);
        Point2f flow_pts[4];
        flow_rect.points(flow_pts);
        if(showEnergy){
            for (int i = 0; i < 4; i++) {
                line(outline, flow_pts[i], flow_pts[(i + 1) % (4)],
                     Scalar(255, 255, 255), 2, LINE_8);
            }
        }
        armors.emplace_back(cv::minAreaRect(armor_contour));//回传所有装甲板到armors容器中
        armor_centers.emplace_back(cv::minAreaRect(armor_contour).center);//回传所有装甲板center到armor_center容器中
    }
    if (armors.empty()) {
        return false;
    }
    return true;

}

/**
 * @brief EnergyDetector::detectFlowStripFan
 * @param Mat& src
 * @return null
 * @remark 检测所有可能的流动条所在的装甲板
 */
bool EnergyDetector::detectFlowStripFan(Mat &src) {
    Mat flow_fan_dilate = src.clone();
    Mat canny;
    //Canny(flow_fan_dilate,canny,100,250);
    //imshow("canny",canny);
    //寻找所有流动条所在扇叶
    vector<vector<Point> > flow_strip_fan_contours;
    findContours(flow_fan_dilate, flow_strip_fan_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE); //只检测外围轮廓
    vector<cv::RotatedRect> candidate_flow_strip_fans;

    for (auto &flow_strip_fan_contour : flow_strip_fan_contours) {

        if (!isValidFlowStripFanContour(flow_fan_dilate, flow_strip_fan_contour)) {
            continue;
        }
        Mat c = src.clone();
        //cout << "flow Area : " << contourArea(flow_strip_fan_contour) << endl;

        RotatedRect flow_rect = cv::minAreaRect(flow_strip_fan_contour);
        //cout << "flow width : "<< flow_rect.size.width << "\tflow height : " << flow_rect.size.height << endl;

        Point2f flow_pts[4];
        flow_rect.points(flow_pts);
        //画出流动条
        if(showEnergy){
            for (int i = 0; i < 4; i++) {
                line(outline, flow_pts[i], flow_pts[(i + 1) % (4)],
                     Scalar(255, 255, 255), 2, LINE_8);
            }
        }

        valid_fan_strip.emplace_back(cv::minAreaRect(flow_strip_fan_contour));
    }

    if (valid_fan_strip.empty())
        return false;

    return true;

#if DEBUG_MSG == 1
    if (valid_fan_strip.empty()) {
        LOGM("flow strip fan false!\n");
    }
    if (!valid_fan_strip.empty()) {
        LOGM("flow strip fan success!\n");
    }
#endif

}
/**
 * @brief 
 * @remark 检测提取风车中心点
 */
bool EnergyDetector::getCircleCenter(Mat &src){
    if(armor_centers.size() < 3){
        Mat img = src.clone();
        Mat CannyEdge;
        std::vector<vector<Point> > circle_contours;
        vector<Vec3f> circle_point;

        Canny(img, CannyEdge, 30, 200);
        //imshow("canny",CannyEdge);
        findContours(CannyEdge, circle_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        for (size_t t = 0; t <  circle_contours.size(); t++) {
            double area = contourArea( circle_contours[t]);
            if (area < _flow.Center_R_Control_area_min | area > _flow.Center_R_Control_area_max) {
                //cout << "circle area : " << area << endl;
                continue; //中心轮廓图像面积
            }
            Rect rect = boundingRect( circle_contours[t]);
            float ratio = float(rect.width) / float(rect.height);
            if (ratio < 2 && ratio > 0.5) { //近似正方形
                int x = rect.x + rect.width / 2;
                int y = rect.y + rect.height / 2;
                Point2f cal_center = calR1P(); //反解的圆心位置用于判断检测圆心的可信度
                circle(outline, cal_center, 3, Scalar(238, 238, 0), 2, 8, 0);
                //cout << "--" << pointDistance(cal_center,Point2f(x,y)) << endl;
                if(pointDistance(cal_center,Point2f(x,y))< 100){
                    circle_center_point = Point(x, y);
                    circle(outline, circle_center_point, 3, Scalar(255, 255, 255), 2, 8, 0);
                    return true;
                }
                //cout << area << '\t' << ratio << endl;
            }
            //else cout << "ratio = " << ratio << endl;
        }
        return false;
    } else {
        circle_center_point = calR3P(); //当同时存在3个装甲板时，直接用三点定圆
    }
}


/**
 * @brief EnergyDetector::getTargetPoint
 * @param Mat& src
 * @return null
 * @remark 根据armor与flowStripFan情况判断给出唯一目标装甲板target_armor以及其中心点target_point
 */
bool EnergyDetector::getTargetPoint(Mat &src) {
    //todo find armor in best_fan 目标armor
    for (int i = 0; i < valid_fan_strip.size(); i++) {
        //为target_armors打分
        for (int j = 0; j < armors.size(); j++) {
            std::vector<cv::Point2f> intersection;
            if (rotatedRectangleIntersection(armors[j], valid_fan_strip[i], intersection) == 0) //计算两个旋转矩形的交集面积
                continue;
            double cur_contour_area = contourArea(intersection);
            if (cur_contour_area > _flow.target_intersection_contour_area_min) {
                target_blades.emplace_back(Blade_(i, j));
            }//返回目标装甲板参数
            //TODO
            //cout << "target_intersection_contour_area" << cur_contour_area << endl;
        }
    }

    if (!target_blades.empty()) {
        if (target_blades.size() == 1) {
            target_blade = target_blades[0];
        } else {
            //为target_armors打分
            auto *grade = (double *) malloc(target_blades.size() * sizeof(double));
            memset(grade, 0, target_blades.size() * sizeof(double));

            for (int i = 0; i < target_blades.size(); i++) {
                *(grade + i) += pointDistance(armors[target_blades[i].armor_index].center,
                                              last_target_armor.center);      //距离远的为新的待打击装甲
            }

            double max_grade = *grade;
            int max_index = 0;

            for (int t = 0; t < target_blades.size(); t++) {
                if (*(grade + t) < max_grade) {
                    max_grade = *(grade + t);
                    max_index = t;
                }
            }

            free(grade);

            target_blade = target_blades[max_index];
        }

        target_armor = armors[target_blade.armor_index];
        last_target_armor = target_armor;        //update
    } else {
        return false;
    }

    target_point = target_armor.center;
    circle(outline, target_point, 2, Scalar(0, 255, 0), 3);
    //虽然我明白target_armor_centers是存储历史中心点的，但是这个条件没看懂
    if (!target_armor_centers.empty() && target_armor_centers.size() < 3 &&
        pointDistance(target_point, target_armor_centers[target_armor_centers.size() - 1]) > 60) {
        target_armor_centers.push_back(target_point);
    } else if (target_armor_centers.empty())
        target_armor_centers.push_back(target_point);
    return true;
}

/**
 * @remark: 计算中心R标所在区域
 */
Point2f EnergyDetector::calR1P() {
    int center_blade_edge_index;
    int max_length = 0;
    Point2f blade_points[4];
    valid_fan_strip[target_blade.flow_strip_fan_index].points(blade_points);

    for (int i = 0; i < 4; i++) {
        float cur_getTargetPoint_length = pointDistance((blade_points[i] + blade_points[(i + 1) % 4]) / 2.0,
                                                        target_armor.center);
        if (cur_getTargetPoint_length > max_length) {
            max_length = cur_getTargetPoint_length;
            center_blade_edge_index = i;
        }
    }

    Point2f blade_bounding_mid = (blade_points[center_blade_edge_index] + blade_points[(center_blade_edge_index+1)%4])/2; //扇叶靠近圆心的边的中点

    Point2f armor_bounding_mid;
    Point2f armor_mid_vec;
    Point2f armor2center_vec;
    for(int j=0;j<4;j++){
        armor_bounding_mid = (pts[j] + pts[(j+1)%4]) / 2;
        armor_mid_vec = armor_bounding_mid - target_point;
        float d = (blade_bounding_mid - target_point).dot(armor_mid_vec);
        if(d > 500){
            armor2center_vec = armor_mid_vec;
            break;
        }
    }
    Point2f center_point = target_point + K * armor2center_vec;
    circle(outline,center_point,2, Scalar(255, 0, 0), 3);
    return center_point;
}
/**
 * @brief 三点计算圆坐标
 * */
Point2f EnergyDetector::calR3P() {
    Point2f A = armor_centers[0];
    Point2f B = armor_centers[1];
    Point2f C = armor_centers[2];
    float k_ab = (B.y-A.y) / (B.x-A.x);
    float k_bc = (C.y-B.y) / (C.x-B.x);
    //float k_ac = (A.y-C.y) / (A.x-C.x);
    Matrix2f mat_right(2,2);
    MatrixXf mat_left(2,1);
    mat_right << 1, 1/k_ab,
            1, 1/k_bc;
    mat_left << (A.y+B.y)/2 + 1/k_ab * (A.x+B.x)/2,
            (B.y+C.y)/2 + 1/k_bc * (B.x+C.x)/2;
    MatrixXf result(2,1);
    result = mat_right.inverse() * mat_left;
    circle(outline,Point2f (result(1,0),result(0,0)),3,Scalar(200,200,50),2);
    return Point2f (result(1,0),result(0,0));
}
/**
 * @param p 变换参考点
 * @param center 极坐标中心
 * @param theta 预测转过角度
 * @return 预测点坐标
 * @remark 计算预测点坐标
 */
Point2f EnergyDetector::calPredict(Point2f p, Point2f center, float theta) const {
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
 * @param Mat src
 * @brief 通过相邻帧的圆心指向目标的向量叉乘来达到既能判断方向，又可以通过叉乘求角度相邻帧转过的角度（因为已知R在图像中为168）
 * @remark 给出预测射击点位置predict_point
 */
void EnergyDetector::getPredictPointSmall(const float fly_t) {
//    cv::Point3i last_target_vector = Point3i(last_target_point - last_circle_center_point);
//    cv::Point3i target_vector = Point3i(target_point - circle_center_point);
//    cv::Point3i last_cross_cur = last_target_vector.cross(target_vector);
//    double theta = asin(last_cross_cur.z / pow(R, 2));
//    double delta_t = (frame_time - last_frame_time) / getTickFrequency();
//    double omega = theta / delta_t;
//    LOGM("----------------------");
//    LOGM("Theta: %.3f", theta);
//    LOGM("Delta_t: %.3lf", delta_t);
//    LOGM("Omega: %.3lf", omega);
    predict_rad = energy_rotation_direction * 1.4 * fly_t;
    predict_point = calPredict(target_point,circle_center_point,predict_rad);
    getPredictRect(predict_rad,pts);
    circle(outline, predict_point, 4, Scalar(0, 0, 255), -1);
    updateLastValues();
}



/**
 * @brief 大能量机关运动预测
 * @param fly_t 计算飞行时间  单位：s
 * */
void EnergyDetector::getPredictPointBig(const float fly_t) {
    vector<float> cut_filter_omega(filter_omega.end()-6,filter_omega.end()); //取 av_omega 的后 6 个数
    vector<float> cut_time_series(time_series.end()-6,time_series.end());
    Eigen::MatrixXd rate = RMTools::LeastSquare(cut_time_series,cut_filter_omega,1); //对上面数据用最小二乘

    // Five consecutive same judge can change the current state.
    int cur_flag = rate(0,0) > 0 ? 1 : 0; //最小二乘法判断斜率
    if (cur_flag != last_flag) {
        predict_cnt = 0;
    } else {
        predict_cnt++;
    }
    if (predict_cnt == 3) {
        flag = cur_flag;
        predict_cnt = 0;
    }
    last_flag = cur_flag;

    if (filter_omega.back() > 0.52 && filter_omega.back() < 2.09) {
        cur_phi = spdPhi(filter_omega.back(), flag);
        //cur_phi = spd_phi(av_omega.back(), flag);
    }
    else if(filter_omega.back() > 2.09)
        cur_phi = CV_PI / 2;
    else
        cur_phi = - CV_PI / 2;
    double t = cur_phi / w_;

    //predict_rad = spd_int(t + 0.5) - spd_int(t);
    predict_rad = energy_rotation_direction * (spdInt(t + fly_t) - spdInt(t));
    VectorXf rad_vec(1,1);
    rad_vec << predict_rad;
    rad_kf.Update(rad_vec);
    predict_rad = rad_kf.state_post_[0];

    predict_point = calPredict(target_point,circle_center_point,predict_rad); //逆时针为负的预测弧度，顺时针为正的预测弧度
    circle(outline,predict_point,2,Scalar(0,0,255),3);
    getPredictRect(predict_rad, pts); //计算预测的装甲板位置
}

/**
 * @brief 计算当前差角及角速度
 * */
void EnergyDetector::FilterOmega(const float dt) {
    //确认启动大幅预测
    total_t += dt;
    time_series.push_back(total_t); //记录下当前的时间戳
    energy_kf.trans_mat_ << 1, dt,0.5*dt*dt,
                            0, 1, dt,
                            0, 0, 1;
    energy_kf.control_mat_ << 1.0/6 * dt*dt*dt,
                              0.5 * dt*dt,
                              dt;
    MatrixXf u_k(1,1);
    u_k << -a_ * w_*w_ * sin(w_*time_series.back() + phi_);
    rad_kf.trans_mat_ = energy_kf.trans_mat_;
    Point2f p = target_point - circle_center_point; //圆心指向目标的向量
    current_theta = atan2(p.y,p.x);     //当前角度 弧度制
    angle.push_back(current_theta);
    if(angle.size() <= 4) {
        if(angle.size() == 4) {
            current_omega = calOmega(angle,time_series,3);
            initRotateKalman();
            initRadKalman();
            total_theta = current_theta;
        }
    } else {
        float d_theta = current_theta - angle[angle.size()-4];

        if(d_theta > 6)
            d_theta -= 2*CV_PI;
        if(d_theta < -6)
            d_theta += 2*CV_PI;
//        current_omega = d_theta / (time_series.back()-time_series[time_series.size()-4]);
        current_omega = calOmega(angle,time_series,3);
        omega.push_back(current_omega);
        total_theta += d_theta; //累积角度
        VectorXf measure_vec(2,1);
        measure_vec << total_theta,
                current_omega;

        energy_kf.predict(u_k);
        energy_kf.correct(measure_vec);
        filter_omega.push_back(energy_rotation_direction*energy_kf.state_post_[1]);
    }
}


void EnergyDetector::initRotateKalman() {
    energy_kf.measure_mat_.setIdentity();
    // 过程噪声协方差矩阵Q
    energy_kf.process_noise_.setIdentity();
    // 测量噪声协方差矩阵R
    energy_kf.measure_noise_.setIdentity();

    energy_kf.measure_noise_ << 10,0,
                                0, 110;
    // 误差估计协方差矩阵P
    energy_kf.error_post_.setIdentity();
    energy_kf.state_post_ << current_theta,
                            current_omega,
                            0;

}


void EnergyDetector::initRadKalman() {
    rad_kf.measure_mat_.setIdentity();
    // 过程噪声协方差矩阵Q
    rad_kf.process_noise_.setIdentity();
    // 测量噪声协方差矩阵R
    rad_kf.measure_noise_.setIdentity();
    rad_kf.measure_noise_ << 10;
    // 误差估计协方差矩阵P
    rad_kf.error_post_.setIdentity();
    rad_kf.state_post_ <<   1,
                            0,
                            0;
    // 后验估计
    //energy_kf.state_post_ << ;
}



float EnergyDetector::spdInt(float t) {
    return -(a_ * cos(w_ * t) / w_) - (a_ - 2.09) * t;
}

/**
 * @brief 变参数大幅反解相位
 * */
float EnergyDetector::spdPhi(float omega, int flag) {
    float sin_phi = (omega - (2.09-a_))/a_;
    float  phi = asin(sin_phi);
    return flag ? phi : CV_PI - phi;
}

/**
 * @remark: 在预测位置画出待打击装甲板
 */
void EnergyDetector::getPredictRect(float theta, vector<Point2f> pts) {
    for(int i = 0;i<4;i++)
        predict_pts[i] = calPredict(pts[i],circle_center_point, theta);
    if(showEnergy)
        for (int i = 0; i < 4; i++)
            line(outline, predict_pts[i], predict_pts[(i + 1) % (4)], Scalar(238, 238, 0), 2, LINE_8);
}

/**
 * @remark:
 */
void EnergyDetector::updateLastValues() {
    last_target_armor = target_armor;
    last_target_point = target_point;
    last_circle_center_point = circle_center_point;
    last_frame_time = frame_time;
}

/**
 * @brief EnergyDetector::isValidArmorContour
 * @param vector<Point>& armor_contour
 * @return bool
 * @remark 判断找到的装甲Armor尺寸是否合格
 */
bool EnergyDetector::isValidArmorContour(const vector<cv::Point> &armor_contour) const {
    double cur_contour_area = contourArea(armor_contour);
//    LOGW("============Vaild Armor=================");
//    LOGW("Count : %d\t Area : %lf", debug_cur_count++, cur_contour_area);
    if (cur_contour_area > _flow.armor_contour_area_max ||
        cur_contour_area < _flow.armor_contour_area_min) {
        //TODO
        //cout << "armor_contour_area:" << cur_contour_area << endl;
        return false;
    }

    RotatedRect cur_rect = minAreaRect(armor_contour);
    Size2f cur_size = cur_rect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
//    LOGW("Length : %f\t Width : %f", length, width);
    if (length < _flow.armor_contour_length_min || width < _flow.armor_contour_width_min ||
        length > _flow.armor_contour_length_max || width > _flow.armor_contour_width_max) {
        //TODO
        //cout << "armor_contour_length:" << length << endl;
        //cout << "armor_contour_width:" << width << endl;
        return false;
    }

    float length_width_ratio = length / width;

    if (length_width_ratio > _flow.armor_contour_hw_ratio_max ||
        length_width_ratio < _flow.armor_contour_hw_ratio_min) {
        //TODO
        //cout << "armor_contour_hw_ratio" << length_width_ratio <<endl;
        return false;
    }
    //cout << "right armor area : " << cur_contour_area << endl;
    //cout << "armor ratio : " << length_width_ratio << endl;
    return true;
}


/**
 * @brief EnergyDetector::isValidFlowStripFanContour
 * @param vector<Point>& flow_strip_fan_contour
 * @return bool
 * @remark 判断找到的含有流动条的扇叶尺寸是否合格
 */
bool EnergyDetector::isValidFlowStripFanContour(cv::Mat &src, const vector<cv::Point> &flow_strip_fan_contour) const {
    double cur_contour_area = contourArea(flow_strip_fan_contour);
//    LOGW("============Vaild Flow=================");
//    LOGW("Area : %lf", cur_contour_area);
    if (cur_contour_area > _flow.flow_strip_fan_contour_area_max ||
        cur_contour_area < _flow.flow_strip_fan_contour_area_min) {
        //TODO
        //cout << "flow_strip_fan_contour_area" << cur_contour_area << endl;
        return false;
    }

    RotatedRect cur_rect = minAreaRect(flow_strip_fan_contour);
    Size2f cur_size = cur_rect.size;

    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
//    LOGW("Length : %f\t Width : %f", length, width);
    if (length < _flow.flow_strip_fan_contour_length_min
        || width < _flow.flow_strip_fan_contour_width_min
        || length > _flow.flow_strip_fan_contour_length_max
        || width > _flow.flow_strip_fan_contour_width_max) {
        //TODO
        //cout << "flow_strip_fan_contour_length" << length << endl;
        //cout << "flow_strip_fan_contour_width" << width << endl;
        return false;
    }

    float length_width_ratio = length / width;
//    LOGW("Ratio : %f", length_width_ratio);
//    LOGW("Rect Size : %f\t Contour Size : %lf", cur_size.area(), cur_contour_area);
    if (length_width_ratio > _flow.flow_strip_fan_contour_hw_ratio_max ||
        length_width_ratio < _flow.flow_strip_fan_contour_hw_ratio_min) {
        //TODO
        //cout << "flow_strip_fan_contour_hw_ratio" << length_width_ratio << endl;
        return false;
    }

    if (cur_contour_area / cur_size.area() < _flow.flow_strip_fan_contour_area_ratio_min
        || cur_contour_area / cur_size.area() > _flow.flow_strip_fan_contour_area_ratio_max) {
        //TODO
        //cout << "flow_strip_fan_contour_area_ratio" << cur_contour_area / cur_size.area() << endl; //流动条轮廓占总面积的比例
        return false;
    }

    return true;
}


//todo 工具类函数
/**
 * @brief EnergyDetector::pointDistance
 * @param Point point_1,Point point_2
 * @return double distance
 * @remark 输入两点，返回两点间距离
 */
double EnergyDetector::pointDistance(const cv::Point& point_1, const cv::Point& point_2) {
    double distance = 0;
    distance = sqrt(
            pow(static_cast<double>(point_1.x - point_2.x), 2) + pow(static_cast<double>(point_1.y - point_2.y), 2));
    return distance;
}


void EnergyDetector::getPts(RotatedRect armor) {
    Point2f rectPoints[4];//定义矩形的4个顶点
    armor.points(rectPoints); //计算矩形的4个顶点

    //judge long side
    if (sqrt(pow((rectPoints[0].x - rectPoints[1].x), 2) + pow((rectPoints[0].y - rectPoints[1].y), 2))
        > sqrt(pow((rectPoints[2].x - rectPoints[1].x), 2) + pow((rectPoints[2].y - rectPoints[1].y), 2))) {
        //pts[0]-pts[1] is long side
        pts[0] = rectPoints[0];
        pts[1] = rectPoints[1];
        pts[2] = rectPoints[2];
        pts[3] = rectPoints[3];
    } else {
        //pts[1]-pts[2] is long side
        pts[0] = rectPoints[1];
        pts[1] = rectPoints[2];
        pts[2] = rectPoints[3];
        pts[3] = rectPoints[0];
    }
}

/**
 * @brief EnergyDetector::isValidCenterRContour
 * @param vector<Point>& center_R_contour
 * @return bool
 * @remark 判断找到的中心点R尺寸是否合格
 */
bool EnergyDetector::isValidCenterRContour(const vector<cv::Point> &center_R_contour) {
    double cur_contour_area = contourArea(center_R_contour);
    if (cur_contour_area > _flow.Center_R_Control_area_max ||
        cur_contour_area < _flow.Center_R_Control_area_min) {
        return false;
    }
    return true;
}
float EnergyDetector::calOmega(vector<float> &angle_vec, vector<float> &time_vec, int step) {
    int step_ = step + 1;
    float d_theta = angle_vec.back() - angle_vec[angle_vec.size()-step_];
    float dt = time_vec.back() - time_vec[time_vec.size()-step_];
    if(d_theta > 6)
        d_theta -= 2*CV_PI;
    if(d_theta < -6)
        d_theta += 2*CV_PI;
    return d_theta / dt;
}