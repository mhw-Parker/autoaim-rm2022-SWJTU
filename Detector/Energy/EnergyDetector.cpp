/*********************************************************************************
  *Copyright(C),2019-2021,西南交通大学HELIOS战队All Rights Reserved
  *FileName:  EnergyDetector.cpp
  *Author:  黄冬婷 罗俊辉 田翊扬 徐润泽
  *Version: 1.4
  *Date:  2022.07.05
  *Description: 能量机关识别及预测
  *Function List:
     1.EnergyTask   能量机关任务执行接口
     2.getTargetPoint   获得目标点
     3.getPredictPoint    寻找攻击点
**********************************************************************************/

#include <fstream>
#include "EnergyDetector.h"


using namespace std;

/**
 * @brief EnergyDetector::EnergyDetector
 * @param null
 * @return null
 * @remark Energy类构造函数，初始化有关参数
 */
EnergyDetector::EnergyDetector() {
    initEnergy();
    initEnergyPartParam();//对能量机关参数进行初始化
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
    pts.resize(4);
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
    _flow.armor_contour_area_max = 1500;//1500
    _flow.armor_contour_area_min = 500;//400
    _flow.armor_contour_length_max = 80;//50
    _flow.armor_contour_length_min = 20;//25
    _flow.armor_contour_width_max = 80;//30
    _flow.armor_contour_width_min = 15;//15
    _flow.armor_contour_hw_ratio_max = 3;//3
    _flow.armor_contour_hw_ratio_min = 1.5;//1

///流动条所在扇叶的相关筛选参数
    _flow.flow_strip_fan_contour_area_max = 5500;
    _flow.flow_strip_fan_contour_area_min = 2500;
    _flow.flow_strip_fan_contour_length_max = 220;
    _flow.flow_strip_fan_contour_length_min = 80;
    _flow.flow_strip_fan_contour_width_max = 140;
    _flow.flow_strip_fan_contour_width_min = 50;
    _flow.flow_strip_fan_contour_hw_ratio_max = 3;
    _flow.flow_strip_fan_contour_hw_ratio_min = 1.2;
    _flow.flow_strip_fan_contour_area_ratio_max = 0.65;
    _flow.flow_strip_fan_contour_area_ratio_min = 0.20;

///流动条到装甲板距离参数
    _flow.Strip_Fan_Distance_max = 56;
    _flow.Strip_Fan_Distance_min = 28;

///流动条相关参数筛选
    _flow.target_intersection_contour_area_min = 40/4;//重合面积

///中心R标筛选相关参数，中心亮灯面积约为装甲板面积的1/2
    _flow.Center_R_Control_area_max = 500;
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
    output_pts.clear(); //输出4点
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
        output_pts = pts;
    } else {
        detect_flag = false;
    }
    if(debug)
        imshow("outline",outline);
}
/**
 * @brief EnergyDetector::preprocess
 * @param Mat& src
 * @return Mat& binary
 * @remark 图像预处理，完成二值化
 */
Mat EnergyDetector::preprocess(Mat &src) {
    Mat gray,binary,sub_mat;
    if(blueTarget) {
        cvtColor(src,gray, COLOR_BGR2GRAY);
        threshold(gray,binary,100,255,THRESH_BINARY);
    }
    else {
        vector<Mat> channels;
        split(src, channels);
        subtract(channels[2],channels[0],sub_mat);
        threshold(sub_mat,binary,110,255,THRESH_BINARY);
    }

    //imshow("sub mat", sub_mat);
     Mat element_dilate_1 = getStructuringElement(MORPH_RECT, Size(3, 3));
     dilate(binary, binary, element_dilate_1);
     GaussianBlur(binary,binary,Size(3,3),0);

    if(showBinaryImg)
        imshow("binary",binary);
    return binary;
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

        //Canny(img, CannyEdge, 30, 200);
        //imshow("canny",CannyEdge);
        findContours(src, circle_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        for (size_t t = 0; t <  circle_contours.size(); t++) {
            double area = contourArea( circle_contours[t]);
            if (area < _flow.Center_R_Control_area_min | area > _flow.Center_R_Control_area_max) {
                //cout << "circle area : " << area << endl;
                continue; //中心轮廓图像面积
            }
            //cout << "circle area : " << area << endl;
            Rect rect = boundingRect( circle_contours[t]);
            float ratio = float(rect.width) / float(rect.height);
            if (ratio < 4 && ratio > 0.8) { //近似正方形
                //cout << "ratio : " << ratio << endl;
                int x = rect.x + rect.width / 2;
                int y = rect.y + rect.height / 2;
                Point2f cal_center = calR1P(); //反解的圆心位置用于判断检测圆心的可信度
                circle(outline, cal_center, 3, Scalar(238, 238, 0), 2, 8, 0);
                //cout << "--" << pointDistance(cal_center,Point2f(x,y)) << endl;
                if (pointDistance(cal_center,Point2f(x,y))< 50) {
                    circle_center_point = Point(x, y);
                    circle(outline, circle_center_point, 3, Scalar(255, 255, 255), 2, 8, 0);
                    //imshow("outline",outline);
                    //cout << "area : " << area << "\n" ;
                    return true;
                }
                //cout << area << '\t' << ratio << endl;
            }
            //else cout << "ratio = " << ratio << endl;
        }
        //return false;
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
    if(debug) {
        printf("----- energy armor param -----\n");
        printf("armor_contour_area : %f\n", cur_contour_area);
        printf("armor_contour_hw_ratio : %f\n", length_width_ratio);
        printf("armor_contour_length : %f\n", length);
        printf("armor_contour_width : %f\n", width);
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
        //cout << "flow_strip_fan_contour_area : " << cur_contour_area << endl;
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
    if(debug) {
        printf("flow_strip_fan_contour_length : %f\n", length);
        printf("flow_strip_fan_contour_width : %f\n", width);
        printf("flow_strip_fan_contour_hw_ratio : %f\n", length_width_ratio);
        printf("flow_strip_fan_contour_area_ratio : %f\n", cur_contour_area / cur_size.area());
        printf("flow_strip_fan_contour_area : %f\n", cur_contour_area);
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
