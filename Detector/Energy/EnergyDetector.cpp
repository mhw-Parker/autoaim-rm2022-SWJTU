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

Mat outline;

/*static Mat polyfit(list<float> &in_point, int n) {
    int size = in_point.size();
    //所求未知数个数
    int x_num = n + 1;
    //构造矩阵U和Y
    Mat mat_u(size, x_num, CV_64F);
    Mat mat_y(size, 1, CV_64F);

    for (int i = 0; i < mat_u.rows; ++i)
        for (int j = 0; j < mat_u.cols; ++j) {
            mat_u.at<double>(i, j) = pow(i * 5, j);
        }

    int i = 0;
    list<float>::iterator it; //声明一个迭代器

    for (it = in_point.begin(); it != in_point.end(); it++) {
        mat_y.at<double>(i, 0) = static_cast<double>(*it);
        i++;
    }

    //矩阵运算，获得系数矩阵K
    Mat mat_k(x_num, 1, CV_64F);
    mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
    return mat_k;
}*/

/**
 * @brief EnergyDetector::EnergyDetector
 * @param null
 * @return null
 * @remark Energy类构造函数，初始化有关参数
 */
EnergyDetector::EnergyDetector() : waveClass(4,600,1000){
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
    show_armors = true;//是否显示图像
    show_target_armor = false;//是否显示调试过程
    show_strip_fan = false;//是否显示灯扇
    show_center_R = true;//是否显示数据
    show_target_point = true;//是否显示目标点
    show_predict_point = true;//是否显示预测点

    last_target_polar_angle_judge_rotation = -1000;//上一帧待击打装甲板的极坐标角度（用于判断旋向）
    clockwise_rotation_init_cnt = 0;//装甲板顺时针旋转次数
    anticlockwise_rotation_init_cnt = 0;//装甲板逆时针旋转s次数
    energy_rotation_init = true;//若仍在判断风车旋转方向，则为true
    predict_rad = 0;//预测提前角
    predict_rad_norm = 25;// 预测提前角的绝对值
    predict_point = Point(0, 0);//预测打击点初始化
    pts.resize(4);
    predict_pts.resize(4);

    delta_theta.resize(angle_length); //长度为4
    angle.resize(angle_length);
    predict_arr.resize(omega_length); //预测角度数组初始化
    omega.resize(omega_length);
    x_list.resize(omega_length);
    av_omega.resize(omega_length);
    for (int i = 0; i < omega.size(); ++i) {
        x_list[i] = i * 0.02;
        av_omega[i] = 0;
    }
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
    _flow.armor_contour_area_max = 1400;//1500
    _flow.armor_contour_area_min = 400;//400
    _flow.armor_contour_length_max = 55;//50
    _flow.armor_contour_length_min = 25;//25
    _flow.armor_contour_width_max = 40;//30
    _flow.armor_contour_width_min = 10;//15
    _flow.armor_contour_hw_ratio_max = 3;//3
    _flow.armor_contour_hw_ratio_min = 1;//1

///流动条所在扇叶的相关筛选参数
    _flow.flow_strip_fan_contour_area_max = 4400;
    _flow.flow_strip_fan_contour_area_min = 2000;
    _flow.flow_strip_fan_contour_length_max = 160;
    _flow.flow_strip_fan_contour_length_min = 90;
    _flow.flow_strip_fan_contour_width_max = 70;
    _flow.flow_strip_fan_contour_width_min = 45;
    _flow.flow_strip_fan_contour_hw_ratio_max = 2.8;
    _flow.flow_strip_fan_contour_hw_ratio_min = 1.2;
    _flow.flow_strip_fan_contour_area_ratio_max = 0.55;
    _flow.flow_strip_fan_contour_area_ratio_min = 0.20;

///流动条到装甲板距离参数
    _flow.Strip_Fan_Distance_max = 56;
    _flow.Strip_Fan_Distance_min = 28;

///流动条相关参数筛选
    _flow.flow_strip_contour_area_max = 400;
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
    _flow.Center_R_Control_area_max = 1000;
    _flow.Center_R_Control_area_min = 150;
    _flow.Center_R_Control_length_max = 35;
    _flow.Center_R_Control_length_min = 6;
    _flow.Center_R_Control_width_max = 35;
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
    centerRs.clear();
    armor_centers.clear();
    /*if(omega.size() > 1000){
        omega(omega.end()-6,omega.end());
    }*/
}

/**
 * @brief EnergyDetector::EnergyTask
 * @param src 摄像头读入的图片
 * @param mode 大小幅模式选择
 * @param deltaT 目前用的三阶差分求 omega 所以 deltaT 为三帧平均时间
 * @remark 能量机关任务执行接口
 */
void EnergyDetector::EnergyTask(const Mat &src, int8_t mode, const float deltaT) {
    clearAll();
    Mat img = src.clone();
    Mat binary;
    BIG_MODE = mode;

    roi = preprocess(img); //预处理
    //binary = preprocess(img);
    //roi = binary;
    //findROI(binary,roi); //设定roi
    //imshow("roi",roi);

    if (detectArmor(roi) && detectFlowStripFan(roi) && getTargetPoint(roi)) {
        getPts(target_armor); //获得的装甲板4点
        detectCircleCenter(roi); //识别旋转圆心
        calOmega(deltaT); //计算当前的角速度 cur_omega 为当前三阶差分计算的角速度 av_omega.back() 为 4 次角速度平滑均值
        if(judgeRotation()) {
            if (mode == SMALL_ENERGY_STATE) getPredictPointSmall();
            else if (mode == BIG_ENERGY_STATE) {
                getPredictPoint();
                waveClass.displayWave(av_omega.back(), predict_rad,"energy rad");
            }
        }
        //waveClass.displayWave(omega.back(), av_omega.back());
        detect_flag = true;
        misscount = 0;
    }else{
        misscount++;
        predict_point = Point2f (0,0);
        if(misscount>5){ //连续5帧丢目标
            misscount = 0;
            detect_flag = false;
        }
    }
    if(showEnergy){
        imshow("binary", roi);
        circle(outline, Point(IMGWIDTH/2, IMGHEIGHT/2), 2, Scalar(255, 255, 255), 3); //画出图像中心
        imshow("outline", outline);
        waitKey(1);
    }
}
/**
 * @brief 每次切换状态清空
 * */
void EnergyDetector::init() {
    clearAll(); //清除容器
    cnt_t = 0;
    cnt_i = 0;

    omega.clear();
    time_series.clear();
    delta_theta.clear();
    angle.clear();
    predict_arr.clear();
    av_omega.clear();

    delta_theta.resize(angle_length); //长度为4
    angle.resize(angle_length);
    predict_arr.resize(omega_length); //预测角度数组初始化
    omega.resize(omega_length);
    av_omega.resize(omega_length);
    for (int i = 0; i < omega.size(); ++i)
        av_omega[i] = 0;

}

/**
 * @brief 初始化判断大幅旋转旋方向，迭代计算大幅速度正弦变化函数参数 a w phi
 * @param src 摄像头读入图像
 * @param startT 任务起始时间
 * */
bool EnergyDetector::judgeRotation() {
    int times = 300;
    if (!cnt_t) {
        startT = getTickCount();//第一帧识别到后记录下当前时间
        cnt_t++;
        return false;
    } else if (cnt_t < times) {
        if (delta_theta.back() < 0) cnt_i++; //记录下差角逆时针变化的次数
        cnt_t++;
        //cout << "次数 ：" << cnt_t++ << endl;
        return false;
    } else if (cnt_t == times) {
        //cout << cnt_t++ << endl;
        cnt_t++;
        if (cnt_i > times / 2 + 10)
            energy_rotation_direction = 1; //顺时针
        else
            energy_rotation_direction = -1; //逆时针
        estimateParam(omega, time_series, times); //分别是时间戳对应的 omega 和用于拟合计算的点的数量
        return true;
    } else
        return true;

}
/**
 * @brief 利用 ceres-solver 对目标 sin 函数进行参数估计
 * @param omega_ 用于参数估计的 omega 数组
 * @param t_ 与 omega 相对应的时间戳
 * @param times 用于曲线拟合的数据点
 * */
void EnergyDetector::estimateParam(vector<float> omega_, vector<float> t_, int times) {
    if(omega_.size() >= times - 1){
        int st = omega_.size() - times + 1;
        float min_w = 5;
        for(int i = 8; i < omega.size(); i++)
            min_w = (av_omega[i] < min_w) ? av_omega[i] : min_w; //找出最小值，进而确定中值

        for (int i = 8; i < omega.size(); ++i) {
            if(abs(omega_[i] - (2.09-min_w)/2) < 0.05 ){
                vector<float> cut_t, cut_o;
                for(int j=0; j<20; j++){
                    cut_t.push_back(t_[i+j] - t_[i]);
                    cut_o.push_back(av_omega[i+j]);
                }
                Eigen::MatrixXd min_sq = RMTools::LeastSquare(cut_t,cut_o,1);
                cout << "--- " << min_sq << endl;
                if(min_sq(0,0)>0){
                    st = i; //从接近 sin 中值的地方开始拟合
                    break;
                }else{
                    a_ = - a_;
                    st = i;
                    break;
                    //continue;
                }

            }
        }
        for(int i = st; i < omega_.size(); i++){
            ceres::CostFunction* cost_func =
                    new ceres::AutoDiffCostFunction<SinResidual,1,1,1,1>(
                            new SinResidual(t_[i]-t_[st],omega_[i])); //确定拟合问题是横坐标问题，需要初始化第一个坐标为 0
            problem.AddResidualBlock(cost_func, NULL, &a_, &w_,&phi_ );
#if SAVE_LOG == 1
            write_energy_data << t_[i]-t_[st] << " " << omega_[i] << " " << endl;
#endif
            //problem.AddResidualBlock(cost_func, NULL, &a_, &phi_ );
            cout << t_[i]-t_[st] << " " << omega_[i] << endl;
            waveClass.displayWave(omega_[i], 1,"curve fitting");
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
        cout << "拟合数据下标起点：" << st << " " << av_omega[st] << " 拟合数据点数 ： " << omega.size() - st << " 函数中值：" << (2.09-min_w)/2 << endl;
#if SAVE_LOG == 1
        write_energy_data << "---Final   a: " << a_ << " w: " << w_ << " phi: " << phi_ << endl;
#endif
        if(a_ < 0.780) a_ = 0.785;
        else if(a_ > 1.045 ) a_ = 1.045;
        if(w_ < 0) w_ = abs(w_);
        if(w_ < 1.884) w_ = 1.884;
        else if(w_ > 2) w_ = 2;

        waitKey(0);
    }
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
    blue_c = channels.at(0);
    red_c = channels.at(2);
//    if (blueTarget) {
//        single = channels.at(0);
//    } else {
//        single = channels.at(2);
//    }

    //threshold(single, binary, 90, 255, THRESH_BINARY);
    threshold(blue_c,blue_binary,150,255,THRESH_BINARY);
    threshold(red_c,red_binary,150,255,THRESH_BINARY);

    binary = blueTarget ? blue_binary - red_binary : red_binary - blue_binary; //滤掉白光

//TODO 用相机时卷积核用3*3，视频测试亮度低用5*5
    Mat element_dilate_1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(binary, binary, element_dilate_1);
    morphologyEx(binary, binary, MORPH_CLOSE, element_dilate_1);
    threshold(binary, binary, 0, 255, THRESH_BINARY);

    GaussianBlur(binary,binary,Size(3,3),0);

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

    }else
    {
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
        valid_armors.push_back(flow_rect);
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
    //flow_strip_fan dilate

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
bool EnergyDetector::detectCircleCenter(Mat &src){
    Mat img = src.clone();
    Mat CannyEdge;
    std::vector<vector<Point> > circle_contours;
    vector<Vec3f> circle_point;

    Canny(img, CannyEdge, 50, 150);
    findContours(CannyEdge, circle_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    for (size_t t = 0; t <  circle_contours.size(); t++) {
        double area = contourArea( circle_contours[t]);
        if (area < _flow.Center_R_Control_area_min | area > _flow.Center_R_Control_area_max) {
            //cout << "circle area : " << area << endl;
            continue; //中心轮廓图像面积
        }
        Rect rect = boundingRect( circle_contours[t]);
        float ratio = float(rect.width) / float(rect.height);
        if (ratio < 1.2 && ratio > 0.80) { //近似正方形
            int x = rect.x + rect.width / 2;
            int y = rect.y + rect.height / 2;
            Point2f cal_center = calR(); //反解的圆心位置用于判断检测圆心的可信度
            circle(outline, cal_center, 3, Scalar(238, 238, 0), 2, 8, 0);
            if(pointDistance(cal_center,Point2f(x,y))< 50){
                circle_center_point = Point(x, y);
                circle(outline, circle_center_point, 3, Scalar(255, 255, 255), 2, 8, 0);
                return true;
            }
            //cout << area << '\t' << ratio << endl;
        }//else cout << "ratio = " << ratio << endl;

    }
    return false;
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
Point2f EnergyDetector::calR() {
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
    return center_point;
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
void EnergyDetector::getPredictPointSmall() {
    cv::Point3i last_target_vector = Point3i(last_target_point - last_circle_center_point);
    cv::Point3i target_vector = Point3i(target_point - circle_center_point);
    cv::Point3i last_cross_cur = last_target_vector.cross(target_vector);
    double theta = asin(last_cross_cur.z / pow(R, 2));
    double delta_t = (frame_time - last_frame_time) / getTickFrequency();
    double omega = theta / delta_t;
    LOGM("----------------------");
    LOGM("Theta: %.3f", theta);
    LOGM("Delta_t: %.3lf", delta_t);
    LOGM("Omega: %.3lf", omega);
    predict_rad = -1.4 * 0.3;
    predict_point = calPredict(target_point,circle_center_point,predict_rad);
    getPredictRect(predict_rad,predict_pts);
    circle(outline, predict_point, 4, Scalar(0, 0, 255), -1);
    updateLastValues();
}



/**
 * @brief 大能量机关运动预测
 * @param src
 * @param deltaT 两帧的间隔时间，可用平均耗时，单位：ms
 * */
void EnergyDetector::getPredictPoint()
{
    /// new 2022/1/13
    vector<float> cut_av_omega(av_omega.end()-6,av_omega.end()); //取 av_omega 的后 6 个数
    Eigen::MatrixXd rate = RMTools::LeastSquare(x_list,cut_av_omega,1); //对上面数据用最小二乘

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


    if(av_omega.back() > 0.52 && av_omega.back() < 2.09){
        cur_phi = spdPhi(av_omega.back(), flag);
        //cur_phi = spd_phi(av_omega.back(), flag);
    }
    else if(av_omega.back() > 2.09)
        cur_phi = CV_PI / 2;
    else
        cur_phi = - CV_PI / 2;
    double t = cur_phi / w_;

    //predict_rad = spd_int(t + 0.5) - spd_int(t);
    predict_rad = spdInt(t + 0.5) - spdInt(t);

    predict_arr.push_back(predict_rad);

    float sum_rad = 0;
    for(int i = 0; i < 3; i++){
        sum_rad += predict_arr[predict_arr.size() - 1 - i];
    }
    predict_rad = sum_rad / 3;
//    if(showEnergy)
//        waveClass.displayWave(av_omega.back(), 1.305);

    predict_point = calPredict(target_point,circle_center_point,-energy_rotation_direction*predict_rad); //逆时针为负的预测弧度，顺时针为正的预测弧度
    circle(outline,predict_point,2,Scalar(0,0,255),3);

    getPredictRect(-energy_rotation_direction*predict_rad, pts); //计算预测的装甲板位置

}

/**
 * @brief 计算当前差角及角速度
 * */
void EnergyDetector::calOmega(float deltaT) {
    if(cnt_t!=0) //确认启动大幅预测
        time_series.push_back(RMTools::CalWasteTime(startT,getTickFrequency())/1000); //记录下当前的时间戳
    Point2f p = circle_center_point - target_point;             //指向圆心的向量
    float cur_theta = atan2(p.y,p.x) / (2*CV_PI) * 360;     //当前角度
    angle.push_back(cur_theta);
    delta_theta.push_back(cur_theta - angle[angle.size()-4]);  //相隔3个数相减  size()-1-3
    if(delta_theta.back() > 300) //解决 180 -180 跳变问题
        delta_theta.back() = 360 - delta_theta.back();
    cur_omega = abs(delta_theta.back() ) / (deltaT * 3/1000) * (2*CV_PI/360); //转为弧度制,算3帧的角速度
    //cout << "--- current spd : " << cur_omega << endl;
    if(cur_omega > 2.15)
        cur_omega = 2.15;
    omega.push_back(cur_omega); //将当前的 cur_omega 存放在 omega 数组中

    //cout << "current omega = " << cur_omega << endl;
    float sum_omega = 0;
    for(int i = 0; i < 4; i++){
        sum_omega += omega[omega.size() - 1 - i];
    }
    av_omega.push_back(sum_omega / 4); //4次角速度平滑均值
}

float EnergyDetector::spd_int(float t) {
    return 1.305*t - 0.4167*cos(1.884*t);
}
float EnergyDetector::spdInt(float t) {
    return -(a_ * cos(w_ * t) / w_) - (a_ - 2.09) * t;
}
/**
 * @brief 计算当前 omega 对应的相位
 * @param omega angle velocity
 * @param flag 1增0减
 * @return 当前的相位
 */
float EnergyDetector::spd_phi(float omega, int flag) {
    float a = (omega - 1.305) / 0.785;
    float phi = asin(a);
    return flag ? phi : CV_PI - phi;
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
void EnergyDetector::em() {
    filter_omega.push_back(av_omega.back());
    if(filter_omega.size() > 150){
        for(int i = 0; i < 149; i++){
            if(filter_omega[filter_omega.size() - i] > max_omega){
                max_omega = filter_omega[filter_omega.size() - i];
                max_t = time_series[filter_omega.size() - i];
            }
            if(filter_omega[filter_omega.size() - i] < min_omega){
                min_omega = filter_omega[filter_omega.size() - i];
                min_t = time_series[filter_omega.size() - i];
            }
            //waveClass.displayWave(filter_omega[filter_omega.size() - i],1.305);
        }
        float cal_w = 2 * CV_PI / (2 * (abs(min_t - max_t)));
        max_omega = 0;
        min_omega = 5;
        cout << min_t<<" " << max_t << "用半周期计算得到的 w 值 ：" << cal_w << endl;
        //waitKey(0);
    }
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
