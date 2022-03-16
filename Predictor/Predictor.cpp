//
// Created by tyy on 2022/1/19.
//
#include "Predictor.h"

Predictor::Predictor() : waveClass(20,600,1000){
    for (int i = 0; i < 7; i++) {
        frame_list.push_back(i);
    }
}

Predictor::~Predictor() = default;


/**
 * @brief
 * @param pyd 绝对 yaw pitch dist
 * @param deltaT 3 帧平均时间
 * */
void Predictor::test1Predict(Vector3f ypd, const float deltaT) {
    abs_pyd.push_back(ypd);
    if(abs_pyd.size()>4){
        Vector3f v_ypd = (abs_pyd.back() - abs_pyd[abs_pyd.size()-4]) / (deltaT*3/1000);
        predict_ypd = abs_pyd.back() + v_ypd * 0.2;
        //predict_yp = tmp.col(1);
    }
}

/**
 * @brief nuc上测试效果还行的绝对角度预测
 * */
 void Predictor::armorPredictor(Vector3f ypd, Vector3f gimbal_ypd, const float deltaT) {
     abs_yaw.push_back(ypd(0,0));
     if(abs_yaw.size() > 8){
         vector<float> cut_yaw (abs_yaw.end() - 7, abs_yaw.end());
         k = RMTools::LeastSquare(frame_list, cut_yaw, 1); //目前用最小二乘法做一次函数拟合，二次有无必要？
         for(int i = 0; i < sizeof (rate)/sizeof (rate[i]) - 1; i++){
             rate[i] = rate[i+1];
             yaw_arr[i] = yaw_arr[i+1];
         }
         rate[3] = k(0,0);
         float avg_k =RMTools::average(rate,4); //移除速度数据突变的可能性
         yaw_arr[3] = abs_yaw.back() + avg_k * frame;
         if(abs_yaw.size() > 13){
             yaw = RMTools::average(yaw_arr,4); //预测后 60 帧 yaw 角度
         }else
             yaw = yaw_arr[3];
         waveClass.displayWave(yaw_arr[2]-gimbal_ypd[0], 0);
     }
 }
 /**
  * @brief 利用云台绝对 yaw 角反解目标相对云台坐标系的 x
  * @param target_ypd 电控的绝对 yaw pitch 角和距离 distance
  * @param yp_speed 电控6轴陀螺仪获得的
  * @param lastT 上一帧时间
  * */
void Predictor::testPredictLineSpeed(Vector3f target_ypd, Vector3f yp_speed, const float lastT) {
    time_list.push_back(lastT);
    float tan_yaw = tan(target_ypd[0] / 360 * 2 * CV_PI);
    float tan_pitch = tan(target_ypd[1] / 360 * 2 * CV_PI);
    float dist2 = pow(target_ypd[2],2); //
    float z_ = sqrt( dist2 / (1+pow(tan_yaw,2)) / (1+pow(tan_pitch,2)) );
    float x_ = sqrt(pow(z_,2) * pow(tan_yaw,2));
    target_x.push_back(x_); //由yaw pitch distance解出的
}

/**
 * @brief
 * */
void Predictor::kalmanPredict(Vector3f target_ypd, Vector3f gimbal_ypd) {
    Vector3f target_xyz;
    calWorldPoint(target_ypd,target_xyz);

    float dx;
    if(kf_flag)
        dx = x_ - target_x.back();
    target_x.push_back(x_);
    target_z.push_back(z_);
    if(target_x.size()>8){
        vector<float> cut_x (target_x.end() - 7, target_x.end());
        k = RMTools::LeastSquare(frame_list, cut_x, 1); //目前用最小二乘法做一次函数拟合，二次有无必要？
    }

    z_k = target_xyz;
    vector<float> show_data;
    for(int len = 0;len<z_k.size();len++)
        show_data.push_back(z_k[len]);

    if(kf_flag){
        kf.Update(z_k);
        for(int len = 0;len<kf.x_k.size();len++)
            show_data.push_back(kf.x_k[len]);
        predict_xyz << kf.x_k[0], kf.x_k[1], kf.x_k[2];
        //cout << "delta x between 2 frame: " << dx << endl;
        string str[] = {"m_x","m_y","m_z","kf_x","kf_y","kf_z","kf_vx","kf_vy","kf_vz"};
        showData(show_data, str);
        if(target_x.size()>8)
            waveClass.displayWave(0, kf.x_k[3]);

    }else{
        kf_flag = true;
        kf.Init(3,6,1); //滤波器初始化
        kf.x_k = kf.H_.transpose() * z_k;  //设置第一次状态估计
        kf.Q_ << 150,0,0,0,0,0,
                 0,35,0,0,0,0,
                 0,0,35,0,0,0,
                 0,0,0,1,0,0,
                 0,0,0,0,1,0,
                 0,0,0,0,0,1 ;

        kf.R_ << 800,0,0,
                 0,65,0,
                 0,0,65 ;
    }
}

/**
 * @brief 变更目标时更新预测器
 * */
void Predictor::Refresh() {
    kf_flag = false;
    abs_pyd.clear();
    abs_yaw.clear();
}

void Predictor::showData(vector<float> data, string *str){
    int c = data.size() * 35;
    Mat background = Mat(c,400,CV_8UC3,Scalar::all(0));
    for(int i = 0;i<data.size();i++){
        putText(background, str[i], Point(0, 30*(i+1)), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2, 8, 0);
        putText(background, to_string(data[i]), Point(150, 30*(i+1)), cv::FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 2, 8, 0);
    }
    imshow("data window",background);
}

void Predictor::calWorldPoint(Vector3f target_ypd, Vector3f &target_xyz) {
    pair<float, float> quadrant[4] = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

    float yaw_ = RMTools::total2circle(target_ypd[0]);

    float tan_yaw = tan(yaw_ * degree2rad);
    float tan_pitch = tan(target_ypd[1] * degree2rad);
    float dist2 = target_ypd[2] * target_ypd[2]; //
    z_ = sqrt( dist2 / (1 + tan_yaw * tan_yaw) / (1 + tan_pitch * tan_pitch) );
    x_ = z_ * fabs(tan_yaw);
    y_ = tan_pitch * sqrt(x_ * x_ + z_ * z_);
    //算x,z符号
    int t = yaw_ / 90;
    x_ *= quadrant[t].first; z_ *= quadrant[t].second;
    target_xyz << x_, y_, z_;
}


