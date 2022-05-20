#include "SolveAngle.hpp"
#include "Kalman.hpp"
#include "preoptions.h"

#define SHOOTFORMULAX(X) (60 - (abs(X - FRAMEWIDTH/2)))
#define SHOOTFORMULAY(Y) (40 - (abs(Y - FRAMEHEIGHT/2)))
#define SHOOTFORMULA(X,Y) (3600 - ((Y - FRAMEHEIGHT/2)*(Y - FRAMEHEIGHT/2) + (X - FRAMEWIDTH/2)*(X - FRAMEWIDTH/2)))

using namespace cv;

SolveAngle::SolveAngle() {
    string filename ="../Math/SolveAngle/camera.xml";
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened()) {
        cout << "no such file" << endl;
        return;
    }
    switch(carName) {
        case HERO:
        case INFANTRY_MELEE0:
        case INFANTRY_MELEE1:
            fs["Distortion_Coefficients5_MIND133GC-0"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND133GC-0"] >> cameraMatrix;
            cv2eigen(cameraMatrix,cam_mat);
            //枪口坐标系相对位置
            //fit_xyz << 0, 50, 50; //x y z
            break;
        case INFANTRY_TRACK:
            break;
        case VIDEO:
            fs["Distortion_Coefficients5_MIND134GC-0"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND134GC-0"] >> cameraMatrix;
            cv2eigen(cameraMatrix,cam_mat);
            yaw_static = 1.6;
            pitch_static = 0.5;
        case SENTRY:
            fs["Distortion_Coefficients5_MIND134GC-0"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND134GC-0"] >> cameraMatrix;
            cv2eigen(cameraMatrix,cam_mat);
            //枪口坐标系相对位置
            //fit_xyz << 0, 50, 50; //x y z
            break;
        case UAV:
            break;
        default:
            break;
    }
    cv2eigen(cameraMatrix,camMat_E);
    fs.release();
}

void SolveAngle::Generate2DPoints(Rect rect) {
    Point2f pt;
    pt.x = rect.x;
    pt.y = rect.y;
    rectPoint2D.push_back(pt);

    pt.x = rect.x + rect.width;
    pt.y = rect.y;
    rectPoint2D.push_back(pt);

    pt.x = rect.x + rect.width;
    pt.y = rect.y + rect.height;
    rectPoint2D.push_back(pt);

    pt.x = rect.x;
    pt.y = rect.y + rect.height;
    rectPoint2D.push_back(pt);
}

/**
 * @brief 将装甲板中心设为 pnp 解算 4 点模型的解算原点
 * @remark 装甲板的实际大小可能要根据情况修改
 * */
void SolveAngle::Generate3DPoints(bool mode) {
    //because the armor is  incline,so the height of the armor should be smaller than reality.
    if (mode){
        targetHeight3D = 125;
        targetWidth3D = 135;
    }
    else{
        targetHeight3D = 140;
        targetWidth3D = 225;
    }
    targetPoints3D.emplace_back(-targetWidth3D / 2, -targetHeight3D / 2, 0);
    targetPoints3D.emplace_back(targetWidth3D / 2, -targetHeight3D / 2, 0);
    targetPoints3D.emplace_back(targetWidth3D / 2, targetHeight3D / 2, 0);
    targetPoints3D.emplace_back(-targetWidth3D / 2, targetHeight3D / 2, 0);
}

/**
 * @brief 用 pnp 解算目标相对相机坐标系的 yaw pitch
 * @param armor_mode 大装甲板还是小装甲
 * @param pts 装甲板 4 点坐标
 * @param v_ 弹速
 * */
void SolveAngle::GetPoseV(const vector<Point2f>& pts, bool armor_mode, Vector3f gimbal_ypd) {
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    Generate3DPoints(armor_mode);
    rvecs = Mat::zeros(3, 1, CV_64FC1);
    tvecs = Mat::zeros(3, 1, CV_64FC1);

    if (pts.size() != 4) return;

    solvePnP(targetPoints3D,
             pts,
             cameraMatrix,
             distortionCoefficients,
             rvecs,
             tvecs,
             false,
             SOLVEPNP_ITERATIVE);
    cv2eigen(tvecs,p_cam_xyz);
    camXYZ2YPD(); //直接输出目标点 yaw pitch dist
    //GunXYZ2YPD(p_cam_xyz);

//    if(carName == SENTRY)
//        gimbal_ypd += Vector3f {-90,0,0};

    float sin_y = sin(gimbal_ypd[0] * degree2rad);
    float cos_y = cos(gimbal_ypd[0] * degree2rad);
    float sin_p, cos_p;
    sin_p = sin(gimbal_ypd[1] * degree2rad);
    cos_p = cos(gimbal_ypd[1] * degree2rad);

    Ry <<   cos_y , 0     , -sin_y ,
            0     , 1     , 0     ,
            sin_y, 0     , cos_y ;
    Rp <<   1     , 0     , 0     ,
            0     , cos_p , -sin_p,
            0     , sin_p , cos_p ;
    cam2world_mat = Ry * Rp;
    world_xyz = Cam2World(p_cam_xyz);

//    string str[] = {"x","y","z"};
//    vector<float> xyz;
//    for(int i=0; i<3; i++)
//        xyz.push_back(world_xyz[i]);
//    RMTools::showData(xyz,str,"xyz");

    rectPoint2D.clear();
    targetPoints3D.clear();
}

/**
 * @brief 相机坐标系 xyz 变为 yaw pitch dist，存入yaw, pitch
 * */
void SolveAngle::camXYZ2YPD() {
    float x = p_cam_xyz[0], y = p_cam_xyz[1], z = p_cam_xyz[2];
    yaw = atan2(x,z) / degree2rad; //arctan(x/z)
    pitch = -atan2(y, sqrt(x*x + z*z) ) / degree2rad; //arctan(y/sqrt(x^2 + z^2))
    dist = sqrt(x*x + y*y + z*z); //sqrt(x^2 + y^2 + z^2)
    // 静态补偿
    yaw = yaw + yaw_static;
    pitch = pitch + pitch_static;
}

void SolveAngle::GunXYZ2YPD(Vector3f cam_xyz) {
    gun_xyz = cam_xyz - fit_xyz; //枪管坐标系
    float x = gun_xyz[0], y = gun_xyz[1], z = gun_xyz[2];
    yaw = atan2(x,z) / degree2rad;
    pitch = -atan2(y,sqrt(x*x+z*z));
    dist = sqrt(x*x + y*y + z*z);
    yaw = yaw + yaw_static;
    pitch = pitch + pitch_static;
}

void SolveAngle::Compensator(Vector3f cam_xyz, float v) {
    gun_xyz = cam_xyz - fit_xyz; //枪管坐标系
    float dt = (gun_xyz[2]/1000) / v; //z轴与枪口中心重合,dt为子弹飞行时间，单位：s
    float dy = 0.5 * 9.8 * dt * dt * 1000; //将补偿量化为 mm
    gun_xyz[2] = gun_xyz[2] - dy;
    ///动态补偿
    yaw = atan2(gun_xyz[0],gun_xyz[2]) / degree2rad;
    pitch = - atan2(gun_xyz[1], sqrt(gun_xyz[0]*gun_xyz[0] + gun_xyz[2]*gun_xyz[2])) / degree2rad;
    dist = sqrt(pow(gun_xyz[0],2) + pow(gun_xyz[1],2) + pow(gun_xyz[2],2));
    ///静态补偿
    yaw = yaw + yaw_static;
    pitch = pitch + pitch_static;
}

float SolveAngle::pitchCompensate(Vector3f target_xyz,float v) {
    float dt = dist/1000 / v;
    float dy = 0.5 * 9.8 * dt*dt * 1000;
    return atan(dy/dist) / degree2rad;
}

/**
 * @brief 计算pitch的补偿角
 * @param target_xyz 目标xyz坐标
 * @param target_ypd 目标yaw pitch distance
 * @param v 子弹速度
 * @return pitch相对于地面的角度
 */
float SolveAngle::CalPitch(Vector3f target_xyz, float v, float &t) const {
    float x = target_xyz[0]/1000, y = target_xyz[1]/1000, z = target_xyz[2]/1000; //转换为m
    float d = sqrt(x*x+z*z);
    float a = 0.25 * g2;
    float b = -(g*y + v*v);
    float c = y*y + d*d;
    float x_2 = (-b-sqrt(b*b-4*a*c)) /2/a;
    t = sqrt(x_2);
    float s_the = (0.5*g*t*t - y) / (v*t);
    float theta = asin(s_the) / degree2rad;
    string str[] = {"x-z","y","t","theta"};
    vector<float> data(4);
    data = {d,y,t,theta};
    //RMTools::showData(data,str,"CalPitch");
//    if(y < -0.4){
//        theta += 6 * -y;
//    }
    return theta;
}

/**
 * @brief 将预测点反投影到图像上
 * @param src 图片
 * @param target_xyz 目标的陀螺仪绝对坐标
 * @param gimbal_ypd 旋转矩阵
 * */
void SolveAngle::backProject2D(Mat &src, const Vector3f target_xyz) {
    Vector3f temp_xyz, cam_xyz, pix_uv1;

    cam_xyz = World2Cam(target_xyz);
    pix_uv1 = Cam2Pixel(cam_xyz);
    circle(src, Point2f(pix_uv1[0],pix_uv1[1]), 5, Scalar(100, 240, 15), 3);
}
Point2f SolveAngle::getBackProject2DPoint(Vector3f target_xyz) {
    Vector3f cam_xyz, pix_uv1;
    cam_xyz = World2Cam(target_xyz);
    pix_uv1 = Cam2Pixel(cam_xyz);
    return Point2f (pix_uv1[0],pix_uv1[1]);
}
Vector3f SolveAngle::Cam2World(Vector3f cam_xyz) {
    Vector3f world_xyz;
    world_xyz = cam2world_mat * cam_xyz;
    return world_xyz;
}
Vector3f SolveAngle::World2Cam(Vector3f world_xyz) {
    Vector3f cam_xyz;
    cam_xyz = cam2world_mat.inverse() * world_xyz;
    return cam_xyz;
}
Vector3f SolveAngle::Cam2Pixel(Vector3f cam_xyz) {
    Vector3f pix_uv1;
    pix_uv1 = cam_mat/cam_xyz[2] * cam_xyz;
    return pix_uv1;
}