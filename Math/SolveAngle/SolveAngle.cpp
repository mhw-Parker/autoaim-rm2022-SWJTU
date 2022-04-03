#include "SolveAngle.hpp"
#include "Kalman.hpp"
#include "preoptions.h"

#define SHOOTFORMULAX(X) (60 - (abs(X - FRAMEWIDTH/2)))
#define SHOOTFORMULAY(Y) (40 - (abs(Y - FRAMEHEIGHT/2)))
#define SHOOTFORMULA(X,Y) (3600 - ((Y - FRAMEHEIGHT/2)*(Y - FRAMEHEIGHT/2) + (X - FRAMEWIDTH/2)*(X - FRAMEWIDTH/2)))

using namespace cv;

SolveAngle::SolveAngle()
{
    string filename ="../Math/SolveAngle/camera.xml";
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened()) {
        cout << "no such file" << endl;
        return;
    }
    switch(carName)
    {
        case HERO:
            fs["Distortion_Coefficients4_Realsense"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_Realsense"] >> cameraMatrix;
            break;
        case INFANTRY_MELEE0:
            fs["Distortion_Coefficients5_MIND"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND"] >> cameraMatrix;
            cv2eigen(cameraMatrix,cam_mat);
            //静态校正值
            yaw_static = 1.6;
            pitch_static = 0.5;
            //枪口坐标系相对位置
            fit_xyz << 0, 50, 50; //x y z
            break;
        case INFANTRY_MELEE1:
            fs["Distortion_Coefficients5_MIND"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND"] >> cameraMatrix;
            cv2eigen(cameraMatrix,cam_mat);
            //静态校正值
            yaw_static = 1.5;
            pitch_static = -0.6;
            //枪口坐标系相对位置
            fit_xyz << 30, 50, 50;
            break;
        case INFANTRY_TRACK:
            break;
        case VIDEO:
            fs["Distortion_Coefficients5_MIND"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND"] >> cameraMatrix;
            cv2eigen(cameraMatrix,cam_mat);
            yaw_static = 1.6;
            pitch_static = 0.5;
        case SENTRY:
            fs["Distortion_Coefficients4_MIND"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND"] >> cameraMatrix;
            cv2eigen(cameraMatrix,cam_mat);
            break;
        case UAV:
            break;
        case NOTDEFINED:
            break;
    }
    cv2eigen(cameraMatrix,camMat_E);
    fs.release();
}

void SolveAngle::Generate2DPoints(Rect rect)
{
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
 * @brief 用 pnp 解算目标相对相机坐标系的 yaw pitch
 * @param armor_mode 大装甲板还是小装甲
 * @param pts 装甲板 4 点坐标
 * @param v_ 弹速
 * */
void SolveAngle::GetPoseV(const vector<Point2f>& pts, bool armor_mode, const float v_)
{
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    Generate3DPoints(armor_mode);
    rvecs = Mat::zeros(3, 1, CV_64FC1);
    tvecs = Mat::zeros(3, 1, CV_64FC1);

    if(pts.size() != 4)return;

    solvePnP(targetPoints3D,
             pts,
             cameraMatrix,
             distortionCoefficients,
             rvecs,
             tvecs,
             false,
             SOLVEPNP_ITERATIVE);

    cv2eigen(tvecs,p_cam_xyz);

    //Compensator(p_cam_xyz,v_);
    camXYZ2YPD(tvecs); //直接输出目标点 yaw pitch dist

    if(fabs(yaw)>1)
        yaw = yaw + (-0.05626 * yaw + 0.204);


    averageX = 0;
    averageY = 0;

    for(auto &p:pts)
    {
        averageX += 1.0/4*p.x;
        averageY += 1.0/4*p.y;
    }

/**********************************************************************************************************************
 *                                            Shoot Logic                                                             *
 *********************************************************************************************************************/
    shootPriority += SHOOTFORMULA(averageX,averageY);
    shootPriority = (shootPriority<0)?(0):((shootPriority > 1000)?(1000):(shootPriority));
    shoot = (shootPriority > 0);

    rectPoint2D.clear();
    targetPoints3D.clear();
}


/**
 * @brief 相机坐标系 xyz 变为 yaw pitch dist
 * */
void SolveAngle::camXYZ2YPD(Mat tvecs)
{
    yaw = atan2(p_cam_xyz[0],p_cam_xyz[2]) / degree2rad ; //arctan(x/z)
    pitch = -atan2(p_cam_xyz[1], sqrt(p_cam_xyz[0]*p_cam_xyz[0] + p_cam_xyz[2]*p_cam_xyz[2]) ) / degree2rad; //arctan(y/sqrt(x^2 + z^2))
    dist = sqrt(p_cam_xyz[0]*p_cam_xyz[0] + p_cam_xyz[1]*p_cam_xyz[1] + p_cam_xyz[2]*p_cam_xyz[2]); //sqrt(x^2 + y^2 + z^2)
    ///静态补偿
    yaw = yaw + yaw_static;
    pitch = pitch + pitch_static;
    ypd << yaw, pitch, dist;
}

void SolveAngle::Compensator(Vector3f cam_xyz, float v)
{
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

    ypd << yaw, pitch, dist;
}

float SolveAngle::pitchCompensate(Vector3f target_xyz, float v) {
    float dist = sqrt(target_xyz[0]*target_xyz[0] + target_xyz[1]*target_xyz[1] + target_xyz[2]*target_xyz[2]);
    float d_2 = dist * dist;
    float dt = dist/1000 / v;
    float dy = 0.5 * 9.8 * dt*dt * 1000;
    return atan(dy/dist) / degree2rad;
}

/**
 * @brief 将装甲板中心设为 pnp 解算 4 点模型的解算原点
 * @remark 装甲板的实际大小可能要根据情况修改
 * */
void SolveAngle::Generate3DPoints(bool mode)
{
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
 * @brief 将预测点反投影到图像上
 * @param src 预测点的（ yaw, pitch, dist ）坐标
 * @param target_xyz 目标的陀螺仪绝对坐标
 * @param gimbal_ypd 旋转矩阵
 * */
void SolveAngle::backProject2D(Mat &src, const Vector3f target_xyz, Vector3f gimbal_ypd) {
    Vector3f temp_xyz, cam_xyz, pix_uv1;
    Vector3f rotate_ypd;
    rotate_ypd << gimbal_ypd[0],
            gimbal_ypd[1],
            gimbal_ypd[2];
    temp_xyz = target_xyz;

    //cout << target_xyz << endl;
    //cout << gim_d_ypd << endl;
    Matrix3f vec_degree2rad;
    vec_degree2rad << degree2rad, 0, 0,
            0, degree2rad, 0,
            0,     0,      1;
    rotate_ypd = vec_degree2rad * rotate_ypd;

    float sin_y = sin(rotate_ypd[0]);
    float cos_y = cos(rotate_ypd[0]);
    float sin_p = sin(rotate_ypd[1]);
    float cos_p = cos(rotate_ypd[1]);

    Matrix3f Ry, Rp;
    Ry <<   cos_y , 0     , sin_y ,
            0     , 1     , 0     ,
            -sin_y, 0     , cos_y ;

    Rp <<   1     , 0     , 0     ,
            0     , cos_p , -sin_p,
            0     , sin_p , cos_p ;

    cam_xyz = Ry * Rp * target_xyz;
    pix_uv1 = cam_mat * cam_xyz / cam_xyz[2];
    //cout << "--- target location :" << endl << target_xyz << endl;
    //cout << "--- camera location :" << endl << cam_xyz << endl;
    //cout << "--- pixel location : " << endl << pix_uv1 << endl;
    circle(src, Point2f(pix_uv1[0],pix_uv1[1]), 2, Scalar(100, 240, 15), 3);
}