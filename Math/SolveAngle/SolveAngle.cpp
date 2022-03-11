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
            direct = 1;
            //静态校正值
            yaw_static = 1.6;
            pitch_static = 0.5;
            //枪口坐标系
            fit_xyz << 0, 50, 50; //x y z
            break;
        case INFANTRY_MELEE1:
            fs["Distortion_Coefficients5_MIND"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND"] >> cameraMatrix;
            direct = -1;
            //静态校正值
            yaw_static = 1.5;
            pitch_static = -0.6;
            //枪口坐标系
            fit_xyz << 30, 50, 50;
            break;
        case INFANTRY_TRACK:
            break;
        case VIDEO:
            fs["Distortion_Coefficients5_MIND"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND"] >> cameraMatrix;
        case SENTRY:
            fs["Distortion_Coefficients4_MIND"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND"] >> cameraMatrix;
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

    rvecs.convertTo(Rvec, CV_32F);    //旋转向量
    tvecs.convertTo(Tvec, CV_32F);   //平移向量

    cv2eigen(tvecs,p_cam_xyz);

    Compensator(p_cam_xyz,v_);
    //camXYZ2YPD(tvecs); //直接输出目标点 yaw pitch dist
    if(fabs(yaw)>1)
        yaw = yaw + (-0.05626 * yaw + 0.204);
    //cout << "yaw = " << yaw << '\t' << "pitch = " << pitch <<endl;

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
 * @brief 小孔成像模型
 * */
void SolveAngle::GetPoseSH(const Point2f p)
{
    float deltaX = p.x - IMAGEWIDTH/2;
    float deltaY = -(p.y - IMAGEHEIGHT/2);
    yaw = atan(deltaX / camMat_E(0,2)) * 180 / CV_PI ;
    pitch = atan(deltaY / camMat_E(1,2)) * 180 / CV_PI ;
}

/**
 * @brief 相机坐标系 xyz 变为 yaw pitch dist
 * */
void SolveAngle::camXYZ2YPD(Mat tvecs)
{
    yaw = atan2(p_cam_xyz[0],p_cam_xyz[2]) / (2*CV_PI) * 360 ; //arctan(x/z)
    pitch = -atan2(p_cam_xyz[1], sqrt(p_cam_xyz[0]*p_cam_xyz[0] + p_cam_xyz[2]*p_cam_xyz[2]) ) / (2*CV_PI) * 360 * direct; //arctan(y/sqrt(x^2 + z^2))
    dist = sqrt(p_cam_xyz[0]*p_cam_xyz[0] + p_cam_xyz[1]*p_cam_xyz[1] + p_cam_xyz[2]*p_cam_xyz[2]); //sqrt(x^2 + y^2 + z^2)
    ///静态补偿
    yaw = yaw + yaw_static;
    pitch = pitch + pitch_static;
}
/**
 * @brief 弹道补偿函数
 * @param fitXYZ 相机相对枪口 左上前为正 单位为：mm
 * @param v 当前弹速
 * */
void SolveAngle::Compensator(Vector3f cam_xyz, float v)
{
    gun_xyz = cam_xyz - fit_xyz; //枪管坐标系
    float dt = (gun_xyz[2]/1000) / v; //z轴与枪口中心重合,dt为子弹飞行时间，单位：s
    float dy = 0.5 * 9.8 * dt * dt * 1000; //将补偿量化为 mm
    gun_xyz[2] = gun_xyz[2] - dy;
    ///动态补偿
    yaw = atan2(gun_xyz[0],gun_xyz[2]) / (2*CV_PI) * 360;
    pitch = - atan2(gun_xyz[1], sqrt(gun_xyz[0]*gun_xyz[0] + gun_xyz[2]*gun_xyz[2])) / (2*CV_PI) * 360 * direct;
    dist = sqrt(pow(gun_xyz[0],2) + pow(gun_xyz[1],2) + pow(gun_xyz[2],2));
    ///静态补偿
    yaw = yaw + yaw_static;
    pitch = pitch + pitch_static;

    ypd << yaw, pitch, dist;
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
 * @param obj_p_ypd 预测点的（ yaw, pitch, dist ）坐标
 * @param tvecs 平移矩阵
 * @param rvecs 旋转矩阵
 * @param img_p 反投影图像上的坐标
 * */
void SolveAngle::backProjection(Vector3f obj_p_ypd, vector<Point2f> &img_p) {
    vector<Point3f> obj_p_xyz;
    float a = tan(obj_p_ypd[0] / 360 * 2*CV_PI); //tan(yaw)，转为弧度值
    float b = tan(obj_p_ypd[1] / 360 * 2*CV_PI); //tan(pitch)
    float d = pow(obj_p_ypd[2],2);        //dist^2
    obj_p_xyz[0].x = sqrt((a*a*d*d)/(a*a+1)/(b*b+1) ) * (obj_p_ypd[0]<0?-1:1); //若yaw为负值，x也为负值
    obj_p_xyz[0].y = sqrt(d/(1+1/(b*b)) ) * (obj_p_ypd[1]<0?1:-1); //若pitch为负值，由于相机坐标系下为正，所以此时y为正值
    obj_p_xyz[0].z = sqrt(d*d/(a*a+1)/(b*b+1) );
    projectPoints(obj_p_xyz,
                  rvecs,
                  tvecs,
                  cameraMatrix,
                  distortionCoefficients,
                  img_p);
}

void SolveAngle::backProject(Point3f obj_p_xyz, Point2f &p) {
    vector<Point3f> obj_p_xyz_vec;
    obj_p_xyz_vec.push_back(obj_p_xyz);

    vector<Point2f> img_p;
    projectPoints(obj_p_xyz_vec,
                  rvecs,
                  tvecs,
                  cameraMatrix,
                  distortionCoefficients,
                  img_p);
    p = img_p.back();
}
