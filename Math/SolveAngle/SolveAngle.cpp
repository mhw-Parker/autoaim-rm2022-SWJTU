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
        case INFANTRY_MELEE:
            fs["Distortion_Coefficients5_MIND"] >> distortionCoefficients;
            fs["Intrinsic_Matrix_MIND"] >> cameraMatrix;
            yaw_static = 1.6 - 0.3;
            pitch_static = 0.5 + 1.6;//-1.2
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
 * */
void SolveAngle::GetPoseV(const vector<Point2f>& pts, bool armor_mode)
{
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    Generate3DPoints(armor_mode);
    rvecs = Mat::zeros(3, 1, CV_64FC1);
    tvecs = Mat::zeros(3, 1, CV_64FC1);

    if(pts.size() != 4)return;

    solvePnP(targetPoints3D, pts, cameraMatrix, distortionCoefficients, rvecs, tvecs,false, SOLVEPNP_ITERATIVE);
    rvecs.convertTo(Rvec, CV_32F);    //旋转向量
    tvecs.convertTo(Tvec, CV_32F);   //平移向量

    camXYZ2YPD(tvecs);
    cout << "yaw = " << yaw << '\t' << "pitch = " << pitch <<endl;

    yaw = yaw + yaw_static;
    pitch = pitch + pitch_static;
//    yaw = atan(tvecs.at<double>(0, 0) / tvecs.at<double>(2, 0)) / 2 / CV_PI * 360;
//    pitch = -1.0*atan(tvecs.at<double>(1, 0) / tvecs.at<double>(2, 0)) / 2 / CV_PI * 360;
//    dist = sqrt(tvecs.at<double>(0, 0)*tvecs.at<double>(0, 0) + tvecs.at<double>(1, 0)*tvecs.at<double>(1, 0) + tvecs.at<double>(2, 0)* tvecs.at<double>(2, 0));

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

void SolveAngle::GetPoseSH(const Point2f p)
{
    float deltaX = p.x - IMAGEWIDTH/2;
    float deltaY = -(p.y - IMAGEHEIGHT/2);
    yaw = atan(deltaX / camMat_E(0,2)) * 180 / CV_PI ;
    pitch = atan(deltaY / camMat_E(1,2)) * 180 / CV_PI ;
}

/**
 * @brief 相机坐标系xyz变为 yaw pitch dist
 * */
void SolveAngle::camXYZ2YPD(Mat tvecs)
{
    cv2eigen(tvecs,p_cam_xyz);
    //cout << " z: " << p_cam_xyz[2] << endl;
    yaw = atan2(p_cam_xyz[0],p_cam_xyz[2]) / (2*CV_PI) * 360 ; //arctan(x/z)
    pitch = -atan2(p_cam_xyz[1], sqrt(p_cam_xyz[0]*p_cam_xyz[0] + p_cam_xyz[2]*p_cam_xyz[2]) ) / (2*CV_PI) * 360; //arctan(y/sqrt(x^2 + z^2))
    dist = sqrt(p_cam_xyz[0]*p_cam_xyz[0] + p_cam_xyz[1]*p_cam_xyz[1] + p_cam_xyz[2]*p_cam_xyz[2]); //sqrt(x^2 + y^2 + z^2)
}
/**
 * @brief 弹道补偿函数
 * @param pitch 当前的 pitch 差角
 * @param dist 目标点距离
 * @param fitY 装配导致的枪口摄像头高度差 在枪口上为正 枪口下为负 单位为：mm
 * */
void SolveAngle::Compensator(float dist, float pitch, float fitY)
{
    float x = p_cam_xyz[0];
    float y = p_cam_xyz[1] - fitY;
    float z = p_cam_xyz[2];
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
void SolveAngle::backProjection(Mat tvecs, Mat rvecs, Vector3d obj_p_ypd, vector<Point2f> &img_p) {
    vector<Point3f> obj_p_xyz;
    float a = tan(obj_p_ypd[0] / 360 * 2*CV_PI); //tan(yaw)，转为弧度值
    float b = tan(obj_p_ypd[1] / 360 * 2*CV_PI); //tan(pitch)
    float d = pow(obj_p_ypd[2],2);        //dist^2
    obj_p_xyz[0].x = sqrt((a*a*d*d)/(a*a+1)/(b*b+1) ) * (obj_p_ypd[0]<0?-1:1); //若yaw为负值，x也为负值
    obj_p_xyz[0].y = sqrt(d/(1+1/(b*b)) ) * (obj_p_ypd[1]<0?1:-1); //若pitch为负值，由于相机坐标系下为正，所以此时y为正值
    obj_p_xyz[0].z = sqrt(d*d/(a*a+1)/(b*b+1) );
    projectPoints(obj_p_xyz, rvecs, tvecs, cameraMatrix, distortionCoefficients, img_p);
}

