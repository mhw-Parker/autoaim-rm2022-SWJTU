//
// Created by tyy on 2021/9/28.
//
#include "MindDriver.h"
#include "utility.hpp"

bool MindDriver::InitCam() {
    CameraSdkInit(1);
    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);

    printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if (iCameraCounts == 0) {
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS) {     //初始化失败
        cout << "CamInit failed !" << endl;
        return false;
    } else {
        //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(hCamera, &tCapability);
        // 数据缓存区
        g_pRgbBuffer = (unsigned char *) malloc(
                tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
        //设置相机的相关参数
        SetCam();
        // 触发方式

        // 设置为彩色
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);

        return true;
    }
}

bool MindDriver::Grab(Mat &src) {
    //CameraGetFrameSpeed(hCamera,&fps);

    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {

        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
        //cout << "get camera buffer: " << RMTools::CalWasteTime(st_,getTickFrequency()) << endl;
        /// it takes almost 99.7% of the whole produce time !
        src = cv::Mat(
                cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer
        );
        //cout << "convert to cv mat: " << RMTools::CalWasteTime(st_,getTickFrequency()) << endl;
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
        return true;
    } else
        return false;

}

bool MindDriver::SetCam() {
    CameraSetAeState(hCamera, false); //设置为手动曝光

    // 调整RGB三个通道增益
    int r_gain, g_gain, b_gain;
    CameraGetGain(hCamera, &r_gain, &g_gain, &b_gain);
    CameraSetGain(hCamera, r_gain + 40, g_gain + 20, b_gain);

    if (carName == SENTRYTOP) { // 134
        CameraSetExposureTime(hCamera, 1250); //设置曝光时间
        CameraSetAnalogGainX(hCamera, 3.5); //设置增益系数
    } else { // 133
        CameraSetExposureTime(hCamera, 1000); //设置曝光时间
        CameraSetAnalogGainX(hCamera, 2.5); //设置增益系数
    }

    /* 让SDK进入工作模式，开始接收来自相机发送的图像
        数据。如果当前相机是触发模式，则需要接收到
        触发帧以后才会更新图像。    */
    CameraPlay(hCamera);
    return true;
}

bool MindDriver::StartGrab() {
    CameraPlay(hCamera);
    return true;
}

bool MindDriver::StopGrab() {
    CameraPause(hCamera);
    CameraReleaseImageBuffer(hCamera, pbyBuffer);
    return false;
}
