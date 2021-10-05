//
// Created by tyy on 2021/9/28.
//
#include <MindVision/include/MindVision.h>

#include "MindVision.h"

bool MindVision::InitCam()
{
    CameraSdkInit(1);
    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);

    printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0){
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

    printf("state = %d\n", iStatus);
    if(iStatus!=CAMERA_STATUS_SUCCESS){     //初始化失败
        cout << "CamInit failed !" << endl;
        return 0;
    }else{
        //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(hCamera,&tCapability);

        g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

        //设置相机的相关参数
        SetCam();

        if(tCapability.sIspCapacity.bMonoSensor){
            channel=1;
            CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
        }else{
            channel=3;
            CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
        }
        return 1;
    }
}

bool MindVision::Grab(Mat& src)
{
    //CameraGetFrameSpeed(hCamera,&fps);
    if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS){

        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);

        cv::Mat matImage(
                cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer
        );
        src = matImage.clone();
        CameraReleaseImageBuffer(hCamera,pbyBuffer);
        return true;
    }else
        return false;

}

bool MindVision::SetCam()
{
    CameraPause(hCamera);
    CameraSetAeState(hCamera, false); //设置为手动曝光
    CameraSetExposureTime(hCamera,4000);
    CameraSetAnalogGain(hCamera,16.4);
    /*让SDK进入工作模式，开始接收来自相机发送的图像
        数据。如果当前相机是触发模式，则需要接收到
        触发帧以后才会更新图像。    */
    CameraPlay(hCamera);
    return true;
}

bool MindVision::StartGrab(){
    return true;
}

bool MindVision::StopGrab() {
    return false;
}
