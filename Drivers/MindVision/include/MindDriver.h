//
// Created by tyy on 2021/9/28.
//

#ifndef MINDVISION_MINDVISION_H
#define MINDVISION_MINDVISION_H

#include "CameraApi.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <iostream>
#include "Driver.h"


using namespace std;
using namespace cv;

class MindDriver : public Driver
{
public:
    bool InitCam() override;
    bool SetCam() override;
    bool StartGrab() override;
    bool Grab(Mat& src) override;
    bool StopGrab() override;

    void Record();

private:
    int                     hCamera;
    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    int                     fps;
    tSdkCameraDevInfo       tCameraEnumList;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    int                     iDisplayFrames = 10000;
    IplImage *iplImage = NULL;
    double*         pfLineTime;
    int                     channel = 3;
    unsigned char           * g_pRgbBuffer;     //处理后数据缓存区
};

#endif //MINDVISION_MINDVISION_H
