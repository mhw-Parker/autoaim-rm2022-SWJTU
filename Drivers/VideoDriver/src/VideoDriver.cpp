//
// Created by root on 2021/3/6.
//
#include "VideoDriver.hpp"

bool VideoDriver::InitCam()
{
    capture.open(videoPath);
    if(!capture.isOpened())
    {
        perror("Video Open Failed!\n");
        return false;
    }else{
        LOGM("Video Open Successfully!\n");
    }
    return true;
}
bool VideoDriver::StartGrab()
{
    return true;
}
bool VideoDriver::SetCam()
{
    return true;
}
bool VideoDriver::Grab(Mat& src)
{
    //printf("Video Capture\n");
    capture.read(src);
    if(src.empty()) printf("grab failed !");
//    capture.set(CAP_PROP_FRAME_WIDTH, 1024);			//设置相机采样宽度
//    capture.set(CAP_PROP_FRAME_HEIGHT, 820);		//设置相机采样高度
    waitKey(3); //加入 delay 模拟相机读图时间
    return !src.empty();
}

bool VideoDriver::StopGrab()
{

}
