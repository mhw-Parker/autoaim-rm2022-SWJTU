//
// Created by sentrydown on 22-6-8.
//

#include "ImageWriter.hpp"

bool ImageDriver::InitCam()
{
    return true;
}
bool ImageDriver::StartGrab()
{
    return true;
}
bool ImageDriver::SetCam()
{
    return true;
}
bool ImageDriver::Grab(Mat& src)
{
    src = imread(srcPath);
    if(src.empty()) printf("Image grab failed!");
    return !src.empty();
}

bool ImageDriver::StopGrab()
{

}
