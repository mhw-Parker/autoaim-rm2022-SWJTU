//
// Created by sentrydown on 22-6-8.
//

#ifndef MASTER_IMAGEWRITER_H
#define MASTER_IMAGEWRITER_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "Driver.h"
#include "mydefine.h"

using namespace cv;

class ImageDriver: public Driver
{
public:
    bool InitCam() override;
    bool StartGrab() override;
    bool SetCam() override;
    bool Grab(Mat& src) override;
    bool StopGrab() override;
};

#endif //MASTER_IMAGEWRITER_H
