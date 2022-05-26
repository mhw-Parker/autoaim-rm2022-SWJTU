//
// Created by root on 2020/12/17.
//

#include "mydefine.h"

bool showArmorBox = false;
bool showOrigin = false;
bool showLamps = false;
bool showEnergy = false;
bool showBinaryImg = false;
bool blueTarget = false;
bool saveVideo = false;
bool saveSVM = false;
bool debug = false;

int FRAMEWIDTH;
int FRAMEHEIGHT;
int detectLostCnt = 0;

CARNAME carName = VIDEO;
int cameraIndex = 0;
std::string videoPath = "../blue_car.avi";

float feedbackDelta  = 1.32;

std::vector<float> time_stamp; // 全局时间戳