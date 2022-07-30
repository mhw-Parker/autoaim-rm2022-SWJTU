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
bool DEBUG = false;
int max_lost = showArmorBox ? 3 : 5;

int FRAMEWIDTH;
int FRAMEHEIGHT;

CARNAME carName = VIDEO;
int cameraIndex = 0;

std::string srcPath = "../Output/blue_car.avi";
std::string timePath = "../Output/blue_car.txt";

float feedbackDelta  = 1.32;

std::vector<float> time_stamp; // 全局时间戳