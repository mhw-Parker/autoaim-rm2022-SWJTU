//
// Created by luojunhui on 2/15/20.
//

#ifndef ROBOMASTER_MYDEFINE_H
#define ROBOMASTER_MYDEFINE_H

#include <string>
#include <vector>

#ifndef AUTO_SHOOT_STATE
#define  AUTO_SHOOT_STATE 0
#endif

#ifndef SMALL_ENERGY_STATE
#define  SMALL_ENERGY_STATE 1
#endif

#ifndef BIG_ENERGY_STATE
#define  BIG_ENERGY_STATE 2
#endif

#ifndef FAR_DISTANCE_SHOOT
#define  FAR_DISTANCE_SHOOT 1
#endif

#ifndef MODEL_MODE
#define   MODEL_MODE 3
#endif

#ifndef TRADITION_MODE
#define  TRADITION_MODE 4
#endif

#ifndef BIG_ARMOR
#define  BIG_ARMOR 1
#endif

#ifndef SMALL_ARMOR
#define  SMALL_ARMOR 2
#endif

#ifndef ENERGY_ARMOR
#define  ENERGY_ARMOR 3
#endif

#ifndef SMALL
#define SMALL 4
#endif

#ifndef LARGE
#define LARGE 5
#endif

#ifndef FIND_ARMOR_YES
#define  FIND_ARMOR_YES 7
#endif

#ifndef FIND_ARMOR_NO
#define  FIND_ARMOR_NO 8
#endif

#ifndef BLUE_ENEMY
#define  BLUE_ENEMY 9
#endif

#ifndef USEROI
#define  USEROI 1
#endif

#ifndef NUM_RECOGNIZE
#define  NUM_RECOGNIZE 1
#endif

/*IMAGEHEIGHT AND IMAGEWIDTH just for initialize the camera, maybe not the real frame format*/
#ifndef IMAGEWIDTH
#define  IMAGEWIDTH 1280
#endif

#ifndef IMAGEHEIGHT
#define  IMAGEHEIGHT 1024
#endif

#ifndef CARNAME_
#define CARNAME_
enum CARNAME {HERO, INFANTRY3, INFANTRY4, INFANTRY5, INFANTRY_TRACK, SENTRYTOP, SENTRYDOWN, UAV, VIDEO, IMAGE, NOTDEFINED};
#endif

#ifndef SHOWTIME
#define SHOWTIME 0
#endif

#ifndef DEBUG_MSG
#define  DEBUG_MSG 0
#endif

#ifndef SAVE_VIDEO
#define  SAVE_VIDEO 0
#endif

#ifndef SAVE_LOG
#define  SAVE_LOG 0
#endif

#ifndef SAVE_TEST_DATA
#define SAVE_TEST_DATA 0
#endif

#ifndef GPUMODE
#define  GPUMODE 0
#endif

#ifndef SVM_PARAM_PATH
#define  SVM_PARAM_PATH "../Detector/resource/svm.xml"
#endif

#ifndef SVM_IMAGE_SIZE
#define  SVM_IMAGE_SIZE 40
#endif

#ifndef NUM_IMG_SIZE
#define  NUM_IMG_SIZE 20
#endif

#include "config.h"

#ifndef OUTPUT_PATH
#define OUTPUT_PATH  "../Output/"
#endif

#ifndef SAVE_SVM_PIC
#define SAVE_SVM_PIC "../Output/SVM/"
#endif

extern bool showArmorBox;
extern bool showOrigin;
extern bool showLamps;
extern bool showBinaryImg;
extern bool showEnergy;
extern bool blueTarget;
extern bool saveVideo;
extern bool saveSVM;
extern bool DEBUG;
extern int FRAMEWIDTH;
extern int FRAMEHEIGHT;
// ??????????????????
extern int max_lost;
extern CARNAME carName;

extern std::string srcPath;
extern std::string timePath;
extern int cameraIndex;
extern float feedbackDelta;

#endif //ROBOMASTER_MYDEFINE_H
