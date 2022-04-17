//
// Created by luojunhui on 1/28/20.
//

#include <csignal>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>
#include <iostream>
#include <memory>
#include "MyThread.hpp"

using namespace std;
using namespace cv;

extern pthread_t producePThreadHandler;
extern pthread_t detectPThreadHandler;
extern pthread_t energyPThreadHandler;
extern pthread_t feedbackPThreadHandler;

string now_time = getSysTime();

#if SAVE_VIDEO == 1
string path = ( string(root_path + now_time).append(".avi"));
VideoWriter videowriter(path, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 50.0, cv::Size(1280, 1024));
#endif

#if SAVE_LOG == 1
std::ofstream logWrite("../Output/log.txt",ios::out);
std::ofstream fileClean("../Output/log.txt",ios::trunc);
#endif

#if SAVE_TEST_DATA == 1
string data_path = ( string(OUTPUT_PATH + now_time + string("_data")).append(".txt") );
ofstream fout(data_path);
ofstream dataWrite(data_path,ios::out);
#endif

namespace rm
{
    bool ImgProdCons::quitFlag = false;// quit flag, all threads would jump out from loop when quitFlag is true

    bool receiveMission = true;//when receive mission completed, receiveMission is true
    bool produceMission = false;//when produce mission completed, produceMission is true, when feedback mission completed, produceMission is false
    bool detectMission = false;//when detect mission completed, detectMission is true, when produce mission completed, detectMission is false
    bool feedbackMission = false;//when feedback mission completed, feedbackMission is true, when produce mission completed, feedbackMission is false

    int8_t curControlState = AUTO_SHOOT_STATE; //current control mode
    //int8_t curControlState = BIG_ENERGY_STATE;
    int8_t lastControlState = BIG_ENERGY_STATE;
    uint8_t curDetectMode = TRADITION_MODE; //tracking or searching

    int direction = 0;
    bool directionChangeFlag = false;
    float yawOffset = 0;
    float avg_yaw;
#ifdef REALSENSE
    RealSenseDriver intelCapture;// Intel D435 Camera Driver, when it is declared in class ImgProdCons, there are a segment fault when program runs
#endif
    bool pauseFlag = false;

#if DEBUG == 1
    int predictX,originalX;
    Mat WaveBackground = Mat(480,640,CV_8UC3,Scalar(0,0,0));
    auto *waveWindowPanel =new DisPlayWaveCLASS(WaveBackground, &originalX,&predictX);



    Mat debugWindowCanvas = Mat(300,500,CV_8UC1,Scalar(0));
#endif

#if DEBUG == 1 || SAVE_LOG == 1
    double timeFlag, taskTime;
    double frequency;
#endif

    float yawTran = 0;
    float pitchTran = 0;

#define YAW_LIST_LEN 20
    double yawList[YAW_LIST_LEN] = {0};
    double yawDeviation = 0;
    int yawListCount = 0;

    /***record distance for debug**/
    int dis_count = 0;
    float dis_sum = 0;

    void ImgProdCons::SignalHandler(int)
    {
        //LOGE("Process Shut Down By SIGINT\n");
        ImgProdCons::quitFlag = true;

#if SAVE_LOG == 1
        logWrite.close();
#endif
        if(pthread_kill(producePThreadHandler,0) == ESRCH)
        {
            //LOGW("Child Thread Produce Thread Close Failed\n");
        }

        if(pthread_kill(detectPThreadHandler,0) == ESRCH)
        {
            //LOGW("Child Thread Detect Thread Close Failed\n");
        }

        if(pthread_kill(energyPThreadHandler,0) == ESRCH)
        {
            //LOGW("Child Thread EnergyDetector Thread Close Failed\n");
        }

        if(pthread_kill(feedbackPThreadHandler,0) == ESRCH)
        {
            //LOGW("Child Thread Feedback Thread Close Failed\n");
        }

        exit(-1);
    }



    void ImgProdCons::InitSignals()
    {
        ImgProdCons::quitFlag = false;

        struct sigaction sigact{};
        sigact.sa_handler = SignalHandler;
        sigemptyset(&sigact.sa_mask);
        sigact.sa_flags = 0;
        /*if interrupt occurs,set _quit_flag as true*/
        sigaction(SIGINT, &sigact, (struct sigaction *)nullptr);
    }


    ImgProdCons::ImgProdCons():
            serialPtr(unique_ptr<Serial>(new Serial())),
            solverPtr(unique_ptr<SolveAngle>(new SolveAngle())),
            armorDetectorPtr(unique_ptr<ArmorDetector>(new ArmorDetector())),
            kalman(unique_ptr<Kalman>(new Kalman())),
            energyPtr(unique_ptr<EnergyDetector>(new EnergyDetector())),
            predictPtr(unique_ptr<Predictor>(new Predictor())),
            armorType(BIG_ARMOR),
            driver(),
            missCount(0),
            showWave(100,300,500)
    {
//#if SAVE_LOG == 1
//        logWrite<<"Find    "<<"TARGET X    "<<"TARGET Y    "<<"TARGET HEIGHT    "<<"TARGET WIDTH    "<<"YAW    "<<"PITCH    "\
//    <<"SHOOT    "<<endl;
//#endif
    }

    bool ImgProdCons::Init()
    {
        /*initialize signal*/
        InitSignals();
        freq = getTickFrequency();
        whole_time_arr.resize(3);
        /*initialize camera*/
#if SAVE_TEST_DATA == 1
        fout.close();
        dataWrite.close();
#endif
        switch (carName) {
            case HERO:
#ifdef MIND
                driver = &mindCapture;
#endif
                break;
            case INFANTRY_MELEE0:
#ifdef MIND
                driver = &mindCapture;
#endif
                break;
            case INFANTRY_MELEE1:
#ifdef MIND
                driver = &mindCapture;
#endif
                break;
            case INFANTRY_TRACK:
                driver = &v4l2Capture;
                break;
            case SENTRY:
#ifdef MIND
                driver = &mindCapture;
#endif
#ifdef DAHUA
                driver = &dahuaCapture;
#endif
                break;
            case UAV:
                driver = &v4l2Capture;
                break;
            case VIDEO:
                driver = &videoCapture;
                break;
            case NOTDEFINED:
                driver = &videoCapture;
                break;
        }
        if(saveVideo){
            path = ( string(OUTPUT_PATH + now_time).append(".avi"));
            videowriter = VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60.0, cv::Size(1280, 1024));
        }
        Mat curImage;
        if((driver->InitCam() && driver->SetCam() && driver->StartGrab()))
        {
            /// log
        }
        else
        {
            driver->StopGrab();

            exit(-1);
        }

        do
        {
            if(driver->Grab(curImage))
            {
                FRAMEWIDTH = curImage.cols;
                FRAMEHEIGHT = curImage.rows;
                armorDetectorPtr->Init();
                break;
            }
            missCount++;
            if(missCount > 5)
            {
                driver->StopGrab();
                exit(-1);
            }
        }while(true);
        missCount = 0;
        //LOGM("Initialization Completed\n");
        return true;
    }

    void ImgProdCons::Produce()
    {
        do {
            double st = (double) getTickCount();
            //Receive();
            //cout << "read data from elc cost : " << CalWasteTime(taskTime,freq) << endl;

            if (!driver->Grab(frame) || frame.rows != FRAMEHEIGHT || frame.cols != FRAMEWIDTH) {
                missCount++;
                //LOGW("FRAME GRAB FAILED!\n");
                if (missCount > 5) {
                    driver->StopGrab();
                    GrabFlag = false;
                    quitFlag = true;
                    raise(SIGINT);
                    break;
                }
            }
            if (carName != VIDEO && saveVideo)
                videowriter.write(frame);
            detectFrame = frame.clone();
            produceTime = CalWasteTime(st,freq);
            produceMission = true;
#if SHOWTIME == 1
            //cout << "Frame Produce Mission Cost : " << produceTime << " ms" << endl;
#endif
        }while(!quitFlag);
    }

    void ImgProdCons::Detect()
    {
        do {
            double st = (double)getTickCount();
            if (produceMission && !detectMission) {
                produceMission = false;
                /** 计算上一次执行耗时 **/
                if(tmp_t!=0)
                    last_mission_time = CalWasteTime(tmp_t,freq); //记录上一次任务运行的时间
                tmp_t = getTickCount();
                total_time = 0;
                for(int i = 0; i<whole_time_arr.size()-1; i++){
                    whole_time_arr[i] = whole_time_arr[i+1];
                    total_time += whole_time_arr[i];
                }
                whole_time_arr.back() = last_mission_time;
                deltat = (total_time + last_mission_time) / whole_time_arr.size();
                //cout << "everage time = " << deltat << "ms" << endl;
                cout << "last t = " << last_mission_time << "ms" <<endl;
                /** ** **/
                if (lastControlState == curControlState) {
                    lastControlState = curControlState;
                } else {
                    if (curControlState == BIG_ENERGY_STATE || curControlState == SMALL_ENERGY_STATE)
                        energyPtr->init();
                    //else
                    /* 装甲板识别初始化，滤波器初始化啥的 */
                    if (curControlState == AUTO_SHOOT_STATE)
                        predictPtr->Refresh();
                }
                lastControlState = curControlState; //更新上一次的状态值
                switch (curControlState) {
                    case AUTO_SHOOT_STATE:
                        Armor();
                        break;
                    case BIG_ENERGY_STATE:
                        Energy();
                        break;
                    case SMALL_ENERGY_STATE:
                        Energy();
                        break;
                    default:
                        Armor();
                }
                detectMission = true;
            }
            detectTime = CalWasteTime(st,freq);
        }while (!quitFlag);
    }

    void ImgProdCons::Armor()
    {
        switch (curDetectMode)
        {
            case MODEL_MODE:
            {
                if (armorDetectorPtr->ModelDetectTask(detectFrame)){
                    curDetectMode = TRADITION_MODE;
                }
                else{
                    curDetectMode = MODEL_MODE;
                }
                break;
            }
            case TRADITION_MODE:
            {
                if (armorDetectorPtr->ArmorDetectTask(detectFrame)){
                    curDetectMode = TRADITION_MODE;
                }
                else{
                    if(++armorDetectorPtr->lossCnt >= 2) {
                        //curDetectMode = MODEL_MODE;
                    }
                }
                break;
            }
        }
        //cout << "Armor Detect Mission Cost : " << CalWasteTime(armorTime,freq) << " ms" << endl;
    }

    void ImgProdCons::Energy()
    {
        /* do energy detection */
        energyPtr->EnergyTask(detectFrame, curControlState, deltat);

#if SAVE_TEST_DATA == 1
        // **** 当前相角  当前角速度  预测弧度值 **** //
        dataWrite << energyPtr->cur_phi << " " << energyPtr->cur_omega << " " << energyPtr->predict_rad << endl;
#endif
#if SHOWTIME == 1
        //cout << "Energy Detect Mission Cost : " << CalWasteTime(energyTime,freq) << " ms" << endl;
#endif
    }

    void ImgProdCons::Feedback() {
        do {
            if (detectMission) {
                detectMission = false;
                double st = (double) getTickCount();
                if (curControlState == AUTO_SHOOT_STATE) {
                    if (armorDetectorPtr->findState && armorDetectorPtr->lostCnt == 0) {
                        /**call solvePnp algorithm function to get the yaw, pitch and distance data**/
                        solverPtr->GetPoseV(armorDetectorPtr->targetArmor.pts,
                                            armorDetectorPtr->targetArmor.armorType);
                        // 云台当前yaw pitch
                        Vector3f gimbal_ypd;

                        if (carName == VIDEO) {
                            // 用于平时的视频测试时
                            gimbal_ypd << 0, 0, solverPtr->dist;
                            target_ypd << gimbal_ypd[0] - solverPtr->yaw,
                                    gimbal_ypd[1] + solverPtr->pitch,
                                    solverPtr->dist;
                            predictPtr->armorPredictor(target_ypd, gimbal_ypd, v_bullet);
                            float pp = solverPtr->pitchCompensate(predictPtr->predict_xyz, v_bullet);
                            float temp_t;
                            float cal_pitch = solverPtr->CalPitch(predictPtr->predict_xyz, v_bullet, temp_t);
//                            string str[] = {"old", "new"};
//                            vector<float> data(2);
//                            data = {solverPtr->pitch + pp, cal_pitch};
//                            RMTools::showData(data, str, "pitch");

                        }
                        else if (carName = SENTRY) {
                            gimbal_ypd << receiveData.yawAngle, receiveData.pitchAngle, 0;
                            target_ypd << receiveData.yawAngle - solverPtr->yaw,
                                    receiveData.pitchAngle - solverPtr->pitch,
                                    solverPtr->dist;
                            predictPtr->test415(target_ypd, v_bullet, last_mission_time / 1000);
                            yaw_abs = target_ypd[0];
                            pitch_abs = target_ypd[1];
                        }
                        else {
                            gimbal_ypd << receiveData.yawAngle, receiveData.pitchAngle, 0;
                            target_ypd << receiveData.yawAngle - solverPtr->yaw,
                                    receiveData.pitchAngle + solverPtr->pitch,
                                    solverPtr->dist;
//                    cout << "solverPtr->yaw: " << solverPtr->yaw << endl;
//                    cout << "Gimbal yaw: " << gimbal_ypd[0] << endl;
//                    cout << "Target yaw: " << target_ypd[0] << endl;

                            //predictPtr->armorPredictor(target_ypd, gimbal_ypd, v_bullet);
                            predictPtr->test415(target_ypd, v_bullet, last_mission_time / 1000);
                            yaw_abs = predictPtr->predict_ypd[0];
                            //pitch_abs = 3 + solverPtr->CalPitch(predictPtr->predict_xyz,target_ypd,v_bullet);
                            pitch_abs = predictPtr->predict_ypd[1];

                        }
                        string str[] = {"re-yaw:",
                                        "re-pitch:",
                                        "tar-yaw:",
                                        "tar-pit:",
                                        "pre-yaw",
                                        "pre-pit",
                                        "v bullet:"};
                        vector<float> data(7);
                        data = {receiveData.yawAngle,
                                receiveData.pitchAngle,
                                target_ypd[0],
                                target_ypd[1],
                                predictPtr->predict_ypd[0],
                                predictPtr->predict_ypd[1],
                                v_bullet};
                        //RMTools::showData(data,str,"abs degree");
                        solverPtr->backProject2D(detectFrame, predictPtr->predict_xyz, gimbal_ypd);

#if SAVE_TEST_DATA == 1
                        // **** 目标陀螺仪 x y z **** //
                        dataWrite << gimbal_ypd[1] << " " << solverPtr->yaw << endl;
#endif
                    } else if (armorDetectorPtr->findState && armorDetectorPtr->lostCnt > 0 &&
                               armorDetectorPtr->lostCnt < 3) {
                        if (armorDetectorPtr->lossCnt == 1)
                            last_xyz = predictPtr->target_xyz;
                        float dt = tt / cnt1 / 1000;
                        predictPtr->target_xyz = predictPtr->target_xyz + predictPtr->target_v_xyz * dt +
                                                 0.5 * predictPtr->target_a_xyz * dt * dt;
                        Vector3f predict_xyz = predictPtr->kalmanPredict(predictPtr->target_xyz, v_bullet);
                        Vector3f predict_ypd = target_ypd + GetDeltaYPD(predict_xyz, last_xyz);
                        yaw_abs = predict_ypd[0];
                        pitch_abs = predict_ypd[1];
                    }


                    /*******************************************************************************************************
                     * when the armor-detector has not detected target armor successfully, that may be caused by the suddenly
                     * movement of robots(myself and the opposite target  robot), but the target armor is still in the view
                     * scoop, we still need to instruct the movement of the holder instead of releasing it to the cruise mode
                     * ****************************************************************************************************/



                    /** package data and prepare for sending data to lower-machine **/
                    serialPtr->pack(yaw_abs,
                                    pitch_abs,
                                    solverPtr->dist,
                                    solverPtr->shoot,
                                    armorDetectorPtr->findState,
                                    AUTO_SHOOT_STATE,
                                    0);
                    feedbackDelta = 1;
#if DEBUG_MSG == 1
                    LOGM("Write Data\n");
#endif
                }
                else {
                    solverPtr->GetPoseV(energyPtr->predict_pts, false);
                    //solverPtr->GetPoseV(energyPtr->pts,false,16);

                    ///电控云台  yaw角：向右为 -  向左为 +    pitch角：向上为 + 向下为 -
                    yaw_abs = receiveData.yawAngle - solverPtr->yaw; //绝对yaw角度
                    pitch_abs = receiveData.pitchAngle + solverPtr->pitch; //绝对pitch角度

                    serialPtr->pack(yaw_abs,
                                    pitch_abs,
                                    solverPtr->dist,
                                    solverPtr->shoot,
                                    energyPtr->detect_flag,
                                    curControlState,
                                    0);

#if SAVE_LOG == 1
                    //            string s = getSysTime();
        //            logWrite <<"=========== produce mission ==========="<< endl;
        //            logWrite << s << endl;
        //            logWrite << "yaw angle : " << solverPtr->yaw << endl;
        //            logWrite << "pitch angle : " << solverPtr->pitch << endl;
        //            logWrite << "detect or not : " << energyPtr->detect_flag << endl;
                    logWrite << solverPtr->tvecs << endl;
#endif
                }

                /** send data from host to low-end machine to instruct holder's movement **/
                if (serialPtr->WriteData()) {
#if SAVE_LOG == 1
                    logWrite<<"[Write Data to USB2TTL SUCCEED]"<<endl;
#endif
                } else {
                    //logWrite<<"[Write Data to USB2TTL FAILED]"<<endl;
                }
                feedbackTime = CalWasteTime(st,freq);
                /** Receive data from low-end machine to update parameters(the color of robot, the task mode, etc) **/
#if SHOWTIME == 1
                cout << "Frame Produce Mission Cost : " << produceTime << " ms" << endl;
                cout << "Detect Mission Cost : " << detectTime << " ms" << endl;
                cout << "FeedBack Mission Cost : " << feedbackTime << " ms" << endl;
                cout << "receive Mission Cost : " << receiveTime << " ms" << endl;
                if(showArmorBox || showEnergy || showOrigin)
                    cout << "Show Image Cost : " << CalWasteTime(showImgTime, freq) << " ms" << endl;
                cout << endl;
#endif
            }
        }while (!quitFlag);
    }

    void ImgProdCons::Receive()
    {
        do{
            double st = (double) getTickCount();
            if (serialPtr->ReadData(receiveData)) {
                curControlState = receiveData.targetMode; //由电控确定当前模式 0：自瞄装甲板 1：小幅 2：大幅
                v_bullet = receiveData.bulletSpeed == 0 ? 14 : receiveData.bulletSpeed;
            }
            receiveTime = CalWasteTime(st,freq);
#if SHOWTIME == 1
            //
#endif

        } while (!quitFlag);
    }

    void ImgProdCons::ShowImage() {
        do{
            double st = (double) getTickCount();
            Mat show_img = detectFrame.clone();
            detectMission = false;
            if (!show_img.empty()) {
                if (showArmorBox || showEnergy) {
                    circle(show_img, Point(FRAMEWIDTH / 2, FRAMEHEIGHT / 2), 2, Scalar(0, 255, 255), 3);

                    putText(show_img, "distance: ", Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255),
                            2,
                            8, 0);
                    putText(show_img, to_string(solverPtr->dist), Point(150, 30), cv::FONT_HERSHEY_PLAIN, 2,
                            Scalar(255, 255, 255), 2, 8, 0);

                    putText(show_img, "yaw: ", Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2,
                            8,
                            0);
                    putText(show_img, to_string(solverPtr->yaw), Point(80, 60), cv::FONT_HERSHEY_PLAIN, 2,
                            Scalar(255, 255, 255), 2, 8, 0);

                    putText(show_img, "pitch: ", Point(0, 90), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2,
                            8,
                            0);
                    putText(show_img, to_string(solverPtr->pitch), Point(100, 90), cv::FONT_HERSHEY_PLAIN, 2,
                            Scalar(255, 255, 255), 2, 8, 0);

                    putText(show_img, "detecting:  ", Point(0, 120), cv::FONT_HERSHEY_SIMPLEX, 1,
                            Scalar(255, 255, 255),
                            2, 8, 0);
                    if (showEnergy && energyPtr->detect_flag) {
                        circle(show_img, Point(165, 115), 4, Scalar(255, 255, 255), 3);
                        for (int i = 0; i < 4; i++) {
                            line(show_img, energyPtr->pts[i], energyPtr->pts[(i + 1) % (4)],
                                 Scalar(255, 255, 255), 2, LINE_8);
                            line(show_img, energyPtr->predict_pts[i], energyPtr->predict_pts[(i + 1) % (4)],
                                 Scalar(0, 255, 255), 2, LINE_8);
                        }
                        circle(show_img, energyPtr->target_point, 2, Scalar(0, 255, 0), 3);
                        circle(show_img, energyPtr->circle_center_point, 3, Scalar(255, 255, 255), 3);
                        circle(show_img, energyPtr->predict_point, 2, Scalar(100, 10, 255), 3);
                    }
                    if (showArmorBox && armorDetectorPtr->findState) {
                        circle(show_img, Point(165, 115), 4, Scalar(255, 255, 255), 3);
                        if (!predictPtr->predict_point.empty())
                            circle(show_img, predictPtr->predict_point.back(), 2, Scalar(100, 240, 15), 3);
                    }
                    imshow("Detect Frame", show_img);
                    waitKey(1);
                }

                if (showOrigin) {
                    circle(frame, Point(FRAMEWIDTH / 2, FRAMEHEIGHT / 2), 5, Scalar(255, 255, 255), -1);

                    if (FRAMEHEIGHT > 1000) {
                        //pyrDown(detectFrame,detectFrame);
                        //pyrDown(detectFrame,detectFrame);
                    }
                    imshow("detect", frame);
                    waitKey(3);
                }
                /**press key 'space' to pause or continue task**/
                if (debug) {
                    if (!pauseFlag && waitKey(30) == 32) { pauseFlag = true; }

                    if (pauseFlag) {
                        while (waitKey() != 32) {}
                        pauseFlag = false;
                    }
                }
#if SHOWTIME == 1

#endif
            }
            showImgTime = CalWasteTime(st,freq);
        }while(!quitFlag);
    }
}
