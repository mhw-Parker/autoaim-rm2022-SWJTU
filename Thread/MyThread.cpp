//
// Created by luojunhui on 1/28/20.
//

#include <csignal>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>
#include <iostream>
#include <memory>
#include <sys/stat.h>
#include <sys/types.h>

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
VideoWriter videoWriter(path, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 50.0, cv::Size(1280, 1024));
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
    bool saveMission = false;
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
    ImgProdCons::~ImgProdCons() {
        driver->StopGrab();
        timeWrite.close();
    }

    bool ImgProdCons::Init()
    {
        /*initialize signal*/
        InitSignals();
        whole_time_arr.resize(3);
        /*initialize camera*/
#if SAVE_TEST_DATA == 1
        fout.close();
        //dataWrite.close();
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
            case SENTRYDOWN:
#ifdef MIND
                driver = &mindCapture;
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
        if(showEnergy)
            curControlState = BIG_ENERGY_STATE;
        Mat curImage;
        if((driver->InitCam() && driver->SetCam() && driver->StartGrab())){
            freq = getTickFrequency();
            startT = getTickCount();
            time_stamp.assign(6000,0);
            /// log
        }
        else
        {
            driver->StopGrab();
            exit(-1);
        }
        //尝试读取5次相机采集
        do
        {
            if(driver->Grab(curImage)) {
                FRAMEWIDTH = curImage.cols;
                FRAMEHEIGHT = curImage.rows;
                armorDetectorPtr->Init();
                break;
            }
            missCount++;
            if(missCount > 5) {
                driver->StopGrab();
                exit(-1);
            }
        }while(true);
        missCount = 0;
        if(saveVideo){
            saveVideoPath = ( string(OUTPUT_PATH + now_time).append(".avi") );
            saveTimeSeriesPath = ( string(OUTPUT_PATH + now_time).append(".txt") );
            timeWrite = ofstream (saveTimeSeriesPath, ios::out);
            videoWriter = VideoWriter(saveVideoPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60.0, cv::Size(FRAMEWIDTH, FRAMEHEIGHT));
        }
        /// 设置对应视频的每帧对应时间的 txt 路径
        if(carName == VIDEO){
            int pos = videoPath.find(".avi");
            if(pos == -1) pos = videoPath.find(".mp4");
            else cout << "my rough code can't cut out the root path !\n";
            if(pos != -1) {
                string rootPath = videoPath.erase(pos);
                timePath = rootPath.append(".txt");
                readTimeTxt = ifstream (timePath);
            }
        }
        //LOGM("Initialization Completed\n");
        return true;
    }

    void ImgProdCons::Armor() {
        switch (curDetectMode) {
            case MODEL_MODE: {
                if (armorDetectorPtr->ModelDetectTask(detectFrame)) {
                    curDetectMode = TRADITION_MODE;
                } else {
                    curDetectMode = MODEL_MODE;
                }
                break;
            }
            case TRADITION_MODE: {
                if (armorDetectorPtr->ArmorDetectTask(detectFrame)) {
                    if (saveSVM) {
                        save_img_cnt++;
                        if (save_img_cnt == 10) {
                            save_img_cnt = 0;
                            if (armorDetectorPtr->armorNumber) {
                                SVMPath = (string(SAVE_SVM_PIC) + to_string(10) + "/" +
                                           to_string(svm_img_num)).append(".png");
                            } else {
                                SVMPath = (string(SAVE_SVM_PIC) + "none/" +
                                           to_string(svm_img_num)).append(".png");
                            }
                            imwrite(SVMPath, armorDetectorPtr->warpPerspective_dst);
                            svm_img_num++;
                        }
                    }
                    curDetectMode = TRADITION_MODE;
                } else {
                    if (++armorDetectorPtr->lossCnt >= 2) {
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
        //energyPtr->EnergyTask(detectFrame, curControlState, last_mission_time, 0.15+fly_t);
        energyPtr->EnergyDetectTask(detectFrame);
#if SAVE_TEST_DATA == 1
        // **** 当前相角  当前角速度  预测弧度值 **** //
        dataWrite << energyPtr->cur_phi << " " << energyPtr->cur_omega << " " << energyPtr->predict_rad << endl;
#endif
#if SHOWTIME == 1
        //cout << "Energy Detect Mission Cost : " << CalWasteTime(energyTime,freq) << " ms" << endl;
#endif
    }

    /** thread function **/
    void ImgProdCons::Produce()
    {
        do {
            if(!produceMission) {
                double st = (double) getTickCount();
                if (!driver->Grab(frame) || frame.rows != FRAMEHEIGHT || frame.cols != FRAMEWIDTH) {
                    missCount++;
                    //LOGW("FRAME GRAB FAILED!\n");
                    if (missCount > 5) {
                        driver->StopGrab();
                        GrabFlag = false;
                        quitFlag = true;
                        cout << "Exit for grabbing fail." << '\n';
                        raise(SIGINT);
                        break;
                    }
                } else {
                    //if(carName != VIDEO)
                        time_stamp[++cnt%6000] = CalWasteTime(startT,freq)/1000;
                    //else {
                    //    readTimeTxt >> time_stamp[++cnt%6000];
                    //}
                    saveMission = true;
                }
//                if (carName != VIDEO && saveVideo) {
//                    videoWriter.write(frame);
//                    //if(timeWrite.is_open()) timeWrite << time_stamp[cnt] << "\n";
//                }
                produceTime = CalWasteTime(st, freq);
                produceMission = true;
            }
        }while(!quitFlag);
    }

    void ImgProdCons::Detect()
    {
        do {
            if (!detectMission && produceMission) {
                double st = (double)getTickCount();
                last_mission_time = time_stamp[cnt%6000] - time_stamp[last_cnt%6000]; //两次图片时间间隔 用当次时间序列值 - 上次时间序列值
                last_cnt = cnt; //更新当次时间序列序号
                detectFrame = frame.clone();
                produceMission = false;
                /** 计算上一次源图像执行耗时 **/
                cout << "last t = " << last_mission_time*1000 << "ms" <<endl;
                /** ** **/
                if (lastControlState == curControlState) {
                    lastControlState = curControlState;
                } else {
                    predictPtr->Refresh();
                }
                lastControlState = curControlState; //更新上一次的状态值
                switch (curControlState) {
                    case AUTO_SHOOT_STATE:
                        Armor();
                        break;
                    case BIG_ENERGY_STATE:
                    case SMALL_ENERGY_STATE:
                        Energy();
                        break;
                    default:
                        Armor();
                }
                detectMission = true;
                detectTime = CalWasteTime(st,freq);
            }
        }while (!quitFlag);
    }

    void ImgProdCons::Feedback() {
        do {
            if (detectMission && receiveMission) {
                double st = (double) getTickCount();

                Vector3f gimbal_ypd;
                if(carName == VIDEO)
                    gimbal_ypd << 0, 0, 0;
                else
                    gimbal_ypd << receiveData.yawAngle, receiveData.pitchAngle, 0;
                receiveMission = false;

                tmp_t = last_mission_time; //同步时间
                if(showArmorBox || showEnergy) {
                    show_img = detectFrame.clone();
                }
                find_state = (curControlState == AUTO_SHOOT_STATE) ? (!armorDetectorPtr->lostState) : energyPtr->detect_flag;
                if(curControlState != AUTO_SHOOT_STATE)
                    rotate_center = energyPtr->circle_center_point;
                detectMission = false;

                if (curControlState == AUTO_SHOOT_STATE) {
                    if (find_state) {
                        predictPtr->ArmorPredictor(armorDetectorPtr->targetArmor.pts, armorDetectorPtr->targetArmor.armorType,
                                                   gimbal_ypd,v_bullet,tmp_t, armorDetectorPtr->lostCnt);
                        Vector2f offset = RMTools::GetOffset(carName);
                        yaw_abs = predictPtr->predict_ypd[0] + offset[0];
                        pitch_abs = predictPtr->predict_ypd[1] + offset[1];
                    }
                }
                else {
                    predictPtr->EnergyPredictor(curControlState,energyPtr->pts,rotate_center,gimbal_ypd,
                                                   v_bullet,tmp_t);
                    yaw_abs = predictPtr->predict_ypd[0];
                    pitch_abs = predictPtr->predict_ypd[1];
                }
                /** package data and prepare for sending data to lower-machine **/
                serialPtr->pack(yaw_abs,
                                pitch_abs,
                                predictPtr->predict_ypd[2],
                                predictPtr->shootCmd,
                                find_state,
                                curControlState,
                                0);
#if SAVE_TEST_DATA == 1
                if(dataWrite.is_open())
                    dataWrite << time_stamp[last_cnt] << " " << predictPtr->cam_yaw << endl;
#endif
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
                    cout << "Show Image Cost : " << showImgTime << " ms" << endl;
                cout << endl;
#endif
                if(showArmorBox || showEnergy || showOrigin){
                    ShowImage();
                }
            }
        }while (!quitFlag);
    }

    void ImgProdCons::Receive()
    {
        do{
            if(produceMission && !receiveMission){
                double st = (double) getTickCount();
                if (serialPtr->ReadData(receiveData)) {
                    curControlState = receiveData.targetMode; //由电控确定当前模式 0：自瞄装甲板 1：小幅 2：大幅
                    v_bullet = receiveData.bulletSpeed;
                    blueTarget = receiveData.targetColor;
                }
                receiveTime = CalWasteTime(st, freq);
                receiveMission = true;
            }
#if SHOWTIME == 1
            //
#endif

        } while (!quitFlag);
    }

    void ImgProdCons::ShowImage() {
        double st = (double) getTickCount();
        if (!show_img.empty()) {
            //Mat show_img = detectFrame.clone();
            if (showArmorBox || showEnergy) {
                circle(show_img, Point(FRAMEWIDTH / 2, FRAMEHEIGHT / 2), 2, Scalar(0, 255, 255), 3);

                putText(show_img, "distance: ", Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255),
                        2,
                        8, 0);
                putText(show_img, to_string(predictPtr->delta_ypd[2]), Point(150, 30), cv::FONT_HERSHEY_PLAIN, 2,
                        Scalar(255, 255, 255), 2, 8, 0);

                putText(show_img, "yaw: ", Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2,
                        8,
                        0);
                putText(show_img, to_string(predictPtr->delta_ypd[0]), Point(80, 60), cv::FONT_HERSHEY_PLAIN, 2,
                        Scalar(255, 255, 255), 2, 8, 0);

                putText(show_img, "pitch: ", Point(0, 90), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2,
                        8,
                        0);
                putText(show_img, to_string(predictPtr->delta_ypd[1]), Point(100, 90), cv::FONT_HERSHEY_PLAIN, 2,
                        Scalar(255, 255, 255), 2, 8, 0);

                putText(show_img, "detecting:  ", Point(0, 120), cv::FONT_HERSHEY_SIMPLEX, 1,
                        Scalar(255, 255, 255),
                        2, 8, 0);

                putText(show_img, "cost:", Point(1060, 28), cv::FONT_HERSHEY_SIMPLEX, 1,
                        Scalar(0, 255, 0),
                        1, 8, 0);
                putText(show_img, to_string(last_mission_time*1000), Point(1140, 30), cv::FONT_HERSHEY_PLAIN, 2,
                        Scalar(0, 255, 0), 1, 8, 0);

                if (showEnergy) {
                    circle(show_img, Point(165, 115), 4, Scalar(255, 255, 255), 3);
                    for (int i = 0; i < 4; i++) {
                        line(show_img, energyPtr->pts[i], energyPtr->pts[(i + 1) % (4)],
                             Scalar(255, 255, 255), 2, LINE_8);
                        line(show_img, predictPtr->predict_pts[i], predictPtr->predict_pts[(i + 1) % (4)],
                             Scalar(0, 255, 255), 2, LINE_8);
                    }
                    circle(show_img, energyPtr->target_point, 2, Scalar(0, 0, 255), 3);
                    circle(show_img, energyPtr->circle_center_point, 3, Scalar(255, 255, 255), 3);
                }
                if (find_state) {
                    circle(show_img, Point(165, 115), 4, Scalar(255, 255, 255), 3);
                    circle(show_img, predictPtr->predict_point, 5, Scalar(100, 240, 15), 2);
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
    }

    void ImgProdCons::Record() {
        do {
            try {
                if(saveMission) {
                    videoWriter.write(frame);
                    if(timeWrite.is_open()) timeWrite << time_stamp[cnt] << "\n";
                    //else cout << "txt close\n" ;
                }
            }
            catch (...) {
                timeWrite.close();
                std::this_thread::sleep_for(500ms);
            }
            //if(timeWrite.is_open()) timeWrite << time_stamp[cnt] << "\n";
            saveMission = false;
        }while(!quitFlag);
    }
}

