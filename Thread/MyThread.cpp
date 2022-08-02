//
// Created by helios on 2021/12
//

#include <csignal>
#include <memory>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <memory>
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
            serialPtr(std::make_unique<Serial>()),
            armorDetectorPtr(std::make_unique<ArmorDetector>()),
            energyPtr(std::make_unique<EnergyDetector>()),
            predictPtr(std::make_unique<Predictor>()),
            armorType(BIG_ARMOR),
            driver(),
            missCount(0),
            showWave(100,300,500) {}

    ImgProdCons::~ImgProdCons() {
        driver->StopGrab();
        //timeWrite.close();
    }

    bool ImgProdCons::Init() {
        /*initialize signal*/
        InitSignals();
        if (!serialPtr->serial_state) {
            com_flag = false;
            cerr << "serial port set failed !" << endl;
        }
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
            case INFANTRY3:
            case INFANTRY4:
            case INFANTRY5:
#ifdef MIND
                driver = &mindCapture;
#endif
                break;
            case INFANTRY_TRACK:
                driver = &v4l2Capture;
                break;
            case SENTRYTOP:
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
            case IMAGE:
                driver = &imageCapture;
                break;
            case NOTDEFINED:
                driver = &videoCapture;
                break;
        }
        if(showEnergy)
            curControlState = BIG_ENERGY_STATE;
        Mat curImage;
        if((driver->InitCam() && driver->StartGrab())){
            freq = getTickFrequency();
            startT = getTickCount();
            /// log
        } else {
            driver->StopGrab();
            exit(-1);
        }
        //尝试5次相机采集，视频模式尝试1次
        do {
            if (driver->Grab(curImage)) {
                FRAMEWIDTH = curImage.cols;
                FRAMEHEIGHT = curImage.rows;
                armorDetectorPtr->Init();
                break;
            }
            missCount++;
            if (carName == VIDEO && missCount) {
                exit(-1);
            }
            if (missCount > 5) {
                driver->StopGrab();
                exit(-1);
            }
        }while(true);
        missCount = 0;
        if(saveVideo){
            saveVideoPath = ( string(OUTPUT_PATH + now_time).append(".avi") );
            saveTimeSeriesPath = ( string(OUTPUT_PATH + now_time).append(".txt") );
            //timeWrite = ofstream (saveTimeSeriesPath, ios::out);
            videoWriter = VideoWriter(saveVideoPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                      85.0, cv::Size(FRAMEWIDTH, FRAMEHEIGHT));
        }
        /// 设置对应视频的每帧对应时间的 txt 路径
        if(carName == VIDEO){
            int pos = srcPath.find(".avi");
            if(pos == -1) pos = srcPath.find(".mp4");
            else cout << "my rough code can't cut out the root path !\n";
            if(pos != -1) {
                string rootPath = srcPath.erase(pos);
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
                }
            }
        }
        //cout << "Armor Detect Mission Cost : " << CalWasteTime(armorTime,freq) << " ms" << endl;
    }

    void ImgProdCons::Energy()
    {
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
            double st = (double) getTickCount();
            ReceiveData rd;
            if (com_flag)
                rd = receive_fifo.wait_and_pop();
            receive_pop_time = CalWasteTime(st, freq);
            double st1 = (double) getTickCount();
            if (!driver->Grab(frame) || frame.rows != FRAMEHEIGHT || frame.cols != FRAMEWIDTH) {
                missCount++;
                //LOGW("FRAME GRAB FAILED!\n");
                if (missCount > 5) {
                    driver->StopGrab();
                    GrabFlag = false;
                    quitFlag = true;
                    cerr << "Exit for grabbing fail." << endl;
                    raise(SIGINT);
                    break;
                }
            } else {
                time_stamp = CalWasteTime(startT,freq)/1000; // save logs which include time_stamp, yaw, pitch
                saveMission = true;
            }
            // put new frame which grab from camera in Fifo
            timeStampMat temp(frame,time_stamp,rd);
            frame_fifo.push(temp);
            produceTime = CalWasteTime(st1, freq);
            // 读取视频空格暂停
            if (carName == VIDEO) {
                if (waitKey(1) == 32) {
                    while (waitKey() != 32) {}
                }
            }
        }while(!quitFlag);
    }

    void ImgProdCons::Detect()
    {
        do {
            double st = (double)getTickCount();
            timeStampMat detect_stamp = frame_fifo.wait_and_pop();
            /** 计算上一次源图像执行耗时 **/
            last_mission_time = detect_stamp.stamp - last_stamp; //两次检测时间间隔 用当次时间序列值 - 上次时间序列值
            //cout << 1/last_mission_time << endl;
            last_stamp = detect_stamp.stamp; //更新当次时间序列序号
            //cout << "t = " << detect_stamp.stamp << " s" <<endl;
            detectFrame = detect_stamp.frame.clone();
            //printf("time : %f\n",detect_stamp.stamp);
            Vector3f gimbal_ypd;
            if(carName == VIDEO || !com_flag) {
                gimbal_ypd << 0,0,0;
            } else {
                gimbal_ypd << detect_stamp.mcuData.yawAngle,
                              detect_stamp.mcuData.pitchAngle,
                              0;
            }

            // if state change, refresh state
            if(lastControlState != curControlState) {
                predictPtr->Refresh();
                lastControlState = curControlState;
            }
            // 用于计算只含有Armor和Energy任务的时间
            double recognitionSt = getTickCount();
            double predictionSt;
            switch (curControlState) {
                case AUTO_SHOOT_STATE:
                    Armor();
                    recognitionTime = CalWasteTime(recognitionSt, freq);
                    find_state = armorDetectorPtr->findState;
                    // 保存未识别的帧
//                    if (!find_state) {
//                        imwrite(OUTPUT_PATH + (string)"LostImg/" + getSysTime() + ".png",
//                                detectFrame);
//                    }
                    predictionSt = getTickCount();
                    if (find_state) {
                        car_num = armorDetectorPtr->armorNumber;
                        find_state = armorDetectorPtr->lostCnt <= max_lost;
                    } else {
                        car_num = 0;
                    }
                    // 如果小于丢失的阈值则认为可以补帧预测
                    if (armorDetectorPtr->lostCnt <= max_lost) {
                        predictPtr->ArmorPredictor(armorDetectorPtr->targetArmor.pts,
                                                   armorDetectorPtr->targetArmor.armorType,
                                                   gimbal_ypd,
                                                   v_bullet,
                                                   last_mission_time,
                                                   armorDetectorPtr->lostCnt);
                    }
                    predictionTime = CalWasteTime(predictionSt, freq);
                    break;
                case BIG_ENERGY_STATE:
                case SMALL_ENERGY_STATE:
                    Energy();
                    recognitionTime = CalWasteTime(recognitionSt, freq);
                    find_state = energyPtr->detect_flag;
                    predictionSt = getTickCount();
                    if (find_state) {
                        predictPtr->EnergyPredictor(curControlState,
                                                    energyPtr->pts,
                                                    energyPtr->circle_center_point,
                                                    gimbal_ypd,
                                                    v_bullet,
                                                    detect_stamp.stamp);
                    }
                    find_state = predictPtr->est_flag && find_state;
                    predictionTime = CalWasteTime(predictionSt, freq);
                    break;
                default:
                    Armor();
            }

            // 读图片模式单线程，识别完按空格退出
            if (carName == IMAGE) {
                show_img = detectFrame.clone();
                ShowImage();
                if (waitKey() == 32) exit(0);
            }
            //
            show_fifo.push(detectFrame);
            SendData send_data(predictPtr->back_ypd[0],
                               predictPtr->back_ypd[1],
                               find_state,
                               car_num,
                               predictPtr->shootCmd);
            send_fifo.push(send_data);
            detectTime = CalWasteTime(st, freq);
        }while (!quitFlag);
    }

    void ImgProdCons::Feedback() {
        do {
            double st = (double) getTickCount();
            SendData data = send_fifo.wait_and_pop();
            /** package data and prepare for sending data to lower-machine **/
            serialPtr->pack(data.yaw,
                            data.pitch,
                            data.find,
                            data.id,
                            data.cmd
                            );
#if SAVE_TEST_DATA == 1
            if(dataWrite.is_open())
                    dataWrite << time_stamp[last_cnt] << " " << predictPtr->cam_yaw << endl;
#endif
            /// 发送数据，除了读取视频模式
            if (carName != VIDEO && carName != IMAGE && serialPtr->WriteData()) {
#if SAVE_LOG == 1
                logWrite<<"[Write Data to USB2TTL SUCCEED]"<<endl;
#endif
            } else {
                //logWrite<<"[Write Data to USB2TTL FAILED]"<<endl;
            }

            // 显示各任务耗时
#if SHOWTIME == 1
            cout << "Received data popping Mission Cost : " << receive_pop_time << " ms" << endl;
            cout << "Frame Produce Mission Cost : " << produceTime << " ms" << endl;
            cout << "Detect Mission Cost : " << detectTime << " ms" << endl;
            cout << "Armor/Energy Task Cost : " << recognitionTime << " ms" << endl;
            cout << "Prediction Task Cost : " << predictionTime << " ms" << endl;
            cout << "FeedBack Mission Cost : " << feedbackTime << " ms" << endl;
            cout << "receive Mission Cost : " << receiveTime << " ms" << endl;
#endif
            // 显示图像
            if (carName != IMAGE && (showArmorBox || showEnergy || showOrigin)) {
                ShowImage();
#if SHOWTIME == 1
                cout << "Show Image Cost : " << showImgTime << " ms" << endl;
#endif
            }
            feedbackTime = CalWasteTime(st,freq);
        }while (!quitFlag);
    }

    void ImgProdCons::Receive()
    {
        do{
            double st = (double) getTickCount();
            if (carName != VIDEO && carName != IMAGE && serialPtr->ReadData(receiveData)) {
                receive_fifo.push(receiveData);
                curControlState = receiveData.targetMode;
                blueTarget = receiveData.targetColor;
                v_bullet = receiveData.bulletSpeed;
            }
            receiveTime = CalWasteTime(st, freq);
            //printf("-- receiver time = %f\n",receiveTime);
#if SHOWTIME == 1
            //
#endif
        } while (!quitFlag);
    }

    void ImgProdCons::ShowImage() {
        double st = (double) getTickCount();
        show_img = show_fifo.wait_and_pop().clone();
        if (!show_img.empty()) {
            //Mat show_img = detectFrame.clone();
            if (showArmorBox || showEnergy) {
                circle(show_img, Point(FRAMEWIDTH / 2, FRAMEHEIGHT / 2), 2,
                       Scalar(0, 255, 255), 3);
                putText(show_img, "distance: ", Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255),2,8, 0);
                putText(show_img, to_string(predictPtr->delta_ypd[2]), Point(150, 30), cv::FONT_HERSHEY_PLAIN, 2,Scalar(255, 255, 255), 2, 8, 0);

                putText(show_img, "yaw: ", Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2,8,0);
                putText(show_img, to_string(predictPtr->delta_ypd[0]), Point(80, 60), cv::FONT_HERSHEY_PLAIN, 2,Scalar(255, 255, 255), 2, 8, 0);

                putText(show_img, "pitch: ", Point(0, 90), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2,8,0);
                putText(show_img, to_string(predictPtr->delta_ypd[1]), Point(100, 90), cv::FONT_HERSHEY_PLAIN, 2,Scalar(255, 255, 255), 2, 8, 0);

                putText(show_img, "detecting:  ", Point(0, 120), cv::FONT_HERSHEY_SIMPLEX, 1,Scalar(255, 255, 255),2, 8, 0);

                putText(show_img, "cost:", Point(1060, 28), cv::FONT_HERSHEY_SIMPLEX, 1,Scalar(0, 255, 0),1, 8, 0);
                putText(show_img, to_string(last_mission_time*1000), Point(1140, 30), cv::FONT_HERSHEY_PLAIN, 2,Scalar(0, 255, 0), 1, 8, 0);

                if (find_state) {
                    for (int i = 0; i < 4; i++) {
                        line(show_img, predictPtr->predict_pts[i], predictPtr->predict_pts[(i + 1) % (4)],Scalar(180, 105, 255), 1, LINE_8);
                    }
                    circle(show_img, predictPtr->predict_point, 4, Scalar(180, 105, 255), 2);
                    circle(show_img, Point(160, 115), 4, Scalar::all(255), 3);
                }
                imshow("Detect Frame", show_img);
                if (carName == IMAGE) {
                    waitKey();
                } else {
                    waitKey(1);
                }
            }
            if (showOrigin) {
                circle(frame, Point(FRAMEWIDTH / 2, FRAMEHEIGHT / 2), 5, Scalar(255, 255, 255), -1);
                imshow("detect", frame);
                waitKey(3);
            }

            /**press key 'space' to pause or continue task**/
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
                    //if(timeWrite.is_open()) timeWrite << time_stamp[cnt] << "\n";
                    //else cout << "txt close\n" ;
                    // 每3分钟自动保存
                    if (RMTools::CalWasteTime(startT, freq) / 1000 > 3 * 60) {
                        startT = getTickCount();
                        videoWriter.release();
                        saveVideoPath = ( string(OUTPUT_PATH + getSysTime()).append(".avi") );
                        videoWriter.open(saveVideoPath,
                                         cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                         70.0, cv::Size(FRAMEWIDTH, FRAMEHEIGHT));
                    }
                    saveMission = false;
                }
            }
            catch (...) {
                //timeWrite.close();
                std::this_thread::sleep_for(500ms);
            }
            //if(timeWrite.is_open()) timeWrite << time_stamp[cnt] << "\n";
        }while(!quitFlag);
    }
}

