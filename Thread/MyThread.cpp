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

string root_path = "../Output/";
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
string data_path = ( string(root_path + now_time + string("_data")).append(".txt") );
ofstream fout(data_path);
ofstream dataWrite(data_path,ios::out);
#endif

namespace rm
{
    bool ImgProdCons::quitFlag = false;// quit flag, all threads would jump out from loop when quitFlag is true

    bool produceMission = false;//when produce mission completed, produceMission is true, when feedback mission completed, produceMission is false
    bool detectMission = false;//when detect mission completed, detectMission is true, when produce mission completed, detectMission is false
    bool energyMission = false;//when energy mission completed, energyMission is true, when produce mission completed, energyMission is false
    bool feedbackMission = false;//when feedback mission completed, feedbackMission is true, when produce mission completed, feedbackMission is false

    //int8_t curControlState = AUTO_SHOOT_STATE; //current control mode
    int8_t curControlState = BIG_ENERGY_STATE;
    uint8_t curDetectMode = MODEL_MODE; //tracking or searching

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
        LOGE("Process Shut Down By SIGINT\n");
        ImgProdCons::quitFlag = true;

#if SAVE_LOG == 1
        logWrite.close();
#endif
#if SAVE_TEST_DATA == 1
        fout.close();
        dataWrite.close();
#endif
        if(pthread_kill(producePThreadHandler,0) == ESRCH)
        {
            LOGW("Child Thread Produce Thread Close Failed\n");
        }

        if(pthread_kill(detectPThreadHandler,0) == ESRCH)
        {
            LOGW("Child Thread Detect Thread Close Failed\n");
        }

        if(pthread_kill(energyPThreadHandler,0) == ESRCH)
        {
            LOGW("Child Thread EnergyDetector Thread Close Failed\n");
        }

        if(pthread_kill(feedbackPThreadHandler,0) == ESRCH)
        {
            LOGW("Child Thread Feedback Thread Close Failed\n");
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
            armorType(BIG_ARMOR),
            driver(),
            missCount(0)
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

        switch (carName) {
            case HERO:
#ifdef REALSENSE
                driver = &intelCapture;
#endif
                break;
            case INFANTRY_MELEE:
#ifdef MIND
                driver = &mindCapture;
#endif
                break;
            case INFANTRY_TRACK:
                driver = &v4l2Capture;
                break;
            case SENTRY:
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

        Mat curImage;
        if((driver->InitCam() && driver->SetCam() && driver->StartGrab()))
        {
            LOGM("Camera Initialized\n");
            LOGM("Camera Set Down\n");
            LOGM("Camera Start to Grab Frames\n");
        }
        else
        {
            driver->StopGrab();
            LOGW("Camera Resource Released\n");
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
        LOGM("Initialization Completed\n");
        return true;
    }

    void ImgProdCons::Produce()
    {
        taskTime = (double)getTickCount();
        /*******/
        if(tmp_t!=0)
            mission_time = CalWasteTime(tmp_t,freq); //记录上一次任务运行的时间
        tmp_t = taskTime;
        total_time = 0;
        for(int i = 0; i<whole_time_arr.size()-1; i++){
            whole_time_arr[i] = whole_time_arr[i+1];
            total_time += whole_time_arr[i];
        }
        whole_time_arr.back() = mission_time;
        deltat = (total_time + mission_time) / whole_time_arr.size();
        cout << "deltat = " << deltat <<endl;
        /*****/

        if (!driver->Grab(frame) || frame.rows != FRAMEHEIGHT || frame.cols != FRAMEWIDTH)
        {
            missCount++;
            LOGW("FRAME GRAB FAILED!\n");
            if(missCount > 5)
            {
                driver->StopGrab();
                GrabFlag = false;
                 raise(SIGINT);
            }
        }
#if SAVE_VIDEO == 1
        if(carName!=VIDEO)
            videowriter.write(frame);
#endif
        //get current gimbal degree while capture
        serialPtr->ReadData(receiveData);
        detectFrame = frame.clone();
        energyFrame = frame.clone();
#if SHOWTIME == 1
        cout << "Frame Produce Mission Cost : " << CalWasteTime(taskTime,freq) << " ms" << endl;
#endif
    }
    
    void ImgProdCons::Detect() 
    {
        switch (curControlState) {
            case AUTO_SHOOT_STATE: Armor(); break;
            case BIG_ENERGY_STATE: Energy(); break;
            case SMALL_ENERGY_STATE: Energy(); break;
            default: Armor(); break;
        }
    }

    void ImgProdCons::Armor()
    {
        taskTime = (double)getTickCount();
        switch (curDetectMode)
        {
            case MODEL_MODE:
            {
                if (armorDetectorPtr->ModelDetectTask(detectFrame)){
                    curDetectMode = MODEL_MODE;
                }
                else{
                    curDetectMode = MODEL_MODE;
                }
            }
            case TRADITION_MODE:
            {
                if (armorDetectorPtr->ArmorDetectTask(detectFrame)){
                    curDetectMode = MODEL_MODE;
                }
                else{
                    if(++armorDetectorPtr->lossCnt >= 2)
                        curDetectMode = TRADITION_MODE;
                }
            }
        }
        cout << "Armor Detect Mission Cost : " << CalWasteTime(taskTime,freq) << " ms" << endl;
    }

    void ImgProdCons::Energy()
    {
        taskTime = (double)getTickCount();
        if(curControlState == BIG_ENERGY_STATE || curControlState == SMALL_ENERGY_STATE)
        {
            /* do energy detection */
            energyPtr->EnergyTask(energyFrame, curControlState, deltat);
        }
#if SAVE_TEST_DATA == 1
        // **** 当前相角  当前角速度  预测弧度值 **** //
        dataWrite << energyPtr->cur_phi << " " << energyPtr->cur_omega << " " << energyPtr->predict_rad << endl;
#endif
#if SHOWTIME == 1
        cout << "Energy Detect Mission Cost : " << CalWasteTime(taskTime,freq) << " ms" << endl;
        tt += CalWasteTime(taskTime,freq);
        cnt1++;

        cout << "average time = " << tt / cnt1 << endl;
#endif
    }

    void ImgProdCons::Feedback()
    {
        taskTime = (double)getTickCount();
        if(curControlState == AUTO_SHOOT_STATE) {
            if (armorDetectorPtr->findState)
            {
                /**call solvePnp algorithm function to get the yaw, pitch and distance data**/
                solverPtr->GetPoseV(armorDetectorPtr->targetArmor.pts,
                                    armorDetectorPtr->IsSmall());
                /**record distance for debug**/
                dis_count++;
                dis_sum += solverPtr->dist;

                // LOGM("DIS AVG : %f\n", dis_sum/dis_count);
            }
#if DEBUG == 1
            frequency = getTickFrequency()/((double)getTickCount() - timeFlag);

                debugWindowCanvas.colRange(0,299).setTo(0);
                putText(debugWindowCanvas,"Yaw: ",Point(10,30),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                putText(debugWindowCanvas,to_string(yawTran).substr(0,5),Point(100,30),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

                putText(debugWindowCanvas,"Pitch: ",Point(10,60),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                putText(debugWindowCanvas,to_string(pitchTran).substr(0,5),Point(100,60),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

                putText(debugWindowCanvas,"Dist: ",Point(10,90),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                if(carName != HERO)
                    putText(debugWindowCanvas,to_string(solverPtr->dist).substr(0,5),Point(100,90),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                else
                    putText(debugWindowCanvas,to_string(dynamic_cast<RealSenseDriver*>(driver)->dist2Armor).substr(0,5),Point(100,90),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                putText(debugWindowCanvas,"Shoot: ",Point(10,120),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                if(solverPtr->shoot)
                    circle(debugWindowCanvas,Point(100,115),8,Scalar(255),-1);

                putText(debugWindowCanvas,"Num: ",Point(10,150),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                putText(debugWindowCanvas,to_string(armorDetectorPtr->armorNumber),Point(100,150),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

                putText(debugWindowCanvas,"Fre: ",Point(10,180),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                putText(debugWindowCanvas,to_string(frequency),Point(100,180),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

                if(armorDetectorPtr->findState)
                    rectangle(debugWindowCanvas,Rect(10,225,50,50),Scalar(255),-1);

                if(armorDetectorPtr->IsSmall())
                    putText(debugWindowCanvas,"S",Point(110,255),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                else
                    putText(debugWindowCanvas,"B",Point(110,255),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

                if(curControlState)
                    putText(debugWindowCanvas,"B",Point(210,255),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);
                else
                    putText(debugWindowCanvas,"R",Point(210,255),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255),1);

                predictX = kalman->p_predictx/5;
                originalX = armorDetectorPtr->targetArmor.center.x/5;

//                printf("Original X:%d\t",originalX);
//                printf("prediect X:%d\n",predictX);
//                pyrDown(debugWindowCanvas,debugWindowCanvas);

                imshow("DEBUG",debugWindowCanvas);

//                waveWindowPanel->DisplayWave2();
#endif

            /*******************************************************************************************************
             * when the armor-detector has not detected target armor successfully, that may be caused by the suddenly
             * movement of robots(myself and the opposite target  robot), but the target armor is still in the view
             * scoop, we still need to instruct the movement of the holder instead of releasing it to the cruise mode
             * ****************************************************************************************************/

            if(showOrigin)
            {
                circle(detectFrame,Point(FRAMEWIDTH/2, FRAMEHEIGHT/2),5,Scalar(255,255,255),-1);

                if(FRAMEHEIGHT > 1000)
                {
                    //pyrDown(detectFrame,detectFrame);
                    //pyrDown(detectFrame,detectFrame);
                }
                imshow("detect",detectFrame);
            }



            /****************************************Control Auxiliary**********************************************
             1. Adjust the target Angle of the holder according to the distance D from the center of the mounting deck
             to the center of the image. When D is large, set the target Angle of the holder to be larger; when D is
             small, set the target Angle of the holder to be smaller, so as to indirectly control the rotation speed
             of the holder.

            2. Through the retreat algorithm, when the assembly deck is not found, the current offset Angle is set to
             1/M of the previous frame offset Angle, which can prevent the jitter of the holder caused by intermittent
             unrecognition to a certain extent.

             3. Judge whether the holder dither on the YAW axis by the dispersion of the deflection Angle of the holder's
             YAW axis. If so, reduce the target Angle of the holder and indirectly slow down the rotation speed of the
             holder.
             *******************************************************************************************************/

            if(!armorDetectorPtr->findState)
            {
                //LOGE("lost_target_count : %d\n", ++lost_target_count);
                yawTran /= 1.2;
                pitchTran /= 1.2;

                /** reset kalman filter **/
                //kalman->SetKF(Point(0,0),true);

                if(fabs(yawTran) > 0.1 && fabs(pitchTran) > 0.1)
                    armorDetectorPtr->findState = true;
            }
            else
            {
                yawTran = solverPtr->yaw - 23;
                pitchTran = solverPtr->pitch + 16.5 + solverPtr->dist/7500;
            }

            /** update yaw list and yawListCount **/
            yawList[yawListCount++] = yawTran;
            yawListCount = yawListCount%YAW_LIST_LEN;

            {
                /** use feedbackDelta to adjust the speed for holder to follow the armor**/

                yawDeviation = stdDeviation(yawList, YAW_LIST_LEN);
                avg_yaw = average(yawList, YAW_LIST_LEN);

                if(direction == 0)
                {
                    if( yawDeviation < 0.7 && avg_yaw > 0.2)
                        yawOffset = -1.5;
                    else if(yawDeviation < 0.7 && avg_yaw < -0.2)
                        yawOffset = 1.5;
                    else
                        yawOffset = 0;
                }
                else if(direction == 1 || (yawDeviation < 0.7 && avg_yaw > 0.2))
                {
                    yawOffset = -1.5;
                }else if(direction == 2 || (yawDeviation < 0.7 && avg_yaw < -0.2))
                {
                    yawOffset = 1.5;
                }
            }

            if(armorDetectorPtr->findState&&((fabs(yawTran) <=  5) || yawDeviation < 1.5))
                solverPtr->shoot = true;
            else
                solverPtr->shoot = false;
#if SAVE_LOG == 1
            logWrite<<"[Shoot Command] : "<<solverPtr->shoot<<endl;
#endif
            //cout<<solverPtr->shoot<<endl;
            //cout<<armorDetectorPtr->findState<<" "<<yawTran<<" "<<pitchTran<<" "<<yawDeviation<<endl;
            /** package data and prepare for sending data to lower-machine **/
            if(carName != HERO)
                serialPtr->pack(receiveData.yawAngle + yawTran + yawOffset,receiveData.pitchAngle + pitchTran, solverPtr->dist, solverPtr->shoot,
                                armorDetectorPtr->findState, AUTO_SHOOT_STATE,0);
            else
            {
#ifdef REALSENSE
                dynamic_cast<RealSenseDriver*>(driver)->measure(armorDetectorPtr->targetArmor.rect);
                serialPtr->pack(receiveData.yawAngle + feedbackDelta*yawTran,receiveData.pitchAngle + pitchTran,1000*static_cast<RealSenseDriver*>(driver)->dist2Armor, solverPtr->shoot,
                                armorDetectorPtr->findState, AUTO_SHOOT_STATE,0);
#endif
            }

            feedbackDelta = 1;
#if DEBUG_MSG == 1
            LOGM("Write Data\n");
#endif
        }
        else
        {
            solverPtr->GetPoseV(energyPtr->predict_pts,false);
            //cout << "by pnp : " << solverPtr->yaw << "\t" << solverPtr->pitch << endl;
            //solverPtr->GetPoseSH(energyPtr->target_point);
            //cout << "by small hole : " << solverPtr->yaw << "\t" << solverPtr->pitch << endl;
            /* do energy things */
            if (showEnergy)
            {
                circle(energyFrame, Point(FRAMEWIDTH / 2, FRAMEHEIGHT / 2), 2, Scalar(0, 255, 255), 3);

                putText(energyFrame, "distance: ", Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2, 8, 0);
                putText(energyFrame, to_string(solverPtr->dist), Point(150, 30), cv::FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 2, 8, 0);

                putText(energyFrame, "yaw: ", Point(0, 60), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2, 8, 0);
                putText(energyFrame, to_string(solverPtr->yaw), Point(80, 60), cv::FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 2, 8, 0);

                putText(energyFrame, "pitch: ", Point(0, 90), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2, 8, 0);
                putText(energyFrame, to_string(solverPtr->pitch), Point(100, 90), cv::FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 2, 8, 0);

                putText(energyFrame, "detecting:  ", Point(0, 180), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2, 8, 0);
                if (energyPtr->detect_flag){
                    circle(energyFrame, Point(165, 175), 4, Scalar(255, 255, 255), 3);
                    for (int i = 0; i < 4; i++) {
                        line(energyFrame, energyPtr->pts[i], energyPtr->pts[(i + 1) % (4)],
                             Scalar(255, 255, 255), 2, LINE_8);
                    }
                    circle(energyFrame, energyPtr->target_point, 2, Scalar(0, 255, 0), 3);
                    circle(energyFrame, energyPtr->circle_center_point, 3, Scalar(255, 255, 255), 3);
                    circle(energyFrame, energyPtr->predict_point, 2, Scalar(100, 10, 255), 3);
                }


                imshow("energy", energyFrame);
                waitKey(1);
            }
            yaw_abs = receiveData.yawAngle - solverPtr->yaw; //绝对yaw角度
            pitch_abs = receiveData.pitchAngle + solverPtr->pitch; //绝对pitch角度

            serialPtr->pack(yaw_abs,
                            pitch_abs,
                            solverPtr->dist,
                            solverPtr->shoot,
                            energyPtr->detect_flag,
                            curControlState,
                            0);
            //cout << yaw_abs << "\t" << pitch_abs << endl;
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

        /**press key 'space' to pause or continue task**/
        if(DEBUG || showOrigin || showEnergy)
        {

            if(!pauseFlag && waitKey(10) == 32){pauseFlag = true;}

            if(pauseFlag)
            {
                while(waitKey() != 32){}
                pauseFlag = false;
            }
        }

        /** send data from host to low-end machine to instruct holder's movement **/
        if(serialPtr->WriteData())
        {
#if SAVE_LOG == 1
            logWrite<<"[Write Data to USB2TTL SUCCEED]"<<endl;
#endif
        }
        else
        {
            //logWrite<<"[Write Data to USB2TTL FAILED]"<<endl;
        }

        /** Receive data from low-end machine to update parameters(the color of robot, the task mode, etc) **/
//        if(serialPtr->ReadData(receiveData))
//        {
//#if DEBUG_MSG == 1
//            LOGM("Receive Data\n");
//#endif
//            /** Update task mode, if receiving data failed, the most reasonable decision may be just keep the status
//             * as the last time **/
//            curControlState = receiveData.targetMode;
//
//            /** because the logic in armor detection task need the color of enemy, so we need to negate to color variable
//             * received, receiveData.targetColor means the color of OUR robot, but not the enemy's **/
//            blueTarget = (receiveData.targetColor) == 0;
//
//            direction = +static_cast<int>(receiveData.direction);
//
//#if SAVE_LOG == 1
//            logWrite<<"[Feedback Time Consume] : "<<((double)getTickCount() - taskTime)/getTickFrequency()<<endl;
//                logWrite<<"[Total Time Consume] : "<<((double)getTickCount() - timeFlag)/getTickFrequency()<<endl;
//                //logWrite<<"[Direction] :" <<receiveData.direction<<endl;
//                //logWrite<<"[Target Color] : "<<blueTarget<<endl;
//                logWrite<<"[Current Yaw Angle] : "<<receiveData.yawAngle<<endl;
//                logWrite<<"[Current Pitch Angle] : "<<receiveData.pitchAngle<<endl;
//#endif
//
//#if DEBUG_MSG == 1
//            LOGM("BlueTarget: %d\n",(int)blueTarget);
//#endif
//        }
//        else
//        {
//#if SAVE_LOG == 1
//            logWrite<<"[Receive Data from USB2TTL FAILED]"<<endl;
//#endif
//        }
#if SHOWTIME == 1
        cout << "FeedBack Mission Cost : " << CalWasteTime(taskTime,freq) << " ms" << endl;
#endif
    }

    void ImgProdCons::Receive()
    {
        serialPtr->ReadData(receiveData);
    }
}
