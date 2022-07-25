//
// Created by root on 2021/1/14.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <mutex>
#include <memory>
#include <shared_mutex>
#include <condition_variable>
#include <sys/timeb.h>
#include <sys/syscall.h>
#include <fstream>

#include "ArmorDetector.hpp"
#include "SerialPort.hpp"
#include "preoptions.h"
#include "mydefine.h"
#include "EnergyDetector.h"
#include "V4L2KAS.h"
#include "VideoDriver.hpp"
#include "ImageWriter.hpp"
#include "utility.hpp"
#include "Predictor/Predictor.h"
#include "Fifo.h"

#ifdef DAHUA
#include "Media/RMDriver.h"
#endif

#ifdef MIND
#include "MindDriver.h"
#endif

#ifdef REALSENSE
#include "RealSenseDriver.h"
#endif

using namespace RMTools;
using namespace std;
using namespace V4L2KAS;

namespace rm
{
    /** create a new structure in order to synchronize the imu data **/
     class timeStampMat {
     public:
         timeStampMat(Mat src, float t, ReceiveData data) : frame(src),stamp(t), mcuData(data){};
         Mat frame;
         float stamp;
         ReceiveData mcuData;
     };
     class sendData {
     public:

     };

    class ImgProdCons
    {
    public:
        ImgProdCons();

        ~ImgProdCons();

        /*initialize*/
        bool Init();

        /**
        * @Brief: Receive self state from the serial port, update task mode if commanded
        */
        void Feedback();

        /**
        * @Brief: keep reading image from the camera into the buffer
        */
        void Produce();

        /**
         * @brief: mainMission
         * */
        void Detect();

        /**
         * @Brief: receive data from lower machine
         */
        void Receive();

        void Record();

        /**
         * @brief show debug images
         * */
        void ShowImage();

        bool GrabFlag = true;

    public:
        /* Camera */
        Driver *driver;
    private:
        /**
         * @Brief: detect armor
         */
        void Armor();

        /**
         * @Brief: detect fan
         */
        void Energy();
        /*
        * To prevent camera from dying!
        */
        static bool quitFlag;
        static void SignalHandler(int);
        static void InitSignals(void);

        double taskTime;
        // 每个线程时间
        double produceTime, detectTime, feedbackTime, receiveTime, showImgTime;
        // 纯识别时间
        double recognitionTime;
        // 纯预测时间
        double predictionTime;
        // getTickFrequency()
        double freq;

        /***/
        double last_mission_time = 0.025; //上一次任务总体耗时
        int time_cnt;
        /***/
        /** new add thread fifo **/
        FIFO<timeStampMat> frame_fifo;
        FIFO<ReceiveData> receive_fifo;
        FIFO<Mat> show_fifo;
        FIFO<SendData> send_fifo;
        bool com_flag = true;

        /* 保存图像相关 */
        string saveVideoPath;
        VideoWriter videoWriter;

        string saveTimeSeriesPath;
        std::ofstream timeWrite;

        std::ifstream readTimeTxt;

        string SVMPath;
        int svm_img_num = 0;
        int save_img_cnt = 0;

        /**/
        int cnt1 = 0;
        float tt;

        /*Camera Driver Instanc -es*/
#ifdef DAHUA
        RMDriver dahuaCapture;
#endif
#ifdef MIND
        MindDriver mindCapture;
#endif
        V4L2Driver v4l2Capture;
        VideoDriver videoCapture;
        ImageDriver imageCapture;

        /* Serial */
        std::unique_ptr<Serial> serialPtr;

        /* Armor detector */
        std::unique_ptr<ArmorDetector> armorDetectorPtr;

        /*EnergyDetector buffer detector*/
        std::unique_ptr<EnergyDetector> energyPtr;

        std::unique_ptr<Predictor> predictPtr;
        //Predictor predictPtr;

        Mat frame;
        Mat detectFrame;
        Mat energyFrame;
        Mat show_img;

        float yaw_abs;
        float pitch_abs;
        Vector3f target_ypd;
        Vector3f last_xyz; //用于存储丢目标前最后一帧的目标位置
        float v_bullet{};
        RMTools::DisPlayWaveCLASS showWave;

        bool find_state;
        uint8_t car_num = 0;
        struct ReceiveData receiveData;
        int armorType;

        int missCount;

    private:
        double startT;
        float time_stamp = 0 , last_stamp = 0;
        long int cnt = 0, last_cnt = 0;

        vector<Point2f> target_pts;
        Point2f rotate_center;
    };

}
