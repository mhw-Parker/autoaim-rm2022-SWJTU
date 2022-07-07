#include <thread>
#include "MyThread.hpp"
#include "preoptions.h"
#include "mydefine.h"
#include "config.h"

using namespace rm;
using namespace cv;
using namespace std;

pthread_t producePThreadHandler;
pthread_t detectPThreadHandler;
pthread_t energyPThreadHandler;
pthread_t feedbackPThreadHandler;

int main(int argc, char** argv)
{
    PreOptions(argc,argv);
    ImgProdCons pro;

    /**  multi thread  **/
    pro.Init();

    std::thread receiveThread(&rm::ImgProdCons::Receive, &pro);   // 接收数据线程
    std::thread frameThread(&rm::ImgProdCons::Produce, &pro);     // 图像采集线程
    std::thread detectThread(&rm::ImgProdCons::Detect, &pro);     // 检测线程
    std::thread feedbackThread(&rm::ImgProdCons::Feedback, &pro); // 反馈线程

    if(saveVideo) {
        std::thread recordThread(&rm::ImgProdCons::Record, &pro);
        recordThread.join();
    }

    receiveThread.join();
    frameThread.join();
    detectThread.join();
    feedbackThread.join();

    /**  multi thread  **/
    return 0;
}
