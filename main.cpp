#include <thread>
#include "MyThread.hpp"
#include "preoptions.h"
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
    if(pro.Init()){
        while(1){
            pro.Produce();
            pro.Detect();
            pro.Feedback();
            cout << endl;
            if(!pro.GrabFlag) break;
        }
    }

    return 0;
}
