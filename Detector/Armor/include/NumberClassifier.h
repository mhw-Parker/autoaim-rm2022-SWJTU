//
// Created by sentrytop on 22-8-3.
//

#ifndef MASTER_NUMBERCLASSIFIER_H
#define MASTER_NUMBERCLASSIFIER_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/dnn.hpp>

#include <iostream>

#include "mydefine.h"

using namespace std;
using namespace cv;

namespace rm{
    class NumberClassifier{
    public:
        NumberClassifier();
        int SVMClassifier(cv::Mat &num_roi);
        int FigureDetection(cv::Mat &num_gray_roi, float &conf);

    private:
        void LoadSvmModel(const char *model_path);
        void LoadOnnxModel(const string &model_path);
        cv::dnn::Net net;
        cv::Mat svm_param_matrix;
        cv::Ptr<cv::ml::SVM> svm;
    };
}


#endif //MASTER_NUMBERCLASSIFIER_H
