//
// Created by sentrytop on 22-8-3.
//

#include "NumberClassifier.h"

namespace rm{
    NumberClassifier::NumberClassifier() {
        LoadSvmModel(SVM_PARAM_PATH);
    }

    void NumberClassifier::LoadSvmModel(const char *model_path) {
        svm = cv::ml::StatModel::load<cv::ml::SVM>(model_path);
        if (svm.empty()) {
            cout << "Svm load error! Please check the path!" << endl;
            exit(0);
        }
    }

    int NumberClassifier::SVMClassifier(cv::Mat &num_roi) {
        if(num_roi.rows == NUM_IMG_SIZE && num_roi.cols == NUM_IMG_SIZE){
            num_roi.reshape(0, 1).convertTo(svm_param_matrix, CV_32F, 1.0 / 255);
            int number = lround(svm->predict(svm_param_matrix));
            return number;
        } else {
            return -1;
        }
    }
}