//
// Created by sentrytop on 22-8-3.
//

#include "NumberClassifier.h"

namespace rm{
    NumberClassifier::NumberClassifier() {
        LoadSvmModel(SVM_PARAM_PATH);
        LoadOnnxModel("../Detector/Armor/model/model_nin.onnx");
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
    /**
     * @Author ChenZhen
     * @brief use onnx to inference
     * */
    void NumberClassifier::LoadOnnxModel(const string &model_path) {
        net = cv::dnn::readNetFromONNX(model_path);
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    }
    /**
     * @brief top task
     * */
    int NumberClassifier::FigureDetection(cv::Mat &num_gray_roi) {
        Mat inputBlob= dnn::blobFromImage(num_gray_roi,1);
        net.setInput(inputBlob);
        vector<float> detectionMat = net.forward();
        Mat dec=net.forward();
        int num=0;
        for(int i=0;i<detectionMat.size();i++){
            if(detectionMat[i]> detectionMat[num])
                num=i;
        }
        return num;
    }

}