//
// Created by sentrytop on 22-8-3.
//

#include "NumberClassifier.h"

namespace rm{
    NumberClassifier::NumberClassifier() {
        LoadSvmModel(SVM_PARAM_PATH);
        LoadOnnxModel("../Detector/Armor/model/model_nin.onnx");
    }
    /**
     * @brief svm number detect
     * */
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
     * @brief use onnx to inference number
     * */
    void NumberClassifier::LoadOnnxModel(const string &model_path) {
        net = cv::dnn::readNetFromONNX(model_path);
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    }
    /**
     * @brief top task
     * @param num_gray_roi input image 20*20
     * */
    int NumberClassifier::FigureDetection(cv::Mat &num_roi, float &conf) {
        if(num_roi.rows == NUM_IMG_SIZE && num_roi.cols == NUM_IMG_SIZE){
            Mat inputBlob= dnn::blobFromImage(num_roi,1);
            net.setInput(inputBlob);
            vector<float> detectionMat = net.forward();
            Mat dec = net.forward();
            int num=0;
            for(int i = 0; i < detectionMat.size(); i++){
                if(detectionMat[i]> detectionMat[num])
                    num = i;
            }
            // confidence
            vector<float> softmax_detect;
            float softmax_sum = 0;
            for(int i=0;i<detectionMat.size();i++){
                softmax_detect.push_back(detectionMat[i]/255.F);
                softmax_sum += exp(softmax_detect[i]);
            }
            conf = exp(softmax_detect[num])/softmax_sum;
            return num;
        } else {
            return -1;
        }

    }

}