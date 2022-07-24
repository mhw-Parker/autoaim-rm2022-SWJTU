#include "ArmorDetector.hpp"
#include <sstream>
#include <algorithm>
#include <thread>

namespace rm {
    /**
    * @brief Armor constructor
    * @param [L1] one lamp of the armor
    * @param [L2] another lamp of the armor
    * @return none
    * @details none
    */
    Armor::Armor(Lamp L1, Lamp L2, double priority_) {
        errorAngle = fabs(L1.lightAngle - L2.lightAngle);
        armorWidth = fabs(static_cast<int>(L1.rect.center.x - L2.rect.center.x));
        armorHeight = fabs(static_cast<int>((L1.rect.size.height + L2.rect.size.height) / 2));
        center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x) / 2);
        center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y) / 2);
        rect = Rect(center - Point2i(armorWidth / 2, armorHeight / 2),
                    Size(armorWidth, armorHeight));
        armorType = (armorWidth / armorHeight > 1.5) ? (BIG_ARMOR) : (SMALL_ARMOR);
        priority = priority_;
        avgRSubBVal = (L1.avgRSubBVal + L2.avgRSubBVal) / 2;

        //need to make sure how to set values to the points
        pts.resize(4);
        Point2f pts_[4];

        if (L1.rect.center.x < L2.rect.center.x) {
            L1.rect.points(pts_);
            if (L1.lightAngle < 0) {
                pts[0] = Point2f((pts_[0] + pts_[3]) / 2);
                pts[3] = Point2f((pts_[1] + pts_[2]) / 2);
            } else {
                pts[3] = Point2f((pts_[0] + pts_[3]) / 2);
                pts[0] = Point2f((pts_[1] + pts_[2]) / 2);
            }

            L2.rect.points(pts_);
            if (L2.lightAngle < 0) {
                pts[1] = Point2f((pts_[0] + pts_[3]) / 2);
                pts[2] = Point2f((pts_[1] + pts_[2]) / 2);
            } else {
                pts[2] = Point2f((pts_[0] + pts_[3]) / 2);
                pts[1] = Point2f((pts_[1] + pts_[2]) / 2);
            }

        } else {
            L2.rect.points(pts_);
            if (L2.lightAngle < 0) {
                pts[0] = Point2f((pts_[0] + pts_[3]) / 2);
                pts[3] = Point2f((pts_[1] + pts_[2]) / 2);
            } else {
                pts[3] = Point2f((pts_[0] + pts_[3]) / 2);
                pts[0] = Point2f((pts_[1] + pts_[2]) / 2);
            }

            L1.rect.points(pts_);
            if (L1.lightAngle < 0) {
                pts[1] = Point2f((pts_[0] + pts_[3]) / 2);
                pts[2] = Point2f((pts_[1] + pts_[2]) / 2);
            } else {
                pts[2] = Point2f((pts_[0] + pts_[3]) / 2);
                pts[1] = Point2f((pts_[1] + pts_[2]) / 2);
            }
        }
    }

    Armor::Armor(Rect &rect) {
        errorAngle = 0;
        center = rect.tl() + Point(rect.width / 2, rect.height / 2);
        this->rect = rect;

        pts.resize(4);

        pts[0] = rect.tl();
        pts[1] = rect.tl() + Point(rect.width, 0);
        pts[2] = rect.tl() + Point(rect.width, rect.height);
        pts[3] = rect.tl() + Point(0, rect.height);

        armorWidth = rect.width;
        armorHeight = rect.height;

        armorType = (armorWidth / armorHeight > 1.5) ? (BIG_ARMOR) : (SMALL_ARMOR);

        priority = 0;

    }

    /**
    * @brief ArmorDetector constructor
    * @param none
    * @return none
    */
    ArmorDetector::ArmorDetector() {
        findState = false;
        isSmall = false;
    }

    /**
     * @brief function used to initialize the armor instance
     * @param none
     * @return none
    */
    void Armor::init() {
        errorAngle = 0;
        armorWidth = 0;
        armorHeight = 0;
        armorType = BIG_ARMOR;
        priority = 10000;
    }

    /**
     * @brief function used to initialize the Armor detector instance
     * @param none
     * @return none
    */
    void ArmorDetector::Init() {
        targetArmor.init();
        lastTarget.init();
        roiRect = Rect(0, 0, FRAMEWIDTH, FRAMEHEIGHT);
        findState = false;
        detectCnt = 0;
        lostCnt = 15;
        armorNumber = 0;
        LoadSvmModel(SVM_PARAM_PATH, Size(SVM_IMAGE_SIZE, SVM_IMAGE_SIZE));
        InitDetectionPrams();
        find_not_engineer = false;
    }

    /**
     * 初始化识别参数
     */
    void ArmorDetector::InitDetectionPrams() {
        FileStorage fs("../Detector/Armor/Armor.yaml", FileStorage::READ);

        string whose_param;
        if (carName == SENTRYDOWN || carName == SENTRYTOP) {
            whose_param = "sentry";
        } else {
            whose_param = "normal";
        }
        FileNode param_set = fs[whose_param];
        // 灯条识别参数
        param_set["minPointNum"] >> param.minPointNum;
        param_set["minLightArea"] >> param.minLightArea;
        param_set["maxLightArea"] >> param.maxLightArea;
        param_set["maxLightAngle"] >> param.maxLightAngle;
        param_set["minLightH2W"] >> param.minLightH2W;
        param_set["maxLightH2W"] >> param.maxLightH2W;
        param_set["maxLightW"] >> param.maxLightW;
        param_set["minLightH"] >> param.minLightH;
        param_set["maxLightH"] >> param.maxLightH;
        param_set["minAverageBrightness"] >> param.minAverageBrightness;
        param_set["maxAverageBrightness"] >> param.maxAverageBrightness;
        // 灯条匹配参数
        param_set["maxAngleError"] >> param.maxAngleError;
        param_set["maxLengthError"] >> param.maxLengthError;
        param_set["minRatio"] >> param.minRatio;
        param_set["maxRatio"] >> param.maxRatio;
        param_set["maxArmorAngle"] >> param.maxArmorAngle;
        param_set["maxDeviationAngle"] >> param.maxDeviationAngle;
    }

    /**
    * @brief: top level detection task
    * @param [img] the image from camera or video that to be processed
    * @return: if ever found armors in this image, return true, otherwise return false
    * @details: none
    */
    bool ArmorDetector::ArmorDetectTask(Mat &img_) {
        GetRoi(img_); //get roi

        Preprocess(imgRoi);

        DetectArmor(img_);

        if (!showArmorBox) {
            printf("----- Armor Detector Info -----\n");
            if (findState) {
                printf("Target Number: %d\n", armorNumber);
            } else
                printf("No target!\n\n");
        }

        return findState;
    }

    /**
    * @brief get he region of interest
    * @param [img] the image from camera or video that to be processed
    * @return none
    * @details none
    */
    void ArmorDetector::GetRoi(const Mat &img) {
        Rect rectTemp = roiRect;
        if (!findState || rectTemp.width == 0 || rectTemp.height == 0) {
            roiRect = Rect(0, 0, FRAMEWIDTH, FRAMEHEIGHT);
        } else if (detectCnt > 0) {
            float scale = 3;
            int w = int(rectTemp.width * scale);
            int h = int(rectTemp.height * scale);
            int x = int(rectTemp.x - (w - rectTemp.width) * 0.5);
            int y = int(rectTemp.y - (h - rectTemp.height) * 0.5);
            roiRect = Rect(x, y, w, h);
            if (!MakeRectSafe(roiRect, Size(FRAMEWIDTH, FRAMEHEIGHT))) {
                roiRect = Rect(0, 0, FRAMEWIDTH, FRAMEHEIGHT);
            }
        }
        imgRoi = img(roiRect);
    }

    /**
    * @brief pre-procession of an image captured
    * @param img the ROI image that clipped by the GetRIO function
    * @param type choose to Preprocess the current image or the lastest two images, when the type is true, parameter
    * img must be the origin image but not the roi image
    * @return none
    * @details if average value in a region of the colorMap is larger than 0, then we can inference that in this region
    * the light is more possible to be red; the reason for why we not just subtract the red channel with blue channel is
    * that the center of lamps always be white and the result of the subtraction is always small. And we have tested that
    * if we use canny to detect the edge of lamps for reducing the computing burden of findContours() function which works
    * for GPU, the result shows that edge detection costs more time(using same video, without edge detection, task costs
    * 4.19548ms;with edge detection, task costs 4.27403ms)
    */
    void ArmorDetector::Preprocess(Mat &img) {
        vector<Mat> channels;
        split(img, channels);
        cvtColor(img, gray, COLOR_BGR2GRAY);
        if (blueTarget)
            subtract(channels[0], channels[2], sub);
        else
            subtract(channels[2], channels[0], sub);
        //imshow("channels-sub",sub);
        threshold(sub, sub, 120, 255, THRESH_BINARY);
        threshold(gray, thresholdMap, 40, 255, THRESH_BINARY);
        colorMap = Mat_<int>(sub);
//        imshow("channels-sub-binary",sub);
//        imshow("gray-binary",thresholdMap);
    }

    /**
    * @brief: detect possible armors
    * @param [img]  the image from camera or video that to be processed
    * @return: if ever found armors in this image, return true, otherwise return false
    * @details: none
    */
    bool ArmorDetector::DetectArmor(Mat &img) {
        findState = false;
        armorNumber = 0;
        if (showBinaryImg) {
            imshow("binary_brightness_img", thresholdMap);
        }

        vector<Lamp> lights;
        // 检测灯条
        lights = LampDetection(imgRoi);
        // 画灯条
        if (showLamps) {
            for (auto &light: lights) {
                Point2f rect_point[4]; //
                light.rect.points(rect_point);
                for (int j = 0; j < 4; j++) {
                    line(img, rect_point[j] + Point2f(roiRect.x, roiRect.y),
                         rect_point[(j + 1) % 4] + Point2f(roiRect.x, roiRect.y),
                         Scalar(0, 255, 255), 1);
                }
                vector<int> data{(int) (light.rect.size.height * light.rect.size.width / 2),
                                 (int) (light.rect.size.height / light.rect.size.width / 2),
                                 (int) light.rect.size.height / 2,
                                 (int) light.rect.size.width,
                                 (int) light.lightAngle,
                                 (int) light.avgRSubBVal};
                Point2f corner = Point2f(roiRect.x + light.rect.center.x + light.rect.size.width / 4,
                                         roiRect.y + light.rect.center.y - light.rect.size.height / 4);
                // 面积，高宽比，高，宽，角度，通道相减图平均权值
                for (int j = 0; j < data.size(); j++) {
                    putText(img, to_string(data[j]), corner + Point2f(0, 20 * j),
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255),
                            2);
                }
            }
        }

        vector<MatchLight> candidate_armors;
        // 匹配灯条
        candidate_armors = MatchLights(lights);

        if (findState) {
            // 寻找最优的目标
            GetOptimalMatch(candidate_armors, lights);

            lostState = false;
            detectCnt++;
            lostCnt = 0;

            MakeRectSafe(targetArmor.rect, img.size());

            targetArmor.rect = targetArmor.rect + Point(roiRect.x, roiRect.y);
            targetArmor.center += Point(roiRect.x, roiRect.y);

            for (int i = 0; i < 4; i++) {
                targetArmor.pts[i] = targetArmor.pts[i] + Point2f(roiRect.x, roiRect.y);
            }

            // 画装甲板
            if (showArmorBox) {
                rectangle(img, roiRect, Scalar(255, 255, 255), 1);
                for (int j = 0; j < 4; j++) {
                    line(img, targetArmor.pts[j], targetArmor.pts[(j + 1) % 4],
                         Scalar(255, 0, 255), 1);
                }
                circle(img, targetArmor.center, 5, Scalar(0, 0, 255), -1);
            }

            /**update roi rect, last armor, average of lamps' R channel subtract B channel value**/
#if USEROI == 1
            roiRect = targetArmor.rect;
#endif

            lastTarget = targetArmor;
            if (showArmorBox) {
                putText(img, "id:", Point(roiRect.x, roiRect.y), cv::FONT_HERSHEY_PLAIN, 2, Scalar(255, 62, 191), 1, 5,
                        0);
                putText(img, to_string(armorNumber), Point(roiRect.x + 35, roiRect.y), cv::FONT_HERSHEY_PLAIN, 2,
                        Scalar(255, 62, 191), 1, 5, 0);
            }
            // 为防止大小装甲板状态抖动，从第五次识别开始，每10次检测一次
            if (detectCnt % 10 == 5) {
                switch (armorNumber) {
                    case 1:
                    case 6:
                        targetArmor.armorType = BIG_ARMOR;
                        break;
                    default:
                        Armor(targetArmor.rect);
                        break;
                }
            }
            return true;
        } else {
            detectCnt = 0;
            lostCnt++;
            if (lostCnt > max_lost) lostState = true;
            cout << "Lost: " << lostCnt << endl;
            return false;
        }

    }

    /**
     * 新版灯条检测
     * @param img
     * @return
     */
    vector<Lamp> ArmorDetector::LampDetection(Mat &img) {
        Mat_<int> lampImage;
        Scalar_<float> avgBrightness;
        RotatedRect possibleLamp;
        Rect rectLamp;
        vector<Lamp> lampVector;
        vector<vector<Point>> contoursLight;

        findContours(thresholdMap, contoursLight, RETR_EXTERNAL, CHAIN_APPROX_NONE);

#pragma omp parallel for  // enable multi-thread run for "for"
        for (auto &i: contoursLight) {
            if (i.size() < param.minPointNum) continue;
            // 1：灯条周长
//            double length = arcLength(i, true);
//            if (length < 10 || length > 800) continue;
            //用椭圆近似形状
            possibleLamp = fitEllipse(i);
            //ellipse(img, possibleLamp, Scalar::all(255));
            // 2：面积
            float lampArea = possibleLamp.size.width * possibleLamp.size.height;
            if ((lampArea > param.maxLightArea) || (lampArea < param.minLightArea)) continue;
            // 3：高，宽
            if (possibleLamp.size.width > param.maxLightW ||
                possibleLamp.size.height > param.maxLightH ||
                possibleLamp.size.height < param.minLightH)
                continue;
            // 4：长宽比例
            float rate_height2width = possibleLamp.size.height / possibleLamp.size.width;
            if ((rate_height2width < param.minLightH2W) || (rate_height2width > param.maxLightH2W)) continue;
            // 5：角度
            float angle_ = (possibleLamp.angle > 90.0f) ? (possibleLamp.angle - 180.0f) : (possibleLamp.angle);
            if (fabs(angle_) >= param.maxLightAngle) continue; //由于灯条形状大致为矩形，将矩形角度限制在 0 ~ 90°
            // 6：矩形区域内平均权值
            rectLamp = possibleLamp.boundingRect(); //根据椭圆得出最小正矩形
            MakeRectSafe(rectLamp, colorMap.size()); //防止灯条矩形越出画幅边界
            mask = Mat::ones(rectLamp.height, rectLamp.width, CV_8UC1); //矩形灯条大小的全1灰度图
            lampImage = colorMap(rectLamp);
            avgBrightness = mean(lampImage, mask); //求均值
            if (avgBrightness[0] < param.minAverageBrightness || avgBrightness[0] > param.maxAverageBrightness)
                continue;
            // 整合所有符合条件的灯条信息
            Lamp lampWithInfo(possibleLamp, angle_, avgBrightness[0]);
            lampVector.emplace_back(lampWithInfo);
        }
        return lampVector;
    }

    /**
     * 寻找符合条件的匹配灯条，此过程不考虑数字
     * @param lights
     * @return 匹配灯条的vector
     */
    vector<MatchLight> ArmorDetector::MatchLights(vector<Lamp> &lights) {
        static float deviationAngle;
        static float nW, nH;
        static float dAngle;
        static float contourLen;
        static float ratio;
        static float nAngle;
        static float dAvgB;
        vector<MatchLight> matchLights;

        //过滤器
        for (auto i = 0; i < lights.size() - 1; i++) {
            for (auto j = i + 1; j < lights.size(); j++) {
                /* the difference between two angles 灯条角度 */
                dAngle = fabs(lights[i].lightAngle - lights[j].lightAngle);
                if (dAngle > param.maxAngleError) continue;

                /*the difference ratio of the two lights' height 灯条长度比例差 */
                contourLen = abs(lights[i].rect.size.height - lights[j].rect.size.height) /
                             (lights[i].rect.size.height + lights[j].rect.size.height) / 2;
                if (contourLen > param.maxLengthError) continue;

                /*the average height of two lights(also the height of the armor defined by these two lights)*/
                nH = (lights[i].rect.size.height + lights[j].rect.size.height) / 2;

                /*the width between the center points of two lights*/
                nW = fabs(sqrt(pow(lights[i].rect.center.x - lights[j].rect.center.x, 2) +
                               pow(lights[i].rect.center.y - lights[j].rect.center.y, 2)));

                /*Ratio of the width and the height in the image */
                ratio = nW / nH;
                if (ratio > param.maxRatio || ratio < param.minRatio) continue;

                /*anyway, the difference of the lights' angle is tiny,so anyone of them can be the angle of the armor*/
                nAngle = fabs((lights[i].lightAngle + lights[j].lightAngle) / 2);
                if (nAngle > param.maxArmorAngle) continue;

                /*the deviation angle of two lamps*/
                deviationAngle = fabs(atan((lights[i].rect.center.y - lights[j].rect.center.y)
                                           / (lights[i].rect.center.x - lights[j].rect.center.x))) * 180 / CV_PI;
                if (deviationAngle > param.maxDeviationAngle) continue;

                /*difference of average brightness*/
                dAvgB = abs(lights[i].avgRSubBVal - lights[j].avgRSubBVal) /
                        (lights[i].avgRSubBVal + lights[j].avgRSubBVal) / 2;

                // 距离目标中心点的距离
                auto center = (lights[i].rect.center + lights[j].rect.center) / 2;
                float center_distance = sqrt(pow(fabs(center.x - FRAMEWIDTH / 2), 2) +
                                             pow(fabs(center.y - FRAMEHEIGHT / 2), 2));
                float center_ratio = center_distance / sqrt(pow(FRAMEHEIGHT / 2, 2) +
                                                            pow(FRAMEWIDTH / 2, 2));

                /*The match factor is still rough now. A formula is more reasonable. */
                float match_factor_;
                if (carName == SENTRYTOP || carName == SENTRYDOWN) {
                    match_factor_ = contourLen + dAvgB +
                                    dAngle / param.maxAngleError + deviationAngle / param.maxDeviationAngle +
                                    exp(MIN(fabs(ratio - 1.2), fabs(ratio - 2.2)));
                } else {
                    match_factor_ = center_ratio + contourLen + dAvgB +
                                    dAngle / param.maxAngleError + deviationAngle / param.maxDeviationAngle +
                                    MIN(fabs(ratio - 1.2), fabs(ratio - 2.2));
                }

                MatchLight matchLight = MatchLight(i, j, match_factor_, nH);
                matchLights.emplace_back(matchLight);
            }
        }

        /*sort these pairs of lamps by match factor*/
        if (matchLights.empty()) {
            findState = false;
            return matchLights;
        }

        findState = true;

        sort(matchLights.begin(), matchLights.end(), compMatchFactor); //元素从小到大排序

        return matchLights;
    }

    /**
     * 从候选匹配灯条中选择最优目标
     * @param armors
     * @return
     */
    void ArmorDetector::GetOptimalMatch(vector<MatchLight> matchLights, vector<Lamp> &lights) {
#if NUM_RECOGNIZE == 1
        uint8_t mostPossibleLampsIndex1 = matchLights[0].matchIndex1,
                mostPossibleLampsIndex2 = matchLights[0].matchIndex2;
        float curSmallestHeightError = 1000;
        int matchPossibleArmorCount = 0;
        int targetMatchIndex = 0;
        Armor curArmor;

        for (int i = 0; i < matchLights.size(); i++) {
            if (matchLights[i].matchIndex1 == mostPossibleLampsIndex1
                || matchLights[i].matchIndex2 == mostPossibleLampsIndex1
                || matchLights[i].matchIndex1 == mostPossibleLampsIndex2
                || matchLights[i].matchIndex2 == mostPossibleLampsIndex2) {
                curArmor = Armor(lights[matchLights[i].matchIndex1], lights[matchLights[i].matchIndex2],
                                 matchLights[i].matchFactor);
                MakeRectSafe(curArmor.rect, roiRect.size());

                //设置用于svm识别的4点区域
                SetSVMRectPoints(curArmor.pts[0], curArmor.pts[1], curArmor.pts[2], curArmor.pts[3]);

                //armorNumber = GetArmorNumber(); //获得装甲板区域对应的数字
                armorNumber = getArmorNumber(curArmor);

                if (armorNumber != 0 && (armorNumber == 1) || (armorNumber == 3) || (armorNumber == 4)) {
                    targetMatchIndex = i;
                    break;
                }
                if (fabs(matchLights[i].lampHeight - lastTarget.armorHeight) < curSmallestHeightError) {
                    targetMatchIndex = i;
                    curSmallestHeightError = fabs(matchLights[i].lampHeight - lastTarget.armorHeight);
                }
                matchPossibleArmorCount++;
                if (matchPossibleArmorCount == 5)
                    break;
            }
        }

        targetArmor = Armor(lights[matchLights[targetMatchIndex].matchIndex1],
                            lights[matchLights[targetMatchIndex].matchIndex2],
                            matchLights[targetMatchIndex].matchFactor);
#else
        targetArmor = Armor(lights[matchLights[0].matchIndex1], lights[matchLights[0].matchIndex2]\
                            ,matchLights[0].matchFactor);
#endif
        MakeRectSafe(targetArmor.rect, roiRect.size());
#if NUM_RECOGNIZE == 1
//        if(armorNumber == 0)
//        {
//            SetSVMRectPoints(targetArmor.pts[0],targetArmor.pts[1],targetArmor.pts[2],targetArmor.pts[3]);
//            armorNumber = GetArmorNumber();
//        }
#endif
    }


    /**
     * @brief compare two matched lamps' priority
     * @param a matched lamp
     * @param b matched lamp
     * @return Sort the elements from smallest matchFractor to largest matchFractor
     */
    bool compMatchFactor(const MatchLight a, const MatchLight b) {
        return a.matchFactor < b.matchFactor;
    }

    /**
    * @brief make the rect safe
    * @param [rect] a rect may be not safe
    * @param [size] the image size, the biggest rect size
    * @return it will never be false
    * @details none
    */
    inline bool MakeRectSafe(cv::Rect &rect, const cv::Size &size) {
        if (rect.x >= size.width || rect.y >= size.height)rect = Rect(0, 0, 0, 0);
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        return !(rect.width <= 0 || rect.height <= 0);
    }

    /**
     * @brief load SVM parameters
     * @param model_path file path
     * @param armorImgSize size of armor
     * @return none
     */
    void ArmorDetector::LoadSvmModel(const char *model_path, const Size &armorImgSize) {
        svm = StatModel::load<SVM>(model_path);
        if (svm.empty()) {
            cout << "Svm load error! Please check the path!" << endl;
            exit(0);
        }

        svmArmorSize = armorImgSize;

        //set dstPoints (the same to armorImgSize, as it can avoid resize armorImg)
//        dstPoints[0] = Point2f(0, 0);
//        dstPoints[1] = Point2f(armorImgSize.width, 0);
//        dstPoints[2] = Point2f(armorImgSize.width, armorImgSize.height);
//        dstPoints[3] = Point2f(0, armorImgSize.height);
    }

    void ArmorDetector::SetSVMRectPoints(Point2f &lt, Point2f &rt, Point2f &lb, Point2f &rb) {
        srcPoints[0] = lt;
        srcPoints[1] = rt;
        srcPoints[2] = lb;
        srcPoints[3] = rb;
    }

    void ArmorDetector::SetSVMRectPoints(Point2f &&lt, Rect &rectArea) {
        srcPoints[0] = lt + Point2f(rectArea.x, rectArea.y);
        srcPoints[1] = srcPoints[0] + Point2f(rectArea.width, 0);
        srcPoints[2] = srcPoints[0] + Point2f(rectArea.width, rectArea.height);
        srcPoints[3] = srcPoints[0] + Point2f(0, rectArea.height);
    }

    /**
     *
     * @param armor
     * @return
     */
    int ArmorDetector::getArmorNumber(Armor &armor) {
        if (armor.armorType == BIG_ARMOR) {
            dstPoints[0] = Point2f(0, 0);
            dstPoints[1] = Point2f(2 * SVM_IMAGE_SIZE, 0);
            dstPoints[2] = Point2f(2 * SVM_IMAGE_SIZE, SVM_IMAGE_SIZE);
            dstPoints[3] = Point2f(0, SVM_IMAGE_SIZE);
            warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints);
            //warpPerspective to get armorImage
            warpPerspective(gray, warpPerspective_dst, warpPerspective_mat,
                            Size(2 * SVM_IMAGE_SIZE, SVM_IMAGE_SIZE));
            warpPerspective_dst = warpPerspective_dst.colRange(12, 68).clone();
        } else {
            dstPoints[0] = Point2f(0, 0);
            dstPoints[1] = Point2f(SVM_IMAGE_SIZE, 0);
            dstPoints[2] = Point2f(SVM_IMAGE_SIZE, SVM_IMAGE_SIZE);
            dstPoints[3] = Point2f(0, SVM_IMAGE_SIZE);
            warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints);
            //warpPerspective to get armorImage
            warpPerspective(gray, warpPerspective_dst, warpPerspective_mat,
                            Size(SVM_IMAGE_SIZE, SVM_IMAGE_SIZE),
                            INTER_NEAREST, BORDER_CONSTANT, Scalar(0));
            warpPerspective_dst = warpPerspective_dst.colRange(8, 32).clone();
        }
        threshold(warpPerspective_dst, warpPerspective_dst, 1, 255,
                  THRESH_BINARY | THRESH_OTSU);
        // 设置尺寸为40x40，实测发现先放大再下采样有更好的效果
        resize(warpPerspective_dst, warpPerspective_dst, Size(SVM_IMAGE_SIZE, SVM_IMAGE_SIZE));
        // 下采样到20x20
        pyrDown(warpPerspective_dst, warpPerspective_dst, Size(20, 20)); //下采样为20*20

        imshow("svm", warpPerspective_dst);
        //
        warpPerspective_dst.reshape(0, 1).convertTo(svmParamMatrix, CV_32F, 1.0 / 255);
        int number = lround(svm->predict(svmParamMatrix));
        return number;
    }

    /**
    * @brief get the Rect instance that describe the target armor's geometry information
    * @param none
    * @return the Rect instance that describe the target armor's geometry information
    * @details none
    */
    Rect ArmorDetector::GetArmorRect() const {
        return targetArmor.rect;
    }

    /**
    * @brief judge wheter the target armor is small armor or big armor
    * @param none
    * @return if the target is small armor then return true, otherwise return false
    * @details this function just used for simplifying the using of targetArmor.armorType
    */
    bool ArmorDetector::IsSmall() const {
        return (targetArmor.armorType == SMALL_ARMOR);
    }

    /**
     * @brief this function shall to serve for building our own database, unfortunately the database built by this way is
     * not good.
     * @param fileP file descriptor
     * @param selectedIndex1 the index of target armor
     * @param selectedIndex2 the index of target armor, because there may be two or more armors in the field of vision at one time
     */
    void ArmorDetector::saveMatchParam(FILE *fileP, int selectedIndex1, int selectedIndex2) {
        if (fileP == nullptr) {
            perror("file open Error!\n");
        }
        /*this section set for make database*/
    }

    bool ArmorDetector::ModelDetectTask(Mat &frame) {
        inputBlob = blobFromImage(frame, 1 / 255.F, Size(320, 320), Scalar(), true,
                                  false);//输入图像设置，input为32的整数倍，不同尺寸速度不同精度不同

        net.setInput(inputBlob);

        net.forward(outs, outNames);

        boxes.clear();
        classIds.clear();
        confidences.clear();

        for (auto &out: outs) {
            // detected objects and C is a number of classes + 4 where the first 4
            float *data = (float *) out.data;
            for (int j = 0; j < out.rows; ++j, data += out.cols) {
                Mat scores = out.row(j).colRange(5, out.cols);
                minMaxLoc(scores, nullptr, &confidence, 0, &classIdPoint);
                if (confidence > 0.5) {
                    int centerX = (int) (data[0] * frame.cols);
                    int centerY = (int) (data[1] * frame.rows);
                    int width = (int) (data[2] * frame.cols);
                    int height = (int) (data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float) confidence);
                    boxes.push_back(Rect(left, top, width, height));
                }
            }
        }

        //---------------------------非极大抑制---------------------------
        NMSBoxes(boxes, confidences, 0.5, 0.5, indices);

        int index;
        findState = false;

        dis2LastCenter = 1 << 30;

        for (index = 0; index < indices.size(); ++index) {
            if ((classIds[indices[index]] < 7 && blueTarget) || (classIds[indices[index]] > 7 && !blueTarget))
                continue;

            if (!find_not_engineer && indices[index] != 1 && indices[index] != 8) {
                targetArmor = Armor(boxes[indices[index]]);
                findState = true;
                break;
            } else if (find_not_engineer) {
                Point curr = (boxes[indices[index]].tl() + boxes[indices[index]].br()) / 2 - lastTarget.center;
                if (dis2LastCenter > curr.x * curr.x + curr.y * curr.y) {
                    targetArmor = Armor(boxes[indices[index]]);
                    findState = true;
                    find_not_engineer = classIds[indices[index]] == 1 || classIds[indices[index]] == 8;
                }
            }
        }

        if (findState) {
            detectCnt++;
            lostCnt = 0;

            //MakeRectSafe(targetArmor.rect, img.size());

            if (showArmorBox) {
                //rectangle(frame,roiRect,Scalar(255,255,255),2);

                for (int j = 0; j < 4; j++) {
                    line(frame, targetArmor.pts[j], targetArmor.pts[(j + 1) % 4], Scalar(0, 255, 255), 2);
                }

                circle(frame, targetArmor.center, 5, Scalar(0, 255, 255), -1);
            }

            /**update roi rect, last armor, average of lamps' R channel subtract B channel value**/
#if USEROI == 1
            roiRect = targetArmor.rect;
#endif

            lastTarget = targetArmor;

            return true;
        } else {
            detectCnt = 0;
            lostCnt++;
            return false;
        }
    }
}
