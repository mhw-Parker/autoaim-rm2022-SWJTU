/**********************************************************************************************************************
 * this file is mainly used for detecting the lamps and matching them into series of armors. First, by preprocessing the
 * image captured from video or camera, we get a binary image, in which we set the target lamps' pixel value as 255. Second,
 * recognizing all the possible lamps in the image by a set of rules made in advance, for example, in our source code,
 * we use 5 parameters to recognize lamps, they are
 * 1. the maximum possible angle of lamp.
 * 2. the minimum possible area of lamp.
 * 3. the maximum possible area of lamp.
 * 4. the maximum possible value of the ratio of lamp's height to lamp's width.
 * 5. the minimum possible value of the ratio of lamp's height to lamp's width.
 * The parameters above are particular for each project, they are related the the ratio of your image, the condition for
 * application ,etc.
 * Third, matching these lamps to a series of armors by some parameters and matching algorithm. And the whole code logic
 * and process you can see at https://github.com/luojunhui1/robomaster, for more detail, you can contact with the active
 * team members of Robomaster in SWJTU.
**********************************************************************************************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/dnn.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <cmath>
#include <ctime>
#include <utility>
#include <vector>
#include <cstring>
#include <omp.h>

#include "mydefine.h"
#include "utility.hpp"
#include "NumberClassifier.h"

using namespace std;
using namespace cv;
using namespace ml;
using namespace cv::dnn;
using namespace Eigen;

namespace rm
{
    /**
     * define the parameters for lamp recognizing and lamp matching.
     */
    struct ArmorParam {
        /// ??????????????????
        // ??????????????????
        float minPointNum = 7;
        // ????????????
        float minLightArea = 10;
        float maxLightArea = 2100;
        // ????????????
        float maxLightAngle = 20;
        // ???????????????
        float minLightH2W = 1.2;
        float maxLightH2W = 7;
        // ????????????
        float maxLightW = 25;
        float minLightH = 0;
        float maxLightH = 85;
        // ??????????????????????????????
        float minAverageBrightness = 20;
        // ???????????????????????????????????????????????????
        // ????????????????????????
        float maxAverageBrightness = 255;

        /// ??????????????????
        // ?????????
        float maxAngleError = 5;
        // ?????????
        float maxLengthError = 0.3;
        // ??????????????????
        float minRatio = 0.68;
        float maxRatio = 2.5;
        // ????????????
        float maxArmorAngle = 23;
        //
        float maxDeviationAngle = 45;
    };

    /**
     * the structure to describe the matched lamps
     */
    typedef struct MatchLight {
        float lampHeight;
        unsigned int matchIndex1 = -1;
        unsigned int matchIndex2 = -1;
        float matchFactor = 10000;

        explicit MatchLight(unsigned int match1 = 0, unsigned int match2 = 0,
                            float factor = 1000, float lampHeight_ = 0) {
            matchIndex1 = match1;
            matchIndex2 = match2;
            matchFactor = factor;
            lampHeight = lampHeight_;
        }
    }MatchLight;

    /**
     * define a structure to describe parameters for matching lamps
     */
    typedef struct MatchParam_Tag
    {
        double dAngle{};
        double conLen1{};
        double conLen2{};
        double ratio{};
        double nAngle{};
        double yDiff{};
        double dBright{};
        double sizeRatio{};

        MatchParam_Tag()= default;;
        MatchParam_Tag(double dAngle_, double conLen1_, double conLen2_, double ratio_, double nAngle_, double yDiff_
                , double dBright_, double sizeRatio_)
        {
            dAngle = dAngle_;
            conLen1 = conLen1_;
            conLen2 = conLen2_;
            ratio = ratio_;
            nAngle = nAngle_;
            yDiff = yDiff_;
            dBright = dBright_;
            sizeRatio = sizeRatio_;
        }
    }MatchParam;

    /**
     * the class to describe the lamp, including a rotated rectangle to describe the lamp's  geometry information and an
     * angle value of the lamp that is more intuitive than the angle member variable in RotateRect. For detail, you can
     * see it at https://blog.csdn.net/xueluowutong/article/details/86069814?utm_medium=distribute.pc_relevant.none-task-blog-title-2&spm=1001.2101.3001.4242
     */
    class Lamp {
    public:
        Lamp() : lightAngle(0), avgRSubBVal(0)
        {
        }

        Lamp(RotatedRect bar, float angle, float avgB) :lightAngle(angle),avgRSubBVal(avgB) {
            ///TODO?????????????????????
            // ?????????k
            cv::Size exLSize(int(bar.size.width), int(bar.size.height * 2.18));
            rect = cv::RotatedRect(bar.center, exLSize, bar.angle);
        }
        RotatedRect rect;

        float lightAngle;
        // ?????????????????????????????????
        float avgRSubBVal;
        /** new **/
        Lamp(RotatedRect bar, float avg_b) : rect(bar), lightAngle(bar.angle), avgRSubBVal(avg_b){
            Point2f pts[4];
            bar.points(pts);
            mid_p = (pts[0]+pts[2])/2;
        }
        Point2f mid_p;

    };

    /**
     * the class to describe the armor, including the differ between the angle of two lamps(errorAngle), the rect to
     * represent the armor(rect), the width of the armor(armorWidth) and the height of the armor(armorWidth), the type
     * of the armor(armorType), the priority of the armor to be attacked(priority)
     */
    class Armor {
    public:
        Armor() : armorWidth(0), armorHeight(0), armorType(BIG_ARMOR), priority(10000) {
        }

        Armor(Lamp L1, Lamp L2, double priority_);

        Armor(Lamp l_1, Lamp l_2, float score);

        explicit Armor(Rect &rect);
        void init();

        Point2i center;
        Rect rect;
        vector<Point2f> pts; // the 4 points of armor lamps
        vector<Point2f> num_pts; // the 4 points of number roi
        float armorWidth;
        float armorHeight;
        int armorType;
        float wh_ratio = 1;

        int id = -1;
        float num_conf = 0;

        double priority;
        float match_score, shoot_score;
    };

    /**
     * the tool class for functions, including the functions of preprocessing, lamp recognizing, lamp matching, ROI
     * updating, tracking and etc.
     */
    class ArmorDetector {
    public:
        ArmorDetector();

        ~ArmorDetector() = default;

        /**core functions**/
        void Init();

        void InitDetectionPrams();

        bool ArmorDetectTask(Mat &img);

    private:
        /***/
        void Preprocess(Mat &img);
        vector<Lamp> LampDetection(Mat& img);
        /***/
        void GetRoi(const Mat& img);

        bool DetectArmor(Mat &img);

        vector<MatchLight> MatchLights(vector<Lamp>&);

        void GetOptimalMatch(vector<MatchLight>, vector<Lamp>&);

        void LoadSvmModel(const char *model_path, const Size& armorImgSize = Size(40, 40));

        void SetSVMRectPoints(Point2f& lt, Point2f& rt, Point2f& lb, Point2f& rb);
        void SetSVMRectPoints(Point2f&& lt, Rect& rectArea);

        int getArmorNumber(Armor &armor);

        /** 7-23 test  **/
        void TopDetectTask(Mat &src);
        void BinaryMat(Mat &roi);
        vector<Lamp> MinLampDetect();
        vector<MatchLight> MatchLamps(vector<Lamp> &possible_lamps);
        vector<Armor> FindArmors(vector<MatchLight> &match_lamps, vector<Lamp> &possible_lamps);
        bool IsArmor(Lamp &l_1, Lamp &l_2, float &score);
        Mat GetNumberRoi(vector<Point2f> &pts, uint8_t armor_type);
        Armor GetTargetArmor(vector<Armor> &candidate_armor);

        Mat gray_mat;
        Mat gray_binary_mat;
        Mat sub_binary_mat;

        const float max_lamp_angle = 5;
        const float max_incline_angle = 30;
        const float small_armor_ratio = 2.33;
        const float big_armor_ratio = 3.83;
        const float max_lamps_height_error = 20;
        const float k_[5] = {2,2,1,1,2}; // score weight
        Point roi_corner;
        // warp perspective parameters
        const int lamp_height = 20;
        const int warp_height = 40;
        const int warp_small_width = 44;
        const int warp_large_width = 76;

        NumberClassifier classifier;
        /**tool functions**/

        Rect GetArmorRect() const;

        bool IsSmall() const;

        static void saveMatchParam(FILE* fileP,int selectedIndex1,int selectedIndex2);

        /**state member variables**/
    public:

        /*the final armor selected to be attacked*/
        Armor targetArmor;

        /*current find state*/
        bool findState = false;

        /*Send this to CONTROL*/
        bool lostState = true;

        /*current armor type*/
        bool isSmall;

        /*ROI rect in image*/
        Rect roiRect;

        /*target armor number*/
        int armorNumber;

        /*loss cnt*/
        int lostCnt;

        /*the number of frames that program get a target armor constantly*/
        unsigned long long detectCnt = 0;

        /** variables would be used in functions**/
    private:
        // gray input
        Mat gray;

        /*a gray image, the difference between rSubB and bSubR*/
        Mat_<int> colorMap;

        /*a gray image, the pixel's value is the difference between red channel and blue channel*/
        Mat rSubB;

        /*a gray image, the pixel's value is the difference between blue channel and red channel*/
        Mat bSubR;

        /*a mask image used to calculate the sum of the values in a region*/
        Mat mask;
        Mat sub;

        /*ROI image*/
        Mat imgRoi;

        /*a binary image*/
        Mat thresholdMap;

        /*last target armor*/
        Armor lastTarget;

        /**the frequency information**/
    private:
        /*the counter for successfully detection of target armor*/
        int armorFoundCounter = 0;

        /**parameters used for lamps recognizing and lamps matching**/
    private:
        // ???????????????????????????
        struct ArmorParam param;

        Ptr<SVM> svm;  //svm model svm??????
        Size svmArmorSize;
        Mat svmBinaryImage;
        Mat svmParamMatrix;		//preRecoginze matrix for svm ?????????SVM??????????????????
        Mat warpPerspective_mat; //warpPerspective transform matrix ???????????????????????????
        Point2f srcPoints[4];   //warpPerspective srcPoints		???????????????????????????????????? tl->tr->br->bl  ?????? ?????? ?????? ??????
        Point2f dstPoints[4];	//warpPerspective dstPoints     ?????????????????????????????????   tl->tr->br->bl  ?????? ?????? ?????? ??????
    public:
        Mat warpPerspective_dst;//warpPerspective dstImage   ??????????????????????????????


        //deep learning
    private:
        String cfgPath;
        String weightPath;
        Net net;
        std::vector<String> outNames;
        std::vector<Mat> outs;

        Mat inputBlob;
        double confidence;
        Point classIdPoint;
        vector<Rect> boxes;
        vector<int> classIds;
        vector<float> confidences;
        vector<int> indices;

        bool find_not_engineer;
        int dis2LastCenter;
    public:
        bool ModelDetectTask(Mat &frame);
    };

    /**
     * @param a an instance of a pair of matched lamps
     * @param b another instance of a pair of matched lamps
     * @return if the match factor of b is larger than a, return true, otherwise return false.
     */
    const bool compMatchFactor(const MatchLight& a, const MatchLight& b);
    const bool compPriority(const Armor &a, const Armor &b);
    bool MakeRectSafe(cv::Rect &rect, const cv::Size& size);
}

