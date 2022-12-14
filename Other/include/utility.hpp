#ifndef _RMTOOLS__
#define _RMTOOLS__

#include <unistd.h>
#include <cerrno>
#include <pthread.h>
#include <mutex>
#include <utility>
#include <thread>
#include <string>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <Eigen/Dense>

#include "mydefine.h"

using namespace cv;
using namespace std;

namespace RMTools {

#define __MYSCALAR__(i) ((i%3==0)?Scalar(i*16.7,i*8,0):((i%3==1)?Scalar(0,i*16.7,255 - i*5):Scalar(255 - i*2,0,i*16.7)))

/**
 * @brief this class is used to display a waveform figure
 * @param src_ background image
 * @param value_ the value to display as wave
 * @param wn_ window name
 * @param stand_ the standard line to paint
 * @param lc_ the standard line color
 * @param wc_ the wave line color
 */
    class DisPlayWaveCLASS {
    private:
        uint8_t count = 0;
        bool m_isExit;
        std::thread m_thread;

        Mat src;
        int *value;
        uint16_t lastValue = 0;

        int *value1{};
        uint16_t lastValue1 = 0;

        string wn;
        int stand = 64;
        int spacing = 10;
        Scalar lc = Scalar(0, 0, 255);
        Scalar wc = Scalar(0, 146, 125);

        Mat left, copy;
        /** new wave form **/
        int h, w, mid_h;
        float ratio;
        Mat background;
        Point2f last_p1;
        Point2f last_p2;

        cv::Scalar color[4] = {Scalar(0,255,0), Scalar(0,255,255),
                               Scalar(255, 0, 255), Scalar(255,255,0)};
        Point2f last_pts[4], cur_pts[4];
        int cnt = 0;

    public:

        DisPlayWaveCLASS(Mat src_, int *value_) : m_isExit(false), src(std::move(src_)), value(value_) {
            copy = src.clone();
        };

        DisPlayWaveCLASS(Mat src_, int *value_, int *value1_) : m_isExit(false), src(std::move(src_)), value(value_),
                                                                value1(value1_) {
            copy = src.clone();
        };

        DisPlayWaveCLASS(Mat src_, int *value_, string wn_, int stand_ = 0, Scalar lc_ = Scalar(0, 0, 255), \
                    Scalar wc_ = Scalar(0, 0, 0)) : m_isExit(false), src(std::move(src_)), value(value_),
                                                    wn(std::move(wn_)), stand(stand_), \
                    lc(std::move(lc_)), wc(std::move(wc_)) {
            copy = src.clone();
        };

        DisPlayWaveCLASS(Mat src_, int *value_, int *value1_, string wn_, int stand_ = 0,
                         Scalar lc_ = Scalar(0, 0, 255), \
                    Scalar wc_ = Scalar(0, 0, 0)) : m_isExit(false), src(std::move(src_)), value(value_),
                                                    wn(std::move(wn_)), stand(stand_), \
                    lc(std::move(lc_)), wc(std::move(wc_)), value1(value1_) {
            copy = src.clone();
        };

        /**
         * @brief a constructor about the wave tool
         * @param dataRange the absolute maximum among the data
         * @param height the window's height
         * @param width the window's width
         * */
        DisPlayWaveCLASS(float dataRange, int height, int width) : ratio((height / 2) / dataRange), h(height), w(width),
                                                                   mid_h(height / 2) {
            background = Mat(h, w, CV_8UC3, Scalar::all(0));
            line(background, Point2f(0, height / 2), Point2f(w, height / 2), Scalar::all(255));
            for(auto &p : last_pts){
                p = Point2f(0,mid_h);
            }
        }

        void DisplayWave() {
            if (*value >= src.rows || stand >= src.rows) {
                perror("Value exceeds the src rows");
                return;
            }
            if ((src.cols / spacing) > count) {
                line(copy, Point2d((count - 1) * spacing, lastValue), Point2d(count * spacing, *(value)), wc);
                lastValue = *(value);
                count++;
            } else {
                copy.colRange(spacing, (count - 1) * spacing + 1).copyTo(left);
                copy.setTo(0);
                left.copyTo(copy.colRange(0, (count - 2) * spacing + 1));
                line(copy, Point2d((count - 2) * spacing, lastValue), Point2d((count - 1) * spacing, *(value)), wc);
                lastValue = *(value);
            }
            line(copy, Point2d(0, stand), Point2d(copy.cols - 1, stand), lc);
            flip(copy, src, 0);
            imshow(wn, src);
        }

        void DisplayWave2() {
            if (*value >= src.rows || stand >= src.rows || *value1 >= src.rows)perror("Value exceeds the src rows");
            if ((src.cols / spacing) > count) {
                line(copy, Point2d((count - 1) * spacing, lastValue), Point2d(count * spacing, *(value)), wc, 1);
                line(copy, Point2d((count - 1) * spacing, lastValue1), Point2d(count * spacing, *(value1)),
                     Scalar(0, 255, 0), 1);
                lastValue = *(value);
                lastValue1 = *(value1);
                count++;
            } else {
                copy.colRange(spacing, (count - 1) * spacing + 1).copyTo(left);
                copy.setTo(0);
                left.copyTo(copy.colRange(0, (count - 2) * spacing + 1));
                line(copy, Point2d((count - 2) * spacing, lastValue), Point2d((count - 1) * spacing, *(value)), wc, 1);
                line(copy, Point2d((count - 2) * spacing, lastValue1), Point2d((count - 1) * spacing, *(value1)),
                     Scalar(0, 255, 0), 1);
                lastValue = *(value);
                lastValue1 = *(value1);
            }
            line(copy, Point2d(0, stand), Point2d(copy.cols - 1, stand), lc);
            flip(copy, src, 0);
            imshow(wn, src);
        }

        /** 2021-12-9 tyy **/
        //?????????????????????????????????????????????????????????????????????
        void displayWave(const float input1, const float input2, string win_name) {
            int amplitude1 = mid_h - ratio * input1;
            int amplitude2 = mid_h - ratio * input2;
            if (amplitude1 < 0 || amplitude2 < 0) {
                cout << "[SHOW WAVE WARNING] -- higher than the dataRange !" << endl;
                return;
            }
            if (amplitude1 > h || amplitude2 > h) {
                cout << "[SHOW WAVE WARNING] -- lower than the dataRange ! " << endl;
                return;
            }

            Point2f cur_p1 = Point2f(cnt, amplitude1);
            Point2f cur_p2 = Point2f(cnt, amplitude2);

            if (last_p1 != Point2f(0, 0)) {
                line(background, cur_p1, last_p1, Scalar(0, 255, 0));     // line input 1
                line(background, cur_p2, last_p2, Scalar(0, 255, 255));   // line input 2
            }

            imshow(win_name, background);
            waitKey(1);

            cnt += 2;
            last_p1 = cur_p1;
            last_p2 = cur_p2;

            if (cnt > w) {
                cnt = 0;
                last_p1 = Point2f(0, 0);
                last_p2 = Point2f(0, 0);
                background = Mat(h, w, CV_8UC3, Scalar::all(0));
                line(copy, Point2f(0, h / 2), Point2f(w, h / 2), Scalar::all(255));
            }
        }
        /**
         * @brief a brief oscilloscope used to show the wave of the data
         * @param line1 green line
         * @param line2 yellow line
         * @param line3 pink line
         * @param line4 sky blue line
         * */
        inline cv::Mat Oscilloscope(float line1=0, float line2=0, float line3=0, float line4=0){
            float data[4] = {line1,line2,line3,line4};
            for(int i=0; i<4; i++){
                int a = mid_h - ratio * data[i];
                a = (a<0) ? 0 : a;
                a = (a>h) ? h : a;
                cur_pts[i] = Point2f(cnt, a);
                line(background, cur_pts[i], last_pts[i], color[i]);
                last_pts[i] = cur_pts[i];
            }
            cnt += 2;
            if(cnt > w) {
                cnt = 0;
                for(auto &p : last_pts)
                    p.x = cnt;
                background = Mat(h, w, CV_8UC3, Scalar::all(0));
                line(background, Point2f(0, mid_h), Point2f(w, mid_h), Scalar::all(255));
            }
            return background;
        }
        /**
         * @brief clear wave
         * */
        void clear(){
            cnt = 0;
            background = Mat(h, w, CV_8UC3, Scalar::all(0));
            line(background, Point2f(0, mid_h), Point2f(w, mid_h), Scalar::all(255));
            for(auto &p : last_pts){
                p = Point2f(0,mid_h);
            }
        }

    };

/**
 * @brief ????????????
 * @param BeginTime ????????????????????? getTickCount() ??????
 * @param freq ?????????????????? getTickFrequency() ??????
 * @return ??????????????? ms
 * */
    inline double CalWasteTime(double BeginTime, double freq = getTickFrequency()) {
        return (getTickCount() - BeginTime) * 1000 / freq;
    }

/**
 * @brief ????????????????????????
 * @return string
 */
    inline string getSysTime() {
        time_t timep;
        time(&timep);
        char tmp[64];
        strftime(tmp, sizeof(tmp), "%Y-%m-%d_%H_%M_%S", localtime(&timep));
        return tmp;
    }
/**
 * @brief ??????????????????
 * @param data ?????? vector<float> ????????????
 * @param *str ?????????????????????
 * */
    inline bool showData(vector<float> data, vector<string> str, const string win_name){
        int c = data.size() * 33;
        if(data.size() != str.size()){
            cout << "The vector length doesn't match !" << endl;
            return false;
        }
        Mat background = Mat(c,500,CV_8UC3,Scalar::all(0));
        for(int i = 0;i<data.size();i++){
            putText(background, str[i], Point(0, 30*(i+1)), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2, 8, 0);
            putText(background, to_string(data[i]), Point(200, 30*(i+1)), cv::FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255), 2, 8, 0);
        }
        imshow(win_name,background);
        waitKey(1);
        return true;
    }
    inline bool Connect4Pts(vector<Point2f> &pts, Mat &img, Scalar color = Scalar::all(255)){
        if(!pts.size()){
            return false;
        } else {
            for(int i = 0; i<4; i++)
                line(img, pts[i], pts[(i + 1) % 4],color, 1);
            return true;
        }
    }

    /**
     * ??????????????????????????????????????????
     * @param receive_v_bullet ?????????????????????
     * @param last_average_v ?????????????????????
     * @return ????????????
     */
    inline bool CheckBulletVelocity(CARNAME carName, float receive_v_bullet) {
        float min_v = 10, max_v = 30;
        switch (carName) {
            case HERO:
                min_v = 10;
                max_v = 16;
                break;
            case INFANTRY3:
            case INFANTRY4:
            case INFANTRY5:
                min_v = 10;
                max_v = 30;
                break;
            case SENTRYTOP:
            case SENTRYDOWN:
                min_v = 24;
                max_v = 32;
                break;
            case UAV:
                break;
            case VIDEO:
                break;
            case NOTDEFINED:
                break;
        }
        if (receive_v_bullet < min_v || receive_v_bullet > max_v) {
            return false;
        }
        return true;
    }

/**
 * ??????????????????0????????????????????????
 */
    inline float average(const float* x, int len) {
        float sum = 0;
        for (int i = 0; i < len; i++) {
            if (x[i] == 0) {
                len = i;
                break;
            }
            sum += x[i];
        }
        return sum / len; // ???????????????
    }

/**
 * ?????????
 */
    inline float variance(float *x, int len) {
        double sum = 0;
        double average1 = average(x, len);
        for (int i = 0; i < len; i++) // ??????
            sum += pow(x[i] - average1, 2);
        return sum / len; // ???????????????
    }

/**
 * ????????????
 */
    inline float stdDeviation(float *x, int len) {
        double variance1 = variance(x, len);
        return sqrt(variance1); // ???????????????
    }

/**
 * @brief ???????????????
 * @param x ?????????????????? x ???????????????????????????????????? 1 ???????????????
 * @param y ???????????????
 * @return ?????? Eigen ??? MatrixXd
 * @remark ?????? CSDN ????????????????????????
 * */
    inline Eigen::MatrixXd LeastSquare(vector<float> x, vector<float> y, int N) {
        Eigen::MatrixXd A(x.size(), N + 1);
        Eigen::MatrixXd B(y.size(), 1);
        Eigen::MatrixXd W;
        for (unsigned int i = 0; i < x.size(); ++i) {
            for (int n = N, dex = 0; n >= 1; --n, ++dex) {
                A(i, dex) = pow(x[i], 2);
            }
            A(i, N) = 1;
            B(i, 0) = y[i]; //???????????? y ?????????
        }
        W = (A.transpose() * A).inverse() * A.transpose() * B;
        return W;
    }

    /**
     * @brief ??????????????????int???
     * */
    inline int round2int(float f) {
        int int_f = f;
        float d = f - int_f;
        if(d >= 0.5) return int_f+1;
        else if(d>0 && d < 0.5) return int_f;
        else if(d<0 && d > -0.5) return int_f;
        else return int_f - 1;
    }
    /**
     * @brief ?????????????????? total ???????????? 0 ~ 360??
     * */
    inline float total2circle(float theta){
        if(theta > 0)
            return theta - (int)(theta / 360) * 360;
        else
            return theta - (int)(theta / 360 - 1) * 360;
    }
    /**
     * @brief ????????????x,z??????????????????rho,theta??????-???+???
     * @param XZ ??????x,z?????????
     * @return {rho, theta}???
     */
    inline Eigen::Vector2f XZ2RhoTheta(const Eigen::Vector2f& XZ) {
        float x = XZ[0], z = XZ[1];
        float rho = sqrt(x * x + z * z);
        float sin_theta = x / rho;
        float theta = z > 0 ? -asin(sin_theta) : CV_PI - asin(sin_theta);
        return {rho, theta};
    }

    /**
     * ?????????????????????
     */
    inline float GetDistance(const Eigen::Vector3f& pos1, const Eigen::Vector3f& pos2) {
        return (pos1 - pos2).norm();
    }

    /**
     * @brief ??????????????????YPD???
     * */
    inline Eigen::Vector3f GetDeltaYPD(Eigen::Vector3f cur_xyz, Eigen::Vector3f last_xyz){
        float lx = last_xyz[0], ly = last_xyz[1], lz = last_xyz[2];
        float ldist = sqrt(lx*lx + ly*ly + lz*lz);
        float x = cur_xyz[0], y = cur_xyz[1], z = cur_xyz[2];
        float dist = sqrt(x*x + y*y + z*z);

        // ??????delta_yaw
        float predict_theta = RMTools::XZ2RhoTheta({cur_xyz[0],cur_xyz[2]}).y();
        float target_theta = RMTools::XZ2RhoTheta({last_xyz[0], last_xyz[2]}).y();
        float delta_yaw = predict_theta - target_theta;
        if (delta_yaw > CV_PI)
            delta_yaw -= CV_PI;
        if (delta_yaw < -CV_PI)
            delta_yaw += CV_PI;

        // ??????delta_pitch
        if (carName == SENTRYTOP || carName == SENTRYDOWN) {
            target_theta = asin(ly / ldist);
            predict_theta = asin(y / dist);
        } else {
            target_theta = -asin(ly / ldist);
            predict_theta = -asin(y / dist);
        }

        float delta_pitch = predict_theta - target_theta;

        Eigen::Vector3f result(delta_yaw, delta_pitch, dist - ldist);
        for (int i = 0; i < 2; i++)
            result[i] = result[i] / CV_PI * 180;
        return result;
    }

    /**
     * @brief calculate distance between 2 points
     * */
    template<class T>
    T point_distance(const Point &pts_1, const Point &pts_2){
        Point d_pts = pts_1 - pts_2;
        return sqrt(d_pts.x*d_pts.x + d_pts.y*d_pts.y);
    }

/**
 *  the descriptor of the point in the route, including the color, location, velocity and the situation of point.
 */
    class RoutePoint {
    public:
        RoutePoint(const Point &p_, Scalar color_, int vx_, int vy_) : p(p_), color(std::move(color_)), vx(vx_),
                                                                       vy(vy_) {};
        Point p;
        Scalar color;
        int vx;
        int vy;
        bool used = false;
    };

/**
 * @brief this class is used to track the feature points that user provided and display the route of these feature points
 *        with unique color.
 * @param src the image that the points drawing on
 * @param pSize the size of the points, which reflect the route
 */
    class FeatureRoute {
    private:
        int pSize;
        int count;
        int expirectl;
        Mat origin;
        Mat srcs[5];
        vector<RoutePoint> lastPts;

        const int wd = 20;
    public:
        explicit FeatureRoute(const Mat &src, int pSize);

        void DisplayRoute(const vector<Point> &pts);

        void FeatureRouteDrawPoint(const Point &p, const Scalar &color, int expirectl);
    };

    inline FeatureRoute::FeatureRoute(const Mat &src, int pSize = 10) {
        this->pSize = pSize;
        count = 1;
        this->origin = src;
        expirectl = 0;
        for (auto &i: srcs) {
            src.copyTo(i);
        }
    }

    inline void FeatureRoute::FeatureRouteDrawPoint(const Point &p_, const Scalar &color_, int expirectl_) {
        for (int i = 0; i < 4; i++) {
            circle(srcs[expirectl_], p_, pSize, color_, -1);
            expirectl_ = (expirectl_ + 1) % 5;
        }
        srcs[expirectl_].setTo(255);
    }

    inline void FeatureRoute::DisplayRoute(const vector<Point> &pts) {
        int i = 0;
        vector<RoutePoint> cur;
        if (lastPts.empty()) {
            for (const auto &p: pts) {
                if (p.x < 0 || p.x > origin.cols || p.y < 0 || p.y > origin.rows) {
                    printf("Point exceed the limitation of window");
                    continue;
                }
                cur.emplace_back(p, __MYSCALAR__(i), 0, 0);
                FeatureRouteDrawPoint(p, __MYSCALAR__(i), expirectl);
                i = i + 1;
            }
        } else {
            vector<Point> lt;
            vector<Point> rb;
            count = pts.size();
            int curError;
            int selectedIndex;
            int error;
            for (const auto &p: pts) {
                lt.emplace_back(Point(lastPts[i].p.x - wd, lastPts[i].p.y - wd));
                rb.emplace_back(Point(lastPts[i].p.x + wd, lastPts[i].p.y + wd));
            }
            for (const auto &p: pts) {
                error = numeric_limits<int>::max();
                selectedIndex = -1;
                for (i = 0; i < lastPts.size(); i++) {
                    if ((p.x > lt[i].x && p.y > lt[i].y && p.x < rb[i].x && p.y < rb[i].y)
                        && !lastPts[i].used
                        || (lastPts[i].vx != 0 && (lastPts[i].p.x - p.x) != 0
                            &&
                            (lastPts[i].vy / lastPts[i].vx > 0) == ((lastPts[i].p.y - p.y) / (lastPts[i].p.x - p.x) > 0)
                            && (abs(lastPts[i].vy * 3) > abs((lastPts[i].p.y - p.y)) &&
                                abs(lastPts[i].vx * 3) > abs((lastPts[i].p.x - p.x)))
                        )
                            ) {
                        curError = abs(lastPts[i].p.y - p.y) + abs(lastPts[i].p.x - p.x);
                        error = (error > curError) ? (selectedIndex = i, curError) : (error);
                    }
                }
                if (selectedIndex != -1) {
                    cur.emplace_back(RoutePoint(p, lastPts[selectedIndex].color, (p.x - lastPts[i].p.x),
                                                (p.y - lastPts[i].p.y)));
                    lastPts[selectedIndex].used = true;
                } else {
                    cur.emplace_back(RoutePoint(p, __MYSCALAR__(count), 0, 0));
                    count = (count + 1) % 18;
                }
            }
            for (const auto &p: cur) {
                if (p.p.x < 0 || p.p.x > origin.cols || p.p.y < 0 || p.p.y > origin.rows) {
                    printf("Point exceed the limitation of window");
                    continue;
                }
                FeatureRouteDrawPoint(p.p, p.color, expirectl);
                //circle(origin,p.p,pSize,p.color,-1);
            }
        }
        imshow("FeatureRoute", srcs[expirectl]);
        expirectl = (expirectl + 1) % 5;

        lastPts = cur;
    }

    template<typename T>
    class MedianFilter {
    private:
        int len;
        int tail = 0;
        bool full_flag = false;
        T *data_buff_ptr;
        T *circular_queue_ptr;
    public :
        explicit MedianFilter(int L = 6) : len(L) {
            data_buff_ptr = (T *) malloc(sizeof(T) * L);
            circular_queue_ptr = (T *) malloc(sizeof(T) * L);

            if (data_buff_ptr == nullptr || circular_queue_ptr == nullptr)
                exit(1);
        }

        ~MedianFilter() {
            free(data_buff_ptr);
            free(circular_queue_ptr);
        }

        T get_middle_value() {
            memcpy(data_buff_ptr, circular_queue_ptr, sizeof(T) * (full_flag ? len : tail));

            if (full_flag) {
                std::sort(data_buff_ptr, data_buff_ptr + len);
                return len & 1 ? data_buff_ptr[len >> 1] : ((data_buff_ptr[len >> 1] + data_buff_ptr[(len + 1) >> 1]) /
                                                            2);
            }

            std::sort(data_buff_ptr, data_buff_ptr + tail);
            return tail & 1 ? data_buff_ptr[tail >> 1] : ((data_buff_ptr[tail >> 1] + data_buff_ptr[(tail + 1) >> 1]) /
                                                          2);
        }

        void insert(T num) {
            circular_queue_ptr[tail++] = num;

            if (tail == len)
                full_flag = true;

            tail %= len;
        }

        bool is_full() { return full_flag; }
    };

    static void sleep_ms(unsigned int secs) {
        struct timeval tval;

        tval.tv_sec = secs / 1000;

        tval.tv_usec = (secs * 1000) % 1000000;

        select(0, NULL, NULL, NULL, &tval);
    }


}

#endif
