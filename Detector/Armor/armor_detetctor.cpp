//
// Created by tyy on 22-7-24.
//
#include "ArmorDetector.hpp"

namespace rm{
    /***/
    void ArmorDetector::TopDetectTask(Mat &roi) {
        double st = getTickCount();
        BinaryMat(roi);
        //cout << "-- preprocess : " << RMTools::CalWasteTime(st,getTickFrequency()) << endl;
        st = getTickCount();
        vector<Lamp> possible_lamps = MinLampDetect(roi);
        /** match light **/
        vector<MatchLight> match_lamps = MatchLamps(possible_lamps);
        FindArmor(match_lamps, possible_lamps, roi);
        //cout << "-- match : " << RMTools::CalWasteTime(st,getTickFrequency()) << endl;
    }
    /**
     * @brief preprocess mat
     * */
    void ArmorDetector::BinaryMat(Mat &roi) {
        Mat gray, sub_mat;
        vector<Mat> channels;
        cvtColor(roi,gray,COLOR_BGR2GRAY);
        threshold(gray, gray_binary_mat, 50, 255, THRESH_BINARY);
        split(roi, channels);
        if(blueTarget)
            subtract(channels[0],channels[2],sub_mat);
        else
            subtract(channels[2],channels[0],sub_mat);
        threshold(sub_mat,sub_binary_mat,100,255,THRESH_BINARY); // 80
        //imshow("binary",gray_binary_mat);
    }
    /**
     * @brief test min area Rect for armor detector
     * */
    vector<Lamp> ArmorDetector::MinLampDetect(Mat &src) {
        vector<vector<Point>> contoursLight;
        vector<Lamp> possible_lamps;
        //vector<RotatedRect> possible_lamps;
        findContours(gray_binary_mat, contoursLight, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        int num = 0;
        /** find possible lamp **/
        for(auto &light : contoursLight) {
            if(light.size() < 25) continue;
            RotatedRect lamp_rect = fitEllipse(light);
            /// limit rect rotate angle
            lamp_rect.angle = lamp_rect.angle > 90 ? lamp_rect.angle-180 : lamp_rect.angle;
            float angle_ = lamp_rect.angle;
            if (fabs(angle_) >= 17) continue;
            /// limit rect height / width ratio
            float ratio_ = lamp_rect.size.height / lamp_rect.size.width;
            if (ratio_ < 2.5 || ratio_ > 16) continue;
            /// limit rect width
            if (lamp_rect.size.width > 35) continue;
            /// judge color
            Rect bounding_lamp = lamp_rect.boundingRect();
            MakeRectSafe(bounding_lamp, sub_binary_mat.size());
            Mat_<u_int8_t > lamp_image = sub_binary_mat(bounding_lamp);
            Scalar_<float> avg_brightness = mean(lamp_image);
            avg_brightness[0] /= cos(angle_/180);
            if(avg_brightness[0] < 35) continue;
            /// store possible lamps
            Lamp lamp_info(lamp_rect, avg_brightness[0]);
            possible_lamps.emplace_back(lamp_info);
            Point2f pts_[4];
            lamp_rect.points(pts_);
            if(showLamps) {
                for(int i = 0; i<4; i++) {
                    line(src, pts_[i], pts_[(i + 1) % 4],Scalar(255, 0, 255), 1);
                }
                circle(src, pts_[0], 2, Scalar(0, 255, 0));
                putText(src, to_string(int(lamp_rect.size.width)), pts_[0],
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255),
                        1);
            }
        }
        return possible_lamps;
        //imshow("test",dst);
        //waitKey();
    }
    /**
     * @brief match couple lamps
     * @param possible_lamps possible lamps selected from binary mat
     * @param src draw lamps on image
     * @return a vector contains all of the MatchLight
     * */
    vector<MatchLight> ArmorDetector::MatchLamps(vector<Lamp> &possible_lamps) {
        vector<MatchLight> match_lamps;
        for(int l1 = 0; l1 < possible_lamps.size(); l1++){
            float max_score = 0;
            vector<MatchLight> pair;
            for(int l2 = l1+1; l2 < possible_lamps.size(); l2++) {
                float score;
                if(possible_armor(possible_lamps[l1], possible_lamps[l2], score)) {
                    if(score > max_score) {
                        pair.emplace_back(MatchLight (l1, l2, score));
                        max_score = score;
                    }
                }
            }
            if(pair.size()) {
                MatchLight ml = pair.back(); // the max score from l1 - ln
                match_lamps.emplace_back(ml);
                for(auto &i : match_lamps) {
                    //cout << i.matchIndex2 << pair.back().matchIndex1 << pair.back().matchIndex1 << endl;
                    if(i.matchIndex2 == match_lamps.back().matchIndex1 || i.matchIndex2 == match_lamps.back().matchIndex2) {  // if twice judge share
                        if(match_lamps.back().matchFactor < i.matchFactor) {
                            cout << "score: " << match_lamps.back().matchFactor << " " << i.matchFactor << endl;
                            match_lamps.pop_back();
                            break;
                        }
                        else if(match_lamps.back().matchFactor > i.matchFactor){
                            cout << "score: " << match_lamps.back().matchFactor << " " << i.matchFactor << endl;
                            swap(i, match_lamps.back());
                            match_lamps.pop_back();
                            break;
                        }
                    }
                }
            }
        }
        sort(match_lamps.begin(), match_lamps.end(), compMatchFactor);
        return match_lamps;
    }
    /**
     * @brief find armor from selected couple lamps
     * @param match_lamps
     * @param possible_lamps
     * */
    void ArmorDetector::FindArmor(vector<MatchLight> &match_lamps, vector<Lamp> &possible_lamps, Mat &src) {
        findState = (lostCnt++ > 5) ? false : true;
        for(int i = 0; i < match_lamps.size(); i++) {
            Armor possible_armor(possible_lamps[match_lamps[i].matchIndex1], possible_lamps[match_lamps[i].matchIndex2]);
            Scalar color = Scalar(255,255,255);
            if(i == match_lamps.size()-1) {
                color = Scalar (0,255,0);
                /* get number */
                armorNumber = 0;
                targetArmor = possible_armor;
                //roiRect = targetArmor.rect;
                lostCnt = 0;
                circle(src, (targetArmor.pts[0]+targetArmor.pts[2])/2, 2, Scalar(0, 255, 0),2);
            }
            if(showArmorBox) {
                for (int j = 0; j < 4; j++) {
                    line(src, possible_armor.pts[j], possible_armor.pts[(j + 1) % 4], color, 1);
                }
                putText(src, to_string(possible_armor.wh_ratio), possible_armor.pts[0],
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255),
                        1);
            }
        }
    }

    /**
     * @brief select possible armor
     * */
    bool ArmorDetector::possible_armor(Lamp &l_1, Lamp &l_2, float &score) {
        Mat background = imgRoi.clone();
        /** lamps angle **/
        float angle_error = fabs(l_1.lightAngle - l_2.lightAngle);
        if(angle_error > max_lamp_angle) return false;
        float norm_angle_err = angle_error / max_lamp_angle;
        /** lamps height **/
        int height_error = abs(l_1.rect.size.height - l_2.rect.size.height);
        if(float(height_error) > max_lamps_height_error) return false;
        float norm_height_err = float(height_error) / max_lamps_height_error;
        /** armor incline angle **/
        Point2f k_pts = l_1.mid_p - l_2.mid_p;
        float incline_angle = atan2(fabs(k_pts.y),fabs(k_pts.x)) / CV_PI * 180;
        if(incline_angle > max_incline_angle) return false;
        float norm_incline_err = incline_angle / max_incline_angle;
        /** w/h ratio **/
        float avg_height = (l_1.rect.size.height + l_2.rect.size.height)/2;
        float lamps_width = RMTools::point_distance<float>(l_1.mid_p, l_2.mid_p);
        float wh_ratio = lamps_width / avg_height;
        if(wh_ratio > 5 || wh_ratio < 1.5) return false;
        float norm_wh_ratio = (wh_ratio < 2.5) ? fabs(wh_ratio-small_armor_ratio)/small_armor_ratio :
                              fabs(wh_ratio-big_armor_ratio)/big_armor_ratio;
        /** lamps brightness **/
        float bright_error = fabs(l_1.avgRSubBVal-l_2.avgRSubBVal);
        float norm_bright = bright_error * 2 / (l_1.avgRSubBVal+l_2.avgRSubBVal);
        /** use exp(-kx) to mark every possible couple lamps **/
        score = exp(-k_[0]*norm_angle_err) + exp(-k_[1]*norm_height_err) +
                exp(-k_[2]*norm_incline_err) + exp(-k_[3]*norm_wh_ratio) +
                exp(-k_[4]*norm_bright);
        score /= 5;

        line(background, l_1.mid_p, l_2.mid_p, Scalar(0,255,0));
        putText(background, to_string(wh_ratio), (l_1.mid_p+l_2.mid_p)/2,
                FONT_HERSHEY_SIMPLEX,0.5, Scalar(255, 255, 255),1);
        //imshow("angle", background);
        //waitKey();
        return true;
    }

    Armor::Armor(Lamp L1, Lamp L2) {
        Point2f pts_[4];
        pts.assign(4,Point2f(0,0));
        if (L1.rect.center.x < L2.rect.center.x) {
            L1.rect.points(pts_);
            pts[0] = (pts_[1] + pts_[2]) / 2;
            pts[3] = (pts_[0] + pts_[3]) / 2;
            L2.rect.points(pts_);
            pts[1] = (pts_[1] + pts_[2]) / 2;
            pts[2] = (pts_[0] + pts_[3]) / 2;
        } else {
            L2.rect.points(pts_);
            pts[0] = (pts_[1] + pts_[2]) / 2;
            pts[3] = (pts_[0] + pts_[3]) / 2;
            L1.rect.points(pts_);
            pts[1] = (pts_[1] + pts_[2]) / 2;
            pts[2] = (pts_[0] + pts_[3]) / 2;
        }
        armorHeight = (L1.rect.size.height + L2.rect.size.height)/2;
        armorWidth = RMTools::point_distance<float>(L1.mid_p, L2.mid_p);
        wh_ratio = armorWidth / armorHeight;
        armorType = (wh_ratio < 3) ? S_ARMOR : B_ARMOR;
        center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x) / 2);
        center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y) / 2);
        rect = Rect(center - Point2i(armorWidth / 2, armorHeight / 2),
                    Size(armorWidth, armorHeight));
    }

}