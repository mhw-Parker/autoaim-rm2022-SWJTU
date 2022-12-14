//
// Created by tyy on 22-7-24.
//
#include "ArmorDetector.hpp"

namespace rm{
    /***/
    void ArmorDetector::TopDetectTask(Mat &img) {
        GetRoi(img);
        double st = getTickCount();
        BinaryMat(imgRoi);
//        cout << "-- preprocess : " << RMTools::CalWasteTime(st) << endl;
        st = getTickCount();
        /** detect lamps **/
        vector<Lamp> possible_lamps = MinLampDetect();
//        cout << "-- lamp : " << RMTools::CalWasteTime(st) << endl;
        if(showLamps) {
            for(auto &l : possible_lamps){
                Point2f pts_[4];
                l.rect.points(pts_);
                for(int i = 0; i<4; i++) {
                    line(img, (Point2f)roi_corner+pts_[i], (Point2f)roi_corner+pts_[(i + 1) % 4],Scalar(255, 0, 255), 1);
                }
                circle(img, (Point2f)roi_corner+pts_[0], 2, Scalar(0, 255, 0));
                putText(img, to_string(int(l.avgRSubBVal)), (Point2f)roi_corner+pts_[0],
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255),
                        1);

            }
        }
        /** match lamps **/
        st = getTickCount();
        vector<MatchLight> match_lamps = MatchLamps(possible_lamps);
        /** find candidate armors **/
        vector<Armor> candidate_armor = FindArmors(match_lamps, possible_lamps);
//        cout << "-- match : " << RMTools::CalWasteTime(st) << endl;
        if(showArmorBox && findState) {
            rectangle(img, roiRect, Scalar(255, 255, 255), 1);
            putText(img, "roi", (Point2f)roi_corner,FONT_HERSHEY_SIMPLEX, 0.5, Scalar::all(255),1);
        }

        if(candidate_armor.size()) {
            lostCnt = 0;
            targetArmor = GetTargetArmor(candidate_armor);
            // reset roi
            MakeRectSafe(targetArmor.rect, img.size());
            targetArmor.rect = targetArmor.rect + roi_corner;
            targetArmor.center += roi_corner;
            roiRect = targetArmor.rect;
            for (int i = 0; i < 4; i++) {
                targetArmor.pts[i] = targetArmor.pts[i] + (Point2f)roi_corner;
            }

            if(showArmorBox && findState){
                for(auto &armor : candidate_armor){
                    for (int i = 0; i < 4; i++) {
                        armor.pts[i] = armor.pts[i] + (Point2f)roi_corner;
                    }
                    RMTools::Connect4Pts(armor.pts, img);
                    string id_str = "id:" + to_string(armor.id) + " " + to_string(armor.num_conf);
                    putText(img, id_str, armor.pts[0],
                            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0),1);
                }
                RMTools::Connect4Pts(targetArmor.pts, img, Scalar(0,255,0));
                circle(img, (targetArmor.pts[0]+targetArmor.pts[2])/2, 2, Scalar(0, 255, 0),2);
            }
        } else {
            lostCnt++;
        }
        findState = (lostCnt == 0);
//        if(showArmorBox && findState)
//            rectangle(img, roiRect, Scalar(255, 255, 255), 1);
        //cout << "-- match : " << RMTools::CalWasteTime(st,getTickFrequency()) << endl;
    }
    /**
     * @brief preprocess mat
     * @param roi maybe roi image or whole size;
     * */
    void ArmorDetector::BinaryMat(Mat &roi) {
        Mat sub_mat;
        vector<Mat> channels;
        cvtColor(roi,gray_mat,cv::COLOR_BGR2GRAY);
        threshold(gray_mat, gray_binary_mat, 50, 255, cv::THRESH_BINARY);
        split(roi, channels);
        if(blueTarget)
            subtract(channels[0],channels[2],sub_mat);
        else
            subtract(channels[2],channels[0],sub_mat);
        threshold(sub_mat,sub_binary_mat,100,255,THRESH_BINARY); // 80
        if(showBinaryImg){
            imshow("binary",gray_binary_mat);
        }
    }
    /**
     * @brief test min area Rect for armor detector
     * */
    vector<Lamp> ArmorDetector::MinLampDetect() {
        vector<vector<Point>> contoursLight;
        vector<Lamp> possible_lamps;
        findContours(gray_binary_mat, contoursLight, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        for(auto &light : contoursLight) {
            if(light.size() < param.minPointNum) continue;
            RotatedRect lamp_rect = fitEllipse(light);
            /** limit rect rotate angle **/
            lamp_rect.angle = lamp_rect.angle > 90 ? lamp_rect.angle-180 : lamp_rect.angle;
            float angle_ = lamp_rect.angle;
            if (fabs(angle_) >= param.maxLightAngle) continue;
            /** limit rect height / width ratio **/
            float ratio_ = lamp_rect.size.height / lamp_rect.size.width;
            if (ratio_ < 1.2 || ratio_ > 16) continue;
            /** limit rect width **/
            if (lamp_rect.size.width > 35) continue;
            /** judge color **/
            Rect bounding_lamp = lamp_rect.boundingRect();
            MakeRectSafe(bounding_lamp, sub_binary_mat.size());
            Mat_<u_int8_t > lamp_image = sub_binary_mat(bounding_lamp);
            Scalar_<float> avg_brightness = mean(lamp_image);
            avg_brightness[0] /= cos(angle_/180);
            if (avg_brightness[0] < param.minAverageBrightness || avg_brightness[0] > param.maxAverageBrightness)
                continue;
            /** store possible lamps **/
            Lamp lamp_info(lamp_rect, avg_brightness[0]);
            possible_lamps.emplace_back(lamp_info);
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
                if(IsArmor(possible_lamps[l1], possible_lamps[l2], score)) {
                    if(score > max_score) {
                        pair.emplace_back(MatchLight (l1, l2, score));
                        max_score = score;
                    }
                }
            }
            if(pair.size()) {
                MatchLight ml = pair.back(); // the max score from l1 - ln
                match_lamps.emplace_back(ml);
                // find all match lights which share the same lamps, and find out the highest match score
                for(auto &i : match_lamps) {
                    // it means this two match lights share a same lamp. So we compare their match score.
                    if(i.matchIndex2 == match_lamps.back().matchIndex1 || i.matchIndex2 == match_lamps.back().matchIndex2) {  // if twice judge share
                        if(match_lamps.back().matchFactor < i.matchFactor) {
                            match_lamps.pop_back();
                            break;
                        }
                        else if(match_lamps.back().matchFactor > i.matchFactor){
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
    vector<Armor> ArmorDetector::FindArmors(vector<MatchLight> &match_lamps, vector<Lamp> &possible_lamps) {
        vector<Armor> candidate_armor;
        for(int i = 0; i < match_lamps.size(); i++) {
            Armor possible_armor(possible_lamps[match_lamps[i].matchIndex1], possible_lamps[match_lamps[i].matchIndex2], match_lamps[i].matchFactor);
            Mat num_roi = GetNumberRoi(possible_armor.pts, possible_armor.armorType);
            /* number detect */
            //possible_armor.id = classifier.SVMClassifier(num_roi);
            //double st = getTickCount();
            possible_armor.id = classifier.FigureDetection(num_roi, possible_armor.num_conf);
            //cout << "onnx inference time: " << RMTools::CalWasteTime(st) << endl;
            imshow("num roi", num_roi);
            candidate_armor.emplace_back(possible_armor);
        }
        sort(candidate_armor.begin(), candidate_armor.end(), compPriority);
        return candidate_armor;
    }

    /**
     * @brief select possible armor
     * */
    bool ArmorDetector::IsArmor(Lamp &l_1, Lamp &l_2, float &score) {
        //Mat background = imgRoi.clone();
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
        score /= 5; // normalize

//        line(background, l_1.mid_p, l_2.mid_p, Scalar(0,255,0));
//        putText(background, to_string(score), (l_1.mid_p+l_2.mid_p)/2,
//                FONT_HERSHEY_SIMPLEX,0.5, Scalar(255, 255, 255),1);
//        imshow("angle", background);

        return true;
    }

    /**
     * @brief get number location roi
     * @param pts the target 4 points
     * @param armor_type small armor or big armor
     * @return Mat
     * */
    Mat ArmorDetector::GetNumberRoi(vector<Point2f> &pts, uint8_t armor_type) {
        int warp_width = armor_type == SMALL ? warp_small_width : warp_large_width;
        int l_top = warp_height/2 - lamp_height/2;
        int l_bot = warp_height/2 + lamp_height/2;
        Point2f dst_pts[4] = {
                Point2f (0, l_top),
                Point2f (warp_width, l_top),
                Point2f (warp_width, l_bot),
                Point2f (0, l_bot)
        };
        int half_num_width = lamp_height - 2;
        int start_col = warp_width/2 - half_num_width;
        int end_col = warp_width/2 + half_num_width;
        Point2f src_pts[4] = {pts[0],pts[1],pts[2],pts[3]};
        auto warp_perspective_mat = getPerspectiveTransform(src_pts, dst_pts);
        Mat warp_dst_img;
        warpPerspective(gray_mat, warp_dst_img, warp_perspective_mat,
                        Size(warp_width, warp_height),
                        INTER_NEAREST, BORDER_CONSTANT, Scalar(0));
        warp_dst_img = warp_dst_img.colRange(start_col, end_col);
        resize(warp_dst_img, warp_dst_img, Size(SVM_IMAGE_SIZE, SVM_IMAGE_SIZE));
        pyrDown(warp_dst_img,warp_dst_img,Size(NUM_IMG_SIZE,NUM_IMG_SIZE));
        threshold(warp_dst_img, warp_dst_img, 0, 255, cv::THRESH_OTSU);
        return warp_dst_img;
    }
    /**
     * @brief select target armor from candidate armors
     * @details select the highest shoot score armor which number isn't 2
     * */
    Armor ArmorDetector::GetTargetArmor(vector<Armor> &candidate_armor) {
        int j = candidate_armor.size()-1;
        for(int i = candidate_armor.size()-1; i >=0; i--){
            if(candidate_armor[i].id != 2){
                j = i;
                break;
            }
        }
        return candidate_armor[j];
    }
    /**
     * @brief calculate the necessary parameter of the possible armor
     * */
    Armor::Armor(Lamp L1, Lamp L2, float score_) {
        Point2f pts_[4];
        match_score = score_;
        pts.resize(4);
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
        shoot_score = armorHeight * armorWidth * score_;
        armorType = (wh_ratio < 3) ? SMALL : LARGE;
        center.x = static_cast<int>((L1.rect.center.x + L2.rect.center.x) / 2);
        center.y = static_cast<int>((L1.rect.center.y + L2.rect.center.y) / 2);
        int angle = (L1.lightAngle + L2.lightAngle)/2;
        // fuck dynamic roi, give a point offset associated to incline angle in order to approximately locate the car center
        Point2i point_offset = Point2i (armorWidth*angle*0.2, -fabs(armorHeight*angle*0.1));
        rect = Rect(center - Point2i(armorWidth / 2, armorHeight / 2 * 2.27) + point_offset,
                    Size(armorWidth, armorHeight * 2.27));
    }

}