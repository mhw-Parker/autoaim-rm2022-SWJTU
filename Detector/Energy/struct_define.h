#ifndef STRUCT_DEFINE_H
#define STRUCT_DEFINE_H
#include<opencv2/opencv.hpp>
#include<iostream>

//风车参数
struct WindmillParamFlow {
	int RED_GRAY_THRESH;//敌方红色时的阈值
	int BLUE_GRAY_THRESH;//敌方蓝色时的阈值
	float armor_contour_area_max;//装甲板的相关筛选参数
	float armor_contour_area_min;
	float armor_contour_length_max;
	float armor_contour_length_min;
	float armor_contour_width_max;
	float armor_contour_width_min;
	float armor_contour_hw_ratio_max;
	float armor_contour_hw_ratio_min;

	float flow_strip_fan_contour_area_max;//流动条所在扇叶的相关筛选参数
	float flow_strip_fan_contour_area_min;
	float flow_strip_fan_contour_length_max;
	float flow_strip_fan_contour_length_min;
	float flow_strip_fan_contour_width_max;
	float flow_strip_fan_contour_width_min;
	float flow_strip_fan_contour_hw_ratio_max;
	float flow_strip_fan_contour_hw_ratio_min;
	float flow_strip_fan_contour_area_ratio_max;
	float flow_strip_fan_contour_area_ratio_min;

	float target_intersection_contour_area_min;

	float twin_point_max;

	float Center_R_Control_area_max;//中心R的相关参数筛选
	float Center_R_Control_area_min;
	float Center_R_Control_length_max;
	float Center_R_Control_length_min;
	float Center_R_Control_width_max;
	float Center_R_Control_width_min;
	float Center_R_Control_radio_max;
	float Center_R_Control_radio_min;
	float Center_R_Control_area_radio_min;
	float Center_R_Control_area_intersection_area_min;

	float flow_area_max;//扇叶相关参数筛选
	float flow_area_min;
	float flow_length_max;
	float flow_length_min;
	float flow_width_max;
	float flow_width_min;
	float flow_aim_max;
	float flow_aim_min;
	float flow_area_ratio_min;
};

typedef struct {
    int radius;
    float angle;
    double time_stamp;
} polarLocal;

typedef struct Blade_{
    int armor_index;
    int flow_strip_fan_index;
    Blade_(int i_, int j_)
    {
        flow_strip_fan_index = i_;
        armor_index = j_;
    }

    Blade_()
    {
        armor_index = 0;
        flow_strip_fan_index = 0;
    }
}Blade;















#endif