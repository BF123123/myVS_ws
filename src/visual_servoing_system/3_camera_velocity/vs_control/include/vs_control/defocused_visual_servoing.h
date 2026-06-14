#ifndef DEFOCUSED_VISUAL_SERVOING_H
#define DEFOCUSED_VISUAL_SERVOING_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "visual_servoing.h"

using namespace cv;
using namespace std;

// 继承自基础的 Visual_Servoing 类
class Defocused_Visual_Servoing : public Visual_Servoing
{
public: 
    Defocused_Visual_Servoing(int resolution_x, int resolution_y, std::string depth_strategy = "constant", double depth_constant_val = 0.24);

    // 【核心】你需要重写这个函数：计算焦流误差和散焦交互矩阵
    virtual void get_feature_error_interaction_matrix() override;

    // 你可以在这里声明你需要的其他辅助计算函数（例如计算模糊度、拉普拉斯算子等）
    // void calculate_blur_gradient(...);

    virtual void save_data_error_feature() override;
    virtual string get_method_name() override;

    // 【新增】为测试纯 DVS 逻辑引入的函数声明
    cv::Mat get_interaction_matrix_gray(cv::Mat& image_gray, cv::Mat& image_depth, cv::Mat& Camera_Intrinsic);
    void get_image_gradient(cv::Mat& image, cv::Mat& Camera_Intrinsic, cv::Mat& I_x, cv::Mat& I_y);
    cv::Mat get_image_gradient_x(cv::Mat& image);
    cv::Mat get_image_gradient_y(cv::Mat& image);

};

#endif