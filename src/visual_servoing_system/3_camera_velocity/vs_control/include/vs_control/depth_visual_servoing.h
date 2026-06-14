#ifndef DEPTH_VISUAL_SERVOING_H
#define DEPTH_VISUAL_SERVOING_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "visual_servoing.h"

using namespace cv;
using namespace std;

class Depth_Visual_Servoing : public Visual_Servoing
{
public:
    // 构造函数
    Depth_Visual_Servoing(int resolution_x = 640, int resolution_y = 480);

    // 【核心】计算深度误差和深度交互矩阵
    virtual void get_feature_error_interaction_matrix();

    // 辅助：计算深度梯度 (Z_x, Z_y)
    // 注意：这里的梯度计算与 DVS 类似，但输入是 Z
    void get_depth_gradient(const Mat& depth_image, const Mat& K, Mat& Z_x, Mat& Z_y);
    Mat get_gradient_x(const Mat& img);
    Mat get_gradient_y(const Mat& img);

    // 辅助：保存调试信息
    virtual void save_data_error_feature();
    virtual string get_method_name();
};

#endif