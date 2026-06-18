#include "vs_control/direct_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime> 
#include <chrono>
#include <ros/ros.h>

using namespace cv;
using namespace std;

Direct_Visual_Servoing::Direct_Visual_Servoing(int resolution_x, int resolution_y, 
                                               string strategy, double depth_val)
    : Visual_Servoing(resolution_x, resolution_y)
{
    // 初始化雅可比矩阵 Le (N pixels * 6 DOF) 和误差向量 e (N pixels * 1)
    this->L_e_ = Mat::zeros(resolution_x*resolution_y, 6, CV_64FC1); 
    this->error_s_ = Mat::zeros(resolution_x*resolution_y, 1, CV_64FC1);
    
    // 设置深度策略
    this->set_depth_strategy(strategy, depth_val);
}

void Direct_Visual_Servoing::get_feature_error_interaction_matrix()
{
    // 1. 检查数据有效性
    if (this->image_gray_current_.empty() || this->image_gray_desired_.empty())
    {
        ROS_WARN_THROTTLE(1.0, "[DVS Error] Matrix Empty! Skipping...");
        this->error_s_ = Mat::zeros(this->resolution_x_*this->resolution_y_, 1, CV_64FC1);
        this->L_e_     = Mat::zeros(this->resolution_x_*this->resolution_y_, 6, CV_64FC1);
        return; 
    }
    
    // 2. Realtime 模式检查
    if (this->depth_strategy_ == "realtime" && this->image_depth_current_.empty())
    {
        ROS_WARN_THROTTLE(1.0, "[DVS Error] Realtime mode requires Current Depth, but it is EMPTY!");
        this->error_s_ = Mat::zeros(this->resolution_x_*this->resolution_y_, 1, CV_64FC1);
        this->L_e_     = Mat::zeros(this->resolution_x_*this->resolution_y_, 6, CV_64FC1);
        return;
    }

    // ==========================================================
    // [修改点 1]：算法启动横幅，只打印一次，增加显眼边框
    // ==========================================================
    ROS_INFO_ONCE("\033[1;32m\n"
                  "==========================================================\n"
                  "     >>> Running Direct Visual Servoing Algorithm <<<\n"
                  "==========================================================\033[0m");


    // ==========================================================
    // [修改点 2]：插入 RAL 缩小 f 焦距的预处理方法
    // 注意：必须 clone 一份局部内参矩阵，否则每次循环都会把焦距缩小 10 倍
    // ==========================================================
    cv::Mat current_intrinsic = this->camera_intrinsic_.clone();
    
    
    if (this->enable_precond_) {
        current_intrinsic.at<double>(0, 0) *= this->precond_scale_; // fx
        current_intrinsic.at<double>(1, 1) *= this->precond_scale_; // fy
        ROS_INFO_ONCE("\033[1;36m[Pre-conditioning] ENABLED! (Scale: %.2f)\033[0m", this->precond_scale_);
    }
    // ==========================================================

    // ==========================================================
    // [修改点 3]：深度模式状态，只打印一次策略名
    // ==========================================================
    ROS_INFO_ONCE("\033[1;33m[Depth Status] DVS Strategy: [%s] \033[0m", this->depth_strategy_.c_str());

    // 步骤 1: 计算图像光度误差向量 e
    // [数学原理] e(t) = I(t) - I*
    // 这里将 HxW 的图像展平为 (H*W)x1 的向量，直接进行像素点对点的减法
    this->error_s_ = this->image_gray_current_.reshape(0, this->image_gray_current_.rows*this->image_gray_current_.cols)
                   - this->image_gray_desired_.reshape(0, this->image_gray_desired_.rows*this->image_gray_desired_.cols);  
    
    // 步骤 2: 计算交互矩阵 L_I
    Mat L_final;
    double debug_mean_depth = 0.0;

    if (this->depth_strategy_ == "realtime") 
    {
        // ================= 【模式: Realtime】 =================
        // [数学原理] Le = 0.5 * ( L(I*, Z*) + L(I(t), Z(t)) )
        // 这属于 ESM (Efficient Second-order Minimization) 类方法，能提高收敛范围和稳定性
        Mat Le_old = get_interaction_matrix_gray(this->image_gray_desired_, this->image_depth_desired_, current_intrinsic);
        Mat Le_new = get_interaction_matrix_gray(this->image_gray_current_, this->image_depth_current_, current_intrinsic);
        L_final = 0.5 * (Le_new + Le_old);

        debug_mean_depth = cv::mean(this->image_depth_current_)[0];
    }
    else if (this->depth_strategy_ == "desired_only")
    {
        // ================= 【模式: Desired Only】 =================
        // [数学原理] Le = L(I*, Z*)
        // 假设当前状态接近目标状态，直接使用目标位置计算出的恒定矩阵
        L_final = get_interaction_matrix_gray(this->image_gray_desired_, this->image_depth_desired_, current_intrinsic);
    
        debug_mean_depth = cv::mean(this->image_depth_desired_)[0];
    }
    else if (this->depth_strategy_ == "constant")
    {
        // ================= 【模式: Constant Depth】 =================
        // [数学原理] Le = L(I*, Z_const)
        // 假设所有点的深度 Z 都是同一个常数 Z_const
        Mat Z_const = Mat::ones(this->image_gray_desired_.size(), CV_64FC1) * this->depth_constant_val_;
        L_final = get_interaction_matrix_gray(this->image_gray_desired_, Z_const, current_intrinsic);
    
        debug_mean_depth = this->depth_constant_val_;
    }
    else
    {
        // 默认处理
        L_final = get_interaction_matrix_gray(this->image_gray_desired_, this->image_depth_desired_, current_intrinsic);
    }

    // [修改点 4]：当前实际应用的平均深度可能会发生变化（比如在 realtime 模式下），因此保留 Throttle 防止刷屏但能动态观测
    ROS_INFO_THROTTLE(2.0, "\033[1;36m[DVS Algo] Current Used Mean Depth: %.4f m\033[0m", debug_mean_depth);
    
    this->L_e_ = L_final;
}

// [核心函数] 计算光度交互矩阵 L_I
Mat Direct_Visual_Servoing::get_interaction_matrix_gray(Mat& image_gray, Mat& image_depth, Mat& Camera_Intrinsic)
{
    Mat I_x, I_y;
    int cnt = 0;
    Mat point_image = Mat::ones(3, 1, CV_64FC1);
    Mat xy = Mat::zeros(3, 1, CV_64FC1);
    double x, y, I_x_temp, I_y_temp;
    double Z_inv;
    Mat L_e = Mat::zeros(image_gray.rows*image_gray.cols, 6, CV_64FC1); 

    // [数学原理] 计算图像梯度
    get_image_gradient(image_gray, Camera_Intrinsic, I_x, I_y);

    for(int i = 0; i < image_gray.rows; i++) // v direction
    {
        point_image.at<double>(1,0) = i;
        for(int j = 0; j < image_gray.cols; j++) // u direction
        {
            // [数学原理] 像素坐标 (u,v) -> 归一化坐标 (x,y)
            point_image.at<double>(0,0) = j;
            xy = Camera_Intrinsic.inv() * point_image;
            x = ((double*)xy.data)[0];
            y = ((double*)xy.data)[1];

            // [数学原理] 逆深度
            Z_inv = 1.0/image_depth.at<double>(i, j);

            // 获取当前点的梯度值
            I_x_temp = ((double*)I_x.data)[i*I_x.cols+j]; 
            I_y_temp = ((double*)I_y.data)[i*I_y.cols+j]; 

            // Col 1: v_x (沿X轴平移) 
            ((double*)L_e.data)[cnt*6+0] = I_x_temp*Z_inv;
            // Col 2: v_y (沿Y轴平移) 
            ((double*)L_e.data)[cnt*6+1] = I_y_temp*Z_inv;
            // Col 3: v_z (沿Z轴平移) 
            ((double*)L_e.data)[cnt*6+2] = -(x*I_x_temp + y*I_y_temp)*Z_inv;
            // Col 4: w_x (绕X轴旋转) 
            ((double*)L_e.data)[cnt*6+3] = -x*y*I_x_temp - (1+y*y)*I_y_temp;
            // Col 5: w_y (绕Y轴旋转) 
            ((double*)L_e.data)[cnt*6+4] = (1+x*x)*I_x_temp + x*y*I_y_temp;
            // Col 6: w_z (绕Z轴旋转) 
            ((double*)L_e.data)[cnt*6+5] = -y*I_x_temp + x*I_y_temp;  
            
            cnt++;        
        }
    }

    return L_e;
}

// 计算图像梯度 (包含链式法则转换)
void Direct_Visual_Servoing::get_image_gradient(Mat& image, Mat& Camera_Intrinsic, Mat& I_x, Mat& I_y)
{
    I_x = get_image_gradient_x(image) * Camera_Intrinsic.at<double>(0, 0);
    I_y = get_image_gradient_y(image) * Camera_Intrinsic.at<double>(1, 1);
}

// 计算 u 方向的像素梯度 (Central Difference)
Mat Direct_Visual_Servoing::get_image_gradient_x(Mat& image)
{
    Mat I_x = Mat::zeros(image.rows, image.cols, CV_64FC1);
    int up, down;
    for(int i = 0; i < image.cols; i++)
    {
        up = i+1;
        down = i-1;
        if (up > image.cols-1) up = image.cols-1;
        if (down < 0) down = 0;
        
        I_x.col(i) = (image.col(up) - image.col(down)) / (up - down); 
    }
    return I_x;
}

// 计算 v 方向的像素梯度
Mat Direct_Visual_Servoing::get_image_gradient_y(Mat& image)
{
    Mat I_y = Mat::zeros(image.rows, image.cols, CV_64FC1);
    int up, down;
    for(int i = 0; i < image.rows; i++)
    {
        up = i+1;
        down = i-1;
        if (up > image.rows-1) up = image.rows-1;
        if (down < 0) down = 0;
        
        I_y.row(i) = (image.row(up) - image.row(down)) / (up - down); 
    }
    return I_y;
}

void Direct_Visual_Servoing::save_data_error_feature()
{
    Mat error_ave = (this->error_s_.t() * this->error_s_) / (this->error_s_.rows * this->error_s_.cols);
    this->data_vs.error_feature_.push_back(error_ave);
}

string Direct_Visual_Servoing::get_method_name()
{
    return "Direct_Visual_Servoing";
}