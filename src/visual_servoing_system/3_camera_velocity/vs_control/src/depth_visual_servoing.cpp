#include "vs_control/depth_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <ros/ros.h> 

Depth_Visual_Servoing::Depth_Visual_Servoing(int resolution_x, int resolution_y)
    : Visual_Servoing(resolution_x, resolution_y)
{
    // 初始化矩阵大小: 雅可比矩阵 L_e 维度为 (像素数 N) x (自由度 6)
    this->L_e_ = Mat::zeros(resolution_x * resolution_y, 6, CV_64FC1);
    this->error_s_ = Mat::zeros(resolution_x * resolution_y, 1, CV_64FC1);
}

void Depth_Visual_Servoing::get_feature_error_interaction_matrix()
{
    // ==================== 【调试探针】 ====================
    if (!this->image_depth_current_.empty()) {
        cv::Scalar mean_depth = cv::mean(this->image_depth_current_);
        ROS_WARN_THROTTLE(2.0, ">>> [VERIFIED] Pure Depth Running! Mean Depth: %.3f m <<<", mean_depth[0]);
    }

    // 1. 安全检查
    if (this->image_depth_current_.empty() || this->image_depth_desired_.empty()) {
        ROS_WARN_THROTTLE(1.0, "[DepthVS] Depth images empty, skipping...");
        return;
    }

    // 2. 计算误差向量
    // [数学原理] 定义误差函数 e(t) = Z(t) - Z*
    // 这是一个 N维向量，表示每个像素点的深度差异
    this->error_s_ = this->image_depth_current_.reshape(0, this->resolution_x_ * this->resolution_y_) - 
                     this->image_depth_desired_.reshape(0, this->resolution_x_ * this->resolution_y_);

    // 计算 RMSE (均方根误差) 用于量化收敛情况
    // [数学原理] RMSE = sqrt( sum((Z_i - Z*_i)^2) / N )
    double depth_error_norm = cv::norm(this->error_s_, cv::NORM_L2);
    double rmse = depth_error_norm / sqrt(this->resolution_x_ * this->resolution_y_);
    ROS_INFO_THROTTLE(0.5, ">>> [DepthVS Status] Real Depth RMSE: %.4f m | Vz_cmd: Check Rqt <<<", rmse);

    // 3. 计算深度梯度 Z_x, Z_y
    // [数学原理] 我们需要的是物理空间梯度 dZ/dx, dZ/dy
    // 而不是图像像素梯度 dZ/du, dZ/dv。这步转换在 get_depth_gradient 函数内完成。
    Mat Z_x, Z_y;
    get_depth_gradient(this->image_depth_current_, this->camera_intrinsic_, Z_x, Z_y);

    // 4. 构建交互矩阵 (Interaction Matrix L_Z)
    // [数学原理] 描述深度 Z 的变化率与相机速度 v_c 的关系: dZ/dt = L_Z * v_c
    int cnt = 0;
    // 获取相机内参: fx, fy, cx, cy
    double cx = this->camera_intrinsic_.at<double>(0, 2);
    double cy = this->camera_intrinsic_.at<double>(1, 2);
    double fx = this->camera_intrinsic_.at<double>(0, 0);
    double fy = this->camera_intrinsic_.at<double>(1, 1);

    double* ptr_Z = (double*)this->image_depth_current_.data;
    double* ptr_Zx = (double*)Z_x.data; // 指向 dZ/dx
    double* ptr_Zy = (double*)Z_y.data; // 指向 dZ/dy
    double* ptr_Le = (double*)this->L_e_.data;

    // [调试参数] 符号翻转因子
    // 数学上标准推导通常不仅包含负号，还取决于坐标系定义 (Z轴向前/向后)
    double sign_factor = -1.0; 

    for (int v = 0; v < this->resolution_y_; v++)
    {
        for (int u = 0; u < this->resolution_x_; u++)
        {
            double Z = ptr_Z[cnt];
            
            // 掩膜处理：忽略无效点或过远的点
            if (Z < 0.1 || Z > 5.0 || std::isnan(Z)) 
            {
                ((double*)this->error_s_.data)[cnt] = 0.0; // 强制误差为0，不产生控制量
                for(int k=0; k<6; ++k) ptr_Le[cnt*6 + k] = 0.0; // 对应的矩阵行设为0
            }
            else
            {
                // [数学原理] 归一化图像平面坐标转换
                // x = (u - c_x) / f_x
                // y = (v - c_y) / f_y
                // 这是将像素坐标 (pixel) 转换为物理成像平面坐标 (无量纲或米)
                double x = (u - cx) / fx;
                double y = (v - cy) / fy;
                
                // 获取当前点的深度梯度
                double zx = ptr_Zx[cnt]; // = dZ/dx
                double zy = ptr_Zy[cnt]; // = dZ/dy

                // [数学原理] 深度交互矩阵 L_Z 的标准公式
                // 每一行对应一个像素点，6列对应 [v_x, v_y, v_z, w_x, w_y, w_z]
                
                // Col 1: v_x 系数 -> (dZ/dx) / Z
                ptr_Le[cnt*6 + 0] = sign_factor * (zx / Z);
                
                // Col 2: v_y 系数 -> (dZ/dy) / Z
                ptr_Le[cnt*6 + 1] = sign_factor * (zy / Z);
                
                // Col 3: v_z 系数 -> -(1 + x*(dZ/dx) + y*(dZ/dy)) / Z
                // 解释: Z轴移动对深度的影响最大，主要由 -1/Z 决定，梯度项是修正
                ptr_Le[cnt*6 + 2] = sign_factor * (-(1.0 + x*zx + y*zy) / Z);
                
                // Col 4: w_x (绕X轴旋转) -> -(y + dZ/dy)
                // 解释: 旋转引起的深度变化由位置 y 和表面倾斜度 dZ/dy 共同决定
                ptr_Le[cnt*6 + 3] = sign_factor * (-(y + zy));
                
                // Col 5: w_y (绕Y轴旋转) -> (x + dZ/dx)
                ptr_Le[cnt*6 + 4] = sign_factor * (x + zx);
                
                // Col 6: w_z (绕Z轴旋转) -> y*(dZ/dx) - x*(dZ/dy)
                // 解释: 面内旋转，主要由图像坐标和梯度方向决定
                ptr_Le[cnt*6 + 5] = sign_factor * (y*zx - x*zy); 
            }
            cnt++;
        }
    }
}

// 辅助函数：计算物理梯度的封装
void Depth_Visual_Servoing::get_depth_gradient(const Mat& depth, const Mat& K, Mat& Z_x, Mat& Z_y)
{
    // 1. 先计算图像像素空间的梯度: dZ/du, dZ/dv
    Mat grad_u = get_gradient_x(depth);
    Mat grad_v = get_gradient_y(depth);

    // 2. [数学原理] 链式法则 (Chain Rule) 转换到归一化坐标系
    // dZ/dx = (dZ/du) * (du/dx)
    // 因为 u = x * f_x + c_x，所以 du/dx = f_x
    // 最终: dZ/dx = grad_u * f_x
    
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);

    Z_x = grad_u * fx; // 物理梯度 X
    Z_y = grad_v * fy; // 物理梯度 Y
}

// 辅助函数：Sobel 算子计算
Mat Depth_Visual_Servoing::get_gradient_x(const Mat& img)
{
    Mat res;
    // 计算 Sobel 梯度 (kernel size = 3)
    cv::Sobel(img, res, CV_64F, 1, 0, 3);
    
    // [数学原理] Sobel 算子归一化
    // 3x3 Sobel 核在 X 方向是 [-1 0 1; -2 0 2; -1 0 1]
    // 对于单纯的线性斜坡 (0, 1, 2)，Sobel 的响应和是 1*1 + 2*1 + 1*1 = 4
    // 还要考虑窗口平滑效应，通常归一化系数取 8.0 以获得正确的一阶导数数值近似
    return res / 8.0; 
}

Mat Depth_Visual_Servoing::get_gradient_y(const Mat& img)
{
    Mat res;
    cv::Sobel(img, res, CV_64F, 0, 1, 3);
    // 同理，Y 方向梯度也需要除以 8.0
    return res / 8.0;
}

void Depth_Visual_Servoing::save_data_error_feature()
{
    double error_norm = cv::norm(this->error_s_, cv::NORM_L2);
    Mat err_mat = (Mat_<double>(1,1) << error_norm);
    this->data_vs.error_feature_.push_back(err_mat);
}

string Depth_Visual_Servoing::get_method_name()
{
    return "Pure_Depth_Visual_Servoing";
}