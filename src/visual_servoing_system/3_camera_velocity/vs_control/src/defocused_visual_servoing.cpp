#include "vs_control/defocused_visual_servoing.h"
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <cmath> // 引入数学库以使用 std::sqrt

Defocused_Visual_Servoing::Defocused_Visual_Servoing(int resolution_x, int resolution_y, std::string depth_strategy, double depth_constant_val)
    : Visual_Servoing(resolution_x, resolution_y)
{
    this->L_e_ = cv::Mat::zeros(resolution_x * resolution_y, 6, CV_64FC1); 
    this->error_s_ = cv::Mat::zeros(resolution_x * resolution_y, 1, CV_64FC1);
    
    // 记录策略和恒定深度值
    this->depth_strategy_ = depth_strategy;
    this->depth_constant_val_ = depth_constant_val;
}

void Defocused_Visual_Servoing::get_feature_error_interaction_matrix()
{
    // 安全检查
    if (this->image_gray_current_.empty() || this->image_gray_desired_.empty()) {
        ROS_WARN_THROTTLE(1.0, "[Defocused VS] Images empty! Skipping...");
        return; 
    }

    // ==========================================================
    // [修改点 1]：算法启动横幅，只打印一次，增加显眼边框
    // ==========================================================
    ROS_INFO_ONCE("\033[1;32m\n"
                  "==========================================================\n"
                  "   >>> Running Exact Defocused Visual Servoing Algorithm <<<\n"
                  "==========================================================\033[0m");

    // 1. 获取相机内参 (焦距与主点)
    double fx = this->camera_intrinsic_.at<double>(0, 0);
    double fy = this->camera_intrinsic_.at<double>(1, 1);
    double cx = this->camera_intrinsic_.at<double>(0, 2);
    double cy = this->camera_intrinsic_.at<double>(1, 2);

    // ==========================================================
    // 插入RAL缩小f焦距预处理方法 (解决旋转平移强耦合)
    // ==========================================================
    
    if (this->enable_precond_) {
        fx *= this->precond_scale_;
        fy *= this->precond_scale_;      
        ROS_INFO_ONCE("\033[1;36m[Pre-conditioning] ENABLED! (Scale: %.2f)\033[0m", this->precond_scale_);
    }
    // ==========================================================

    // 2. 散焦模型光学参数
    double Z_f = 0.23;         // 镜头的聚焦深度/工作距离 (米)
    double R_f = 0.004;        // 镜头的光圈半径 (m)
    double f_metric = 0.016;   // 镜头的物理焦距 (米)，例如 16mm 镜头

    double D = 2.0 * R_f;      // 光圈直径 (m) 

    // 提前在循环外计算精确散焦系数的常数部分
    double K_exact_const = (D * D * fx * fx) / (36.0 * Z_f);//double K_exact_const = (D * D * fx * fx) / 36.0 * Z_f;居然也收敛

    // 3. 计算图像特征
    cv::Mat I_current = this->image_gray_current_;
    cv::Mat I_desired = this->image_gray_desired_;

    cv::Mat I_u, I_v, delta_I;
    
    // 计算 X 和 Y 方向的一阶像素梯度
    cv::Sobel(I_current, I_u, CV_64F, 1, 0, 3, 1.0 / 8.0);
    cv::Sobel(I_current, I_v, CV_64F, 0, 1, 3, 1.0 / 8.0);
    
    // 计算二阶拉普拉斯算子 
    cv::Laplacian(I_current, delta_I, CV_64F, 1, 1.0);

    int M = I_current.rows;
    int N = I_current.cols;
    int num_pixels = M * N;

    // ==========================================================
    // [修改点 3]：深度状态配置，只在程序开始时打印一次当前生效的模式
    // ==========================================================
    if (this->depth_strategy_ == "constant" || this->image_depth_current_.empty()) {
        ROS_INFO_ONCE("\033[1;33m[Depth Status] CONSTANT MODE OR NO DEPTH! Using Constant Z = %.3f m \033[0m", this->depth_constant_val_);
    } else {
        double center_z = this->image_depth_current_.at<double>(M / 2, N / 2);
        // 注意：这里如果用 ONCE，它只会打印第一帧的 center_z。如果需要动态观察中心深度，请改回 THROTTLE
        ROS_INFO_ONCE("\033[1;32m[Depth Status] DEPTH CAMERA ACTIVE! Initial Center Z = %.3f m \033[0m", center_z);
    }

    // 4. 重置交互矩阵 L_e 和 误差向量 error_s_
    this->L_e_ = cv::Mat::zeros(num_pixels, 6, CV_64F);
    this->error_s_ = cv::Mat::zeros(num_pixels, 1, CV_64F);

    /* =================== [实验1.5: 符号与方向解剖 - 初始化] =================== */
    double exp_sum_abs_opt = 0.0, exp_sum_abs_def = 0.0;
    int exp_count = 0, count_same_sign = 0, count_oppo_sign = 0;
    /* ===================================================================== */

    int idx = 0;
    
    // 5. 遍历每个像素，构建大规模精确交互矩阵
    for (int v = 0; v < M; ++v)
    {
        const double* ptr_I_curr = I_current.ptr<double>(v);
        const double* ptr_I_des  = I_desired.ptr<double>(v);
        const double* ptr_I_u    = I_u.ptr<double>(v);
        const double* ptr_I_v    = I_v.ptr<double>(v);
        const double* ptr_delta  = delta_I.ptr<double>(v);
        const double* ptr_Z      = this->image_depth_current_.empty() ? nullptr : this->image_depth_current_.ptr<double>(v);

        for (int u = 0; u < N; ++u)
        {
            // [A. 计算特征误差]
            this->error_s_.at<double>(idx, 0) = ptr_I_curr[u] - ptr_I_des[u];

            // [B. 获取深度 Z]
            double Z;
            if (this->depth_strategy_ == "constant" || this->image_depth_current_.empty() || ptr_Z == nullptr) {
                Z = this->depth_constant_val_;  
            } else {
                Z = (ptr_Z[u] > 0.001) ? ptr_Z[u] : this->depth_constant_val_; 
            }
            
            // 归一化图像坐标
            double x_n = (u - cx) / fx;
            double y_n = (v - cy) / fy;

            double I_du = ptr_I_u[u];
            double I_dv = ptr_I_v[u];
            double I_lap = ptr_delta[u];

            // [C. 计算绝对精确的散焦项系数 K_defocus]
            double K_defocus = (K_exact_const / Z) * (1.0 / Z_f - 1.0 / Z);

            // [D. 构建精确焦流交互矩阵 L_cam]
            double L1 =  (I_du * fx) / Z;
            double L2 =  (I_dv * fy) / Z;
            
            double L3 = -(x_n * I_du * fx + y_n * I_dv * fy) / Z - I_lap * K_defocus;

            /* =================== [实验1.5: 符号与方向解剖 - EXACT] =================== */
            double term_opt = -(x_n * I_du * fx + y_n * I_dv * fy) / Z;
            double term_def = -I_lap * K_defocus; 

            exp_sum_abs_opt += std::abs(term_opt);
            exp_sum_abs_def += std::abs(term_def);
            
            if (term_opt * term_def > 0) count_same_sign++;
            else if (term_opt * term_def < 0) count_oppo_sign++;
            
            exp_count++;
            
            // 旋转项
            double L4 = -I_dv * fy + y_n * Z * L3;
            double L5 =  I_du * fx - x_n * Z * L3;
            double L6 =  x_n * I_dv * fy - y_n * I_du * fx;

            // 填入交互矩阵
            double* row_Le = this->L_e_.ptr<double>(idx);
            row_Le[0] = L1;
            row_Le[1] = L2;
            row_Le[2] = L3;
            row_Le[3] = L4;
            row_Le[4] = L5;
            row_Le[5] = L6;

            idx++;
        }
    }

    /* =================== [实验1.5: 结果打印] =================== */
    if (exp_count > 0) {
        double avg_opt = exp_sum_abs_opt / exp_count;
        double avg_def = exp_sum_abs_def / exp_count;
        double align_rate = (double)count_same_sign / (count_same_sign + count_oppo_sign + 1e-6) * 100.0;
        
        // 实时计算的数据依然使用 THROTTLE，否则只会打印无意义的第一帧
        ROS_INFO_THROTTLE(1.0, "\033[1;36m[EXP 1.5] Ratio: %.1f | Same Direction: %.1f%% | Opp Direction: %.1f%%\033[0m", 
                          (avg_def > 1e-12) ? (avg_opt / avg_def) : -1.0, align_rate, 100.0 - align_rate);
    }
}

// ==========================================================

void Defocused_Visual_Servoing::save_data_error_feature()
{
    cv::Mat error_ave = (this->error_s_.t() * this->error_s_) / (this->error_s_.rows * this->error_s_.cols);
    this->data_vs.error_feature_.push_back(error_ave);
}

std::string Defocused_Visual_Servoing::get_method_name()
{
    return "Defocused_Visual_Servoing";
}