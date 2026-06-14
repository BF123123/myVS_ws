#include "vs_control/defocused_visual_servoing_RAL.h"
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

using namespace cv;
using namespace std;

Defocused_Visual_Servoing_RAL::Defocused_Visual_Servoing_RAL(int resolution_x, int resolution_y, std::string depth_strategy, double depth_constant_val)
    : Visual_Servoing(resolution_x, resolution_y)
{
    this->L_e_ = Mat::zeros(resolution_x * resolution_y, 6, CV_64FC1); 
    this->error_s_ = Mat::zeros(resolution_x * resolution_y, 1, CV_64FC1);
    
    // 记录策略和恒定深度值
    this->depth_strategy_ = depth_strategy;
    this->depth_constant_val_ = depth_constant_val;
}

void Defocused_Visual_Servoing_RAL::get_feature_error_interaction_matrix()
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
                  "   >>> Running Defocused Visual Servoing (RAL) <<<\n"
                  "==========================================================\033[0m");

    // 1. 获取相机内参 (对应论文 Eq 12)
    double fx = this->camera_intrinsic_.at<double>(0, 0);
    double fy = this->camera_intrinsic_.at<double>(1, 1);
    double cx = this->camera_intrinsic_.at<double>(0, 2);
    double cy = this->camera_intrinsic_.at<double>(1, 2);

    // ==========================================================
    // 插入RAL缩小f焦距方法
    // ==========================================================
    bool enable_precond = true;     // 开关：设为 true 开启预处理，false 关闭
    double precond_scale = 0.1;     // 缩悉因子：论文使用的是 0.1
    
    if (enable_precond) {
        fx *= precond_scale;
        fy *= precond_scale;
        // ==========================================================
        // [修改点 2]：预处理配置只打印一次
        // ==========================================================
        ROS_INFO_ONCE("\033[1;36m[Pre-conditioning] ENABLED! (Scale: %.2f)\033[0m", precond_scale);
    }
    // ==========================================================

    // ==========================================================
    // 2. 散焦模型光学参数 (对应论文 Sec III-B)
    // ==========================================================
    double Z_f = 0.23;   // 镜头的聚焦深度 (m)
    double R_f = 0.004;  // 镜头的光圈半径 (m)

    // 【新增优化】：将 Eq (20) 中与像素深度 Z 无关的常数部分提取到循环外计算，大幅提升帧率
    // 论文 V.A 节实验设置中提到 Yakumo 镜头物理焦距 f 为 17mm
    double f_metric = 0.016;      // 物理焦距 (m)
    double D = 2.0 * R_f;         // 光圈直径 (m) 
    
    // 对应 Eq (20) 中散焦系数的常数部分: (D * f_x) / ( 6 * (Z_f - f) )
    // 这里利用了 f_x = f / k_u 的关系进行了化简替换
    double K_paper_const = (D * fx) / (6.0 * (Z_f - f_metric)); 

    // 3. 计算图像特征
    cv::Mat I_current = this->image_gray_current_;
    cv::Mat I_desired = this->image_gray_desired_;

    cv::Mat I_u, I_v, delta_I;
    
    // 计算 X 和 Y 方向的一阶像素梯度 (对应 Eq 20 向量的第一项 \nabla_u I_d^\top)
    cv::Sobel(I_current, I_u, CV_64F, 1, 0, 3, 1.0 / 8.0);
    cv::Sobel(I_current, I_v, CV_64F, 0, 1, 3, 1.0 / 8.0);
    
    // 计算二阶拉普拉斯算子 (对应 Eq 20 向量的第二项 \Delta_u I_d)
    cv::Laplacian(I_current, delta_I, CV_64F, 1, 1.0);

    int M = I_current.rows;
    int N = I_current.cols;
    int num_pixels = M * N;

    // ==========================================================
    // [修改点 3]：深度状态配置，只打印一次
    // ==========================================================
    if (this->depth_strategy_ == "constant" || this->image_depth_current_.empty()) {
        ROS_INFO_ONCE("\033[1;33m[Depth Status] CONSTANT MODE OR NO DEPTH! Using Constant Z = %.3f m \033[0m", this->depth_constant_val_);
    } else {
        double center_z = this->image_depth_current_.at<double>(M / 2, N / 2);
        ROS_INFO_ONCE("\033[1;32m[Depth Status] DEPTH CAMERA ACTIVE! Initial Center Z = %.3f m \033[0m", center_z);
    }
    // 移除了重复的 "Running RAL algorithm" 打印，保持控制台整洁

    // 4. 重置交互矩阵 L_e 和 误差向量 error_s_
    this->L_e_ = cv::Mat::zeros(num_pixels, 6, CV_64F);
    this->error_s_ = cv::Mat::zeros(num_pixels, 1, CV_64F);

    /* =================== [实验1.5: 符号与方向解剖 - 初始化] =================== */
    double exp_sum_abs_opt = 0.0, exp_sum_abs_def = 0.0;
    int exp_count = 0, count_same_sign = 0, count_oppo_sign = 0;
    /* ===================================================================== */

    int idx = 0;
    // 5. 遍历每个像素，构建大规模交互矩阵
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
            // [A. 计算特征误差] (对应 Eq 15)
            this->error_s_.at<double>(idx, 0) = ptr_I_curr[u] - ptr_I_des[u];

            // [B. 获取深度 Z]
            double Z;
            if (this->depth_strategy_ == "constant" || this->image_depth_current_.empty() || ptr_Z == nullptr) {
                Z = this->depth_constant_val_;  
            } else {
                Z = (ptr_Z[u] > 0.001) ? ptr_Z[u] : this->depth_constant_val_; 
            }
            
            // 归一化图像坐标 (对应 Eq 12, 及后续 L_u 中的 X/Z, Y/Z)
            double x_n = (u - cx) / fx;
            double y_n = (v - cy) / fy;

            double I_du = ptr_I_u[u];
            double I_dv = ptr_I_v[u];
            double I_lap = ptr_delta[u];

            // ==========================================================
            // [C. 计算散焦项系数]
            // ==========================================================
            
            // +++ [修正代码]：严格对应论文 Eq (20) +++
            // 代入深度 Z，得到最终的 K_paper = (D * fx) / (6 * (Z_f - f) * Z)
            double K_RAL = K_paper_const / Z; 
            // ==========================================================

            // [D. 构建交互矩阵 L_cam] (严格对应 Eq 20 展开式)
            
            // 平移 X, Y (纯光流部分，Eq 21 的前两列为0)
            double L1 =  (I_du * fx) / Z;
            double L2 =  (I_dv * fy) / Z;
            
            // 平移 Z (光流的 T_z 项 + 散焦拉普拉斯项)
            // 对应 Eq 20: -\nabla_u I_d^\top L_u(对应项) - \Delta_u I_d * K * (-1)
            double L3 = -(x_n * I_du * fx + y_n * I_dv * fy) / Z + I_lap * K_RAL;

            /* =================== [实验1.5: 符号与方向解剖 - RAL] =================== */
            double term_opt = -(x_n * I_du * fx + y_n * I_dv * fy) / Z;
            double term_def = +I_lap * K_RAL; // RAL 的散焦项符号是加号

            exp_sum_abs_opt += std::abs(term_opt);
            exp_sum_abs_def += std::abs(term_def);
            
            // 统计方向是否一致
            if (term_opt * term_def > 0) count_same_sign++;
            else if (term_opt * term_def < 0) count_oppo_sign++;
            
            exp_count++;
            /* ===================================================================== */

            // 旋转 X, Y (完美利用 Eq 21 中 -Y 和 X 结构进行的化简)
            double L4 = -I_dv * fy + y_n * Z * L3;
            double L5 =  I_du * fx - x_n * Z * L3;
            
            // 旋转 Z (纯光流部分，Eq 21 最后一列为0)
            double L6 =  x_n * I_dv * fy - y_n * I_du * fx;

            // 填入交互矩阵的对应行
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
        // 计算同方向的比例
        double align_rate = (double)count_same_sign / (count_same_sign + count_oppo_sign + 1e-6) * 100.0;
        
        // 实验数据保持 Throttle 打印
        ROS_INFO_THROTTLE(1.0, "\033[1;36m[EXP 1.5] Ratio: %.1f | Same Direction: %.1f%% | Opp Direction: %.1f%%\033[0m", 
                          (avg_def > 1e-12) ? (avg_opt / avg_def) : -1.0, align_rate, 100.0 - align_rate);
    }
    /* ======================================================= */

    // ==========================================================
    // 6. 实时计算并打印交互矩阵 L_e 的条件数 (如果开启的话，保留 Throttle)
    // ==========================================================
    /*
    cv::Mat H = this->L_e_.t() * this->L_e_;
    cv::Mat S; 
    cv::SVD::compute(H, S, cv::SVD::NO_UV);
    double max_sv_sq = S.at<double>(0);      
    double min_sv_sq = S.at<double>(5);      

    if (min_sv_sq > 1e-12) 
    {
        double condition_number = std::sqrt(max_sv_sq / min_sv_sq);
        ROS_INFO_THROTTLE(1.0, "\033[1;33m[Matrix Condition] cond(L_e) = %.3f \033[0m", condition_number);
    } 
    else 
    {
        ROS_WARN_THROTTLE(1.0, "\033[1;31m[Matrix Condition] Interaction Matrix is NEAR SINGULAR! cond(L_e) is infinite.\033[0m");
    }
    */
    // ==========================================================
}

void Defocused_Visual_Servoing_RAL::save_data_error_feature()
{
    Mat error_ave = (this->error_s_.t() * this->error_s_) / (this->error_s_.rows * this->error_s_.cols);
    this->data_vs.error_feature_.push_back(error_ave);
}

string Defocused_Visual_Servoing_RAL::get_method_name()
{
    return "Defocused_Visual_Servoing_RAL";
}