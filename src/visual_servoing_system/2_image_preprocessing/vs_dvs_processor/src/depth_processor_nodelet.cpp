#include "vs_dvs_processor/depth_processor_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <numeric>

namespace vs_dvs_processor
{

DepthProcessorNodelet::DepthProcessorNodelet() 
    : gaussian_kernel_size_(7), gaussian_sigma_(1.5), visualize_scale_(800.0),
      use_temporal_filter_(true), temporal_alpha_(0.2), frame_count_(0)
{
}

DepthProcessorNodelet::~DepthProcessorNodelet()
{
}

void DepthProcessorNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    NODELET_INFO("Initializing Depth Processor (Quantified)...");

    private_nh_.param("use_gaussian_blur", use_gaussian_blur_, true);
    private_nh_.param("gaussian_kernel_size", gaussian_kernel_size_, 7);
    private_nh_.param("gaussian_sigma", gaussian_sigma_, 1.5);
    private_nh_.param("visualize_scale", visualize_scale_, 800.0);
    
    // 【新增】时间滤波配置
    // alpha = 0.2 意味着新数据占 20%，历史数据占 80%，非常平滑
    private_nh_.param("use_temporal_filter", use_temporal_filter_, true);
    private_nh_.param("temporal_alpha", temporal_alpha_, 0.2);

    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    
    pub_depth_processed_ = it_->advertise("/vs/depth/processed", 1);      
    pub_depth_float_     = it_->advertise("/vs/depth/processed_float", 1); 
    
    // 可视化：梯度幅值 (模长)
    pub_grad_mag_ = it_->advertise("/vs/debug/grad_magnitude", 1);

    sub_depth_ = it_->subscribe("/camera/aligned_depth_to_color/image_raw", 1, 
                                &DepthProcessorNodelet::depthCallback, this, 
                                image_transport::TransportHints("raw"));

    NODELET_INFO("Depth Processor Ready. Temporal Filter: %s (alpha=%.2f)", 
                 use_temporal_filter_ ? "ON" : "OFF", temporal_alpha_);
}

void DepthProcessorNodelet::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        std::lock_guard<std::mutex> lock(img_mutex_);
        
        cv::Mat raw_depth = cv_bridge::toCvShare(msg, "mono16")->image;
        
        // 1. 处理深度 (转浮点 + 空间高斯 + 时间滤波)
        cv::Mat depth_f = processDepth(raw_depth);

        // 2. 发布数据
        std_msgs::Header header = msg->header;
        pub_depth_float_.publish(cv_bridge::CvImage(header, "32FC1", depth_f).toImageMsg());

        cv::Mat depth_u16;
        depth_f.convertTo(depth_u16, CV_16U, 1000.0); 
        pub_depth_processed_.publish(cv_bridge::CvImage(header, "mono16", depth_u16).toImageMsg());

        // 3. 量化分析与可视化
        analyzeAndVisualize(depth_f, header);

    } catch (cv_bridge::Exception& e) {
        NODELET_ERROR("Depth callback exception: %s", e.what());
    }
}

cv::Mat DepthProcessorNodelet::processDepth(const cv::Mat& raw_depth_mm)
{
    cv::Mat depth_curr;
    // 转米
    raw_depth_mm.convertTo(depth_curr, CV_32F, 0.001);

    // A. 空间滤波 (Gaussian)
    if (use_gaussian_blur_)
    {
        int k = gaussian_kernel_size_;
        if (k % 2 == 0) k++; 
        cv::GaussianBlur(depth_curr, depth_curr, cv::Size(k, k), gaussian_sigma_);
    }

    // B. 时间滤波 (Exponential Moving Average)
    // Formula: Out = alpha * New + (1 - alpha) * Old
    if (use_temporal_filter_)
    {
        if (img_depth_filtered_.empty() || img_depth_filtered_.size() != depth_curr.size())
        {
            depth_curr.copyTo(img_depth_filtered_); // 第一帧直接赋值
        }
        else
        {
            // img_depth_filtered_ = img_depth_filtered_ * (1-alpha) + depth_curr * alpha
            cv::addWeighted(depth_curr, temporal_alpha_, img_depth_filtered_, 1.0 - temporal_alpha_, 0.0, img_depth_filtered_);
        }
        return img_depth_filtered_.clone();
    }

    return depth_curr;
}

void DepthProcessorNodelet::analyzeAndVisualize(const cv::Mat& img_float, const std_msgs::Header& header)
{
    // 1. 计算梯度
    cv::Mat grad_x, grad_y;
    cv::Sobel(img_float, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(img_float, grad_y, CV_32F, 0, 1, 3);

    // 2. 计算梯度幅值 (Magnitude) = sqrt(gx^2 + gy^2)
    cv::Mat grad_mag;
    cv::magnitude(grad_x, grad_y, grad_mag);

    // 3. 量化指标计算
    // (1) 平均梯度强度 (Signal Strength)
    cv::Scalar mean_mag = cv::mean(grad_mag);
    double signal_strength = mean_mag[0];

    // (2) 噪声抖动 (Temporal Jitter)
    double jitter_level = 0.0;
    if (!img_last_grad_mag_.empty())
    {
        // 计算前后两帧梯度的平均绝对差 (L1 Norm)
        cv::Mat diff;
        cv::absdiff(grad_mag, img_last_grad_mag_, diff);
        jitter_level = cv::mean(diff)[0];
    }
    grad_mag.copyTo(img_last_grad_mag_); // 更新历史

    // 4. 终端打印 (每30帧打印一次，避免刷屏)
    frame_count_++;
    if (frame_count_ % 30 == 0)
    {
        // 简单评级
        std::string quality = "UNKNOWN";
        if (signal_strength < 0.002) quality = "BAD (Too Flat)";
        else if (jitter_level > signal_strength * 0.5) quality = "BAD (Too Noisy)";
        else quality = "GOOD";

        // 注意：梯度单位是 m/pixel，数值通常很小 (e.g. 0.005)
        // 放大 1000 倍显示方便阅读 (mm/pixel)
        NODELET_INFO("[Quality] Signal(Grad): %.2f | Noise(Jitter): %.2f | Ratio: %.1f%% -> %s", 
                     signal_strength * 1000, 
                     jitter_level * 1000,
                     (signal_strength > 1e-6) ? (jitter_level / signal_strength * 100.0) : 0.0,
                     quality.c_str());
    }

    // 5. 可视化发布 (Magnitude)
    if (pub_grad_mag_.getNumSubscribers() > 0)
    {
        cv::Mat viz_mag;
        // 映射：0 -> 0 (黑), High -> 255 (白)
        grad_mag.convertTo(viz_mag, CV_8U, visualize_scale_); 
        // 伪彩色增强对比度
        cv::applyColorMap(viz_mag, viz_mag, cv::COLORMAP_JET);
        
        pub_grad_mag_.publish(cv_bridge::CvImage(header, "bgr8", viz_mag).toImageMsg());
    }
}

} // namespace
PLUGINLIB_EXPORT_CLASS(vs_dvs_processor::DepthProcessorNodelet, nodelet::Nodelet)