#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace vs_dvs_processor
{

class DepthProcessorNodelet : public nodelet::Nodelet
{
public:
    DepthProcessorNodelet();
    ~DepthProcessorNodelet();
    virtual void onInit() override;

private:
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);

    // 核心处理：转浮点 + 空间高斯 + 时间滤波
    cv::Mat processDepth(const cv::Mat& raw_depth_mm);
    
    // 计算梯度、量化指标并发布可视化
    void analyzeAndVisualize(const cv::Mat& img_float, const std_msgs::Header& header);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::shared_ptr<image_transport::ImageTransport> it_;

    image_transport::Subscriber sub_depth_;
    image_transport::Publisher pub_depth_processed_; 
    image_transport::Publisher pub_depth_float_;     
    
    // 改为发布“梯度幅值”，更直观
    image_transport::Publisher pub_grad_mag_; 

    // 参数
    bool use_gaussian_blur_;
    int gaussian_kernel_size_;
    double gaussian_sigma_;
    double visualize_scale_; 
    
    // 【新增】时间滤波参数
    bool use_temporal_filter_;
    double temporal_alpha_; // 滤波系数 (0~1)，越小越平滑但滞后越大

    // 数据缓存
    std::mutex img_mutex_;
    cv::Mat img_depth_filtered_; // 上一帧滤波后的深度图 (用于时间滤波)
    cv::Mat img_last_grad_mag_;  // 上一帧的梯度幅值 (用于计算噪声抖动)
    
    // 计数器 (用于打印频率控制)
    int frame_count_;
};

} // namespace vs_dvs_processor