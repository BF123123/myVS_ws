#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

// 服务与同步机制
#include <std_srvs/Trigger.h> 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mutex> 

namespace vs_dvs_processor
{

class DvsProcessorNodelet : public nodelet::Nodelet
{
public:
    DvsProcessorNodelet();
    ~DvsProcessorNodelet();
    virtual void onInit() override;

private:
    // 回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rgbDepthCallback(const sensor_msgs::ImageConstPtr& rgb_msg, 
                          const sensor_msgs::ImageConstPtr& depth_msg);

    // 截图服务回调
    bool snapshotCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    // 图像处理函数
    cv::Mat processRGB(const cv::Mat& raw_img);
    
    // 加载与保存
    void loadDesiredImages();
    // 【修改】改为接受参数，不再隐式保存成员变量
    void saveDesiredImages(const cv::Mat& raw_rgb, const cv::Mat& raw_depth); 

    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::shared_ptr<image_transport::ImageTransport> it_;

    // 发布者
    image_transport::Publisher pub_processed_;    
    image_transport::Publisher pub_depth_processed_; 
    image_transport::Publisher pub_desired_rgb_;   
    image_transport::Publisher pub_desired_depth_; 
    image_transport::Publisher pub_error_img_;    

    // 服务
    ros::ServiceServer srv_snapshot_;

    // 订阅者
    image_transport::Subscriber sub_rgb_only_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    std::shared_ptr<image_transport::SubscriberFilter> sub_rgb_filter_;
    std::shared_ptr<image_transport::SubscriberFilter> sub_depth_filter_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // --- 图像数据 ---
    
    // 1. 期望图像 (内存中是已处理过的，用于计算误差)
    cv::Mat img_desired_gray_;   
    cv::Mat img_desired_depth_;
    
    // 2. 当前帧缓存 (新增 raw 缓存)
    cv::Mat img_current_raw_;    // 【关键新增】原始未处理的 RGB (用于保存)
    cv::Mat img_current_gray_;   // 处理后的灰度 (用于显示/计算)
    cv::Mat img_current_depth_;  // 原始深度
    
    std::mutex img_mutex_;       // 线程锁
    
    // 状态与路径
    bool desired_loaded_;   
    std::string desired_rgb_path_;
    std::string desired_depth_path_;
    
    // 参数
    bool use_realtime_depth_; 
    bool use_gaussian_blur_;
    int gaussian_kernel_size_;
    double gaussian_sigma_;
    bool use_histogram_eq_;

    // 初始化定时器声明
    ros::Timer init_timer_;
    void initTimerCb(const ros::TimerEvent& event);
};

} // namespace vs_dvs_processor