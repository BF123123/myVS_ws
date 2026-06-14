#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/Trigger.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mutex>

namespace vs_dvs_processor
{

class BaslerProcessorNodelet : public nodelet::Nodelet
{
public:
    BaslerProcessorNodelet();
    ~BaslerProcessorNodelet();
    virtual void onInit() override;

private:
    // 同步回调函数：同时接收 Basler 和 L515 深度
    void rgbDepthCallback(const sensor_msgs::ImageConstPtr& rgb_msg, 
                          const sensor_msgs::ImageConstPtr& depth_msg);

    // 截图服务 (接管 M 键)
    bool snapshotCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    void loadDesiredImages();
    void saveDesiredImages(const cv::Mat& raw_rgb, const cv::Mat& raw_depth);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::shared_ptr<image_transport::ImageTransport> it_;

    // 发布者：发布给算法和可视化窗口
    image_transport::Publisher pub_processed_;    
    image_transport::Publisher pub_depth_processed_; 
    image_transport::Publisher pub_desired_rgb_;   
    image_transport::Publisher pub_desired_depth_; 

    // 截图服务提供者
    ros::ServiceServer srv_snapshot_;

    // 软同步订阅器
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    std::shared_ptr<image_transport::SubscriberFilter> sub_basler_filter_;
    std::shared_ptr<image_transport::SubscriberFilter> sub_depth_filter_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // 数据缓存
    cv::Mat img_desired_gray_;   
    cv::Mat img_desired_depth_;
    cv::Mat img_current_raw_;    
    cv::Mat img_current_gray_;   
    cv::Mat img_current_depth_;  
    std::mutex img_mutex_;       
    
    // 参数
    bool desired_loaded_;   
    std::string desired_rgb_path_;
    std::string desired_depth_path_;
    int target_width_;
    int target_height_;

    ros::Timer init_timer_;
    void initTimerCb(const ros::TimerEvent& event);
    bool use_realtime_depth_;
    image_transport::Subscriber sub_basler_only_;
    void baslerOnlyCallback(const sensor_msgs::ImageConstPtr& rgb_msg);
};

} // namespace vs_dvs_processor