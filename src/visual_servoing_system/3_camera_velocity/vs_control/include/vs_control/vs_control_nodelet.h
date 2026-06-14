#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "vs_control/visual_servoing.h" 
#include <std_srvs/Trigger.h>

// 【新增】 多线程支持库
#include <thread>
#include <mutex>
#include <atomic>

namespace vs_control
{

class VsControlNodelet : public nodelet::Nodelet
{
public:
    VsControlNodelet(); 
    ~VsControlNodelet();
    virtual void onInit() override;

private:
    void desiredImgCallback(const sensor_msgs::ImageConstPtr& gray_msg, 
                            const sensor_msgs::ImageConstPtr& depth_msg);
    void controlLoopCallback(const sensor_msgs::ImageConstPtr& gray_msg, 
                             const sensor_msgs::ImageConstPtr& depth_msg);
    bool enableControlCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool reloadConfigCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void loadAlgorithm();
    void setupSubscribers();
    
    // 【新增】 异步发布线程函数
    void publishThreadFunc();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    Visual_Servoing* dvs_algo_; 
    bool algo_initialized_; 
    bool control_running_;

    ros::Publisher pub_cmd_vel_;
    ros::ServiceServer srv_enable_;
    ros::ServiceServer srv_reload_;

    std::shared_ptr<image_transport::ImageTransport> it_;
    
    // 【新增】 错误图像/可视化图像发布者 (之前代码里似乎没写，这里补上)
    image_transport::Publisher pub_error_img_; 

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    std::shared_ptr<image_transport::SubscriberFilter> sub_gray_;
    std::shared_ptr<image_transport::SubscriberFilter> sub_depth_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    std::shared_ptr<image_transport::SubscriberFilter> sub_des_gray_;
    std::shared_ptr<image_transport::SubscriberFilter> sub_des_depth_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_des_;

    std::string camera_frame_; 

    // ================= 【新增】 多线程相关变量 =================
    std::thread pub_thread_;            // 后台线程对象
    std::atomic<bool> thread_running_;  // 线程运行标志
    
    std::mutex img_mutex_;              // 互斥锁，保护共享数据
    cv::Mat shared_curr_img_;          // 共享的图像缓存
    std_msgs::Header shared_header_;    // 共享的时间戳
    bool new_data_ready_;               // 标志位：是否有新数据待发送
    // ==========================================================
    std::string primary_camera_;
    std::string depth_strategy_;
    image_transport::Subscriber sub_gray_only_;
    image_transport::Subscriber sub_des_gray_only_;

    void grayOnlyCallback(const sensor_msgs::ImageConstPtr& gray_msg);
    void desGrayOnlyCallback(const sensor_msgs::ImageConstPtr& gray_msg);

    
};

}