#include "vs_dvs_processor/basler_processor_nodelet.h"
#include <pluginlib/class_list_macros.h>

namespace vs_dvs_processor
{

BaslerProcessorNodelet::BaslerProcessorNodelet() 
    : target_width_(640), target_height_(480), desired_loaded_(false) {}

BaslerProcessorNodelet::~BaslerProcessorNodelet() {}

void BaslerProcessorNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    NODELET_INFO("Initializing Basler HYBRID Processor Nodelet...");

    private_nh_.param("target_width", target_width_, 640);
    private_nh_.param("target_height", target_height_, 480);
    private_nh_.param<std::string>("desired_rgb_path", desired_rgb_path_, "");
    private_nh_.param<std::string>("desired_depth_path", desired_depth_path_, "");

    loadDesiredImages();

    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    
    // 1. 发布话题 (必须和 DVS 处理器名称一致，骗过控制节点)
    pub_processed_ = it_->advertise("/vs/image_processed", 1);
    pub_depth_processed_ = it_->advertise("/vs/depth_processed", 1);
    pub_desired_rgb_   = it_->advertise("/vs/image_desired_gray", 1, true);
    pub_desired_depth_ = it_->advertise("/vs/image_desired_depth", 1, true);

    // 2. 注册服务 (保持名字为 /vs_dvs/snapshot，这样 RobotManager 的 M 键可以直接调)
    srv_snapshot_ = nh_.advertiseService("/vs_dvs/snapshot", &BaslerProcessorNodelet::snapshotCb, this);


    private_nh_.param("use_realtime_depth", use_realtime_depth_, false);

    if (use_realtime_depth_) {
        // 3. 时间软同步订阅 (Basler 原始图 + L515 深度图)
    sub_basler_filter_ = std::make_shared<image_transport::SubscriberFilter>(
        *it_, "/basler_camera/image_raw", 1, image_transport::TransportHints("raw"));
    sub_depth_filter_ = std::make_shared<image_transport::SubscriberFilter>(
        *it_, "/camera/aligned_depth_to_color/image_raw", 1, image_transport::TransportHints("raw"));
    
    // Basler 和 L515 硬件时钟不同步，队列稍微给大一点 (15) 以包容延迟
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(15), *sub_basler_filter_, *sub_depth_filter_);
    sync_->registerCallback(boost::bind(&BaslerProcessorNodelet::rgbDepthCallback, this, _1, _2));
    } else {
        NODELET_WARN("Basler Processor: Realtime depth OFF. Subscribing to Basler only.");
        sub_basler_only_ = it_->subscribe("/basler_camera/image_raw", 1, &BaslerProcessorNodelet::baslerOnlyCallback, this);
    }
    

    init_timer_ = nh_.createTimer(ros::Duration(1.0), &BaslerProcessorNodelet::initTimerCb, this, true);
    NODELET_INFO("Basler Hybrid Processor Ready.");
}

void BaslerProcessorNodelet::rgbDepthCallback(const sensor_msgs::ImageConstPtr& rgb_msg, 
                                              const sensor_msgs::ImageConstPtr& depth_msg)
{
    try {
        std::lock_guard<std::mutex> lock(img_mutex_);
        
        // 1. 获取并缩放 Basler 图像
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
        cv::Mat raw_basler = cv_ptr->image;
        cv::resize(raw_basler, img_current_raw_, cv::Size(target_width_, target_height_));
        
        // 确保转为单通道灰度 (Basler 有些是 bayer 或 bgr8)
        if (img_current_raw_.channels() == 3)
            cv::cvtColor(img_current_raw_, img_current_gray_, cv::COLOR_BGR2GRAY);
        else
            img_current_raw_.copyTo(img_current_gray_);

        // 2. 获取 L515 深度图
        cv::Mat raw_depth = cv_bridge::toCvShare(depth_msg, "mono16")->image;
        raw_depth.copyTo(img_current_depth_);

        // 3. 发布出去，供算法和可视化使用
        std_msgs::Header header = rgb_msg->header;
        pub_processed_.publish(cv_bridge::CvImage(header, "mono8", img_current_gray_).toImageMsg());
        pub_depth_processed_.publish(cv_bridge::CvImage(header, "mono16", img_current_depth_).toImageMsg());

    } catch (cv_bridge::Exception& e) {
        NODELET_ERROR("Sync callback exception: %s", e.what());
    }
}

bool BaslerProcessorNodelet::snapshotCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    std::lock_guard<std::mutex> lock(img_mutex_);
    if (img_current_raw_.empty()) {
        res.success = false;
        res.message = "No Basler image received!";
        return true;
    }

    saveDesiredImages(img_current_raw_, img_current_depth_);
    img_current_gray_.copyTo(img_desired_gray_);
    img_current_depth_.copyTo(img_desired_depth_);
    desired_loaded_ = true;

    // 发布给可视化界面 (Target View 窗口将会刷新)
    std_msgs::Header header;
    header.stamp = ros::Time::now(); 
    header.frame_id = "camera_color_frame"; 
    pub_desired_rgb_.publish(cv_bridge::CvImage(header, "mono8", img_desired_gray_).toImageMsg());
    pub_desired_depth_.publish(cv_bridge::CvImage(header, "mono16", img_desired_depth_).toImageMsg());

    res.success = true;
    res.message = "Hybrid Snapshot Saved!";
    return true;
}

void BaslerProcessorNodelet::saveDesiredImages(const cv::Mat& raw_rgb, const cv::Mat& raw_depth)
{
    if (!desired_rgb_path_.empty() && !raw_rgb.empty()) cv::imwrite(desired_rgb_path_, raw_rgb);
    if (!desired_depth_path_.empty() && !raw_depth.empty()) cv::imwrite(desired_depth_path_, raw_depth);
}

void BaslerProcessorNodelet::loadDesiredImages()
{
    if (desired_rgb_path_.empty()) return;
    cv::Mat raw_rgb = cv::imread(desired_rgb_path_, cv::IMREAD_GRAYSCALE);
    if (!raw_rgb.empty()) {
        img_desired_gray_ = raw_rgb;
        desired_loaded_ = true;
    }
    if (!desired_depth_path_.empty()) {
        img_desired_depth_ = cv::imread(desired_depth_path_, cv::IMREAD_UNCHANGED);
    }
}

void BaslerProcessorNodelet::initTimerCb(const ros::TimerEvent& event)
{
    if (desired_loaded_) {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "camera_color_frame"; 
        pub_desired_rgb_.publish(cv_bridge::CvImage(header, "mono8", img_desired_gray_).toImageMsg());
    }
}

void BaslerProcessorNodelet::baslerOnlyCallback(const sensor_msgs::ImageConstPtr& rgb_msg) 
{
    sensor_msgs::ImagePtr empty_depth(new sensor_msgs::Image);
    empty_depth->header = rgb_msg->header;
    empty_depth->encoding = "mono16";
    empty_depth->width = target_width_;
    empty_depth->height = target_height_;
    empty_depth->step = target_width_ * 2;
    empty_depth->data.resize(empty_depth->step * empty_depth->height, 0);
    
    // 借用原来的同步回调处理
    rgbDepthCallback(rgb_msg, empty_depth);
}

} // namespace vs_dvs_processor

PLUGINLIB_EXPORT_CLASS(vs_dvs_processor::BaslerProcessorNodelet, nodelet::Nodelet)