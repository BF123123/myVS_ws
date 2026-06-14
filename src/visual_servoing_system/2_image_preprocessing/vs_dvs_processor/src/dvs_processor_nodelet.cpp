#include "vs_dvs_processor/dvs_processor_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <fstream>

namespace vs_dvs_processor
{

DvsProcessorNodelet::DvsProcessorNodelet() 
    : desired_loaded_(false), gaussian_kernel_size_(5), gaussian_sigma_(1.0)
{
}

DvsProcessorNodelet::~DvsProcessorNodelet()
{
}

void DvsProcessorNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    NODELET_INFO("Initializing DVS Processor Nodelet...");

    // 1. 参数获取
    private_nh_.param("use_realtime_depth", use_realtime_depth_, false); 
    private_nh_.param("use_gaussian_blur", use_gaussian_blur_, true);
    private_nh_.param("gaussian_kernel_size", gaussian_kernel_size_, 5);
    private_nh_.param("gaussian_sigma", gaussian_sigma_, 1.0);
    private_nh_.param("use_histogram_eq", use_histogram_eq_, false);
    
    // 路径参数
    private_nh_.param<std::string>("desired_rgb_path", desired_rgb_path_, "");
    private_nh_.param<std::string>("desired_depth_path", desired_depth_path_, "");

    // 2. 加载初始图像 (从磁盘读取Raw -> 处理为Desired)
    loadDesiredImages();

    // 3. 初始化通讯
    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    pub_processed_ = it_->advertise("/vs/image_processed", 1);
    pub_depth_processed_ = it_->advertise("/vs/depth_processed", 1);
    
    pub_desired_rgb_   = it_->advertise("/vs/image_desired_gray", 1, true);
    pub_desired_depth_ = it_->advertise("/vs/image_desired_depth", 1, true);
    pub_error_img_ = it_->advertise("/vs/image_error", 1);

    // ================= 刚启动时发布一次初始图像 =================
    init_timer_ = nh_.createTimer(ros::Duration(1.0), &DvsProcessorNodelet::initTimerCb, this, true);
    
    NODELET_INFO("DvsProcessor Initialized. Waiting 1s to publish initial target...");
    // ======================================================================

    srv_snapshot_ = nh_.advertiseService("/vs_dvs/snapshot", &DvsProcessorNodelet::snapshotCb, this);

    // 4. 订阅逻辑
    if (use_realtime_depth_)
    {
        sub_rgb_filter_ = std::make_shared<image_transport::SubscriberFilter>(
            *it_, "/camera/color/image_raw", 1, image_transport::TransportHints("raw"));
        sub_depth_filter_ = std::make_shared<image_transport::SubscriberFilter>(
            *it_, "/camera/aligned_depth_to_color/image_raw", 1, image_transport::TransportHints("raw"));
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(5), *sub_rgb_filter_, *sub_depth_filter_);
        sync_->registerCallback(boost::bind(&DvsProcessorNodelet::rgbDepthCallback, this, _1, _2));
    }
    else
    {
        sub_rgb_only_ = it_->subscribe("/camera/color/image_raw", 1, 
                                       &DvsProcessorNodelet::imageCallback, this, 
                                       image_transport::TransportHints("raw"));
    }
    
    NODELET_INFO("DVS Processor Initialized. Save path: %s", desired_rgb_path_.c_str());
}

// ==================== 截图服务 (核心逻辑修改) ====================
bool DvsProcessorNodelet::snapshotCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    std::lock_guard<std::mutex> lock(img_mutex_);
    
    // 检查是否有数据
    if (img_current_raw_.empty())
    {
        res.success = false;
        res.message = "Snapshot Failed: No image received yet!";
        NODELET_WARN("%s", res.message.c_str());
        return true;
    }

    // 1. 保存到磁盘 (保存的是原始 RAW 数据)
    // 目的：下次启动时，程序会读取这个 RAW 图，再重新执行 processRGB
    saveDesiredImages(img_current_raw_, img_current_depth_);

    // 2. 更新内存中的期望图像
    // 既然我们刚刚保存了新目标，为了不重启程序就能继续实验，
    // 我们需要在内存中手动对这张 Raw 图做一次处理，作为新的 Desired
    img_desired_gray_ = processRGB(img_current_raw_);
    
    if (!img_current_depth_.empty()) {
        img_current_depth_.copyTo(img_desired_depth_);
    }

    // 3. 立即发布更新 (通知算法层)
    desired_loaded_ = true;
    std_msgs::Header header;
    header.stamp = ros::Time::now(); 
    header.frame_id = "camera_color_frame"; 
    
    pub_desired_rgb_.publish(cv_bridge::CvImage(header, "mono8", img_desired_gray_).toImageMsg());
    if(!img_desired_depth_.empty())
        pub_desired_depth_.publish(cv_bridge::CvImage(header, "mono16", img_desired_depth_).toImageMsg());

    res.success = true;
    res.message = "Snapshot Saved (RAW) & Updated (Processed)!";
    NODELET_INFO("Snapshot captured. Raw saved to disk, Processed set to memory.");
    return true;
}

// ==================== 保存函数 ====================
void DvsProcessorNodelet::saveDesiredImages(const cv::Mat& raw_rgb, const cv::Mat& raw_depth)
{
    // 保存 RGB (Raw)
    if (!desired_rgb_path_.empty() && !raw_rgb.empty()) {
        try {
            cv::imwrite(desired_rgb_path_, raw_rgb);
            NODELET_INFO("Saved RAW RGB to: %s", desired_rgb_path_.c_str());
        } catch (cv::Exception& e) {
            NODELET_ERROR("Failed to save RGB: %s", e.what());
        }
    } else {
        NODELET_WARN("RGB path empty or image empty, skip saving.");
    }

    // 保存 Depth (Raw 16-bit)
    if (!desired_depth_path_.empty() && !raw_depth.empty()) {
        try {
            cv::imwrite(desired_depth_path_, raw_depth);
            NODELET_INFO("Saved RAW Depth to: %s", desired_depth_path_.c_str());
        } catch (cv::Exception& e) {
            NODELET_ERROR("Failed to save Depth: %s", e.what());
        }
    } else {
        if(desired_depth_path_.empty()) 
            NODELET_WARN("Desired depth path NOT set! Depth image NOT saved.");
    }
}

// ==================== 图像处理 ====================
cv::Mat DvsProcessorNodelet::processRGB(const cv::Mat& raw_img)
{
    cv::Mat gray;
    if (raw_img.channels() == 3)
        cv::cvtColor(raw_img, gray, cv::COLOR_BGR2GRAY);
    else
        gray = raw_img.clone();

    if (use_histogram_eq_) cv::equalizeHist(gray, gray);

    if (use_gaussian_blur_)
    {
        if (gaussian_kernel_size_ % 2 == 0) gaussian_kernel_size_++;
        cv::GaussianBlur(gray, gray, cv::Size(gaussian_kernel_size_, gaussian_kernel_size_), gaussian_sigma_);
    }
    return gray;
}

// ==================== 加载函数 ====================
void DvsProcessorNodelet::loadDesiredImages()
{
    if (desired_rgb_path_.empty()) return;
    
    // 读取时：读取 Raw -> 处理 -> 存入内存
    NODELET_INFO("Loading RGB from: %s", desired_rgb_path_.c_str());
    cv::Mat raw_rgb = cv::imread(desired_rgb_path_, cv::IMREAD_COLOR);
    if (!raw_rgb.empty()) {
        img_desired_gray_ = processRGB(raw_rgb); // 在这里进行处理
        desired_loaded_ = true;
        NODELET_INFO("RGB loaded and processed successfully.");
    } else {
        NODELET_WARN("Failed to load RGB image.");
    }

    if (!desired_depth_path_.empty()) {
        img_desired_depth_ = cv::imread(desired_depth_path_, cv::IMREAD_UNCHANGED);
        NODELET_INFO("Depth loaded.");
    }
}

// ==================== 接收回调 (同步模式) ====================
void DvsProcessorNodelet::rgbDepthCallback(const sensor_msgs::ImageConstPtr& rgb_msg, 
                                           const sensor_msgs::ImageConstPtr& depth_msg)
{
    try {
        std::lock_guard<std::mutex> lock(img_mutex_);
        
        cv::Mat raw_img = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
        cv::Mat raw_depth = cv_bridge::toCvShare(depth_msg, "mono16")->image;
        
        // 【关键】备份原始数据 (用于截图)
        raw_img.copyTo(img_current_raw_);
        raw_depth.copyTo(img_current_depth_);

        // 生成处理后的图 (用于控制和显示)
        cv::Mat processed = processRGB(raw_img);
        processed.copyTo(img_current_gray_);

        // 发布
        std_msgs::Header header = rgb_msg->header;
        pub_processed_.publish(cv_bridge::CvImage(header, "mono8", processed).toImageMsg());
        pub_depth_processed_.publish(cv_bridge::CvImage(header, "mono16", raw_depth).toImageMsg());

        // 误差计算
        if (desired_loaded_)
        {
            // pub_desired_rgb_.publish(cv_bridge::CvImage(header, "mono8", img_desired_gray_).toImageMsg());
            // pub_desired_depth_.publish(cv_bridge::CvImage(header, "mono16", img_desired_depth_).toImageMsg());
            
            if (processed.size() == img_desired_gray_.size()) {
                 cv::Mat diff;
                 cv::subtract(processed, img_desired_gray_, diff, cv::noArray(), CV_16S);
                 cv::add(diff, cv::Scalar(128), diff);
                 cv::Mat diff_8u;
                 diff.convertTo(diff_8u, CV_8U);
                 pub_error_img_.publish(cv_bridge::CvImage(header, "mono8", diff_8u).toImageMsg());
            }
        }
    } catch (cv_bridge::Exception& e) {
        NODELET_ERROR("Sync callback exception: %s", e.what());
    }
}

// ==================== 接收回调 (RGB only) ====================
void DvsProcessorNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        std::lock_guard<std::mutex> lock(img_mutex_);
        cv::Mat raw_img = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        // 【关键】备份
        raw_img.copyTo(img_current_raw_);

        cv::Mat processed = processRGB(raw_img);
        processed.copyTo(img_current_gray_);
        
        std_msgs::Header header = msg->header;
        pub_processed_.publish(cv_bridge::CvImage(header, "mono8", processed).toImageMsg());
        
        if (desired_loaded_)
        {
            // pub_desired_rgb_.publish(cv_bridge::CvImage(header, "mono8", img_desired_gray_).toImageMsg());
            
            if (processed.size() == img_desired_gray_.size()) {
                 cv::Mat diff;
                 cv::subtract(processed, img_desired_gray_, diff, cv::noArray(), CV_16S);
                 cv::add(diff, cv::Scalar(128), diff);
                 cv::Mat diff_8u;
                 diff.convertTo(diff_8u, CV_8U);
                 pub_error_img_.publish(cv_bridge::CvImage(header, "mono8", diff_8u).toImageMsg());
            }
        }
    } catch (cv_bridge::Exception& e) {}
}

// ================= 【新增】 定时器回调函数 =================
void DvsProcessorNodelet::initTimerCb(const ros::TimerEvent& event)
{
    // 这里再判断加载状态
    if (desired_loaded_)
    {
        // 双重检查
        if (img_desired_gray_.empty()) {
            NODELET_WARN("Timer Triggered: desired_loaded_ is true but image is EMPTY!");
            return;
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "camera_color_frame"; 

        // 此时连接通常已经建立好了，发布更安全
        pub_desired_rgb_.publish(cv_bridge::CvImage(header, "mono8", img_desired_gray_).toImageMsg());
        
        if (!img_desired_depth_.empty()) {
            pub_desired_depth_.publish(cv_bridge::CvImage(header, "mono16", img_desired_depth_).toImageMsg());
        }
        
        NODELET_WARN(">>> [DELAYED 1.0s] Initial Desired Image PUBLISHED! Size: %d x %d <<<", img_desired_gray_.cols, img_desired_gray_.rows);
    }
    else
    {
        NODELET_WARN("Timer Triggered: No desired image loaded to publish.");
    }
}

} // namespace vs_dvs_processor

PLUGINLIB_EXPORT_CLASS(vs_dvs_processor::DvsProcessorNodelet, nodelet::Nodelet)