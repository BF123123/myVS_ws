#include "vs_control/vs_control_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include "vs_control/direct_visual_servoing.h" 
#include "vs_control/depth_visual_servoing.h"
#include "vs_control/defocused_visual_servoing.h"
#include "vs_control/defocused_visual_servoing_RAL.h"

namespace vs_control
{

VsControlNodelet::VsControlNodelet() 
    : dvs_algo_(nullptr), algo_initialized_(false), control_running_(false)
{
    // 【新增】 初始化线程标志
    thread_running_ = false;
    new_data_ready_ = false;
}

VsControlNodelet::~VsControlNodelet()
{
    // 【修改】 析构时安全停止线程
    thread_running_ = false; 
    if (pub_thread_.joinable()) {
        pub_thread_.join();
    }

    if (dvs_algo_) delete dvs_algo_;
}

void VsControlNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();
    
    NODELET_INFO("Initializing VS Control Nodelet (Async Visualization)...");

    int res_x, res_y;
    private_nh_.param("resolution_x", res_x, 640);
    private_nh_.param("resolution_y", res_y, 480);
    private_nh_.param<std::string>("camera_frame", camera_frame_, "camera_color_frame");

    //加载算法
    loadAlgorithm();

    std::string algo_type;
    private_nh_.param<std::string>("algorithm_type", algo_type, "dvs");
    
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/vs/camera_velocity_raw", 1);
    srv_enable_  = nh_.advertiseService("/vs/enable_control", &VsControlNodelet::enableControlCb, this);
    srv_reload_  = nh_.advertiseService("/vs/reload_config", &VsControlNodelet::reloadConfigCb, this);
    
    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    
    //  初始化错误图像发布者
    pub_error_img_ = it_->advertise("/vs/image_error_control", 1);

    private_nh_.param<std::string>("primary_camera", primary_camera_, "basler");
    private_nh_.param<std::string>("depth_strategy", depth_strategy_, "constant");

    // 可选：安全校验提示
    if (primary_camera_ == "basler" && depth_strategy_ == "realtime") {
        NODELET_WARN("VS Control: 双机模式 (Basler+L515) 启动。请确保预处理节点已完成两者的深度空间对齐！");
    }

    // 核心：只根据 depth_strategy 决定订阅方式，与 algorithm_type 解绑
    if (depth_strategy_ == "constant" || depth_strategy_ == "desired_only") 
    {
        NODELET_WARN("VS Control: CONSTANT/DESIRED depth mode. 仅订阅 RGB。");
        sub_des_gray_only_ = it_->subscribe("/vs/image_desired_gray", 1, &VsControlNodelet::desGrayOnlyCallback, this);
        sub_gray_only_     = it_->subscribe("/vs/image_processed", 1, &VsControlNodelet::grayOnlyCallback, this);
    } 
    else // realtime 模式
    {
        NODELET_INFO("VS Control: REALTIME depth mode. 等待 RGB 与 Depth 同步...");

        sub_des_gray_ = std::make_shared<image_transport::SubscriberFilter>(
            *it_, "/vs/image_desired_gray", 1, image_transport::TransportHints("raw"));
        sub_des_depth_ = std::make_shared<image_transport::SubscriberFilter>(
            *it_, "/vs/image_desired_depth", 1, image_transport::TransportHints("raw"));
        sync_des_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *sub_des_gray_, *sub_des_depth_);
        sync_des_->registerCallback(boost::bind(&VsControlNodelet::desiredImgCallback, this, _1, _2));

        sub_gray_ = std::make_shared<image_transport::SubscriberFilter>(
            *it_, "/vs/image_processed", 1, image_transport::TransportHints("raw"));
        sub_depth_ = std::make_shared<image_transport::SubscriberFilter>(
            *it_, "/vs/depth_processed", 1, image_transport::TransportHints("raw")); 
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(5), *sub_gray_, *sub_depth_);
        sync_->registerCallback(boost::bind(&VsControlNodelet::controlLoopCallback, this, _1, _2));
    }
    // =========================================================

    

    // 启动异步发布线程
    thread_running_ = true;
    pub_thread_ = std::thread(&VsControlNodelet::publishThreadFunc, this);

    NODELET_INFO("VS Control Ready.");
}

bool VsControlNodelet::reloadConfigCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    // 为了线程安全，这里最好暂停一下控制线程（如果是多线程环境），但由于此时 control_running_ 通常为 false，暂不加锁
    try {
        loadAlgorithm();
        res.success = true;
        res.message = "Algorithm Reloaded Successfully!";
        NODELET_INFO("%s", res.message.c_str());
    } catch (std::exception& e) {
        res.success = false;
        res.message = std::string("Reload Failed: ") + e.what();
        NODELET_ERROR("%s", res.message.c_str());
    }
    return true;
}

void VsControlNodelet::loadAlgorithm()
{
    // 1. 读取动态参数
    std::string algo_type;
    private_nh_.param<std::string>("algorithm_type", algo_type, "dvs");
    
    int res_x, res_y;
    private_nh_.param("resolution_x", res_x, 640);
    private_nh_.param("resolution_y", res_y, 480);

    // 2. 清理旧对象
    if (dvs_algo_) { 
        delete dvs_algo_; 
        dvs_algo_ = nullptr; 
        algo_initialized_ = false; 
    }

    // ================= 【核心修复：向下兼容底层算法】 =================
    std::string strategy;
    private_nh_.param<std::string>("depth_strategy", strategy, "realtime");
    
    // =================================================================

    // 3. 实例化工厂
    if (algo_type == "dvs" || algo_type == "direct") 
    {
        double depth_val;
        private_nh_.param("depth_constant_value", depth_val, 0.24);
        
        NODELET_INFO("[ReLoad] Algo: DVS | Strategy: %s | Depth: %.2f", strategy.c_str(), depth_val);
        dvs_algo_ = new Direct_Visual_Servoing(res_x, res_y, strategy, depth_val);
    }
    else if (algo_type == "depth"|| algo_type == "pure_depth") 
    {
        NODELET_INFO("[ReLoad] Algo: Pure Depth Visual Servoing selected.");
        dvs_algo_ = new Depth_Visual_Servoing(res_x, res_y);
    }
    else if (algo_type == "defocused") 
    {
        double depth_val;
        private_nh_.param("depth_constant_value", depth_val, 0.24);

        NODELET_INFO("[ReLoad] Algo: Defocused VS | Strategy: %s | Depth: %.2f", strategy.c_str(), depth_val);
        dvs_algo_ = new Defocused_Visual_Servoing(res_x, res_y, strategy, depth_val);
    }
      else if (algo_type == "defocused_RAL") 
    {
        double depth_val;
        private_nh_.param("depth_constant_value", depth_val, 0.24);

        NODELET_INFO("[ReLoad] Algo: Defocused RAL | Strategy: %s | Depth: %.2f", strategy.c_str(), depth_val);
        dvs_algo_ = new Defocused_Visual_Servoing_RAL(res_x, res_y, strategy, depth_val);
    }
    else
    {
        NODELET_WARN("Unknown algo '%s', using DVS.", algo_type.c_str());
        dvs_algo_ = new Direct_Visual_Servoing(res_x, res_y);
    }
}

void VsControlNodelet::setupSubscribers()
{
    // 1. 强行关闭并清理旧的所有订阅器 (防止幽灵回调和话题冲突)
    sub_des_gray_only_.shutdown();
    sub_gray_only_.shutdown();
    
    if (sync_des_) sync_des_.reset();
    if (sub_des_gray_) sub_des_gray_.reset();
    if (sub_des_depth_) sub_des_depth_.reset();
    
    if (sync_) sync_.reset();
    if (sub_gray_) sub_gray_.reset();
    if (sub_depth_) sub_depth_.reset();

    // 2. 读取最新的深度策略
    private_nh_.param<std::string>("depth_strategy", depth_strategy_, "constant");

    // 3. 根据新策略，重新建立订阅管道
    if (depth_strategy_ == "constant" || depth_strategy_ == "desired_only") 
    {
        NODELET_WARN("VS Control Pipeline: [%s] 模式，建立纯 RGB 订阅管道。", depth_strategy_.c_str());
        sub_des_gray_only_ = it_->subscribe("/vs/image_desired_gray", 1, &VsControlNodelet::desGrayOnlyCallback, this);
        sub_gray_only_     = it_->subscribe("/vs/image_processed", 1, &VsControlNodelet::grayOnlyCallback, this);
    } 
    else // realtime 模式
    {
        NODELET_INFO("VS Control Pipeline: [realtime] 模式，建立 RGB-Depth 同步管道...");
        sub_des_gray_ = std::make_shared<image_transport::SubscriberFilter>(
            *it_, "/vs/image_desired_gray", 1, image_transport::TransportHints("raw"));
        sub_des_depth_ = std::make_shared<image_transport::SubscriberFilter>(
            *it_, "/vs/image_desired_depth", 1, image_transport::TransportHints("raw"));
        sync_des_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *sub_des_gray_, *sub_des_depth_);
        sync_des_->registerCallback(boost::bind(&VsControlNodelet::desiredImgCallback, this, _1, _2));

        sub_gray_ = std::make_shared<image_transport::SubscriberFilter>(
            *it_, "/vs/image_processed", 1, image_transport::TransportHints("raw"));
        sub_depth_ = std::make_shared<image_transport::SubscriberFilter>(
            *it_, "/vs/depth_processed", 1, image_transport::TransportHints("raw")); 
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(5), *sub_gray_, *sub_depth_);
        sync_->registerCallback(boost::bind(&VsControlNodelet::controlLoopCallback, this, _1, _2));
    }
}

bool VsControlNodelet::enableControlCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    control_running_ = req.data;
    res.success = true;
    res.message = control_running_ ? "VS Control STARTED" : "VS Control STOPPED";
    if (!control_running_) pub_cmd_vel_.publish(geometry_msgs::Twist());
    NODELET_INFO("%s", res.message.c_str());
    return true;
}

void VsControlNodelet::desiredImgCallback(const sensor_msgs::ImageConstPtr& gray_msg, 
                                          const sensor_msgs::ImageConstPtr& depth_msg)
{
    try {
        cv::Mat I_star = cv_bridge::toCvShare(gray_msg, "mono8")->image;
        cv::Mat Z_star = cv_bridge::toCvShare(depth_msg, "mono16")->image;
        cv::Mat I_star_f, Z_star_f;
        I_star.convertTo(I_star_f, CV_64F, 1.0/255.0);
        Z_star.convertTo(Z_star_f, CV_64F, 1.0/1000.0);

        double lambda, epsilon;
        private_nh_.param("lambda", lambda, 0.5); 
        private_nh_.param("epsilon", epsilon, 1e-4); 
        
        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        std::vector<double> K_list;
        if(private_nh_.getParam("camera_intrinsic", K_list) && K_list.size() == 9) {
            memcpy(K.data, K_list.data(), 9*sizeof(double));
        } else {
            K.at<double>(0,0)=600; K.at<double>(0,2)=320;
            K.at<double>(1,1)=600; K.at<double>(1,2)=240;
        }

        cv::Mat pose_des = cv::Mat::eye(4,4,CV_64F);
        dvs_algo_->init_VS(lambda, epsilon, I_star_f, Z_star_f, I_star_f, K, pose_des);
        algo_initialized_ = true;
        NODELET_INFO("Algorithm Initialized.");
    } catch (cv_bridge::Exception& e) { NODELET_ERROR("Desired img error: %s", e.what()); }
}

void VsControlNodelet::controlLoopCallback(const sensor_msgs::ImageConstPtr& gray_msg, 
                                           const sensor_msgs::ImageConstPtr& depth_msg)
{
    // 1. 如果算法还没初始化（还没按 M 保存目标，也没加载 Slot），肯定没法算误差，直接返回
    if (!algo_initialized_) return; 

    try {
        // --- A. 数据转换 (无论是否控制，都要做) ---
        cv::Mat I_curr = cv_bridge::toCvShare(gray_msg, "mono8")->image;
        cv::Mat Z_curr = cv_bridge::toCvShare(depth_msg, "mono16")->image;
        cv::Mat I_f, Z_f;
        I_curr.convertTo(I_f, CV_64F, 1.0/255.0);
        Z_curr.convertTo(Z_f, CV_64F, 1.0/1000.0);

        // --- B. 还没按 V 的时候，就能看到热力图 ---
        if (pub_error_img_.getNumSubscribers() > 0)
        {
            std::lock_guard<std::mutex> lock(img_mutex_);
            shared_curr_img_ = I_f.clone(); 
            shared_header_ = gray_msg->header; 
            new_data_ready_ = true;            
        }
        // -------------------------------------------------------

        // 2. 如果没开启控制 (没按 V)，到这里就结束，不发速度
        if (!control_running_) {
            // 可选：为了防止上一帧的速度残留，可以发个 0 速度，或者什么都不做
            return; 
        }

        // --- C. 以下是控制逻辑 (只有按了 V 才执行) ---
        dvs_algo_->set_image_gray_current(I_f);
        dvs_algo_->set_image_depth_current(Z_f);
        
        cv::Mat T_curr = cv::Mat::eye(4, 4, CV_64F); 
        dvs_algo_->save_data(T_curr);

        cv::Mat v_cam = dvs_algo_->get_camera_velocity(); 

        // 调试打印
        double error_norm = cv::norm(I_f, dvs_algo_->image_gray_desired_, cv::NORM_L2);
        double v_z_val = v_cam.at<double>(2);
        NODELET_INFO_THROTTLE(1.0, "Error=%.2f, Vz=%.4f", error_norm, v_z_val);

        // 成功判定
        if (dvs_algo_->is_success()) {
            NODELET_INFO("Visual Servoing SUCCESS!");
            control_running_ = false; // 自动停止
            dvs_algo_->write_data();
            pub_cmd_vel_.publish(geometry_msgs::Twist());
            return;
        }

        // 发布速度
        geometry_msgs::Twist msg;
        msg.linear.x = v_cam.at<double>(0);
        msg.linear.y = v_cam.at<double>(1);
        msg.linear.z = v_cam.at<double>(2);
        msg.angular.x = v_cam.at<double>(3);
        msg.angular.y = v_cam.at<double>(4);
        msg.angular.z = v_cam.at<double>(5);
        
        pub_cmd_vel_.publish(msg);

    } catch (std::exception& e) {
        NODELET_ERROR("Control loop error: %s", e.what());
    }
}

void VsControlNodelet::publishThreadFunc()
{
    ros::Rate r(30); 

    while (ros::ok() && thread_running_)
    {
        cv::Mat raw_curr_img;
        std_msgs::Header header_to_pub;
        bool has_data = false;

        // 1. 取数据
        {
            std::lock_guard<std::mutex> lock(img_mutex_);
            if (new_data_ready_) {
                raw_curr_img = shared_curr_img_;
                header_to_pub = shared_header_;
                new_data_ready_ = false;
                has_data = true;
            }
        }

// 2. 处理与发布
        if (has_data && !raw_curr_img.empty() && dvs_algo_ && !dvs_algo_->image_gray_desired_.empty()) 
        {
            try {
                if (pub_error_img_.getNumSubscribers() > 0) {
                    
                    // ================= 【修正：强制归零的红绿蓝热力图】 =================
                    
                    cv::Mat diff_float;
                    // 1. 计算带符号差值: (Current - Desired)
                    // 范围大约在 -1.0 到 1.0 之间
                    cv::subtract(raw_curr_img, dvs_algo_->image_gray_desired_, diff_float);
                    
                    // 2. 【关键步骤】 去除平均光照误差 (Zero-Mean)
                    // 计算整张图的平均偏差（例如 +0.2 说明整体变亮了）
                    cv::Scalar avg_diff = cv::mean(diff_float);
                    // 减去这个平均值，强制让全图的中心值回到 0.0
                    cv::subtract(diff_float, avg_diff, diff_float);

                    // 3. 线性映射到 0~255
                    // 目标：负数->蓝(0~100), 0->绿(127), 正数->红(150~255)
                    // 公式：Pixel = Value * Alpha + Beta
                    // Beta (偏移) = 127.5 (保证 0.0 映射为绿色)
                    // Alpha (增益) = 400.0 (增益系数，值越大越灵敏)
                    //    举例：误差 0.1 * 400 + 127.5 = 167 (微红)
                    //    举例：误差 -0.1 * 400 + 127.5 = 87 (微蓝)
                    cv::Mat err_img_vis;
                    diff_float.convertTo(err_img_vis, CV_8U, 100.0, 70); 

                    // 4. 上色 (JET 色图：0=蓝, 127=绿, 255=红)
                    cv::applyColorMap(err_img_vis, err_img_vis, cv::COLORMAP_JET);
                    
                    // ===============================================================

                    pub_error_img_.publish(
                        cv_bridge::CvImage(header_to_pub, "bgr8", err_img_vis).toImageMsg()
                    );
                }
            } catch (...) {}
        }
        
        r.sleep();
    }
}

void VsControlNodelet::desGrayOnlyCallback(const sensor_msgs::ImageConstPtr& gray_msg) 
{
    sensor_msgs::ImagePtr empty_depth(new sensor_msgs::Image);
    empty_depth->header = gray_msg->header;
    empty_depth->encoding = "mono16";
    empty_depth->width = gray_msg->width;
    empty_depth->height = gray_msg->height;
    empty_depth->step = gray_msg->width * 2;
    empty_depth->data.resize(empty_depth->step * empty_depth->height, 0);
    // 复用原有的逻辑
    desiredImgCallback(gray_msg, empty_depth);
}

void VsControlNodelet::grayOnlyCallback(const sensor_msgs::ImageConstPtr& gray_msg) 
{
    sensor_msgs::ImagePtr empty_depth(new sensor_msgs::Image);
    empty_depth->header = gray_msg->header;
    empty_depth->encoding = "mono16";
    empty_depth->width = gray_msg->width;
    empty_depth->height = gray_msg->height;
    empty_depth->step = gray_msg->width * 2;
    empty_depth->data.resize(empty_depth->step * empty_depth->height, 0);
    // 复用原有的逻辑
    controlLoopCallback(gray_msg, empty_depth);
}

} 
PLUGINLIB_EXPORT_CLASS(vs_control::VsControlNodelet, nodelet::Nodelet)