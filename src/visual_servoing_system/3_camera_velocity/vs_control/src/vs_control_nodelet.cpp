#include "vs_control/vs_control_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include "vs_control/direct_visual_servoing.h" 
#include "vs_control/depth_visual_servoing.h"
#include "vs_control/defocused_visual_servoing.h"
#include "vs_control/defocused_visual_servoing_RAL.h"

namespace vs_control
{

VsControlNodelet::VsControlNodelet() 
    : dvs_algo_(nullptr), algo_initialized_(false), control_running_(false), enable_precond_(true),    // 【新增】默认开启预处理
      precond_scale_(0.1)       // 【新增】默认缩放系数为 0.1
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

// 1. 修改 reloadConfigCb
bool VsControlNodelet::reloadConfigCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    try {
        std::lock_guard<std::mutex> lock(algo_mutex_); // 【新增】加锁
        
        loadAlgorithm();
        setupSubscribers();  // 【核心修复】重建订阅管道，原本遗漏了这一句

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

// 2. 修改 loadAlgorithm，加入状态继承
void VsControlNodelet::loadAlgorithm()
{
    // 将 private_nh_ 改为直接读取 GUI 写入的全局命名空间，防止参数读不到
    ros::NodeHandle global_nh("/vs_controller");
    std::string algo_type;
    global_nh.param<std::string>("algorithm_type", algo_type, "dvs");
    
    int res_x, res_y;
    private_nh_.param("resolution_x", res_x, 640);
    private_nh_.param("resolution_y", res_y, 480);
    
    std::string strategy;
    global_nh.param<std::string>("depth_strategy", strategy, "realtime");

    global_nh.param<bool>("enable_precond", enable_precond_, true);
    global_nh.param<double>("precond_scale", precond_scale_, 0.1);
    NODELET_INFO("[ReLoad] Precond: %s | Scale: %.3f", enable_precond_ ? "ON" : "OFF", precond_scale_);

    // 【新增】在销毁旧算法前，暂存已保存的目标图像状态
    cv::Mat old_des_gray, old_des_depth, old_K, old_pose;
    double old_lambda = 0.5, old_epsilon = 1e-4;
    bool had_old_state = false;

    if (dvs_algo_ && algo_initialized_) {
        old_des_gray = dvs_algo_->image_gray_desired_.clone();
        old_des_depth = dvs_algo_->image_depth_desired_.clone();
        old_K = dvs_algo_->camera_intrinsic_.clone();
        old_pose = dvs_algo_->pose_desired_.clone();
        old_lambda = dvs_algo_->lambda_;
        old_epsilon = dvs_algo_->epsilon_;
        had_old_state = true;
    }

    // 清理旧对象
    if (dvs_algo_) { 
        delete dvs_algo_; 
        dvs_algo_ = nullptr; 
        algo_initialized_ = false; 
    }

    // 实例化新算法 (原本的代码)
    if (algo_type == "dvs" || algo_type == "direct") {
        double depth_val; private_nh_.param("depth_constant_value", depth_val, 0.24);
        dvs_algo_ = new Direct_Visual_Servoing(res_x, res_y, strategy, depth_val);
    }
    else if (algo_type == "depth" || algo_type == "pure_depth") {
        dvs_algo_ = new Depth_Visual_Servoing(res_x, res_y);
    }
    else if (algo_type == "defocused") {
        double depth_val; private_nh_.param("depth_constant_value", depth_val, 0.24);
        dvs_algo_ = new Defocused_Visual_Servoing(res_x, res_y, strategy, depth_val);
    }
    else if (algo_type == "defocused_RAL") {
        double depth_val; private_nh_.param("depth_constant_value", depth_val, 0.24);
        dvs_algo_ = new Defocused_Visual_Servoing_RAL(res_x, res_y, strategy, depth_val);
    }
    else {
        dvs_algo_ = new Direct_Visual_Servoing(res_x, res_y);
    }

    if (dvs_algo_) {
        dvs_algo_->set_precondition(enable_precond_, precond_scale_);
    }

    // 【新增】如果旧算法已经按过 [M] 初始化了，自动灌入新算法中实现无缝切换
    if (had_old_state) {
        dvs_algo_->init_VS(old_lambda, old_epsilon, old_des_gray, old_des_depth, old_des_gray, old_K, old_pose);
        algo_initialized_ = true;
        NODELET_INFO("Transferred desired state to new algorithm seamlessly.");
    }
}

void VsControlNodelet::setupSubscribers()
{
    // 1. 强行关闭并清理旧的所有订阅器
    sub_des_gray_only_.shutdown();
    sub_gray_only_.shutdown();
    
    if (sync_des_) sync_des_.reset();
    if (sub_des_gray_) sub_des_gray_.reset();
    if (sub_des_depth_) sub_des_depth_.reset();
    
    if (sync_) sync_.reset();
    if (sub_gray_) sub_gray_.reset();
    if (sub_depth_) sub_depth_.reset();

    // ================== 【核心修复：统一参数命名空间】 ==================
    // 2. 读取 GUI 修改后的全局命名空间下的最新深度策略
    ros::NodeHandle global_nh("/vs_controller");
    global_nh.param<std::string>("depth_strategy", depth_strategy_, "constant");
    // ===================================================================

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
    std::lock_guard<std::mutex> lock(algo_mutex_); // 【新增】
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
    // ================== 锁外操作：图像转换与缓存 (耗时操作) ==================
    cv::Mat I_f, Z_f;
    try {
        cv::Mat I_curr = cv_bridge::toCvShare(gray_msg, "mono8")->image;
        cv::Mat Z_curr = cv_bridge::toCvShare(depth_msg, "mono16")->image;
        I_curr.convertTo(I_f, CV_64F, 1.0/255.0);
        Z_curr.convertTo(Z_f, CV_64F, 1.0/1000.0);
    } catch (std::exception& e) {
        NODELET_ERROR("Image convert error: %s", e.what());
        return;
    }

    if (pub_error_img_.getNumSubscribers() > 0)
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        shared_curr_img_ = I_f.clone(); 
        shared_header_ = gray_msg->header; 
        new_data_ready_ = true;            
    }

    // ================== 锁内操作：核心算法状态机 ==================
    std::lock_guard<std::mutex> lock(algo_mutex_); // 【关键优化：推迟加锁，仅保护核心】
    
    if (!algo_initialized_) {
        if (control_running_) {
            NODELET_WARN_THROTTLE(2.0, "Auto Servo is ON, but Target is NOT SET! Press [M] first.");
        }
        return; 
    }

    if (!control_running_) return;

    try {
        dvs_algo_->set_image_gray_current(I_f);
        dvs_algo_->set_image_depth_current(Z_f);
        
        cv::Mat T_curr = cv::Mat::eye(4, 4, CV_64F); 
        dvs_algo_->save_data(T_curr);

        cv::Mat v_cam = dvs_algo_->get_camera_velocity(); 

        double error_norm = cv::norm(I_f, dvs_algo_->image_gray_desired_, cv::NORM_L2);
        double v_z_val = v_cam.at<double>(2);
        NODELET_INFO_THROTTLE(1.0, "Error=%.2f, Vz=%.4f", error_norm, v_z_val);

        if (dvs_algo_->is_success()) {
            NODELET_INFO("Visual Servoing SUCCESS!");
            control_running_ = false; 
            dvs_algo_->write_data();
            pub_cmd_vel_.publish(geometry_msgs::Twist());
            return;
        }

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

        // 1. 取当前图像数据 (用 img_mutex_ 保护)
        {
            std::lock_guard<std::mutex> lock(img_mutex_);
            if (new_data_ready_) {
                raw_curr_img = shared_curr_img_;
                header_to_pub = shared_header_;
                new_data_ready_ = false;
                has_data = true;
            }
        }

        // 2. 取目标图像数据 (用 algo_mutex_ 保护，【关键优化：瞬间拷贝并释放锁】)
        cv::Mat des_img_safe;
        {
            std::lock_guard<std::mutex> lock(algo_mutex_); 
            if (dvs_algo_ && !dvs_algo_->image_gray_desired_.empty()) {
                des_img_safe = dvs_algo_->image_gray_desired_.clone(); // 只做深度拷贝，耗时极短
            }
        }

        // 3. 处理与发布热力图 (完全脱离锁的束缚，不再阻塞主控循环)
        if (has_data && !raw_curr_img.empty() && !des_img_safe.empty()) 
        {
            try {
                if (pub_error_img_.getNumSubscribers() > 0) {
                    
                    cv::Mat diff_float;
                    cv::subtract(raw_curr_img, des_img_safe, diff_float); // 用刚才深拷贝的 des_img_safe
                    
                    cv::Scalar avg_diff = cv::mean(diff_float);
                    cv::subtract(diff_float, avg_diff, diff_float);

                    cv::Mat err_img_vis;
                    diff_float.convertTo(err_img_vis, CV_8U, 100.0, 70); 
                    cv::applyColorMap(err_img_vis, err_img_vis, cv::COLORMAP_JET);
                    
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