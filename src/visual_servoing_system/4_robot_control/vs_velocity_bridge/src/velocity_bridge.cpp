#include "vs_velocity_bridge/velocity_bridge.h"

namespace vs_velocity_bridge {

VelocityBridge::VelocityBridge() : pnh_("~"), is_enabled_(false), first_run_(true)
{
    // 【新增】 初始化状态变量
    last_v_cmd_.setZero();
    last_w_cmd_.setZero();
}

VelocityBridge::~VelocityBridge()
{
}

void VelocityBridge::init()
{

    //  替换为直接读取节点私有参数
    pnh_.param<std::string>("camera_frame", camera_frame_, "basler_camera_optical_frame");
    pnh_.param<std::string>("tool_frame", tool_frame_, "tool0");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");

    ROS_INFO("[VelocityBridge] Target Camera Frame: [%s]", camera_frame_.c_str());
        // ======
    pnh_.param<std::string>("tool_frame", tool_frame_, "tool0");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");

    // ================= 【新增】 加载安全参数 (带默认值) =================
    pnh_.param<double>("max_linear_vel", max_linear_vel_, 0.2);   // 默认 0.2 m/s
    pnh_.param<double>("max_angular_vel", max_angular_vel_, 0.5); // 默认 0.5 rad/s
    pnh_.param<double>("max_linear_acc", max_linear_acc_, 1.0);   // 默认 1.0 m/s^2
    pnh_.param<double>("max_angular_acc", max_angular_acc_, 2.0); // 默认 2.0 rad/s^2

    ROS_INFO("[VelocityBridge] Safety Limits: LinVel=%.2f, AngVel=%.2f", max_linear_vel_, max_angular_vel_);
    // ====================================================================

    // 2. 订阅与发布
    sub_cam_vel_ = nh_.subscribe("/vs/camera_velocity_raw", 1, &VelocityBridge::velocityCallback, this);
    pub_robot_vel_ = nh_.advertise<geometry_msgs::Twist>("/twist_controller/command", 1);
    srv_enable_ = nh_.advertiseService("/vs_bridge/enable", &VelocityBridge::enableCallback, this);

    ROS_INFO("[VelocityBridge] Initialized.");
    ROS_INFO("[VelocityBridge] Waiting for service call to /vs_bridge/enable ...");
}

// 【新增】限幅与平滑核心逻辑
// 【新增】限幅与平滑核心逻辑 (带醒目警告)
void VelocityBridge::limitTwist(Eigen::Vector3d& v, Eigen::Vector3d& w, double dt)
{
    // --- 1. 绝对值限幅 (Saturation) ---
    // 保持方向，按比例缩小
    double v_norm = v.norm();
    if (v_norm > max_linear_vel_) {
        // \033[1;31m 为亮红色，\033[0m 为恢复默认颜色
        ROS_WARN_THROTTLE(0.5, "\033[1;31m[LIMITER] LINEAR VELOCITY CLAMPED! Req: %.3f m/s, Max: %.3f m/s\033[0m", v_norm, max_linear_vel_);
        v = v * (max_linear_vel_ / v_norm);
    }
    
    double w_norm = w.norm();
    if (w_norm > max_angular_vel_) {
        ROS_WARN_THROTTLE(0.5, "\033[1;31m[LIMITER] ANGULAR VELOCITY CLAMPED! Req: %.3f rad/s, Max: %.3f rad/s\033[0m", w_norm, max_angular_vel_);
        w = w * (max_angular_vel_ / w_norm);
    }

    // --- 2. 加速度平滑 (Ramping) ---
    // 只有非首次运行且dt合理时才计算
    if (!first_run_ && dt > 0.001) 
    {
        // 计算当前帧允许的最大增量 (dv = a * dt)
        double max_v_inc = max_linear_acc_ * dt;
        double max_w_inc = max_angular_acc_ * dt;

        // 计算期望的增量
        Eigen::Vector3d dv = v - last_v_cmd_;
        Eigen::Vector3d dw = w - last_w_cmd_;

        // 如果变化太剧烈，则限制在最大加速度范围内
        double dv_norm = dv.norm();
        if (dv_norm > max_v_inc) {
            // \033[1;33m 为亮黄色
            ROS_WARN_THROTTLE(0.5, "\033[1;33m[LIMITER] LINEAR ACCEL CLAMPED (Ramping)! Req_dv: %.4f, Max_dv: %.4f\033[0m", dv_norm, max_v_inc);
            v = last_v_cmd_ + dv * (max_v_inc / dv_norm);
        }

        double dw_norm = dw.norm();
        if (dw_norm > max_w_inc) {
            ROS_WARN_THROTTLE(0.5, "\033[1;33m[LIMITER] ANGULAR ACCEL CLAMPED (Ramping)! Req_dw: %.4f, Max_dw: %.4f\033[0m", dw_norm, max_w_inc);
            w = last_w_cmd_ + dw * (max_w_inc / dw_norm);
        }
    }

    // 更新历史记录
    last_v_cmd_ = v;
    last_w_cmd_ = w;
    first_run_ = false;
}

bool VelocityBridge::enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    is_enabled_ = req.data;
    res.success = true;
    res.message = is_enabled_ ? "Bridge ENABLED" : "Bridge DISABLED";
    
    if (is_enabled_) {
        // 【新增】每次开启时，重置状态，防止速度突变
        last_v_cmd_.setZero();
        last_w_cmd_.setZero();
        first_run_ = true;
        last_time_ = ros::Time::now();
    }
    else {
        // 关闭时发送全零速度，防止机器人漂移
        pub_robot_vel_.publish(geometry_msgs::Twist());
    }
    
    ROS_INFO_STREAM("[VelocityBridge] " << res.message);
    return true;
}

void VelocityBridge::velocityCallback(const geometry_msgs::TwistConstPtr& msg)
{
    if (!is_enabled_) return; 

    // 【新增】 计算时间间隔 dt
    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    // 异常处理：如果是刚启动或系统卡顿(dt过大)，给予一个安全的默认值(如20Hz)
    if (first_run_ || dt > 1.0) dt = 0.05; 
    last_time_ = now;

    // 1. 获取输入速度 (相机坐标系下的期望速度)
    Eigen::Vector3d v_cam(msg->linear.x, msg->linear.y, msg->linear.z);
    Eigen::Vector3d w_cam(msg->angular.x, msg->angular.y, msg->angular.z);

    // 定义 TF 变换容器
    tf::StampedTransform tf_tc; // Tool -> Camera
    tf::StampedTransform tf_bt; // Base -> Tool

    try {
        // 等待 TF 树稳定
        listener_.waitForTransform(tool_frame_, camera_frame_, ros::Time(0), ros::Duration(0.01));
        listener_.waitForTransform(base_frame_, tool_frame_, ros::Time(0), ros::Duration(0.01));

        // 获取变换关系
        listener_.lookupTransform(tool_frame_, camera_frame_, ros::Time(0), tf_tc);
        listener_.lookupTransform(base_frame_, tool_frame_, ros::Time(0), tf_bt);
    }
    catch (tf::TransformException &ex) {
        ROS_WARN_THROTTLE(2.0, "[VelocityBridge] TF Error: %s", ex.what());
        
        // 【新增】 TF 失败时，为了安全，重置加速度记录
        first_run_ = true;
        pub_robot_vel_.publish(geometry_msgs::Twist()); // 发送0速
        return;
    }

    // ... (中间的刚体变换逻辑保持不变) ...
    // =================================================================================
    // 步骤 A: Cam -> Tool
    tf::Matrix3x3 tf_R_tc = tf_tc.getBasis();
    Eigen::Matrix3d R_tc;
    for(int i=0; i<3; i++) for(int j=0; j<3; j++) R_tc(i,j) = tf_R_tc[i][j];
    Eigen::Vector3d P_tc(tf_tc.getOrigin().x(), tf_tc.getOrigin().y(), tf_tc.getOrigin().z());

    Eigen::Vector3d w_tool_local = R_tc * w_cam;
    Eigen::Vector3d v_tool_local = (R_tc * v_cam) - w_tool_local.cross(P_tc);

    // =================================================================================
    // 步骤 B: Tool -> Base
    tf::Matrix3x3 tf_R_bt = tf_bt.getBasis();
    Eigen::Matrix3d R_bt;
    for(int i=0; i<3; i++) for(int j=0; j<3; j++) R_bt(i,j) = tf_R_bt[i][j];

    Eigen::Vector3d v_cmd_base = R_bt * v_tool_local;
    Eigen::Vector3d w_cmd_base = R_bt * w_tool_local;


    // ================= 【新增】 调用安全限幅 =================
    // 在发布给 robot 之前，进行限幅和平滑处理
    limitTwist(v_cmd_base, w_cmd_base, dt);
    // =======================================================


    // 4. 发布结果
    geometry_msgs::Twist cmd;
    cmd.linear.x = v_cmd_base.x(); 
    cmd.linear.y = v_cmd_base.y(); 
    cmd.linear.z = v_cmd_base.z();
    cmd.angular.x = w_cmd_base.x(); 
    cmd.angular.y = w_cmd_base.y(); 
    cmd.angular.z = w_cmd_base.z();

    pub_robot_vel_.publish(cmd);
}

} // namespace