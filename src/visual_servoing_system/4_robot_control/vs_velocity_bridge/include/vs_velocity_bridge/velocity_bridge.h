#ifndef VS_VELOCITY_BRIDGE_H
#define VS_VELOCITY_BRIDGE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <string>

namespace vs_velocity_bridge {

class VelocityBridge
{
public:
    VelocityBridge();
    ~VelocityBridge();

    void init();
    // void run(); // 如果cpp里没实现run，这里声明了会报错，假设你在main里用了ros::spin

private:
    bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    void velocityCallback(const geometry_msgs::TwistConstPtr& msg);

    // 【新增】安全限幅辅助函数
    void limitTwist(Eigen::Vector3d& v, Eigen::Vector3d& w, double dt);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    ros::Subscriber sub_cam_vel_;
    ros::Publisher pub_robot_vel_;
    ros::ServiceServer srv_enable_;
    
    tf::TransformListener listener_;
    
    bool is_enabled_;
    std::string camera_frame_;
    std::string tool_frame_;
    std::string base_frame_;

    // ================= 【新增】 安全设置变量 =================
    // 限制参数
    double max_linear_vel_;       // 最大线速度 (m/s)
    double max_angular_vel_;      // 最大角速度 (rad/s)
    double max_linear_acc_;       // 最大线加速度 (m/s^2)
    double max_angular_acc_;      // 最大角加速度 (rad/s^2)

    // 状态记录 (用于计算加速度和平滑)
    Eigen::Vector3d last_v_cmd_;  // 上一时刻下发的线速度
    Eigen::Vector3d last_w_cmd_;  // 上一时刻下发的角速度
    ros::Time last_time_;         // 上一时刻的时间戳
    bool first_run_;              // 是否刚启用
    // ========================================================
};

} // namespace

#endif