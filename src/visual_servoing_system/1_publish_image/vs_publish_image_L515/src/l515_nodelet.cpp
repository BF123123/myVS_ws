#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h> 

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <thread>
#include <atomic>
#include <mutex>

namespace vs_publish_image_L515
{

class L515Nodelet : public nodelet::Nodelet
{
private:
    // --- 成员变量 ---
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 发布者
    image_transport::Publisher pub_rgb_;
    image_transport::Publisher pub_depth_raw_;
    image_transport::Publisher pub_depth_show_;
    ros::Publisher pub_cam_info_;

    // 【修复点 1】 将 frame_id_ 声明为类的成员变量，这样 devicePoll 才能访问
    std::string frame_id_; 
    
    // 同时也把宽高FPS作为成员变量是一个好习惯（虽然你目前在Init里用了）
    int width_;
    int height_;
    int fps_;

    // 硬件相关
    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::align* align_to_color_;
    rs2::colorizer color_map_;

    // 线程控制
    std::thread device_thread_;
    std::atomic<bool> is_running_;

    // 内参缓存
    sensor_msgs::CameraInfo current_cam_info_;
    bool is_intrinsics_received_;

public:
    L515Nodelet() : align_to_color_(nullptr), is_running_(false), is_intrinsics_received_(false) {}

    ~L515Nodelet()
    {
        is_running_ = false;
        if (device_thread_.joinable()) device_thread_.join();
        if (align_to_color_) delete align_to_color_;
    }

    virtual void onInit()
    {
        nh_ = getNodeHandle();
        private_nh_ = getPrivateNodeHandle();

        NODELET_INFO("Starting L515 Nodelet...");

        // 参数名建议统一使用 "camera_frame" 
        private_nh_.param<std::string>("camera_frame", frame_id_, "camera_color_optical_frame");
        
        // 读取分辨率参数
        private_nh_.param("image_width", width_, 640);
        private_nh_.param("image_height", height_, 480);
        private_nh_.param("fps", fps_, 30);

        // 打印一下确认读取正确
        NODELET_INFO("L515 Config -> Frame: %s, Res: %dx%d @ %d FPS", 
                     frame_id_.c_str(), width_, height_, fps_);

        image_transport::ImageTransport it(nh_);
        // 建议：话题名也可以参数化，这里暂时保持硬编码
        pub_rgb_ = it.advertise("/camera/color/image_raw", 1);
        pub_depth_raw_ = it.advertise("/camera/aligned_depth_to_color/image_raw", 1);
        pub_depth_show_ = it.advertise("/camera/aligned_depth_to_color/image_show", 1);
        pub_cam_info_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1);

        try 
        {
            // 使用读取到的成员变量配置流
            cfg_.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
            cfg_.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
            
            // 启动硬件
            rs2::pipeline_profile profile = pipe_.start(cfg_);
            align_to_color_ = new rs2::align(RS2_STREAM_COLOR);

            // 获取内参
            auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            current_cam_info_ = get_camera_info(stream);
            is_intrinsics_received_ = true;

            NODELET_INFO("RealSense L515 initialized! Intrinsics loaded.");

            is_running_ = true;
            device_thread_ = std::thread(&L515Nodelet::devicePoll, this);
        }
        catch (const std::exception& e)
        {
            NODELET_ERROR("Exception: %s", e.what());
        }
    }

private:
    void devicePoll()
    {
        // 获取深度比例尺
        float depth_scale = 0.001f;
        try {
            depth_scale = get_depth_scale(pipe_.get_active_profile().get_device());
        } catch(...) { /* ignore */ }
        
        while(ros::ok() && is_running_)
        {
            try
            {
                rs2::frameset frameset = pipe_.wait_for_frames();
                rs2::frameset aligned_frames = align_to_color_->process(frameset);
                rs2::frame depth_frame = aligned_frames.get_depth_frame();
                rs2::frame color_frame = aligned_frames.get_color_frame();

                if (!depth_frame || !color_frame) continue;

                ros::Time now = ros::Time::now();
                
                // 【修复点 3】 现在这里可以正常访问成员变量 frame_id_ 了
                std::string current_frame_id = frame_id_;

                // --- 1. 处理图像 (OpenCV) ---
                auto w = depth_frame.as<rs2::video_frame>().get_width();
                auto h = depth_frame.as<rs2::video_frame>().get_height();
                
                // 硬件滤波
                depth_frame = apply_filters(depth_frame);

                cv::Mat color_mat(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                cv::Mat depth_mat(cv::Size(w, h), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

                // 简单的填孔
                hole_fill(depth_mat);

                // 单位确认 (如果不是毫米单位则转换)
                if (std::abs(depth_scale - 0.001f) > 1e-6) 
                {
                    depth_mat.convertTo(depth_mat, CV_64FC1);
                    depth_mat = depth_mat * (1000.0 * depth_scale);
                    depth_mat.convertTo(depth_mat, CV_16UC1);
                }
                
                // 可视化深度图
                rs2::frame depth_colorized = depth_frame.apply_filter(color_map_);
                cv::Mat depth_show_mat(cv::Size(w, h), CV_8UC3, (void*)depth_colorized.get_data(), cv::Mat::AUTO_STEP);

                // --- 2. 打包并发布消息 ---
                std_msgs::Header header;
                header.stamp = now;
                header.frame_id = current_frame_id; // 使用正确的 Frame ID

                pub_rgb_.publish(cv_bridge::CvImage(header, "bgr8", color_mat).toImageMsg());
                pub_depth_raw_.publish(cv_bridge::CvImage(header, "mono16", depth_mat).toImageMsg());
                pub_depth_show_.publish(cv_bridge::CvImage(header, "bgr8", depth_show_mat).toImageMsg());

                if (is_intrinsics_received_)
                {
                    current_cam_info_.header = header; 
                    pub_cam_info_.publish(current_cam_info_);
                }

            }
            catch (const std::exception& e)
            {
                NODELET_WARN_THROTTLE(1.0, "Polling error: %s", e.what());
            }
        }
    }

    sensor_msgs::CameraInfo get_camera_info(const rs2::video_stream_profile& stream)
    {
        sensor_msgs::CameraInfo info;
        rs2_intrinsics intrinsics = stream.get_intrinsics();

        info.width = intrinsics.width;
        info.height = intrinsics.height;
        info.distortion_model = "plumb_bob";
        
        info.D.resize(5);
        for(int i=0; i<5; i++) info.D[i] = intrinsics.coeffs[i];

        info.K.fill(0.0);
        info.K[0] = intrinsics.fx; info.K[2] = intrinsics.ppx;
        info.K[4] = intrinsics.fy; info.K[5] = intrinsics.ppy;
        info.K[8] = 1.0;

        info.P.fill(0.0);
        info.P[0] = intrinsics.fx; info.P[2] = intrinsics.ppx;
        info.P[5] = intrinsics.fy; info.P[6] = intrinsics.ppy;
        info.P[10] = 1.0;
        
        info.R.fill(0.0);
        info.R[0] = 1.0; info.R[4] = 1.0; info.R[8] = 1.0;

        return info;
    }

    float get_depth_scale(rs2::device dev)
    {
        for (rs2::sensor& sensor : dev.query_sensors())
            if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
                return dpt.get_depth_scale();
        return 0.001f;
    }

    rs2::frame apply_filters(rs2::frame depth_frame)
    {
        // 这里的 filter 定义为静态或成员变量会更好，减少每次创建开销，但功能上没问题
        rs2::decimation_filter dec_filter;
        dec_filter.set_option(rs2_option::RS2_OPTION_FILTER_MAGNITUDE, 1);
        rs2::spatial_filter spat_filter;
        rs2::temporal_filter temp_filter;
        
        rs2::frame filtered = dec_filter.process(depth_frame);
        filtered = spat_filter.process(filtered);
        filtered = temp_filter.process(filtered);
        return filtered;
    }

    void hole_fill(cv::Mat& img_depth)
    {
        if (img_depth.empty()) return;
        cv::Mat img_temp;
        img_depth.copyTo(img_temp);
        // 注意：简单的用均值填充整图的0点可能在复杂场景不准确，但作为示例可用
        unsigned short ave = cv::mean(img_temp)[0]; 
        for(int i = 0; i < img_temp.rows; i++)
        {
            unsigned short* row_ptr = img_temp.ptr<unsigned short>(i);
            for(int j = 0; j < img_temp.cols; j++)
            {
                if(row_ptr[j] == 0) row_ptr[j] = ave;
            }
        }
        img_temp.copyTo(img_depth);
    }
};

} // namespace

PLUGINLIB_EXPORT_CLASS(vs_publish_image_L515::L515Nodelet, nodelet::Nodelet)