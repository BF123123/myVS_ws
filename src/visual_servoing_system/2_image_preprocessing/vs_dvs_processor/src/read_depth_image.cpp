#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 1. 极其重要：必须加上 cv::IMREAD_ANYDEPTH，否则会被强制压缩成 8 位！
    cv::Mat depth_image = cv::imread("current_desired_depth.png", cv::IMREAD_ANYDEPTH);

    // 2. 检查图片是否成功读取，以及数据类型是否正确
    if (depth_image.empty()) {
        std::cerr << "错误：无法读取图片！" << std::endl;
        return -1;
    }
    
    // 检查深度图类型，CV_16U 对应的整数值是 2
    if (depth_image.type() != CV_16U) {
        std::cerr << "警告：这张图不是 16 位深度图！当前类型为: " << depth_image.type() << std::endl;
        std::cerr << "真实的物理深度数据可能已经在保存时丢失了。" << std::endl;
        return -1;
    }

    // 3. 读取特定像素的深度值 (假设你要查 x=320, y=240 中心的点)
    int x = 320;
    int y = 240;
    
    // 注意：OpenCV 中访问像素是 .at<类型>(行y, 列x)
    uint16_t depth_value = depth_image.at<uint16_t>(y, x);

    // 4. 打印结果
    if (depth_value == 0) {
        std::cout << "坐标 (" << x << ", " << y << ") 的深度值为 0。说明该点无效（可能太近出现盲区，或反光严重丢失了深度）。" << std::endl;
    } else {
        // 通常 RealSense 保存的 16 位 PNG，单位是毫米 (mm)
        float distance_meters = depth_value / 1000.0f; 
        std::cout << "坐标 (" << x << ", " << y << ") 的像素值为: " << depth_value << std::endl;
        std::cout << "换算成物理距离大约为: " << distance_meters << " 米" << std::endl;
    }

    return 0;
}