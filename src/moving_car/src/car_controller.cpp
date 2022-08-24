#include <cstring>
#include <functional>
#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/publisher_options.hpp"

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class CarController : public rclcpp::Node
{
public:
    CarController(const std::string &name) : Node(name)
    {
        camera_subs_ = this->create_subscription<sensor_msgs::msg::Image>("/car_camera_raw", 10, std::bind(&CarController::camera_image_callback, this, _1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

protected:
    void camera_image_callback(const sensor_msgs::msg::Image &img)
    {
        const double target_center = img.width / 2.0;
        const double kp = 0.2;

        // 转换原始图像
        auto cv_img = cv_bridge::toCvCopy(img, "bgr8");
        cv::Mat gray_img;

        // 灰度化
        cv::cvtColor(cv_img->image, gray_img, cv::COLOR_BGR2GRAY);

        // 二值化
        cv::Mat bin_img;
        cv::threshold(gray_img, bin_img, 128, 1, cv::THRESH_BINARY_INV);

        // 计算图像矩
        cv::Moments m = cv::moments(bin_img, true);
        double center = m.m10 / m.m00;

        // 更新速度控制指令
        geometry_msgs::msg::Twist vel;
        if (std::isfinite(center))
        {
            vel.linear.x = 0.4;
            vel.angular.z = -kp * (center - target_center);
        }

        cmd_vel_pub_->publish(vel);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subs_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarController>("car_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}