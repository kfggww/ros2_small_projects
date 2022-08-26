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
        params_change_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        auto callback = std::bind(&CarController::on_param_change, this, _1);
        add_param_register_callback("kp1", 10.1, callback);
        add_param_register_callback("kp2", 20.1, callback);
        add_param_register_callback("v_normal", 0.3, callback);
        add_param_register_callback("R", 0.08, callback);
        add_param_register_callback("vline_ratio", 2.08, callback);
    }

protected:
    void camera_image_callback(const sensor_msgs::msg::Image &img)
    {
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
        double mx = m.m10 / m.m00;
        double my = m.m01 / m.m00;

        // 更新速度控制指令
        int width = img.width;
        int height = img.height;

        double kp1 = param_settings_map_["kp1"].param_val;
        double kp2 = param_settings_map_["kp2"].param_val;
        double v_normal = param_settings_map_["v_normal"].param_val;
        double R = param_settings_map_["R"].param_val;
        double vline_ratio = param_settings_map_["vline_ratio"].param_val;

        int vline_1 = width / vline_ratio;
        int vline_2 = width - vline_1;

        double w = 0;

        geometry_msgs::msg::Twist vel;
        if (std::isfinite(mx) && std::isfinite(my))
        {
            if (mx >= vline_1 && mx <= vline_2)
            {
                vel.linear.x = v_normal;
            }
            else if (mx < vline_1)
            {
                w = kp1 * (vline_1 - mx) / vline_1 + kp2 * my / height;
                vel.linear.x = w * R;
                vel.angular.z = w;
            }
            else
            {
                w = kp1 * (mx - vline_2) / (width - vline_2) + kp2 * my / height;
                vel.linear.x = w * R;
                vel.angular.z = -w;
            }
        }
        cmd_vel_pub_->publish(vel);
    }

    void on_param_change(const rclcpp::Parameter &param)
    {
        const std::string param_name = param.get_name();
        double val = param.as_double();

        ParamSetting &ps = param_settings_map_.at(param_name);
        ps.param_val = val;

        RCLCPP_INFO(this->get_logger(), "Set param [%s] to [%.3f]", param_name.c_str(), val);
    }

    void add_param_register_callback(const std::string &name, double value, rclcpp::ParameterEventHandler::ParameterCallbackType callback)
    {

        struct ParamSetting ps;
        ps.param_name = name;
        ps.param_val = value;

        this->declare_parameter(name, value);
        ps.param_cb_handle = params_change_handler_->add_parameter_callback(name, callback);

        param_settings_map_.insert(std::make_pair(name, ps));
    }

private:
    struct ParamSetting
    {
        std::string param_name;
        double param_val;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> param_cb_handle;
    };

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subs_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::shared_ptr<rclcpp::ParameterEventHandler> params_change_handler_;
    std::map<std::string, struct ParamSetting> param_settings_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarController>("car_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}