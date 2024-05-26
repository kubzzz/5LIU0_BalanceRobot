#include <memory>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

float time_prev = 0;
float theta_prev = 0;

class ImuSubscriber : public rclcpp::Node
{
    public:
        ImuSubscriber()
        : Node("imu_subscriber")
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu_plugin/out", 10, std::bind(&ImuSubscriber::topic_callback, this, _1));
        }

    private:
        void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
        {
            float time_nanosec = msg->header.stamp.nanosec;
            float time_sec = msg->header.stamp.sec;

            float time = time_sec + (time_nanosec / 1000000000);
            float period = (time - time_prev);

            float g_x = msg->angular_velocity.x;
            float a_y = msg->linear_acceleration.y;
            float a_z = msg->linear_acceleration.z;

            float phi = atan2(a_y, a_z)*57.295780;

            float theta = (0.95 * (theta_prev + (g_x * (period)))) + (0.05 * phi);

            time_prev = time;
            theta_prev = theta;

            RCLCPP_INFO(this->get_logger(), "phi: %f", phi);
            RCLCPP_INFO(this->get_logger(), "period: %f", period);
            RCLCPP_INFO(this->get_logger(), "theta: %f", theta);
        }
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuSubscriber>());
    rclcpp::shutdown();
    return 0;
}