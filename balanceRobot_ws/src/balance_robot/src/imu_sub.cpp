#include <memory>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

uint32_t time_prev = 0;
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
            uint32_t time_nanosec = msg->header.stamp.nanosec;
            int32_t time_sec = msg->header.stamp.sec;

            uint32_t time = time_sec + (time_nanosec / 1000000000);
            uint32_t period = (time - time_prev);

            float g_x = msg->angular_velocity.x;
            float a_y = msg->linear_acceleration.y;
            float a_z = msg->linear_acceleration.z;

            float phi = atan2(a_y, a_z)*57.295780;

            float theta = (0.95 * (theta_prev + (g_x * (period)))) + (0.05 * phi);

            RCLCPP_INFO(this->get_logger(), "phi: %f", phi);
            RCLCPP_INFO(this->get_logger(), "period: %d", period);
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