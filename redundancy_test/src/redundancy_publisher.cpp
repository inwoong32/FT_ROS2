#include <chrono>
#include <memory>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class RedundancyPublisher : public rclcpp::Node
{
public:
    RedundancyPublisher() : Node("redundancy_publisher"), count_(0)
    {
        std::cout << "=== C++ Publisher Node Created ===" << std::endl;
        std::cout << "Node Name: " << this->get_name() << std::endl;
        std::cout << "Sub Title: " << this->get_sub_title() << std::endl;
        std::cout << "Node Type: " << this->get_node_type() << std::endl;
        std::cout << "Namespace: " << this->get_namespace() << std::endl;
        std::cout << "Fully Qualified Name: " << this->get_fully_qualified_name() << std::endl;
        
        // Test use_sim_time
        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        std::cout << "Use Sim Time: " << (use_sim_time ? "true" : "false") << std::endl;
        
        // Test parameters
        this->declare_parameter("use_sim_time", false);
        this->declare_parameter("test_string_param", "default_string");
        this->declare_parameter("test_int_param", 0);
        this->declare_parameter("test_double_param", 0.0);
        this->declare_parameter("test_bool_param", false);
        this->declare_parameter("node_specific_param", "default_value");
        
        std::string test_string = this->get_parameter("test_string_param").as_string();
        int test_int = this->get_parameter("test_int_param").as_int();
        double test_double = this->get_parameter("test_double_param").as_double();
        bool test_bool = this->get_parameter("test_bool_param").as_bool();
        std::string node_specific = this->get_parameter("node_specific_param").as_string();
        
        std::cout << "Parameters:" << std::endl;
        std::cout << "  test_string_param: " << test_string << std::endl;
        std::cout << "  test_int_param: " << test_int << std::endl;
        std::cout << "  test_double_param: " << test_double << std::endl;
        std::cout << "  test_bool_param: " << (test_bool ? "true" : "false") << std::endl;
        std::cout << "  node_specific_param: " << node_specific << std::endl;
        
        // Test command line arguments
        std::cout << "Command Line Arguments:" << std::endl;
        for (size_t i = 0; i < this->get_node_options().arguments().size(); ++i) {
            std::cout << "  [" << i << "]: " << this->get_node_options().arguments()[i] << std::endl;
        }
        
        std::cout << "===================================" << std::endl;

        // Create publisher with remapped topic name
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("redundancy_topic", 10);
        timer_ = this->create_wall_timer(
            10s, std::bind(&RedundancyPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Redundancy Publisher started");
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Int32();
        message.data = count_++;

        std::cout << "=== Publishing Message ===" << std::endl;
        std::cout << "Node: " << this->get_name() << " | Sub Title: " << this->get_sub_title()
                  << " | Type: " << this->get_node_type() << std::endl;
        std::cout << "Publishing Value: " << message.data << std::endl;
        std::cout << "==========================" << std::endl;

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RedundancyPublisher>());
    rclcpp::shutdown();
    return 0;
}