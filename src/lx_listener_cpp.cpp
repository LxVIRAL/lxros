// src/lx_listener_cpp.cpp
#include <std_msgs/msg/string.hpp>
#include "lxros/lxros.hpp"

int callback(const std_msgs::msg::String & msg)
{    
    RCLCPP_INFO(rclcpp::get_logger("lx_listener_cpp"), "I heard: '%s'", msg.data.c_str());
    return 0;
}

int main(int argc, char ** argv)
{
    lxros::LxNode node("lx_listener_cpp");
    auto sub = node.sub<std_msgs::msg::String>("chatter_cpp",callback);

    /*
    auto sub = node.sub<std_msgs::msg::String>("chatter_cpp",
        [&node](const std_msgs::msg::String & msg) {
            RCLCPP_INFO(node.logger(), "Received: '%s'", msg.data.c_str());
        }
    );
    */
    lxros::run();
    return 0;
}
