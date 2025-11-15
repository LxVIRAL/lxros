// src/lx_listener_cpp.cpp
#include <std_msgs/msg/string.hpp>
#include "lxros/lxros.hpp"



class ListenerClass
{
public:
    ListenerClass() : node("lx_listener_cpp") {        
        this->k=0;
        
        auto sub = node.sub<std_msgs::msg::String>("chatter_cpp",&ListenerClass::callback,this);    
    }
    void callback(const std_msgs::msg::String & msg)
    {    
        RCLCPP_INFO(node.logger(), "C++ method callback heard: '%s' %d", msg.data.c_str(),k);
        this->k--;
    }
    int k;
    lxros::LxNode node;
};

int main(int argc, char ** argv)
{
    lxros::init(argc, argv);
    ListenerClass cl;
    lxros::LxNode node("lx_listener_cpp_2");
    

    
    auto sub = node.sub<std_msgs::msg::String>("chatter_cpp",
        [&node](const std_msgs::msg::String & msg) {
            RCLCPP_INFO(node.logger(), "C++ lambda callback heard: '%s'", msg.data.c_str());
        }
    );
    
    lxros::run();
    return 0;
}
