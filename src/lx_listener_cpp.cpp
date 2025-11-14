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
        RCLCPP_INFO(rclcpp::get_logger("lx_listener_cpp"), "I heard: '%s' %d", msg.data.c_str(),k);
        this->k++;
    }
    int k;
    lxros::LxNode node;
};

int main(int argc, char ** argv)
{

    ListenerClass cl;
    

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
