#include <chrono>
#include <std_msgs/msg/string.hpp>
#include "lxros/lxros.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    lxros::init(argc, argv);
    lxros::LxNode node("lx_talker_cpp");
    auto pub = node.pub<std_msgs::msg::String>("chatter_cpp");

    /*
    auto cb = std::function<void()>(
        [pub]() mutable {
            std_msgs::msg::String msg;
            msg.data = "hello from lx_talker_cpp";
            pub.publish(msg);
        }
    );

    auto timer = node.rcl_node()->create_wall_timer(1000ms, cb);

    lxros::run();                          // spins executor
*/

    int k=0;
    while(rclcpp::ok())
    {
        k++;
        std_msgs::msg::String msg;
        msg.data = "hello from lx_talker_cpp"+std::to_string(k);
        pub.publish(msg);
        //lxros::spin_for(1s);
        std::this_thread::sleep_for (std::chrono::seconds(1));
    }


    return 0;
}
