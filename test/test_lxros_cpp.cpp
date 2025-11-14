// test/test_lxros_cpp.cpp
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include <std_msgs/msg/string.hpp>

#include "lxros/lxros.hpp"

using namespace std::chrono_literals;

TEST(LxRos, TwoNodesSameProcess)
{
    lxros::LxNode talker("lx_talker");
    lxros::LxNode listener("lx_listener");

    auto pub = talker.pub<std_msgs::msg::String>("chatter");

    std::atomic<int> received{0};

    auto sub = listener.sub<std_msgs::msg::String>(
        "chatter",
        [&](const std_msgs::msg::String & msg) {
            (void)msg;
            received.fetch_add(1, std::memory_order_relaxed);
        }
    );

    auto start = std::chrono::steady_clock::now();

    // For up to 1 second, publish periodically and spin
    while (received.load(std::memory_order_relaxed) < 3 &&
           std::chrono::steady_clock::now() - start < 1s)
    {
        std_msgs::msg::String msg;
        msg.data = "hello from talker";
        pub.publish(msg);

        lxros::spin_for(10ms);
        std::this_thread::sleep_for(10ms);
    }

    EXPECT_GE(received.load(std::memory_order_relaxed), 1);
}
