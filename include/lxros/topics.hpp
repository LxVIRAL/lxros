#pragma once
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "lxros/detail/lx_context.hpp"

namespace lxros {

// ------------------------
// Publisher wrapper (ROS1-style)
// ------------------------

template<class Msg>
class Publisher {
public:
    using RclPublisher     = rclcpp::Publisher<Msg>;
    using RclPublisherPtr  = std::shared_ptr<RclPublisher>;
    using WeakRclPublisher = std::weak_ptr<RclPublisher>;

    Publisher() = default;

    explicit Publisher(const WeakRclPublisher & weak_pub)
    : weak_pub_(weak_pub)
    {}

    // ROS1-style API
    void publish(const Msg & msg) const
    {
        if (auto p = weak_pub_.lock()) {
            p->publish(msg);
        }
        // If expired: node was destroyed → do nothing.
    }

    // Escape hatch
    RclPublisherPtr raw() const
    {
        return weak_pub_.lock();
    }

    explicit operator bool() const
    {
        return !weak_pub_.expired();
    }

private:
    WeakRclPublisher weak_pub_;
};


// ------------------------
// Subscription wrapper
// ------------------------

template<class Msg>
class Subscription {
public:
    using RclSubscription     = rclcpp::Subscription<Msg>;
    using RclSubscriptionPtr  = std::shared_ptr<RclSubscription>;
    using WeakRclSubscription = std::weak_ptr<RclSubscription>;

    Subscription() = default;

    explicit Subscription(const WeakRclSubscription & weak_sub)
    : weak_sub_(weak_sub)
    {}

    RclSubscriptionPtr raw() const
    {
        return weak_sub_.lock();
    }

    explicit operator bool() const
    {
        return !weak_sub_.expired();
    }

private:
    WeakRclSubscription weak_sub_;
};

} // namespace lxros