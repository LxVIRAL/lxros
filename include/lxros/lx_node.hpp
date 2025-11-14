#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "lxros/detail/lx_context.hpp"   // <-- important

namespace lxros {
namespace detail {

// Implemented in detail/lx_context.hpp
void register_node(const std::shared_ptr<rclcpp::Node> & node);

} // namespace detail


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


// ------------------------
// LxNode
// ------------------------

class LxNode {
public:
    using RclNodePtr = std::shared_ptr<rclcpp::Node>;

    LxNode() = delete;

    explicit LxNode(const std::string & name)
    {
        detail::ensure_init();                          // <-- NEW
        node_ = std::make_shared<rclcpp::Node>(name);
        detail::register_node(node_);
    }

    LxNode(const std::string & name,
           const std::string & ns)
    {
        detail::ensure_init();                          // <-- NEW
        node_ = std::make_shared<rclcpp::Node>(name, ns);
        detail::register_node(node_);
    }

    LxNode(const std::string & name,
           const rclcpp::NodeOptions & options)
    {
        detail::ensure_init();                          // <-- NEW
        node_ = std::make_shared<rclcpp::Node>(name, options);
        detail::register_node(node_);
    }

    static LxNode from_existing(const RclNodePtr & node)
    {
        detail::ensure_init();                          // safer if existing node
        LxNode wrapper(node);
        detail::register_node(wrapper.node_);
        return wrapper;
    }

    // Copy/move: all copies share the same underlying node
    LxNode(const LxNode &) = default;
    LxNode(LxNode &&) noexcept = default;
    LxNode & operator=(const LxNode &) = default;
    LxNode & operator=(LxNode &&) noexcept = default;

    // Basic info
    std::string name() const
    {
        return node_->get_name();
    }

    std::string namespace_() const
    {
        return node_->get_namespace();
    }

    rclcpp::Logger logger() const
    {
        return node_->get_logger();
    }

    // Escape hatch
    RclNodePtr rcl_node() const
    {
        return node_;
    }

    bool valid() const
    {
        return static_cast<bool>(node_);
    }

    // ------------------------
    // Publisher helper
    // ------------------------
    template<class Msg>
    Publisher<Msg> pub(const std::string & topic,
                       std::size_t queue_size = 10)
    {
        auto pub = node_->create_publisher<Msg>(topic, queue_size);

        // Keep it alive for the lifetime of this node
        publishers_.push_back(pub);

        // Wrap it with a weak_ptr for the user-facing Publisher
        return Publisher<Msg>(pub);
    }

    // ------------------------
    // Subscription helper – generic callable
    //   Callback: void(const Msg &)
// ------------------------
    template<class Msg, class Callback>
    Subscription<Msg> sub(const std::string & topic,
                          Callback && cb,
                          std::size_t queue_size = 10)
    {
        auto wrapper_cb =
            [fn = std::forward<Callback>(cb)](const std::shared_ptr<Msg> msg) {
                fn(*msg);
            };

        auto sub = node_->create_subscription<Msg>(topic, queue_size, wrapper_cb);

        // Keep it alive
        subscriptions_.push_back(sub);

        return Subscription<Msg>(sub);
    }

    // ------------------------
    // Subscription helper – member function
    //   memfn: void(T::*)(const Msg &)
    //   obj:   T*
// ------------------------
    template<class Msg, class T>
    Subscription<Msg> sub(const std::string & topic,
                          void (T::*memfn)(const Msg &),
                          T * obj,
                          std::size_t queue_size = 10)
    {
        auto wrapper =
            [obj, memfn](const Msg & msg) {
                (obj->*memfn)(msg);
            };

        return sub<Msg>(topic, std::move(wrapper), queue_size);
    }

    // (timer(), service(), action helpers will be added later)

private:
    explicit LxNode(const RclNodePtr & node)
    : node_(node)
    {}

    RclNodePtr node_;

    // Keep pubs/subs/timers alive for the lifetime of this node
    std::vector<rclcpp::PublisherBase::SharedPtr>    publishers_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    std::vector<rclcpp::TimerBase::SharedPtr>        timers_;
};

} // namespace lxros
