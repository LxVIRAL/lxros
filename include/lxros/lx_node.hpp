#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "lxros/detail/lx_context.hpp"
#include "topics.hpp"
#include "actions.hpp"   
#include "services.hpp"

namespace lxros {
namespace detail {

// Implemented in detail/lx_context.hpp
void register_node(const std::shared_ptr<rclcpp::Node> & node);

} // namespace detail


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
    
    template<class ActionT>
    ActionClient<ActionT> action_client(const std::string & name)
    {
        auto client = rclcpp_action::create_client<ActionT>(node_, name);

        action_clients_.push_back(
            std::static_pointer_cast<rclcpp_action::ClientBase>(client));

        return ActionClient<ActionT>(client);
    }

    template<class ActionT, class Handler>
    ActionServer<ActionT>
    action_server(const std::string & name, Handler && handler)
    {
        auto server = detail::make_action_server<ActionT>(
            node_, name, std::forward<Handler>(handler));

        action_servers_.push_back(
            std::static_pointer_cast<rclcpp_action::ServerBase>(server));

        return ActionServer<ActionT>(server);
    }

    template<class ActionT, class Handler, class GoalCb, class CancelCb>
    ActionServer<ActionT>
    action_server(const std::string & name,
                Handler && handler,
                GoalCb && goal_cb,
                CancelCb && cancel_cb)
    {
        auto server = detail::make_action_server<ActionT>(
            node_, name,
            std::forward<Handler>(handler),
            std::forward<GoalCb>(goal_cb),
            std::forward<CancelCb>(cancel_cb));

        action_servers_.push_back(
            std::static_pointer_cast<rclcpp_action::ServerBase>(server));

        return ActionServer<ActionT>(server);
    }

    template<typename T>
    T get_param(const std::string & name, const T & default_value) const
    {
        T value;
        if (node_->get_parameter(name, value))
            return value;
        else
            return default_value;
    }

    // ------------------------
    // Service client helper
    // ------------------------
    template<class Srv>
    ServiceClient<Srv> service_client(const std::string & name)
    {
        auto client = node_->create_client<Srv>(name);

        service_clients_.push_back(
            std::static_pointer_cast<rclcpp::ClientBase>(client));

        return ServiceClient<Srv>(node_, client);
    }

    // ------------------------
    // Service server helper – generic callable
    //   Handler: void(const Req&, Res&)
    // ------------------------
    template<class Srv, class Handler>
    ServiceServer<Srv> service_server(const std::string & name, Handler && handler)
    {
        using Req = typename Srv::Request;
        using Res = typename Srv::Response;

        auto wrapper_cb =
            [fn = std::forward<Handler>(handler)](const std::shared_ptr<Req> req,
                                                  std::shared_ptr<Res> res) {
                fn(*req, *res);
            };

        auto srv = node_->create_service<Srv>(name, wrapper_cb);

        service_servers_.push_back(
            std::static_pointer_cast<rclcpp::ServiceBase>(srv));

        return ServiceServer<Srv>(srv);
    }

    // ------------------------
    // Service server helper – member function
    //   memfn: void(T::*)(const Req&, Res&)
    //   obj:   T*
    // ------------------------
    template<class Srv, class T>
    ServiceServer<Srv> service_server(const std::string & name,
                                      void (T::*memfn)(const typename Srv::Request &,
                                                       typename Srv::Response &),
                                      T * obj)
    {
        auto wrapper =
            [obj, memfn](const typename Srv::Request & req,
                         typename Srv::Response & res) {
                (obj->*memfn)(req, res);
            };

        return service_server<Srv>(name, std::move(wrapper));
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
    std::vector<rclcpp_action::ClientBase::SharedPtr> action_clients_;
    std::vector<rclcpp_action::ServerBase::SharedPtr> action_servers_;
    std::vector<rclcpp::ClientBase::SharedPtr>  service_clients_;
    std::vector<rclcpp::ServiceBase::SharedPtr> service_servers_;

};

} // namespace lxros
