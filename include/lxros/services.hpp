#pragma once

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace lxros {

// ------------------------
// ServiceClient
// ------------------------
template<class Srv>
class ServiceClient {
public:
    using ClientPtr = typename rclcpp::Client<Srv>::SharedPtr;
    using Request   = typename Srv::Request;
    using Response  = typename Srv::Response;

    ServiceClient() = default;

    explicit ServiceClient(const std::shared_ptr<rclcpp::Node> & node,
                           const ClientPtr & client)
    : node_(node), client_(client)
    {}

    bool valid() const
    {
        return !node_.expired() && !client_.expired();
    }

    bool service_is_ready() const
    {
        auto c = client_.lock();
        if (!c) return false;
        return c->service_is_ready();
    }

    bool wait_for_service(std::chrono::milliseconds timeout) const
    {
        auto c = client_.lock();
        if (!c) return false;
        return c->wait_for_service(timeout);
    }

    // Async request (returns SharedFuture)
    auto call_async(const Request & req)
    {
        auto c = client_.lock();
        if (!c) {
            throw std::runtime_error("ServiceClient: client expired");
        }
        auto req_ptr = std::make_shared<Request>(req);
        return c->async_send_request(req_ptr);
    }

    // Sync request (spins this node until completion)
    Response call(const Request & req,
                  std::chrono::milliseconds timeout = std::chrono::seconds(2)) const
    {
        auto n = node_.lock();
        auto c = client_.lock();
        if (!n || !c) {
            throw std::runtime_error("ServiceClient: node/client expired");
        }

        if (!c->wait_for_service(timeout)) {
            throw std::runtime_error("ServiceClient: service not available");
        }

        auto fut = c->async_send_request(std::make_shared<Request>(req));

        auto ret = rclcpp::spin_until_future_complete(n, fut, timeout);
        if (ret != rclcpp::FutureReturnCode::SUCCESS) {
            throw std::runtime_error("ServiceClient: call failed or timed out");
        }

        return *(fut.get());
    }

    // Escape hatch
    ClientPtr raw() const
    {
        return client_.lock();
    }

private:
    std::weak_ptr<rclcpp::Node> node_;
    std::weak_ptr<rclcpp::Client<Srv>> client_;
};

// ------------------------
// ServiceServer
// ------------------------
template<class Srv>
class ServiceServer {
public:
    using ServicePtr = typename rclcpp::Service<Srv>::SharedPtr;

    ServiceServer() = default;

    explicit ServiceServer(const ServicePtr & srv)
    : srv_(srv)
    {}

    bool valid() const
    {
        return !srv_.expired();
    }

    ServicePtr raw() const
    {
        return srv_.lock();
    }

private:
    std::weak_ptr<rclcpp::Service<Srv>> srv_;
};

} // namespace lxros
