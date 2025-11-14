// include/lxros/detail/lx_context.hpp
#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

namespace lxros {
namespace detail {

class LxContext {
public:
    static LxContext & instance()
    {
        static LxContext ctx;
        return ctx;
    }

    void ensure_init()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) {
            int argc = 0;
            char ** argv = nullptr;
            rclcpp::init(argc, argv);
            executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
            initialized_ = true;
        }
    }

    void register_node(const std::shared_ptr<rclcpp::Node> & node)
    {
        // assume ensure_init() was already called by LxNode ctor
        std::lock_guard<std::mutex> lock(mutex_);
        nodes_.push_back(node);
        executor_->add_node(node);
    }

    void spin()
    {
        ensure_init();
        executor_->spin();
    }

    void spin_some()
    {
        ensure_init();
        executor_->spin_some();
    }

    void spin_for(std::chrono::nanoseconds duration)
    {
        ensure_init();
        executor_->spin_some(duration);
    }

    void shutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) {
            executor_->cancel();
            executor_.reset();
            rclcpp::shutdown();
            initialized_ = false;
        }
    }

private:
    LxContext() = default;
    ~LxContext() { shutdown(); }

    LxContext(const LxContext &) = delete;
    LxContext & operator=(const LxContext &) = delete;

    bool initialized_ = false;
    std::mutex mutex_;
    std::unique_ptr<rclcpp::Executor> executor_;
    std::vector<std::weak_ptr<rclcpp::Node>> nodes_;
};

// free functions used by LxNode / lxros.hpp
inline void register_node(const std::shared_ptr<rclcpp::Node> & node)
{
    LxContext::instance().register_node(node);
}

inline void ensure_init()
{
    LxContext::instance().ensure_init();
}

} // namespace detail
} // namespace lxros
