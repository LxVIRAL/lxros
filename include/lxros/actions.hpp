#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "lxros/detail/lx_context.hpp"

namespace lxros {
namespace detail {

// Helper for static_assert in dependent contexts
template<typename T>
struct dependent_false : std::false_type {};

} // namespace detail

// -----------------------------------------------------------------------------
// ActionOutcome: (ResultCode, Result) pair
// -----------------------------------------------------------------------------

template<class ActionT>
struct ActionOutcome {
  using Result = typename ActionT::Result;
  rclcpp_action::ResultCode code;
  Result                    result;

  static ActionOutcome succeeded(const Result & r) {
    return {rclcpp_action::ResultCode::SUCCEEDED, r};
  }
  static ActionOutcome aborted(const Result & r) {
    return {rclcpp_action::ResultCode::ABORTED, r};
  }
  static ActionOutcome canceled(const Result & r) {
    return {rclcpp_action::ResultCode::CANCELED, r};
  }
};

// Forward declaration
template<class ActionT> class ActionClient;

// -----------------------------------------------------------------------------
// ActionGoalHandle
// -----------------------------------------------------------------------------

template<class ActionT>
class ActionGoalHandle {
public:
  using Result        = typename ActionT::Result;
  using GoalHandle    = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using ResultCode    = rclcpp_action::ResultCode;

  ActionGoalHandle() = default;

  bool valid() const noexcept { return static_cast<bool>(impl_); }

  bool is_done() const noexcept {
    return impl_ && impl_->done.load(std::memory_order_acquire);
  }

  Result wait_for_result(
      std::optional<std::chrono::nanoseconds> timeout = std::nullopt) const
  {
    if (!impl_) {
      throw std::runtime_error("ActionGoalHandle: invalid handle");
    }

    using namespace std::chrono;
    auto & ctx   = detail::LxContext::instance();
    const auto t0 = steady_clock::now();

    while (rclcpp::ok()) {
      if (impl_->done.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        return impl_->result;  // copy
      }

      if (timeout) {
        const auto elapsed = steady_clock::now() - t0;
        if (elapsed >= *timeout) {
          throw std::runtime_error("wait_for_result() timed out");
        }
        const auto remaining = *timeout - elapsed;
        const auto dt = std::min(remaining, nanoseconds(10'000'000)); // 10 ms
        ctx.spin_for(dt);
      } else {
        ctx.spin_for(std::chrono::milliseconds(10));
      }
    }

    throw std::runtime_error("wait_for_result() interrupted (rclcpp shutdown)");
  }

  ResultCode status() const
  {
    if (!impl_) {
      throw std::runtime_error("ActionGoalHandle: invalid handle");
    }
    return impl_->code.load(std::memory_order_acquire);
  }

private:
  struct Impl {
    std::atomic<bool>       done{false};
    std::atomic<ResultCode> code{ResultCode::UNKNOWN};
    mutable std::mutex      mutex;
    Result                  result{};
  };

  explicit ActionGoalHandle(std::shared_ptr<Impl> impl)
  : impl_(std::move(impl))
  {}

  std::shared_ptr<Impl> impl_;

  friend class ActionClient<ActionT>;
};

// -----------------------------------------------------------------------------
// ActionClient wrapper
// -----------------------------------------------------------------------------

template<class ActionT>
class ActionClient {
public:
  using Goal          = typename ActionT::Goal;
  using Feedback      = typename ActionT::Feedback;
  using Result        = typename ActionT::Result;
  using Client        = rclcpp_action::Client<ActionT>;
  using ClientPtr     = std::shared_ptr<Client>;
  using WeakClient    = std::weak_ptr<Client>;
  using GoalHandle    = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  using FeedbackCallback = std::function<void(const Feedback &)>;
  using ResultCallback   = std::function<void(const Result &)>;

  ActionClient() = default;

  explicit ActionClient(const ClientPtr & client)
  : weak_client_(client)
  {}

  explicit ActionClient(const WeakClient & weak)
  : weak_client_(weak)
  {}

  explicit operator bool() const noexcept {
    return !weak_client_.expired();
  }

  ClientPtr raw() const {
    return weak_client_.lock();
  }

  bool wait_for_server(std::chrono::nanoseconds timeout) const
  {
    auto client = weak_client_.lock();
    if (!client) return false;
    return client->wait_for_action_server(timeout);
  }

  ActionGoalHandle<ActionT> send_goal(
      const Goal & goal,
      FeedbackCallback feedback_cb = {},
      ResultCallback   result_cb   = {})
  {
    auto client = weak_client_.lock();
    if (!client) {
      throw std::runtime_error(
          "ActionClient::send_goal: underlying client expired");
    }

    using Impl = typename ActionGoalHandle<ActionT>::Impl;
    auto impl  = std::make_shared<Impl>();
    ActionGoalHandle<ActionT> handle{impl};

    typename Client::SendGoalOptions options;

    if (feedback_cb) {
      options.feedback_callback =
          [feedback_cb](typename Client::GoalHandle::SharedPtr,
                        const std::shared_ptr<const Feedback> feedback) {
            feedback_cb(*feedback);
          };
    }

    options.result_callback =
        [impl, result_cb](const WrappedResult & wrapped) {
          {
            std::lock_guard<std::mutex> lock(impl->mutex);
            impl->result = wrapped.result;
            impl->code.store(wrapped.code, std::memory_order_release);
          }
          impl->done.store(true, std::memory_order_release);

          if (result_cb) {
            result_cb(wrapped.result);
          }
        };

    client->async_send_goal(goal, options);
    return handle;
  }

private:
  WeakClient weak_client_;
};

// -----------------------------------------------------------------------------
// ActionServer wrapper
// -----------------------------------------------------------------------------

template<class ActionT>
class ActionServer {
public:
  using Server     = rclcpp_action::Server<ActionT>;
  using ServerPtr  = std::shared_ptr<Server>;
  using WeakServer = std::weak_ptr<Server>;

  ActionServer() = default;

  explicit ActionServer(const ServerPtr & server)
  : weak_server_(server)
  {}

  explicit ActionServer(const WeakServer & weak)
  : weak_server_(weak)
  {}

  explicit operator bool() const noexcept {
    return !weak_server_.expired();
  }

  ServerPtr raw() const {
    return weak_server_.lock();
  }

private:
  WeakServer weak_server_;
};

// -----------------------------------------------------------------------------
// detail::make_action_server helpers
//   Heavy implementation lives here; LxNode just forwards.
// -----------------------------------------------------------------------------

namespace detail {

template<class ActionT, class Handler>
typename rclcpp_action::Server<ActionT>::SharedPtr
make_action_server(const std::shared_ptr<rclcpp::Node> & node,
                   const std::string & name,
                   Handler && handler)
{
  using Goal = typename ActionT::Goal;

  auto default_goal_cb = [](const Goal &) { return true; };
  auto default_cancel_cb = [](const Goal &) { return true; };

  return make_action_server<ActionT>(
      node,
      name,
      std::forward<Handler>(handler),
      default_goal_cb,
      default_cancel_cb);
}

template<class ActionT, class Handler, class GoalCb, class CancelCb>
typename rclcpp_action::Server<ActionT>::SharedPtr
make_action_server(const std::shared_ptr<rclcpp::Node> & node,
                   const std::string & name,
                   Handler && handler,
                   GoalCb && goal_cb,
                   CancelCb && cancel_cb)
{
  using Server     = rclcpp_action::Server<ActionT>;
  using ServerPtr  = typename Server::SharedPtr;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using Goal       = typename ActionT::Goal;
  using Feedback   = typename ActionT::Feedback;
  using Result     = typename ActionT::Result;
  using Outcome    = ActionOutcome<ActionT>;

  // Wrap goal_cb(const Goal&) -> bool or GoalResponse
  auto goal_wrap =
      [gcb = std::forward<GoalCb>(goal_cb)]
      (const rclcpp_action::GoalUUID &,
       std::shared_ptr<const Goal> goal_msg)
       -> rclcpp_action::GoalResponse
  {
    using Ret = std::invoke_result_t<GoalCb, const Goal &>;
    if constexpr (std::is_same_v<Ret, bool>) {
      return gcb(*goal_msg)
          ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
          : rclcpp_action::GoalResponse::REJECT;
    } else if constexpr (std::is_same_v<Ret, rclcpp_action::GoalResponse>) {
      return gcb(*goal_msg);
    } else {
      static_assert(dependent_false<Ret>::value,
                    "Goal callback must return bool or GoalResponse");
    }
  };

  // Wrap cancel_cb(const Goal&) -> bool or CancelResponse
  auto cancel_wrap =
      [ccb = std::forward<CancelCb>(cancel_cb)]
      (const std::shared_ptr<GoalHandle> goal_handle)
       -> rclcpp_action::CancelResponse
  {
    const Goal & goal = *goal_handle->get_goal();
    using Ret = std::invoke_result_t<CancelCb, const Goal &>;
    if constexpr (std::is_same_v<Ret, bool>) {
      return ccb(goal)
          ? rclcpp_action::CancelResponse::ACCEPT
          : rclcpp_action::CancelResponse::REJECT;
    } else if constexpr (std::is_same_v<Ret, rclcpp_action::CancelResponse>) {
      return ccb(goal);
    } else {
      static_assert(dependent_false<Ret>::value,
                    "Cancel callback must return bool or CancelResponse");
    }
  };

    /// Execute wrapper: call handler(goal, publish_feedback, is_cancel)
    auto execute_wrap =
        [h = std::forward<Handler>(handler)]
        (const std::shared_ptr<GoalHandle> goal_handle) mutable
    {
        const Goal & goal = *goal_handle->get_goal();

        auto publish_feedback =
            [goal_handle](const Feedback & fb) {
            auto fb_ptr = std::make_shared<Feedback>(fb);
            goal_handle->publish_feedback(fb_ptr);
            };

        auto is_cancel_requested =
            [goal_handle]() {
            return goal_handle->is_canceling();
            };

        using Ret = std::invoke_result_t<
            Handler,
            const Goal &,
            decltype(publish_feedback),
            decltype(is_cancel_requested)>;

        if constexpr (std::is_same_v<Ret, Result>) {
        Result res = h(goal, publish_feedback, is_cancel_requested);
        auto res_ptr = std::make_shared<Result>(res);
        goal_handle->succeed(res_ptr);
        } else if constexpr (std::is_same_v<Ret, Outcome>) {
        Outcome out = h(goal, publish_feedback, is_cancel_requested);
        auto res_ptr = std::make_shared<Result>(out.result);
        switch (out.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            goal_handle->succeed(res_ptr);
            break;
            case rclcpp_action::ResultCode::ABORTED:
            goal_handle->abort(res_ptr);
            break;
            case rclcpp_action::ResultCode::CANCELED:
            goal_handle->canceled(res_ptr);
            break;
            default:
            // treat UNKNOWN as aborted
            goal_handle->abort(res_ptr);
            break;
        }
        } else {
        static_assert(detail::dependent_false<Ret>::value,
                        "Handler must return Result or ActionOutcome<ActionT>");
        }
    };

    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    auto handle_accepted_wrap =
        [exec = std::move(execute_wrap)]
        (const std::shared_ptr<GoalHandle> goal_handle) mutable
    {
        std::thread{[exec, goal_handle]() mutable {
            exec(goal_handle);   // run your execute logic on a separate thread
        }}.detach();
    };

    auto server = rclcpp_action::create_server<ActionT>(
        node,
        name,
        goal_wrap,
        cancel_wrap,
        handle_accepted_wrap);   // ✅ per-goal thread

  return server;
}

} // namespace detail

} // namespace lxros


