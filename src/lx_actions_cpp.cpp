#include <lxros/lxros.hpp>
#include <example_interfaces/action/fibonacci.hpp>

using Action   = example_interfaces::action::Fibonacci;
using Goal     = Action::Goal;
using Feedback = Action::Feedback;
using Result   = Action::Result;


lxros::ActionOutcome<Action> execute_fibonacci(
    const Goal & goal,
    std::function<void(const Feedback &)> publish_feedback,
    std::function<bool()>                 is_cancel_requested)
{
    Feedback fb;
    Result   result;
    int a = 0;
    int b = 1;
    result.sequence.clear();
    result.sequence.push_back(a);
    for (int i = 1; i <= goal.order; ++i) {
        if (is_cancel_requested()) {            
            RCLCPP_WARN(rclcpp::get_logger("fib_server"),"Fibonacci goal canceled at order %d", i);              
            return lxros::ActionOutcome<Action>::canceled(result);                        
        }

        fb.sequence = result.sequence;
        publish_feedback(fb);

        int next = a + b;
        a = b;
        b = next;
        result.sequence.push_back(a);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));        
    }
    return lxros::ActionOutcome<Action>::succeeded(result);
}

bool goal_can_be_accepted(const Goal & goal)
{
    return goal.order > 0;
}

bool cancel_can_be_accepted(const Goal & /*goal*/)
{
    return true;
}

int main(int argc, char ** argv)
{
    lxros::init(argc, argv);

    auto node = lxros::LxNode("fib_server");

    auto server = node.action_server<Action>(
        "fibonacci",
        execute_fibonacci,
        goal_can_be_accepted,
        cancel_can_be_accepted);

    auto server2 = node.action_server<Action>(
        "fibonacci_simple",
        execute_fibonacci);

    lxros::run();
    return 0;
}
