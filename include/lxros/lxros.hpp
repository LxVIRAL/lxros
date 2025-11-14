#pragma once

#include "lxros/lx_node.hpp"
#include "lxros/detail/lx_context.hpp"

#include <chrono>

namespace lxros {

inline void init(int & argc, char ** argv)
{
    detail::LxContext::instance().init(argc, argv);
}

inline void run()
{
    detail::LxContext::instance().spin();
}

inline void spin_some()
{
    detail::LxContext::instance().spin_some();
}

inline void spin_for(std::chrono::nanoseconds dt)
{
    detail::LxContext::instance().spin_for(dt);
}

} // namespace lxros
