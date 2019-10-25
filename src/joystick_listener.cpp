// Copyright 2019 Zhihao Zhang License MIT

#include "joystick_listener.hpp"

#include "student_helper.hpp"

#include <memory>
#include <string>
#include <utility>

namespace assignment2
{
JoystickListener::JoystickListener(
    std::string const & zid, JoystickConfig config)
    // TODO(STUDENT): CODE HERE
    : rclcpp::Node{helper::joy_node_name(zid)}
    , config_{}
{
    // TODO(STUDENT): CODE HERE
}

// ReSharper disable once CppMemberFunctionMayBeConst
auto JoystickListener::joy_message_callback(
    sensor_msgs::msg::Joy::UniquePtr joy_message) -> void
{
    // TODO(STUDENT): CODE HERE
}
} // namespace assignment2
