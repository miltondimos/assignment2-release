// Copyright 2019 Zhihao Zhang License MIT

#include "config_parser.hpp"

#include <iostream>
#include <string>

namespace assignment2
{
ConfigReader::ConfigReader(std::istream & config_file)
{
    // TODO(STUDENT): CODE HERE
}

auto ConfigReader::find_config(std::string const & key,
    std::string const & default_value) const -> std::string
{
    // TODO(STUDENT): CODE HERE
    return std::string{};
}

ConfigParser::ConfigParser(ConfigReader const & config)
    : zid_{config.find_config("zid", std::string{"z0000000"})}
    // TODO(STUDENT): CODE HERE
    , joy_config_{}
    , kinematic_config_{}
    , refresh_period_{}
{
    // TODO(STUDENT): CODE HERE
}

auto ConfigParser::get_zid() const -> std::string { return zid_; }

auto ConfigParser::get_refresh_period() const -> std::chrono::milliseconds
{
    return refresh_period_;
}

auto ConfigParser::get_joystick_config() const -> JoystickConfig
{
    return joy_config_;
}

auto ConfigParser::get_kinematic_config() const -> KinematicLimits
{
    return kinematic_config_;
}
} // namespace assignment2
