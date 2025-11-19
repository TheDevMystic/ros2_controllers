// Copyright (c) 2022 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JOINT_TRAJECTORY_CONTROLLER__INTERPOLATION_METHODS_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__INTERPOLATION_METHODS_HPP_

#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

namespace joint_trajectory_controller
{

static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("joint_trajectory_controller.interpolation_methods");

namespace interpolation_methods
{

/**
 * @brief Enum class for interpolation methods.
 * Enum class to define multiple interpolation methods between two points.
 */
enum class InterpolationMethod
{
  NONE,  ///< No interpolation; each point is executed exactly as provided.
  LINEAR,  ///< Straight-line interpolation with constant velocity and zero acceleration.
  VARIABLE_DEGREE_SPLINE  ///< Smooth spline interpolation with continuous velocities and accelerations.
};

/// Default interpolation method used when no parameter is specified.
const InterpolationMethod DEFAULT_INTERPOLATION = InterpolationMethod::VARIABLE_DEGREE_SPLINE;

/// Maps interpolation method enums to their corresponding parameter strings.
const std::unordered_map<InterpolationMethod, std::string> InterpolationMethodMap({
  {InterpolationMethod::NONE, "none"}, 
  {InterpolationMethod::LINEAR, "linear"},
  {InterpolationMethod::VARIABLE_DEGREE_SPLINE, "splines"}
});


/**
 * @brief Converts a string into the corresponding interpolation method.
 * @returns Corresponding InterpolationMethod.
 * @note When no parameter is matched (i.e., unknown parameter) defaults to VARIABLE_DEGREE_SPLINE.
 */
[[nodiscard]] inline InterpolationMethod from_string(const std::string & interpolation_method)
{
  if (interpolation_method == InterpolationMethodMap.at(InterpolationMethod::NONE))
  {
    return InterpolationMethod::NONE;
  }
  else if (interpolation_method == InterpolationMethodMap.at(InterpolationMethod::LINEAR))
  {
    return InterpolationMethod::LINEAR;
  }
  else if (interpolation_method == InterpolationMethodMap.at(InterpolationMethod::VARIABLE_DEGREE_SPLINE))
  {
    return InterpolationMethod::VARIABLE_DEGREE_SPLINE;
  }
  // Default
  else
  {
    RCLCPP_INFO(
      LOGGER,
      "Unknown interpolation method was given. Using default: VARIABLE_DEGREE_SPLINE.");
    return InterpolationMethod::VARIABLE_DEGREE_SPLINE;
  }
}
}  // namespace interpolation_methods
}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__INTERPOLATION_METHODS_HPP_
