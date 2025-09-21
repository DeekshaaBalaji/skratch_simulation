/******************************************************************************
 * Copyright (c) 2021
 * KELO Robotics GmbH
 *
 * Author:
 * Sushant Chavan
 * Walter Nowak
 *
 * Maintainer:
 * Chaitanya Gumudala
 *
 * Changes:
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "skratch_gazebo/SkratchPlatformController.hpp"

std::shared_ptr<SkratchPlatformController> platformController;

void cmdVelCallback(const geometry_msgs::msg::Twist &msg) {
  if (platformController) {
    platformController->setCmdVel(msg.linear.x, msg.linear.y, msg.angular.z);
  }
}

void joyCallback(const sensor_msgs::msg::Joy &joy) {
  if (platformController) {
    platformController->setCmdVel(joy.axes[1], joy.axes[0], joy.axes[3]);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_sim_time", true}});

  rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>(
      "skratch_gazebo_platform_controller", options);

  auto cmdVelSubscriber = nh->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, cmdVelCallback);

  auto joySubscriber =
      nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 1000, joyCallback);

  double platformMaxLinVel;
  if (!nh->has_parameter("platform_max_lin_vel")) {
    platformMaxLinVel = 1.0;
  } else {
    nh->get_parameter("platform_max_lin_vel", platformMaxLinVel);
  }

  double platformMaxAngVel;
  if (!nh->has_parameter("platform_max_ang_vel")) {
    platformMaxAngVel = 1.0;
  } else {
    nh->get_parameter("platform_max_ang_vel", platformMaxAngVel);
  }

  platformController = std::make_shared<SkratchPlatformController>(nh);
  platformController->setMaxPlatformVelocity(platformMaxLinVel,
                                             platformMaxAngVel);

  rclcpp::WallRate loopRate(100);

  while (rclcpp::ok()) {
    rclcpp::spin_some(nh);
    platformController->step();
    platformController->publishOdomToBaseLinkTF();
    platformController->publishOdom();
    platformController->publishPivotMarkers();

    loopRate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
