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

#include "skratch_gazebo/KeloDrive.hpp"

KeloDrive::KeloDrive(rclcpp::Node::SharedPtr nh, std::string name, double xPos,
                     double yPos, double zPos, double pivotOrientation)
    : _name(name), _xPos(xPos), _yPos(yPos), _zPos(zPos),
      _pivotOrientation(pivotOrientation) {

  std::string ctrl_topic = "/skratch_base_controller/commands";
  hubWheelPub =
      nh->create_publisher<std_msgs::msg::Float64MultiArray>(ctrl_topic, 1);
}

void KeloDrive::getPos(double &xPos, double &yPos, double &zPos) const {
  xPos = _xPos;
  yPos = _yPos;
  zPos = _zPos;
}

void KeloDrive::setPivotOrientation(double orientation) {
  _pivotOrientation = orientation;
}

visualization_msgs::msg::Marker KeloDrive::getPivotMarker() const {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = rclcpp::Clock().now();
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  getPos(marker.pose.position.x, marker.pose.position.y,
         marker.pose.position.z);
  tf2::Quaternion quat;
  quat.setRPY(0, 0, _pivotOrientation);
  quat.normalize();
  marker.pose.orientation = tf2::toMsg(quat);
  marker.scale.x = 0.25;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  return marker;
}

void KeloDrive::setHubWheelVelocities(const std::vector<double> &velocities) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = velocities;
  hubWheelPub->publish(msg);
}
