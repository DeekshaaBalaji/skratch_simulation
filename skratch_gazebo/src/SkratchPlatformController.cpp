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

#include "skratch_gazebo/SkratchPlatformController.hpp"

SkratchPlatformController::SkratchPlatformController(rclcpp::Node::SharedPtr nh)
    : _nh(nh), _drives(), _initialized(false),
      _tfBuffer(std::make_shared<tf2_ros::Buffer>(nh->get_clock())),
      _tfListener(std::make_shared<tf2_ros::TransformListener>(*_tfBuffer)),
      _cmdVelX(0.0), _cmdVelY(0.0), _cmdVelA(0.0),
      _odomDuration(rclcpp::Duration::from_seconds(1.0)) {
  _jointStatesSub = nh->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1,
      std::bind(&SkratchPlatformController::jointStatesCallBack, this,
                std::placeholders::_1));
  _linkStatesSub = nh->create_subscription<tf2_msgs::msg::TFMessage>(
      "/link_states", 1,
      std::bind(&SkratchPlatformController::gazeboLinkStatesCallBack, this,
                std::placeholders::_1));
  _odomPublisher = nh->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
  _tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
  _pivotMarkersPublisher =
      nh->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/pivot_markers", 1);
  setOdomFrequency(10.0);
}

void SkratchPlatformController::setCmdVel(double vx, double vy, double va) {
  _cmdVelX = vx;
  _cmdVelY = vy;
  _cmdVelA = va;
}

void SkratchPlatformController::setMaxPlatformVelocity(double linearVel,
                                                       double angularVel) {
  _controller.setPlatformMaxLinVelocity(linearVel);
  _controller.setPlatformMaxAngVelocity(angularVel);
}

void SkratchPlatformController::publishPivotMarkers() const {
  if (_pivotMarkersPublisher->get_subscription_count() == 0) {
    visualization_msgs::msg::MarkerArray markerArray;
    int markerId = 0;
    for (const auto &drive : _drives) {
      visualization_msgs::msg::Marker marker = drive.second.getPivotMarker();
      marker.id = markerId++;
      markerArray.markers.push_back(marker);
    }
    _pivotMarkersPublisher->publish(markerArray);
  }
}

void SkratchPlatformController::setOdomFrequency(double frequency) {
  _odomDuration = rclcpp::Duration::from_seconds(1.0 / frequency);
}

void SkratchPlatformController::step() {
  if (_initialized) {
    _controller.setPlatformTargetVelocity(_cmdVelX, _cmdVelY, _cmdVelA);
    _controller.calculatePlatformRampedVelocities();
    std::vector<double> allWheelVelocities;

    for (const auto &drive : _drives) {
      const std::string &driveName = drive.first;
      int wheelNumber = _wheelConfigs[driveName].ethercatNumber;
      float setpoint1, setpoint2;
      _controller.calculateWheelTargetVelocity(
          wheelNumber, drive.second.getPivotOrientation(), setpoint1,
          setpoint2);
      allWheelVelocities.push_back(-setpoint1);
      allWheelVelocities.push_back(-setpoint2);
    }
    for (auto &drive : _drives) {
      drive.second.setHubWheelVelocities(allWheelVelocities);
    }
  }
}

void SkratchPlatformController::initDrives(
    const std::map<std::string, double> &pivotJointData) {
  if (!_drives.empty())
    return;

  rclcpp::Time now = rclcpp::Clock().now();
  for (auto &joint : pivotJointData) {
    std::string driveName = getRobileBrickName(joint.first);
    std::string pivotLink = driveName + std::string("_drive_pivot_link");

    RCLCPP_INFO(_nh->get_logger(), "Checking transform for: %s",
                pivotLink.c_str());

    RCLCPP_INFO(_nh->get_logger(), "Transform OK for drive: %s",
                driveName.c_str());
    if (!_tfBuffer->canTransform("base_link", pivotLink, rclcpp::Time(0),
                                 rclcpp::Duration(std::chrono::seconds(2)))) {
      RCLCPP_WARN(_nh->get_logger(),
                  "Did not receive transform from base_link to %s",
                  pivotLink.c_str());
      _drives.clear();
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform =
        _tfBuffer->lookupTransform("base_link", pivotLink, rclcpp::Time(0));

    _drives.insert(std::make_pair(
        driveName, KeloDrive(_nh, driveName, transform.transform.translation.x,
                             transform.transform.translation.y,
                             transform.transform.translation.z, joint.second)));
  }

  std::vector<kelo::WheelConfig> wheelConfigsVector;
  int wheelNumber = 0;
  double zDummy = 0;
  for (const auto &drive : _drives) {
    kelo::WheelConfig wc;
    wc.ethercatNumber = wheelNumber;
    drive.second.getPos(wc.x, wc.y, zDummy);
    wc.a = 0;
    _wheelConfigs[drive.first] = wc;
    wheelConfigsVector.push_back(wc);
    wheelNumber++;
  }

  _controller.initialise(wheelConfigsVector);
  _initialized = true;
  RCLCPP_INFO(_nh->get_logger(), "Initialized %lu Kelo drives", _drives.size());
}

void SkratchPlatformController::jointStatesCallBack(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  const std::vector<std::string> &jointNames = msg->name;
  const std::vector<double> &jointPositions = msg->position;

  if (jointNames.empty() || jointNames.size() != jointPositions.size()) {
    return;
  }

  std::map<std::string, double> pivotJointData;
  std::string pivotJointNameIdentifier = "drive_pivot_joint";

  for (size_t i = 0; i < jointNames.size(); ++i) {
    std::string jointName = jointNames[i];
    if (jointName.find(pivotJointNameIdentifier) != std::string::npos) {
      double pivotAngle = jointPositions[i] -
                          (int(jointPositions[i] / (2 * M_PI))) * (2 * M_PI);
      pivotJointData[jointName] = pivotAngle;
    }
  }
  if (_drives.empty()) {
    initDrives(pivotJointData);
  } else {
    setPivotOrientations(pivotJointData);
  }
}

geometry_msgs::msg::Pose SkratchPlatformController::getSkratchPose(
    const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  geometry_msgs::msg::Pose skratch_pose;
  for (const auto &transform : msg->transforms) {
    if (transform.child_frame_id == "skratch") {
      skratch_pose.position.x = transform.transform.translation.x;
      skratch_pose.position.y = transform.transform.translation.y;
      skratch_pose.position.z = transform.transform.translation.z;
      skratch_pose.orientation = transform.transform.rotation;
      break;
    }
  }
  return skratch_pose;
}

void SkratchPlatformController::gazeboLinkStatesCallBack(
    const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  static geometry_msgs::msg::Pose last_pose;
  static rclcpp::Time last_time;
  static bool first_call = true;

  geometry_msgs::msg::Pose current_pose = getSkratchPose(msg);
  if (current_pose.position.x == 0.0 && current_pose.position.y == 0.0 &&
      current_pose.position.z == 0.0) {
    return;
  }

  rclcpp::Time current_time = rclcpp::Clock().now();
  double dt = (current_time - last_time).seconds();

  _odomMsg.pose.pose = current_pose;

  if (!first_call && dt > 0.0) {
    double dx = current_pose.position.x - last_pose.position.x;
    double dy = current_pose.position.y - last_pose.position.y;

    double vx = dx / dt;
    double vy = dy / dt;

    double qw = current_pose.orientation.w;
    double qx = current_pose.orientation.x;
    double qy = current_pose.orientation.y;
    double qz = current_pose.orientation.z;
    double theta = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

    _odomMsg.twist.twist.linear.x = vx * cos(theta) + vy * sin(theta);
    _odomMsg.twist.twist.linear.y = vx * sin(theta) - vy * cos(theta);
    _odomMsg.twist.twist.angular.z = 0.0;
  } else {
    _odomMsg.twist.twist.linear.x = 0.0;
    _odomMsg.twist.twist.linear.y = 0.0;
    _odomMsg.twist.twist.angular.z = 0.0;
  }

  last_pose = current_pose;
  last_time = current_time;
  first_call = false;
}

void SkratchPlatformController::setPivotOrientations(
    const std::map<std::string, double> &pivotJointData) {
  for (const auto &joint : pivotJointData) {
    std::string driveName = getRobileBrickName(joint.first);
    if (_drives.find(driveName) == _drives.end()) {
      RCLCPP_ERROR(_nh->get_logger(),
                   "Cannot set pivot orientation for drive %s",
                   driveName.c_str());
      continue;
    }
    KeloDrive &drive = _drives.at(driveName);
    drive.setPivotOrientation(joint.second);
  }
}

std::string
SkratchPlatformController::getRobileBrickName(const std::string &jointName) {
  return jointName.substr(0, jointName.find("_drive_"));
}

void SkratchPlatformController::publishOdom() {
  rclcpp::Time now = rclcpp::Clock().now();
  if (_initialized && now - _lastOdomPublishTime >= _odomDuration) {
    _odomMsg.header.stamp = _nh->now();
    _odomMsg.header.frame_id = "odom";
    _odomMsg.child_frame_id = "base_link";
    _odomPublisher->publish(_odomMsg);
    _lastOdomPublishTime = now;
  }
}

void SkratchPlatformController::publishOdomToBaseLinkTF() {
  if (_initialized) {
    tf2_msgs::msg::TFMessage tfMsg;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.header.stamp = _nh->now();
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = _odomMsg.pose.pose.position.x;
    transform.transform.translation.y = _odomMsg.pose.pose.position.y;
    transform.transform.translation.z = _odomMsg.pose.pose.position.z;
    transform.transform.rotation = _odomMsg.pose.pose.orientation;
    tfMsg.transforms.push_back(transform);
    _tfBroadcaster->sendTransform(transform);
  }
}