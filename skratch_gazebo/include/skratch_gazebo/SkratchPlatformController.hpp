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

#ifndef SKRATCH_PLATFORM_CONTROLLER_H
#define SKRATCH_PLATFORM_CONTROLLER_H

#include "kelo_tulip/VelocityPlatformController.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "skratch_gazebo/KeloDrive.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

class SkratchPlatformController {
public:
  /**
   * @brief Construct a new Robile Platform Controller object
   *
   * @param nh Node handle for the ros node managing the platform controller
   */
  SkratchPlatformController(rclcpp::Node::SharedPtr nh);

  /**
   * @brief Set the linear and angular velocities for the ROBILE platform
   *
   * @param vx Linear velocity (in m/s) along the positive x axis in robot
   * frame
   * @param vy Linear velocity (in m/s) along the positive y axis in robot
   * frame
   * @param va Angular velocity in (rad/s) around the positive z axis in robot
   * frame
   */
  void setCmdVel(double vx, double vy, double va);

  /**
   * @brief Set the maximum linear and angular velocities that the ROBILE
   * platform could achieve. Any commanded velocities above these values will
   * be clipped.
   *
   * @param linearVel Max linear velocity the ROBILE platform could achieve
   * @param angularVel Max angular velocity the ROBILE platform could achieve
   */
  void setMaxPlatformVelocity(double linearVel, double angularVel);
  /**
   * @brief Controller step which computes and sets the desired hub wheel
   * velocities to each hub wheel based on the commanded platform velocity
   *
   */
  void step();

  /**
   * @brief Publish RViz markers for each active wheel's pivot pose
   *
   */
  void publishPivotMarkers() const;

  /**
   * @brief Set the frequency at which odometry messages are published. The
   * frequency must be less that or equal to the controller's frequency.
   *
   * @param frequency Frequency (in Hz) at which odometry messages should be
   * published
   */
  void setOdomFrequency(double frequency);

  /**
   * @brief Publish the latest odometry received from Gazebo on '/odom' topic
   *
   */
  void publishOdom();

  /**
   * @brief Publish the latest transfrom from odom to base_link on the '/tf'
   * topic
   *
   */
  void publishOdomToBaseLinkTF();

protected:
  /**
   * @brief Callback function to receive and process the robot joint states
   * from Gazebo.
   *
   * The first message is used to get the positions of all Kelo drives
   * attached to the platform and initialize the Velocity controller from
   * kelo_tulip. The next messages are used to store the latest pivot
   * orientation of each Kelo drive.
   *
   * @param msg Ros message from Gazebo with the Joint state information
   */
  void jointStatesCallBack(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief Get the current pose of the ROBILE platform from the TF message
   *
   * @param msg The TF message containing the transforms
   * @return The current pose of the ROBILE platform
   */
  geometry_msgs::msg::Pose
  getSkratchPose(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  /**
   * @brief Callback function to receive and process the robot link states
   * from Gazebo
   *
   * The base_link pose and twist published by Gazebo is stored and used to
   * construct the odometry message.
   *
   * @param msg Ros message from Gazebo with the Link state information
   */
  void gazeboLinkStatesCallBack(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  /**
   * @brief Intialize the Kelo drive data structures and the Velocity
   * controller from kelo_tulip
   *
   * @param pivotJointData Map of Kelo drive name vs its latest pivot
   * orientation
   */
  void initDrives(const std::map<std::string, double> &pivotJointData);

  /**
   * @brief Set the Pivot Orientations for every Kelo drive in the ROBILE
   * platform
   *
   * @param pivotJointData Map of Kelo drive name vs the latest pivot
   * orientation
   */
  void
  setPivotOrientations(const std::map<std::string, double> &pivotJointData);

  /**
   * @brief Extract the ROBILE brick name from the pivot joint name
   *
   * @param jointName Joint name of the Kelo drive pivot
   * @return std::string Name of the ROBILE brick
   */
  std::string getRobileBrickName(const std::string &jointName);

  /**
   * @brief Node handle for the ros node managing the platform controller
   *
   */
  rclcpp::Node::SharedPtr _nh;

  /**
   * @brief A store for kelo drives attached to the ROBILE platform.
   * The key for the store is the name of the kelo drive and the value is the
   * kelo drive object
   *
   */
  std::map<std::string, KeloDrive> _drives;

  /**
   * @brief Flag to indicate if the platform has been successfully initialized
   * and is ready for operation
   *
   */
  bool _initialized;

  /**
   * @brief A shared pointer to the tf2 buffer used to store and manage
   * transforms
   *
   */
  std::shared_ptr<tf2_ros::Buffer> _tfBuffer;

  /**
   * @brief A shared pointer to the tf2 transform listener used to listen
   * for transforms from Gazebo
   *
   */
  std::shared_ptr<tf2_ros::TransformListener> _tfListener;

  /**
   * @brief A shared pointer to the tf2 transform broadcaster used to publish
   * transforms to ROS
   *
   */
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tfBroadcaster;

  /**
   * @brief A ROS subscriber to receive Joint state information of all
   * robot joints
   *
   */
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _jointStatesSub;

  /**
   * @brief A ROS subscriber to receive Link state information of all
   * robot links from Gazebo
   *
   */
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr _linkStatesSub;

  /**
   * @brief A ROS publisher to publish the platform odometry information
   *
   */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odomPublisher;

  /**
   * @brief A ROS publisher to publish the transform from odom to base_link
   * frames
   *
   */
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr _odomTFPublisher;

  /**
   * @brief A ROS publisher to publish RViz markers representing the pivot
   * poses of every kelo drive attached to the platform
   *
   */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      _pivotMarkersPublisher;

  /**
   * @brief The desired linear platform velocity along the x-axis of base_link
   *
   */
  double _cmdVelX;

  /**
   * @brief The desired linear platform velocity along the y-axis of base_link
   *
   */
  double _cmdVelY;

  /**
   * @brief The desired angular platform velocity around the z-axis of
   * base_link
   *
   */
  double _cmdVelA;

  /**
   * @brief A ROS odometry message with the latest odometry information that
   * can be published
   *
   */
  nav_msgs::msg::Odometry _odomMsg;

  /**
   * @brief Minimum time duration between two consecutive odometry messages
   * being published. This is set based on the desired odom publish frequency
   *
   */
  rclcpp::Duration _odomDuration;

  /**
   * @brief Time at which the last odom message was published
   *
   */
  rclcpp::Time _lastOdomPublishTime;

  /**
   * @brief Velocity platform controller to convert platform velocity to each
   * individual hub wheel velocities
   *
   */
  kelo::VelocityPlatformController _controller;

  /**
   * @brief Store of Wheel configuration required by the Velocity platform
   * controller
   *
   */
  std::map<std::string, kelo::WheelConfig> _wheelConfigs;
};
#endif // ROBILE_PLATFORM_CONTROLLER_H