/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
 *  
 *  Created by: a. whit. (nml@whit.contact)
 *  
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Functionality related to receiving ROS2 messages.


// Import node header.
#include "node.hpp"

// Import the Force Dimension haptics header.
#include "dhdc.h"

// Import package headers.
#include "messages.hpp"
#include "topics.hpp"
#include "qos.hpp"
#include <iostream>

// Scope namespace elements.
using force_dimension::Node;

// Subscribes to ROS messages that indicate an instantaneous force to be 
// applied by the robot.
void Node::SubscribeForceAndTorqueAndGripperForce(void) {
  auto force_callback = [this](const ForceMessage::SharedPtr force) { this->force_callback(force); };
  auto torque_callback = [this](const TorqueMessage::SharedPtr torque) { this->torque_callback(torque); };
  auto gripper_force_callback = [this](const GripperForceMessage::SharedPtr gripper_force) { this->gripper_force_callback(gripper_force); };
  auto force_topic = FORCE_COMMAND_TOPIC;
  auto torque_topic = TORQUE_COMMAND_TOPIC;
  auto gripper_force_topic = GRIPPER_FORCE_COMMAND_TOPIC;
  auto qos = DefaultQoS();
  force_subscription_ = this->create_subscription<ForceMessage>(force_topic, qos, force_callback);
  torque_subscription_ = this->create_subscription<TorqueMessage>(torque_topic, qos, torque_callback);
  gripper_force_subscription_ = this->create_subscription<GripperForceMessage>(gripper_force_topic, qos, gripper_force_callback);
  robot_command_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Node::set_force_and_torque_and_gripper_force, this));
}

// The force callback
void Node::force_callback(const ForceMessage::SharedPtr force_msg)
{
  last_force_msg_ = force_msg;
}

// The torque callback
void Node::torque_callback(const TorqueMessage::SharedPtr torque_msg)
{
  last_torque_msg_ = torque_msg;
}

// The gripper force callback
void Node::gripper_force_callback(const GripperForceMessage::SharedPtr gripper_force_msg)
{
  last_gripper_force_msg_ = gripper_force_msg;
}


// Applies a force to the robotic manipulandum
void Node::set_force_and_torque_and_gripper_force(void) {

    double force_x = 0.0, force_y = 0.0, force_z = 0.0;
    double torque_x = 0.0, torque_y = 0.0, torque_z = 0.0;
    double gripper_force = 0.0;

    if (last_force_msg_ != nullptr) {
        force_x = last_force_msg_->x;
        force_y = last_force_msg_->y;
        force_z = last_force_msg_->z;
    }
    if (last_torque_msg_ != nullptr) {
        torque_x = last_torque_msg_->x;
        torque_y = last_torque_msg_->y;
        torque_z = last_torque_msg_->z;
    }
    if (last_gripper_force_msg_ != nullptr) {
        gripper_force = last_gripper_force_msg_->data;
    }
    

    RCLCPP_INFO(this->get_logger(), "Commanding Robot: Force(%.2f, %.2f, %.2f), Torque(%.2f, %.2f, %.2f), Gripper %.2f",
                force_x, force_y, force_z, torque_x, torque_y, torque_z, gripper_force);

    auto result = hardware_disabled_
                  ? 0
                  : dhdSetForceAndTorqueAndGripperForce(force_x, force_y, force_z, torque_x, torque_y, torque_z, gripper_force);

    if ((result != 0) && (result != DHD_MOTOR_SATURATED)) {
        std::string error_message = "Cannot set force: ";
        error_message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
        Log(error_message);
        on_error();
    }
}
