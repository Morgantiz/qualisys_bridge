// Copyright 2023 Carlo Morganti <carloski@live.it>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef QUALISYS_BRIDGE
#define QUALISYS_BRIDGE
#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "qualisys_bridge/qualisys_bridge_core.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace qualisys_bridge
{

class QualisysBridgeNode : public rclcpp::Node
{
public:
  QualisysBridgeNode(std::string const & name) : rclcpp::Node(name)
  {
    // default parameters setttings
    this->declare_parameter("server_ip", "localhost");
    this->declare_parameter("server_port", 2222);
    this->declare_parameter("udp_port_", 6734);
    this->declare_parameter("qualisys_frame", "qualisys_map");
    this->declare_parameter("cycle_time", int(10));  // ms
    this->declare_parameter("publish_tf", true);
    this->declare_parameter("publish_pose", false);

    // gets user parameters setttings
    server_ip_ = this->get_parameter("server_ip").as_string();
    server_port_ = static_cast<uint16_t>(this->get_parameter("server_port").as_int());
    udp_port_ = static_cast<uint16_t>(this->get_parameter("udp_port_").as_int());
    qualisys_frame_ = this->get_parameter("qualisys_frame").as_string();
    cycle_time_ = this->get_parameter("cycle_time").as_int();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    publish_pose_ = this->get_parameter("publish_pose").as_bool();

    // clock
    clock_ = this->get_clock();

    // qualisys connection
    while (rclcpp::ok() && !connect()) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *clock_, 1000, "Trying to connect again ...");
    }
    // creates first objects
    update_active_6dof_objects();
    // activates frames streaming
    if (!protocol_.StreamFrames(
          CRTProtocol::RateAllFrames, 0, udp_port_, NULL, CRTProtocol::cComponent6d)) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *clock_, 1000, "Error on starting stream. Error: %s",
        protocol_.GetErrorString());
      return;
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), "Frames streaming activated");
    }

    // creates tf broadcaster, used in case of publish tf
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // allows to dynamically change parameters
    parameters_callback_ = this->add_on_set_parameters_callback(std::bind(
      &qualisys_bridge::QualisysBridgeNode::parameters_callback, this, std::placeholders::_1));
    // update cycle
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time_),
      std::bind(&qualisys_bridge::QualisysBridgeNode::update, this));
  }

  void update()
  {
    if (!connect()) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *clock_, 1000, "Update error, connection lost!");
      return;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Streaming frames ...");

    // publishes data
    CRTPacket::EPacketType data_type;
    if (protocol_.Receive(data_type, true) == CNetwork::ResponseType::success) {
      if (data_type == CRTPacket::PacketData) {
        auto time = get_clock()->now();
        float x, y, z;
        float rotation[9];
        // data
        CRTPacket * data = protocol_.GetRTPacket();
        for (unsigned int i = 0; i < data->Get6DOFBodyCount(); i++) {
          if (!data->Get6DOFBody(i, x, y, z, rotation)) {
            // computing
            const char * name = protocol_.Get6DOFBodyName(i);
            tf2::Quaternion quat;
            const tf2::Matrix3x3 matrix(
              rotation[0], rotation[3], rotation[6], rotation[1], rotation[4], rotation[7],
              rotation[2], rotation[5], rotation[8]);
            matrix.getRotation(quat);

            // tf
            if (publish_tf_) {
              geometry_msgs::msg::TransformStamped tf;
              tf.header.stamp = time;
              tf.header.frame_id = qualisys_frame_;
              tf.child_frame_id = name;
              tf.transform.translation.x = x;
              tf.transform.translation.y = y;
              tf.transform.translation.z = z;
              tf.transform.rotation.x = quat.x();
              tf.transform.rotation.y = quat.y();
              tf.transform.rotation.z = quat.z();
              tf.transform.rotation.w = quat.w();
              tf_broadcaster_->sendTransform(tf);
              RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 10000, "Publishing tf ...");
            }
            // pose
            if (publish_pose_) {
              geometry_msgs::msg::PoseStamped pose;
              pose.header.frame_id = qualisys_frame_;
              pose.pose.position.x = x;
              pose.pose.position.y = y;
              pose.pose.position.z = z;
              pose.pose.orientation.x = quat.x();
              pose.pose.orientation.y = quat.y();
              pose.pose.orientation.z = quat.z();
              pose.pose.orientation.w = quat.w();
              RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 10000, "Publishing pose ...");
            }

            if (!publish_tf_ && !publish_pose_) {
              RCLCPP_WARN_THROTTLE(
                this->get_logger(), *clock_, 10000,
                "No publishing. publish_pose and publish_tf "
                "params set to false!");
            }
          } else {
            RCLCPP_ERROR_THROTTLE(
              this->get_logger(), *clock_, 1000,
              "Error body number %i does not exist anymore. Error: %s", i,
              protocol_.GetErrorString());
          }
        }
      } else {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *clock_, 1000, "Data type is not of type PacketData. Error: %s",
          protocol_.GetErrorString());
      }
    } else {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *clock_, 1000, "Error on receiving data. Error: %s",
        protocol_.GetErrorString());
    }
    return;
  }

  ~QualisysBridgeNode()
  {
    protocol_.StopCapture();
    protocol_.Disconnect();
    return;
  }

protected:
  bool connect()
  {
    if (!protocol_.Connected()) {
      if (!protocol_.Connect(server_ip_.c_str(), server_port_, &udp_port_, 1, 19, false)) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *clock_, 1000,
          "Error on connection to server %s, server port %i, udp port %i .... Error: %s",
          server_ip_.c_str(), server_port_, udp_port_, protocol_.GetErrorString());
        return false;
      }
    }
    if (!protocol_.Read6DOFSettings(read_settings_)) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *clock_, 1000, "Error while reading data ... Error: %s",
        protocol_.GetErrorString());
      return false;
    }
    return true;
  }
  void update_active_6dof_objects()
  {
    std::map<std::string, unsigned int> new_map;
    // gets new objects
    for (unsigned int i = 0; i < protocol_.Get6DOFBodyCount(); i++) {
      std::string const name = protocol_.Get6DOFBodyName(i);
      if (new_map.find(name) == new_map.end()) {
        new_map.emplace(name, i);
      } else {
        RCLCPP_ERROR_STREAM(
          this->get_logger(), "Qualisys returned 6dof objects with the same name");
      }
      if (objects_6dof.find(name) == objects_6dof.end()) {
        RCLCPP_INFO_STREAM(this->get_logger(), "New 6dof object " << name << " found");
      }
      // creates new publishers
      if (publish_pose_ && pose_publishers_.find(name) == pose_publishers_.end()) {
        pose_publishers_.emplace(
          name, create_publisher<geometry_msgs::msg::PoseStamped>("~/" + name, 1));
      }
    }
    // checks removed objects
    for (auto const & object : objects_6dof) {
      if (new_map.find(object.first) == new_map.end()) {
        RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Qualisys does not returns 6dof object " << object.first << " anymore");
      }
    }
    // updates object list
    objects_6dof.clear();
    objects_6dof = new_map;
    return;
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";
    for (auto const & param : parameters) {
      if (param.get_name() == "publish_tf") {
        publish_tf_ = param.as_bool();
      } else if (param.get_name() == "publish_pose") {
        publish_pose_ = param.as_bool();
      } else if (param.get_name() == "qualisys_frame") {
        qualisys_frame_ == param.as_string();
      } else {
        result.successful = false;
        result.reason += "Param " + param.get_name() + " cannot be updated! ";
      }
    }
    return result;
  }

  // parameters
  int cycle_time_;
  std::string server_ip_;
  uint16_t server_port_;
  uint16_t udp_port_;
  std::string qualisys_frame_;
  bool publish_tf_;
  bool publish_pose_;

  // qualisys
  bool steaming_{false};
  bool read_settings_{false};
  CRTProtocol protocol_;
  std::unique_ptr<CRTPacket> packet_;
  std::map<std::string, unsigned int> objects_6dof;

  // ros
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::map<std::string, std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>>>
    pose_publishers_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;
};

}  // namespace qualisys_bridge

#endif
