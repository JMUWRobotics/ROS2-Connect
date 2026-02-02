// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "statusPublisher.hpp"

#include <connect/msg/connect_status.hpp>

#include "global/globalConfig.hpp"

StatusPublisher::StatusPublisher(const rclcpp::Node::SharedPtr &node, std::function<void()> callback) : node(node), publishCallback(std::move(callback)) {
    this->statusPublisher = node->create_publisher<connect::msg::ConnectStatus>("connect/status", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default));
    this->statusTimerCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
    this->statusTimer = node->create_wall_timer(std::chrono::seconds(1), [this]() { this->publishCallback(); }, this->statusTimerCallbackGroup, true);
}

void StatusPublisher::publishStatus(const bool connected) {
    connect::msg::ConnectStatus msg;
    msg.header.stamp = this->node->now();
    msg.authentication_omit = GlobalConfig::authenticationOmit;
    msg.domain_id = GlobalConfig::nodeDomainId;
    msg.name_space = GlobalConfig::nodeNamespace;
    msg.client_connected = connected;

    rclcpp::Publisher<connect::msg::ConnectStatus>::SharedPtr statusPublisher = std::static_pointer_cast<rclcpp::Publisher<connect::msg::ConnectStatus>>(this->statusPublisher);
    statusPublisher->publish(msg);
}
