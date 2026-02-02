// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sharedPublisher.hpp"

SharedPublisher::SharedPublisher(Topic_t &topic, const rclcpp::Node::SharedPtr &node, ConnectionBase &connection) : PublisherBase(topic, node, connection) {
}

void SharedPublisher::publish(std::unique_ptr<MessageBase> message) {
    if (message->isCompressed())
        RCLCPP_ERROR(this->logger.get(), "Received a compressed message on SharedPublisher for topic %s on channel %d, dropping message", this->topic.topic.c_str(), this->topic.channel);
    else
        this->rosPublish(message);  // passed by reference, no need to transfer ownership
}
