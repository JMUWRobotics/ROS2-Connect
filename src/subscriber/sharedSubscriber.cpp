// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sharedSubscriber.hpp"

#include "message/vectorMessage.hpp"

SharedSubscriber::SharedSubscriber(Topic_t &topic, ConnectionBase &connection): SubscriberBase(topic, connection) {
}

void SharedSubscriber::callback(const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMessage) {
    const std::shared_ptr<const std::vector<uint8_t> > combinedVector = VectorMessage::fromSerializedMessage(this->topic.channel, *serializedMessage)->getCombinedVector();
    this->connection.send(combinedVector);
}
