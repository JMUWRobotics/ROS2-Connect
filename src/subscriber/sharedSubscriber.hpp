// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SHAREDSUBSCRIBER_HPP
#define SHAREDSUBSCRIBER_HPP

#include "subscriberBase.hpp"

class SharedSubscriber final : public SubscriberBase {
public:
    /**
     * Constructs a new shared subscriber
     *
     * @param topic the topic to which this should subscribe
     * @param connection the connection for which this subscriber is created
     */
    explicit SharedSubscriber(Topic_t &topic, ConnectionBase &connection);

protected:
    /**
     * The callback function of the rclcpp::subscriber
     *
     * @param serializedMessage the serialized message
     */
    void callback(const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMessage) override;
};


#endif //SHAREDSUBSCRIBER_HPP
