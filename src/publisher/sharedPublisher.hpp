// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SHAREDPUBLISHER_HPP
#define SHAREDPUBLISHER_HPP

#include "publisherBase.hpp"

/**
 * This Publisher implementation just publishes right away.
 * It it shared in the context that many instances of this are managed by a single thread.
 */
class SharedPublisher final : public PublisherBase {
public:
    /**
     * Constructs a new shared publisher
     *
     * @param topic the topic which should be advertised and published on
     * @param node the node for which the rclcpp::publisher should be created
     * @param connection the connection for which this publisher is created
     */
    SharedPublisher(Topic_t &topic, const rclcpp::Node::SharedPtr &node, ConnectionBase &connection);

    /**
     * Publishes a message.
     * The actual implementation can make use of a queue and a publishing thread or directly publish it
     *
     * @param message the message to publish
     */
    void publish(std::unique_ptr<MessageBase> message) override;
};


#endif //SHAREDPUBLISHER_HPP
