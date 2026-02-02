// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef PUBLISHERBASE_HPP
#define PUBLISHERBASE_HPP

#include "connect/logger.hpp"
#include "connect/types.hpp"
#include "connect/messageBase.hpp"
#include "message/vectorMessage.hpp"
#include "connection/base/connectionBase.hpp"
#include "global/globalConfig.hpp"

#include <rclcpp/rclcpp.hpp>

class PublisherBase {
public:
    /**
     * Constructs a new publisher base
     *
     * @param topic the topic which should be advertised and published on
     * @param node the node for which the rclcpp::publisher should be created
     * @param connection the connection for which this publisher is created
     */
    explicit PublisherBase(Topic_t &topic, const rclcpp::Node::SharedPtr &node, ConnectionBase &connection) : connection(connection), topic(topic), hasSubscriber(false), subscriberGoneAwayTimePoint(0) {
        this->clock = rclcpp::Clock(RCL_ROS_TIME);

        if (topic.qos == nullptr) {
            RCLCPP_ERROR(this->logger.get(), "Topic %s on channel %d has no QoS definition, falling back to default profile", topic.topic.c_str(), topic.channel);
            this->publisher = node->create_generic_publisher(topic.topic, topic.type, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default));
        } else {
            this->publisher = node->create_generic_publisher(topic.topic, topic.type, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*topic.qos), *topic.qos));
        }

        if (topic.useOwnThread) {
            if (!this->connection.getRemoteEndpoint().empty()) this->logger.setName(std::to_string(topic.channel) + "-t-publisher", this->connection.getRemoteEndpoint());
            else this->logger.setName(std::to_string(topic.channel) + "-t-publisher");

            RCLCPP_INFO(this->logger.get(), "ThreadedPublisher for topic %s with type %s on channel %d created", topic.topic.c_str(), topic.type.c_str(), topic.channel);
        } else {
            if (!this->connection.getRemoteEndpoint().empty()) this->logger.setName(std::to_string(topic.channel) + "-s-publisher", this->connection.getRemoteEndpoint());
            else this->logger.setName(std::to_string(topic.channel) + "-s-publisher");

            RCLCPP_INFO(this->logger.get(), "SharedPublisher for topic %s with type %s on channel %d created", topic.topic.c_str(), topic.type.c_str(), topic.channel);
        }
    }

    virtual ~PublisherBase() {
        if (this->publisher != nullptr) {
            RCLCPP_INFO(this->logger.get(), "Publisher for topic %s with type %s on channel %d destroyed", this->topic.topic.c_str(), this->topic.type.c_str(), this->topic.channel);

            this->publisher.reset();
            this->publisher = nullptr;
        }
    }

    /**
     * Publishes a message.
     * The actual implementation can make use of a queue and a publishing thread or directly publish it
     *
     * @param message the message to publish
     */
    virtual void publish(std::unique_ptr<MessageBase> message) = 0;

    /**
     * Tests if there is still or a new subscriber for this publish.
     * If so, this notifies the connection.
     */
    void testForSubscriber() {
        const size_t subscriptionCount = this->publisher->get_subscription_count();

        // if the subscriber has gone away
        if (this->hasSubscriber && subscriptionCount == 0) {
            // if its the first iteration we notice this, we store the time
            if (this->subscriberGoneAwayTimePoint == 0) {
                this->subscriberGoneAwayTimePoint = this->clock.now().nanoseconds();
            }
                // otherwise we test if 10s have passed, only then we notify the "other side" about the subscriber going away
            else if ((this->clock.now().nanoseconds() - this->subscriberGoneAwayTimePoint) >= 10000000000) {
                this->notifyAboutSubscriber(false);
                this->hasSubscriber = false;
            }
        }
            // if the subscriber re-appeared, we clear the stored time
        else if (this->hasSubscriber && this->subscriberGoneAwayTimePoint > 0 && subscriptionCount > 0) {
            this->subscriberGoneAwayTimePoint = 0;
        }
            // if a subscriber as appeared, notify the "other side" about the subscriber appearing
        else if (!this->hasSubscriber && subscriptionCount > 0) {
            this->notifyAboutSubscriber(true);
            this->subscriberGoneAwayTimePoint = 0;
            this->hasSubscriber = true;
        }
    }

protected:
    Logger logger;

    ConnectionBase &connection;

    Topic_t &topic;

    rclcpp::GenericPublisher::SharedPtr publisher = nullptr;

    bool hasSubscriber;

    rclcpp::Clock clock;
    rcl_time_point_value_t subscriberGoneAwayTimePoint;

    /**
     * Actual publishing mechanism on the rclcpp::publisher
     * If possible, using memory loaning to be able to publish in a zero-copy-mechanism
     *
     * @param message the message to publish
     */
    void rosPublish(const std::unique_ptr<MessageBase> &message) const {
        if (rclcpp::ok()) this->publisher->publish(*message->getSerializedMessage());
    }

    /**
    * This method whill send a Message holding op-code SUBSCRIPTION to inform the "other side"
    * that this publisher has now at least one subscription or none
    *
    * @param value true if at least one subscription, false if none
    */
    void notifyAboutSubscriber(const bool value) const {
        const std::shared_ptr<const std::vector<uint8_t> > combinedVector = VectorMessage::forOpCodeSubscription(this->topic.channel, value)->getCombinedVector();
        this->connection.send(combinedVector);
    }
};

#endif //PUBLISHERBASE_HPP
