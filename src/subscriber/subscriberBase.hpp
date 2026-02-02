// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SUBSCRIBERBASE_HPP
#define SUBSCRIBERBASE_HPP

#include "connect/logger.hpp"
#include "connect/types.hpp"
#include "connection/base/connectionBase.hpp"

#include <rclcpp/rclcpp.hpp>

class SubscriberBase : public std::enable_shared_from_this<SubscriberBase> {
public:
    /**
     * Constructs a new subscriber base
     *
     * @param topic the topic to which this should subscribe
     * @param connection the connection for which this subscriber is created
     */
    explicit SubscriberBase(Topic_t &topic, ConnectionBase &connection) : connection(connection), topic(topic) {
        // store if we need compression to allow fast evaluation later on
        if (topic.compression->compressor != Compressor::NONE) this->compression = true;
        else this->compression = false;
    }

    /**
     * Actually initializes the subscriber base by creating the rclcpp::subscriber
     * This is needed since we need to construct a weak_pointer from this which is not possible in the constructor.
     *
     * @param node the node for which the rclcpp::subscription should be created
     * @param callbackGroup the callback group in which the callback of this rclcpp::subscription should be executed
     */
    void init(const rclcpp::Node::SharedPtr &node, const rclcpp::CallbackGroup::SharedPtr &callbackGroup) {
        rclcpp::SubscriptionOptions options;
        options.callback_group = callbackGroup; // use the given callback group to work either in parallel on a thread pool or mutual exclusive on a single thread
        options.ignore_local_publications = true; // ignore all publications which are made by the process in which this application runs (no dds implementation does support this though)

        if (topic.qos == nullptr) {
            RCLCPP_ERROR(this->logger.get(), "Topic %s on channel %d has no QoS definition, falling back to default profile", topic.topic.c_str(), topic.channel);
            this->subscription = node->create_generic_subscription(
                this->topic.topic,
                this->topic.type,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default),
                // we need to use a weak_ptr for this since the callback may be executed by the executor even after the this was destroyed
                // this is due to the lifecycle management of subscriptions in ROS2
                // another option would be to shutdown the executor before destroying this, however the executor is used by several subscriptions so this is not an option
                [weak = this->weak_from_this()](const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMessage) {
                    if (const std::shared_ptr<SubscriberBase> self = weak.lock()) {
                        self->callback(serializedMessage);
                    }
                },
                options
            );
        } else {
            this->subscription = node->create_generic_subscription(
                this->topic.topic,
                this->topic.type,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*topic.qos), *topic.qos),
                // we need to use a weak_ptr for this since the callback may be executed by the executor even after the this was destroyed
                // this is due to the lifecycle management of subscriptions in ROS2
                // another option would be to shutdown the executor before destroying this, however the executor is used by several subscriptions so this is not an option
                [weak = this->weak_from_this()](const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMessage) {
                    if (const std::shared_ptr<SubscriberBase> self = weak.lock()) {
                        self->callback(serializedMessage);
                    }
                },
                options
            );
        }

        if (this->topic.useOwnThread) {
            if (!this->connection.getRemoteEndpoint().empty()) this->logger.setName(std::to_string(this->topic.channel) + "-t-subscriber", this->connection.getRemoteEndpoint());
            else this->logger.setName(std::to_string(this->topic.channel) + "-t-subscriber");

            if (this->compression) {
                if (this->topic.compression->compressor == Compressor::LZ4_DEFAULT)
                    RCLCPP_INFO(this->logger.get(), "ThreadedSubscriber with LZ4_Default (%d) compression for topic %s with type %s on channel %d created", this->topic.compression->rate, this->topic.topic.c_str(), this->topic.type.c_str(), this->topic.channel);
                else if (this->topic.compression->compressor == Compressor::LZ4_HC)
                    RCLCPP_INFO(this->logger.get(), "ThreadedSubscriber with LZ4_HC (%d) compression for topic %s with type %s on channel %d created", this->topic.compression->rate, this->topic.topic.c_str(), this->topic.type.c_str(), this->topic.channel);
                else if (this->topic.compression->compressor == Compressor::ZLIB)
                    RCLCPP_INFO(this->logger.get(), "ThreadedSubscriber with ZLIB (%d) compression for topic %s with type %s on channel %d created", this->topic.compression->rate, this->topic.topic.c_str(), this->topic.type.c_str(), this->topic.channel);
            } else
                RCLCPP_INFO(this->logger.get(), "ThreadedSubscriber without compression for topic %s with type %s on channel %d created", this->topic.topic.c_str(), this->topic.type.c_str(), this->topic.channel);
        } else {
            if (!this->connection.getRemoteEndpoint().empty()) this->logger.setName(std::to_string(this->topic.channel) + "-s-subscriber", this->connection.getRemoteEndpoint());
            else this->logger.setName(std::to_string(this->topic.channel) + "-s-subscriber");

            RCLCPP_INFO(this->logger.get(), "SharedSubscriber without compression for topic %s with type %s on channel %d created", this->topic.topic.c_str(), this->topic.type.c_str(), this->topic.channel);
            if (this->compression)
                RCLCPP_WARN(this->logger.get(), "A compression was defined for SharedSubscriber for topic %s with type %s on channel %d, ignoring it", this->topic.topic.c_str(), this->topic.type.c_str(), this->topic.channel);
        }
    }

    virtual ~SubscriberBase() {
        if (this->subscription != nullptr) {
            RCLCPP_INFO(this->logger.get(), "Subscriber for topic %s with type %s on channel %d destroyed", this->topic.topic.c_str(), this->topic.type.c_str(), this->topic.channel);

            this->subscription.reset();
            this->subscription = nullptr;
        }
    }

protected:
    Logger logger;

    ConnectionBase &connection;

    Topic_t &topic;
    bool compression;

    rclcpp::GenericSubscription::SharedPtr subscription = nullptr;

    /**
     * The callback function of the rclcpp::subscriber
     *
     * @param serializedMessage the serialized message
     */
    virtual void callback(const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMessage) = 0;
};

#endif //SUBSCRIBERBASE_HPP
