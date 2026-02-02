// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SUBSCRIBERMANAGER_HPP
#define SUBSCRIBERMANAGER_HPP

#include "subscriberBase.hpp"
#include "clockSubscriber.hpp"
#include "connect/messageBase.hpp"
#include "connect/logger.hpp"
#include "common/thread.hpp"
#include "connect/types.hpp"
#include "connection/base/connectionBase.hpp"

#include <memory>
#include <map>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

class SubscriberManager final : public Thread {
public:
    /**
     * Constructs a new subscriber manager for a certain connection
     *
     * @param connection the connection for which this subscriber manager is created
     * @param node the node for which the rclcpp::subscriber should be created
     */
    explicit SubscriberManager(ConnectionBase &connection, const rclcpp::Node::SharedPtr &node);

    ~SubscriberManager() override;

    /**
     * Initializes this manager which will create all the needed permanent Subscriber instances according to GlobalConfig
     * This should be called AFTER the connection was authorized
     */
    void init();

    /**
     * Processes an op-code SUBSCRIPTION message
     *
     * @param message the message to process
     */
    void subscribe(std::unique_ptr<MessageBase> message);

protected:

    /**
     * Called before the underlying thread is started.
     * This must return true for the start procedure to continue.
     * Hence this can be used to do some checks before actually starting the thread.
     *
     * @return true if thread start procedure should be continued
     */
    bool onBeforeStartThread() override;

    /**
     * Called right after the underlying thread is stopped by setting the stopping flag to true.
     */
    void onStopThread() override;

    /**
     * Run method which will be executed by the underlying std::thread.
     * This method must respond to this->stopping becoming true.
     */
    void run() override;

private:
    Logger logger;

    std::map<const uint8_t, std::shared_ptr<SubscriberBase> > subscribers;

    ClockSubscriber clockSubscriber;

    rclcpp::Node::SharedPtr node;
    rclcpp::CallbackGroup::SharedPtr reentrantCallbackGroup;
    rclcpp::CallbackGroup::SharedPtr mutualExclusiveCallbackGroup;

    ConnectionBase &connection;

    std::queue<std::unique_ptr<MessageBase> > toSubscribe;
    std::mutex toSubscribeMutex;
    std::condition_variable toSubscribeCV;

    /**
     * Unsubscribe the given subscriber
     *
     * @param pair subscriber to unsubscribe from
     */
    void unsubscribe(const std::pair<const uint8_t, std::shared_ptr<SubscriberBase> > &pair);

    /**
     * Subscribe to the given topic
     *
     * @param topic topic to subscribe to
     * @param startThread if the underlying thread for a threaded subscriber should be started right away
     */
    void subscribe(Topic_t &topic, bool startThread);
};


#endif //SUBSCRIBERMANAGER_HPP
