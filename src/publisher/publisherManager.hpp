// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef PUBLISHERMANAGER_HPP
#define PUBLISHERMANAGER_HPP

#include "publisherBase.hpp"

#include "connect/logger.hpp"
#include "connect/messageBase.hpp"
#include "common/thread.hpp"
#include "connection/base/connectionBase.hpp"

#include <map>
#include <queue>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"

class PublisherManager final : public Thread {
public:
    /**
     * Create a new publisher manager for a certain connection
     *
     * @param connection the connection for which this publish manager is created
     */
    explicit PublisherManager(ConnectionBase &connection);

    ~PublisherManager() override;

    /**
     * Initializes this manager which will create all the needed Publisher instances according to GlobalConfig
     * This should be called AFTER the connection was authorized
     *
     * @param node the node for which the rclcpp::publisher should be created
     */
    void init(const rclcpp::Node::SharedPtr &node);

    /**
     * Queues a message to be published
     *
     * @param message message to be published
     */
    void publish(std::unique_ptr<MessageBase> message);

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

    /**
     * Run method of the subscription-monitor thread.
     * This thread follows the stopping variable.
     */
    void subscriptionMonitorRun() const;

private:
    Logger logger;

    std::map<const uint8_t, std::unique_ptr<PublisherBase> > publishers;

    ConnectionBase &connection;

    std::queue<std::unique_ptr<MessageBase> > toPublish;
    std::mutex toPublishMutex;
    std::condition_variable toPublishCV;

    bool hasSharedPublisher;
    std::thread subscriptionMonitor;
};


#endif //PUBLISHERMANAGER_HPP
