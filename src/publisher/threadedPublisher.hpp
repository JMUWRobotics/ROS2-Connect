// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef THREADEDPUBLISHER_HPP
#define THREADEDPUBLISHER_HPP

#include "publisherBase.hpp"
#include "common/thread.hpp"
#include "connection/base/connectionBase.hpp"

#include <queue>
#include <mutex>
#include <condition_variable>

/**
* This Publisher implementation makes use of a own thread and a publishing queue.
* With this, the publish method only queues a new message to publish to speed up overall.
* This also can make use of compression.
*/
class ThreadedPublisher final : public PublisherBase, public Thread {
public:
    /**
    * Constructs a new threaded publisher
    *
    * @param topic the topic which should be advertised and published on
    * @param node the node for which the rclcpp::publisher should be created
    * @param connection the connection for which this publisher is created
    */
    ThreadedPublisher(Topic_t &topic, const rclcpp::Node::SharedPtr &node, ConnectionBase &connection);

    ~ThreadedPublisher() override;

    /**
     * Publishes a message.
     * The actual implementation can make use of a queue and a publishing thread or directly publish it
     *
     * @param message the message to publish
     */
    void publish(std::unique_ptr<MessageBase> message) override;

protected:
    /**
     * Run method which will be executed by the underlying std::thread.
     * This method must respond to this->stopping becoming true.
     */
    void run() override;

    /**
     * Called right after the underlying thread is stopped by setting the stopping flag to true.
     */
    void onStopThread() override;

private:
    std::queue<std::unique_ptr<MessageBase> > toPublish;
    std::mutex toPublishMutex;
    std::condition_variable toPublishCV;
};

#endif //THREADEDPUBLISHER_HPP
