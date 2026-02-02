// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef THREADEDSUBSCRIBER_HPP
#define THREADEDSUBSCRIBER_HPP

#include "subscriberBase.hpp"
#include "common/thread.hpp"
#include "connect/messageBase.hpp"

#include <queue>
#include <mutex>
#include <condition_variable>

class ThreadedSubscriber final : public SubscriberBase, public Thread {
public:
    /**
     * Constructs a new threaded subscriber
     *
     * @param topic the topic to which this should subscribe
     * @param connection the connection for which this subscriber is created
     */
    explicit ThreadedSubscriber(Topic_t &topic, ConnectionBase &connection);

    ~ThreadedSubscriber() override;

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

    /**
    * The callback function of the rclcpp::subscriber
    *
    * @param serializedMessage the serialized message
    */
    void callback(const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMessage) override;

private:
    std::queue<std::unique_ptr<MessageBase> > toSend;
    std::mutex toSendMutex;
    std::condition_variable toSendCV;
};


#endif //THREADEDSUBSCRIBER_HPP
