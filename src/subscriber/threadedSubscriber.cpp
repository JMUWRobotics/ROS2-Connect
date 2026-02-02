// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "threadedSubscriber.hpp"
#include "message/vectorMessage.hpp"

ThreadedSubscriber::ThreadedSubscriber(Topic_t &topic, ConnectionBase &connection) : SubscriberBase(topic, connection) {
}

ThreadedSubscriber::~ThreadedSubscriber() {
    // stop the subscription before beginning to clear the queue
    if (this->subscription != nullptr) {
        RCLCPP_INFO(this->logger.get(), "Subscriber for topic %s with type %s on channel %d destroyed", this->topic.topic.c_str(), this->topic.type.c_str(), this->topic.channel);

        this->subscription.reset();
        this->subscription = nullptr;
    }
    while (!this->toSend.empty()) this->toSend.pop();
}

void ThreadedSubscriber::onStopThread() {
    this->toSendCV.notify_all();
}

void ThreadedSubscriber::run() {
    while (!this->stopping) {
        // get a unique lock for the toSend mutex
        std::unique_lock<std::mutex> lock(this->toSendMutex);
        // release the lock and wait until notified. only wake up if:
        //  - the toSend queue is not empty
        //  - or the thread is stopping
        this->toSendCV.wait(lock, [this]() {
            return !this->toSend.empty() || this->stopping;
        });
        // make sure to break early
        if (this->stopping) break;
        // make sure we have a message
        if (this->toSend.empty()) continue;

        // get the next message and release the lock again
        const std::unique_ptr<MessageBase> message = std::move(this->toSend.front());
        this->toSend.pop();
        lock.unlock();

        // convert the message into a combined vector for sending applying compression if necessary
        std::shared_ptr<const std::vector<uint8_t> > combinedVector;
        if (this->compression) combinedVector = message->getCompressedCombinedVector(*this->topic.compression);
        else combinedVector = message->getCombinedVector();

        // finally send the message
        if (!this->stopping) this->connection.send(combinedVector);
    }
}

void ThreadedSubscriber::callback(const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMessage) {
    // copy the underlying data and putting it into a Message
    std::unique_ptr<MessageBase> message = VectorMessage::fromSerializedMessage(this->topic.channel, serializedMessage);

    // now queueing the message
    std::unique_lock<std::mutex> lock(this->toSendMutex);
    this->toSend.push(std::move(message));
    this->toSendCV.notify_all();
}
