// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "threadedPublisher.hpp"

ThreadedPublisher::ThreadedPublisher(Topic_t &topic, const rclcpp::Node::SharedPtr &node, ConnectionBase &connection) : PublisherBase(topic, node, connection) {
}

ThreadedPublisher::~ThreadedPublisher() {
    while (!this->toPublish.empty()) this->toPublish.pop();
}

void ThreadedPublisher::onStopThread() {
    this->toPublishCV.notify_all();
}

void ThreadedPublisher::publish(std::unique_ptr<MessageBase> message) {
    std::unique_lock<std::mutex> lock(this->toPublishMutex);
    this->toPublish.push(std::move(message));
    this->toPublishCV.notify_all();
}

void ThreadedPublisher::run() {
    while (!this->stopping) {
        // get a unique lock for the toPublish mutex
        std::unique_lock<std::mutex> lock(this->toPublishMutex);
        // release the lock and wait until notified OR until the given time passed. only wake up on notify if:
        //  - the toPublish queue is not empty
        //  - or the thread is stopping
        this->toPublishCV.wait_for(lock, this->hasSubscriber ? std::chrono::seconds(1) : std::chrono::milliseconds(100), [this]() {
            return !this->toPublish.empty() || this->stopping;
        });
        // make sure to break early
        if (this->stopping) break;

        // if there is a message to publish get it to release the lock
        std::unique_ptr<MessageBase> message = nullptr;
        if (!this->toPublish.empty()) {
            message = std::move(this->toPublish.front());
            this->toPublish.pop();
        }
        lock.unlock();

        // check the subscriber count for this topic
        if (!this->stopping) this->testForSubscriber();

        // publish the message if necessary
        if (message != nullptr) {
            if (!this->stopping) {
                // if the message is compressed, try to decompress it
                // after decompressing the rclcpp::SerializedMessage will hold decompressed data
                // if it fails, we can not continue
                if (message->isCompressed() && !message->decompress()) {
                    RCLCPP_ERROR(this->logger.get(), "Received compressed message for topic %s on channel %d which cannot be decompressed, dropping message", this->topic.topic.c_str(), this->topic.channel);
                    continue;
                }
                
                // publish
                this->rosPublish(message);  // passed by reference, no need to transfer ownership
            }
        }
    }
}
