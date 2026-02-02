// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "publisherManager.hpp"
#include "threadedPublisher.hpp"
#include "sharedPublisher.hpp"

#include "global/globalConfig.hpp"
#include "connect/types.hpp"

PublisherManager::PublisherManager(ConnectionBase &connection) : logger("publisher-manager"), connection(connection), hasSharedPublisher(false) {
}

PublisherManager::~PublisherManager() {
    // empty the toPublish queue
    while (!this->toPublish.empty()) this->toPublish.pop();
}

void PublisherManager::init(const rclcpp::Node::SharedPtr &node) {
    if (!this->connection.getRemoteEndpoint().empty()) this->logger.setName("publisher-manager", this->connection.getRemoteEndpoint());

    for (Topic_t &topic: GlobalConfig::publisher) {
        // create the publisher for the topic
        std::unique_ptr<PublisherBase> publisher;
        if (topic.useOwnThread) publisher = std::make_unique<ThreadedPublisher>(topic, node, this->connection);
        else {
            publisher = std::make_unique<SharedPublisher>(topic, node, this->connection);
            this->hasSharedPublisher = true;
        }

        // insert it into the map
        this->publishers.insert(std::pair<const uint8_t, std::unique_ptr<PublisherBase> >(topic.channel, std::move(publisher)));
    }
}

void PublisherManager::publish(std::unique_ptr<MessageBase> message) {
    std::unique_lock<std::mutex> lock(this->toPublishMutex);
    this->toPublish.push(std::move(message));
    this->toPublishCV.notify_all();
}

bool PublisherManager::onBeforeStartThread() {
    // start every threaded publisher
    for (const std::pair<const uint8_t, std::unique_ptr<PublisherBase> > &pair: this->publishers) {
        if (ThreadedPublisher *threaded = dynamic_cast<ThreadedPublisher *>(pair.second.get())) {
            threaded->startThread("TDPUBR");
        }
    }

    // then start the subscription monitor thread if needed
    if (this->hasSharedPublisher) {
        this->subscriptionMonitor = std::thread(std::bind(&PublisherManager::subscriptionMonitorRun, this));
    }

    // good to go
    return true;
}

void PublisherManager::onStopThread() {
    // we need to wake up the thread so it can exit the run-loop
    this->toPublishCV.notify_all();

    // then we need to start the monitor thread if applicable
    if (this->subscriptionMonitor.joinable()) this->subscriptionMonitor.join();

    // then we stop all threaded publisher
    while (!this->publishers.empty()) {
        const std::pair<const uint8_t, std::unique_ptr<PublisherBase> > &pair = *this->publishers.begin();
        if (ThreadedPublisher *threaded = dynamic_cast<ThreadedPublisher *>(pair.second.get())) {
            threaded->stopThread();
            threaded->joinThread();
        }
        this->publishers.erase(pair.first);
    }
}

void PublisherManager::run() {
    while (!this->stopping) {
        // get a unique lock for the toPublish mutex
        std::unique_lock<std::mutex> lock(this->toPublishMutex);
        // release the lock and wait until notified. only wake up if:
        //  - the toPublish queue is not empty
        //  - or the thread is stopping
        this->toPublishCV.wait(lock, [this]() {
            return !this->toPublish.empty() || this->stopping;
        });
        // make sure to break early
        if (this->stopping) break;
        // make sure we have a message
        if (this->toPublish.empty()) continue;

        // get the next message to send and release the lock again
        std::unique_ptr<MessageBase> message = std::move(this->toPublish.front());
        this->toPublish.pop();
        lock.unlock();

        // get the correct publisher and move the message over
        // depending on the actual implementation of the publisher this may either
        // enqueue the message (threaded publisher) or publish it right away (shared)
        const std::map<uint8_t, std::unique_ptr<PublisherBase> >::iterator it = this->publishers.find(message->getChannel());
        if (it == this->publishers.end() && !this->stopping) {
            RCLCPP_WARN(this->logger.get(), "Received a message to publish for unknown channel %d, dropping it", message->getChannel());
        } else if (!this->stopping) {
            it->second->publish(std::move(message));
        }
    }
}

void PublisherManager::subscriptionMonitorRun() const {
    while (!this->stopping) {
        // check for every shared publisher if it has a change in its subscriber count
        // if so, this also directly notifies the connection
        for (const std::pair<const uint8_t, std::unique_ptr<PublisherBase> > &pair: this->publishers) {
            if (SharedPublisher *shared = dynamic_cast<SharedPublisher *>(pair.second.get())) {
                if (!this->stopping) shared->testForSubscriber();
                else break;
            }
        }
        // now sleep for 100ms and repeat
        if (!this->stopping) usleep(100000);
    }
}
