// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "subscriberManager.hpp"
#include "threadedSubscriber.hpp"
#include "sharedSubscriber.hpp"
#include "global/globalConfig.hpp"

SubscriberManager::SubscriberManager(ConnectionBase &connection, const rclcpp::Node::SharedPtr &node) : logger("subscriber-manager"), clockSubscriber(connection), node(node), connection(connection) {
    this->reentrantCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
    this->mutualExclusiveCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
}

SubscriberManager::~SubscriberManager() {
    // empty the toSubscribe queue
    while (!this->toSubscribe.empty())
        this->toSubscribe.pop();
}

void SubscriberManager::init() {
    if (!this->connection.getRemoteEndpoint().empty()) this->logger.setName("subscriber-manager", this->connection.getRemoteEndpoint());

    // create all permanent subscriber
    for (Topic_t &topic: GlobalConfig::subscriber) {
        if (topic.permanent) this->subscribe(topic, false);
    }
}

void SubscriberManager::subscribe(std::unique_ptr<MessageBase> message) {
    std::unique_lock<std::mutex> lock(this->toSubscribeMutex);
    this->toSubscribe.push(std::move(message));
    this->toSubscribeCV.notify_all();
}

bool SubscriberManager::onBeforeStartThread() {
    // start every permanent subscribers thread
    for (const std::pair<const uint8_t, std::shared_ptr<SubscriberBase> > &pair: this->subscribers) {
        if (ThreadedSubscriber *threaded = dynamic_cast<ThreadedSubscriber *>(pair.second.get())) {
            threaded->startThread("TDSUBR");
        }
    }
    // start the clock subscriber
    if (GlobalConfig::subscribeClock) this->clockSubscriber.startThread("CLKSUB");

    // good to go
    return true;
}

void SubscriberManager::onStopThread() {
    // we need to wake up the thread so it can exit the run-loop
    this->toSubscribeCV.notify_all();

    // then we stop all threaded subscriber
    while (!this->subscribers.empty()) {
        const std::pair<const uint8_t, std::shared_ptr<SubscriberBase> > &pair = *this->subscribers.begin();
        this->unsubscribe(pair);
    }

    // stop the clock subscriber
    this->clockSubscriber.stopThread();
    this->clockSubscriber.joinThread();
}

void SubscriberManager::run() {
    while (!this->stopping) {
        // get a unique lock for the toSubscribe mutex
        std::unique_lock<std::mutex> lock(this->toSubscribeMutex);
        // release the lock and wait until notified. only wake up if:
        //  - the toSubscribe queue is not empty
        //  - or the thread is stopping
        this->toSubscribeCV.wait(lock, [this]() { return !this->toSubscribe.empty() || this->stopping; });
        // make sure to break early
        if (this->stopping)
            break;
        // make sure we have a message
        if (this->toSubscribe.empty())
            continue;

        // get the next message and release the lock again
        const std::unique_ptr<MessageBase> message = std::move(this->toSubscribe.front());
        this->toSubscribe.pop();
        lock.unlock();

        // make sure the message has the correct opcode
        if (message->getChannel() != OpCode::SUBSCRIPTION) {
            RCLCPP_INFO(this->logger.get(), "Received a message which is not of op-code SUBSCRIPTION, has channel %d, dropping message", message->getChannel());
            continue;
        }

        // extract the channel and if we want to subscribe or unsubscribe
        const std::pair<const uint8_t, const bool> subscription = message->toSubscription();

        // if the topic to subscribe is op-code CLOCK
        // start or stop the clock subscriber
        if (subscription.first == OpCode::CLOCK) {
            if (GlobalConfig::subscribeClock) {
                if (subscription.second) this->clockSubscriber.start();
                else if (!subscription.second) this->clockSubscriber.stop();
            } else {
                RCLCPP_WARN(this->logger.get(), "Received a message to (un-)subscribe to /clock but subscribe_clock = false, dropping message");
            }
            continue;
        }

        // find the topic definition
        Topic_t *topic = nullptr;
        for (Topic_t &sub: GlobalConfig::subscriber) {
            if (sub.channel == subscription.first) {
                topic = &sub;
                break;
            }
        }
        if (topic == nullptr) {
            RCLCPP_WARN(this->logger.get(), "Received a message to (un-)subscribe to unknown channel %d, dropping message", subscription.first);
            continue;
        }

        // get the instantiated subscriber for the channel if there is one
        const std::map<const uint8_t, std::shared_ptr<SubscriberBase> >::iterator it = this->subscribers.find(subscription.first);

        // we want to subscribe, therefore we need to test if we know the channel and are not yet subscribed
        if (subscription.second) {
            // already subscribed (includes permanent)
            if (it != this->subscribers.end() && !this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Received a message to subscribe to already subscribed topic %s, dropping message", topic->topic.c_str());
            }
            // ok
            else if (!this->stopping) {
                this->subscribe(*topic, true);
            }
        }
        // we want to unsubscribe, therefore we need to test if we are subscribed
        else {
            // not subscribed
            if (it == this->subscribers.end() && !this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Received a message to unsubscribe from not subscribed topic %s, dropping message", topic->topic.c_str());
            }
            // permanent subscriber
            else if (!this->stopping && topic->permanent) {
                RCLCPP_WARN(this->logger.get(), "Received a message to unsubscribe from a permanent topic %s, dropping message", topic->topic.c_str());
            }
            // ok
            else if (!this->stopping) {
                this->unsubscribe(*it);
            }
        }
    }
}

void SubscriberManager::unsubscribe(const std::pair<const uint8_t, std::shared_ptr<SubscriberBase> > &pair) {
    if (ThreadedSubscriber *threaded = dynamic_cast<ThreadedSubscriber *>(pair.second.get())) {
        threaded->stopThread();
        threaded->joinThread();
    }
    this->subscribers.erase(pair.first);
}

void SubscriberManager::subscribe(Topic_t &topic, const bool startThread) {
    if (topic.useOwnThread) {
        std::shared_ptr<ThreadedSubscriber> subscriber = std::make_shared<ThreadedSubscriber>(topic, this->connection);
        subscriber->init(this->node, this->reentrantCallbackGroup);
        if (startThread) subscriber->startThread("TDSUBR");
        this->subscribers.insert(std::pair<const uint8_t, std::shared_ptr<SubscriberBase> >(topic.channel, subscriber));
    } else {
        std::shared_ptr<SharedSubscriber> subscriber = std::make_shared<SharedSubscriber>(topic, this->connection);
        subscriber->init(this->node, this->mutualExclusiveCallbackGroup);
        this->subscribers.insert(std::pair<const uint8_t, std::shared_ptr<SubscriberBase> >(topic.channel, subscriber));
    }
}
