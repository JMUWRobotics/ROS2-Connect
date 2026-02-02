// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef STATUS_PUBLISHER_HPP
#define STATUS_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>

/**
 * This class allows publishing messages of type ConnectStatus.msg
 *
 * This hack is necessary due to the fact that the boost library defines the namespace "connect".
 * However, ConnectStatus.msg is also defined in the namespace "connect", by the ros idl generator.
 * Therefore, we must hide ConnectStatus to avoid a re-definition of the "connect" namespace.
 */
class StatusPublisher final {
   public:
    /**
     * Creates a new status publisher
     *
     * @param node the node to create publisher, callbackgroup and timer on
     * @param callback a callback which is called by the timer periodically; the callback must call publishStatus(...)
     */
    StatusPublisher(const rclcpp::Node::SharedPtr &node, std::function<void()> callback);

    /**
     * Stops the periodical timer and destructs everything
     */
    ~StatusPublisher() {
        if (this->statusTimer != nullptr) {
            this->statusTimer->cancel();

            this->statusTimer.reset();
            this->statusTimer = nullptr;
        }
    }

    /**
     * Actually publishes the status
     *
     * @param connected if at least one client is connected
     */
    void publishStatus(const bool connected);

   private:
    rclcpp::Node::SharedPtr node = nullptr;

    rclcpp::CallbackGroup::SharedPtr statusTimerCallbackGroup = nullptr;
    rclcpp::TimerBase::SharedPtr statusTimer = nullptr;
    rclcpp::PublisherBase::SharedPtr statusPublisher = nullptr;

    std::function<void()> publishCallback = []() {
    };
};

#endif  // STATUS_PUBLISHER