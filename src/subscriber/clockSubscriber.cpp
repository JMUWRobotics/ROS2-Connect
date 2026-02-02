// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "clockSubscriber.hpp"
#include "connect/types.hpp"
#include "message/vectorMessage.hpp"

#include <cstdint>

ClockSubscriber::ClockSubscriber(ConnectionBase &connection) : connection(connection), timer((this->ioc)), timerRunning(false) {
    this->logger.setName(std::to_string(OpCode::CLOCK) + "-t-subscriber");
    this->clock = rclcpp::Clock(RCL_ROS_TIME);
}

void ClockSubscriber::start() {
    this->timerRunning = true;
    this->restartTimer();

    RCLCPP_INFO(this->logger.get(), "ThreadedSubscriber for topic /clock with type rosgraph_msgs/msg/Clock on channel %d created", OpCode::CLOCK);
}

void ClockSubscriber::stop() {
    const bool timerWasRunning = this->timerRunning;
    this->timerRunning = false;
    this->timer.cancel();

    if (timerWasRunning)
        RCLCPP_INFO(this->logger.get(), "Subscriber for topic /clock on channel %d destroyed", OpCode::CLOCK);
}

void ClockSubscriber::restartTimer() {
    // set time to expire after 10ms
    this->timer.expires_after(std::chrono::milliseconds(10));
    // publish the clock and restart the timer
    this->timer.async_wait([this](const boost::system::error_code & /* ec */) {
        this->sendClock();
        if (this->timerRunning) this->restartTimer();
    });
}

void ClockSubscriber::sendClock() const {
    // get the current ros time
    const rclcpp::Time time = this->clock.now();

    // put it in a message
    rosgraph_msgs::msg::Clock message;
    message.clock = time;

    // serialize it
    rclcpp::SerializedMessage serializedMessage;
    this->serialization.serialize_message(&message, &serializedMessage);

    // send it to the other side on op-code CLOCK
    const std::shared_ptr<const std::vector<uint8_t> > combinedVector = VectorMessage::fromSerializedMessage(OpCode::CLOCK, serializedMessage)->getCombinedVector();
    this->connection.send(combinedVector);
}

void ClockSubscriber::onStopThread() {
    this->stop();
    this->ioc.stop();
}

void ClockSubscriber::run() {
    // re-name the logger again, the connection should have an enpoint address
    if (!this->connection.getRemoteEndpoint().empty()) this->logger.setName(std::to_string(OpCode::CLOCK) + "-t-subscriber", this->connection.getRemoteEndpoint());
    else this->logger.setName(std::to_string(OpCode::CLOCK) + "-t-subscriber");

    while (!this->stopping) {
        // create a work guard to keep the ioc alive without it waking up the cpu
        boost::asio::executor_work_guard<boost::asio::io_context::basic_executor_type<std::allocator<void>, 0> > workGuard = boost::asio::make_work_guard(this->ioc);

        // start the ioc
        try {
            this->ioc.run();
        } catch (...) {
            const std::exception_ptr p = std::current_exception();
            RCLCPP_ERROR(this->logger.get(), "I/O context unexpectedly threw an exception: %s", p ? p.__cxa_exception_type()->name() : "null");
        }

        // restart the ioc if not stopping
        if (!this->stopping) {
            workGuard.reset();
            this->ioc.restart();
        }
    }
}
