// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef CLOCK_HPP
#define CLOCK_HPP

#include "common/thread.hpp"
#include "connect/logger.hpp"
#include "connection/base/connectionBase.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>

class ClockSubscriber final : public Thread {
public:
    /**
     * Creates a new clock subscriber
     *
     * This is a special kind of subscriber since it does not actually subscriber to a topic
     * but rather sends the current ros-time of this node over the connection to the other side
     * on channel op-code CLOCK
     *
     * @param connection connection to send clock to
     */
    explicit ClockSubscriber(ConnectionBase &connection);

    /**
     * Start the subscriber and therefore timer with an expiration of 10ms
     * The timer publishes the clock and restarts itself
     */
    void start();

    /**
     * Stops the subscriber and therefore timer
     */
    void stop();

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
    Logger logger;

    ConnectionBase &connection;

    rclcpp::Clock clock;
    rclcpp::Serialization<rosgraph_msgs::msg::Clock> serialization;

    boost::asio::io_context ioc{};
    boost::asio::steady_timer timer{ioc};

    bool timerRunning;

    /**
     * Restarts the timer
     */
    void restartTimer();

    /**
     * Send the current ros-time on the op-code channel CLOCK over the stored connection
     */
    void sendClock() const;
};


#endif //CLOCK_HPP
