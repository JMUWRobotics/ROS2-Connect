// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <rclcpp/rclcpp.hpp>

/**
 * This class provides named logging capabilities
 */
class Logger {
public:
    /**
     * Construct a named logger
     * The loggers name will be "$node_name"
     */
    explicit Logger();

    /**
     * Construct a named logger
     * The loggers name will be "$node_name.$name"
     *
     * @param name name of logger
     */
    explicit Logger(const std::string &name);

    /**
     * Constructs a named logger
     * The loggers name will be "$node_name.$name.$remoteEndpoint"
     *
     * @param name name of logger
     * @param remoteEndpoint remote endpoint of logger
     */
    explicit Logger(const std::string &name, const std::string &remoteEndpoint);

    /**
     * @return named logger
     */
    const rclcpp::Logger &get() const;

    /**
     * Resets the name of the logger
     * The loggers name will be "$node_name$
     */
    void resetName();

    /**
     * Sets the name of the logger
     * The loggers name will be "$node_name.$name"
     *
     * @param name name of logger
     */
    void setName(const std::string &name);

    /**
     * Sets the name of the logger
     * The loggers name will be "$node_name.$name.$remoteEndpoint"
     *
     * @param name name of logger
     * @param remoteEndpoint remote endpoint of logger
     */
    void setName(const std::string &name, const std::string &remoteEndpoint);

private:
    rclcpp::Logger logger;
};

#endif //LOGGER_HPP
