// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "connect/logger.hpp"
#include "global/globalConfig.hpp"

Logger::Logger() : logger(rclcpp::get_logger(GlobalConfig::nodeName)) {
}

Logger::Logger(const std::string &name) : logger(rclcpp::get_logger(GlobalConfig::nodeName + "." + name)) {
}

Logger::Logger(const std::string &name, const std::string &remoteEndpoint) : logger(rclcpp::get_logger(GlobalConfig::nodeName + "." + name + "." + remoteEndpoint)) {
}

const rclcpp::Logger & Logger::get() const {
    return this->logger;
}

void Logger::resetName() {
    this->logger = rclcpp::get_logger(GlobalConfig::nodeName);
}

void Logger::setName(const std::string &name) {
    this->logger = rclcpp::get_logger(GlobalConfig::nodeName + "." + name);
}

void Logger::setName(const std::string &name, const std::string &remoteEndpoint) {
    this->logger = rclcpp::get_logger(GlobalConfig::nodeName + "." + name + "." + remoteEndpoint);
}
