// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef GLOBALCONFIG_HPP_
#define GLOBALCONFIG_HPP_

#include <cstdint>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "connect/types.hpp"

/**
 * This class implements global properties.
 *
 * Every property must only be written by the main class of the application during initialization.
 * After that, only read access is allowed.
 */
class GlobalConfig {
   public:
    static std::string nodeName;
    static std::string nodeNamespace;
    static size_t nodeDomainId;

    static std::string host;
    static std::string path;
    static std::string port;

    static bool ssl;

    static std::string authenticationHost;
    static std::string authenticationPort;
    static bool authenticationSsl;

    static std::string authenticationPlugin;
    static std::string authenticationEndpoint;
    static bool authenticationOmit;
    static uint16_t authenticationTimeout;

    static bool fragmentation;
    static size_t fragmentationSize;

    static size_t maxMessageSize;

    static bool publishStatus;

    static std::string userKey;

    static bool subscribeClock;
    static bool publishClock;

    static bool subscribeTf;
    static bool publishTf;

    static std::map<uint8_t, rmw_qos_profile_t> qos;
    static std::map<uint8_t, compression_t> compression;

    static std::vector<Topic_t> subscriber;
    static std::vector<Topic_t> publisher;

    static std::vector<Service_Action_t> serviceServer;
    static std::vector<Service_Action_t> serviceClient;

    static std::vector<Service_Action_t> actionServer;
    static std::vector<Service_Action_t> actionClient;

   private:
    GlobalConfig();

    GlobalConfig(const GlobalConfig &);
};

#endif /* GLOBALCONFIG_HPP_ */
