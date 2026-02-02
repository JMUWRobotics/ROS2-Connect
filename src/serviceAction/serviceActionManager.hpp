// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SERVICEACTION_MANAGER_HPP
#define SERVICEACTION_MANAGER_HPP

#include "common/thread.hpp"
#include "connection/base/connectionBase.hpp"
#include "connect/messageBase.hpp"
#include "connect/logger.hpp"

#include "connect/service/serviceClient.hpp"
#include "connect/service/serviceServer.hpp"

#include "connect/action/actionClient.hpp"
#include "connect/action/actionServer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

class ServiceActionManager final : public Thread, public std::enable_shared_from_this<ServiceActionManager> {
public:
    /**
     * Creates a new service and action manager for a certain connection
     *
     * @param connection the connection for which the service and action manager is created
     * @param node the node for which the clients and servers should be created
     */
    explicit ServiceActionManager(ConnectionBase &connection, const rclcpp::Node::SharedPtr &node);

    ~ServiceActionManager() override;

    /**
     * Initialize this manager which will create all the needed service an action server and client instances by loading their implementation using pluginlib according to GlobalConfig
     * This should be called AFTER the connection was authorized
     */
    void init();

    /**
     * This handles a message
     * @param message
     */
    void handle(std::unique_ptr<MessageBase> message);

protected:
    /**
     * Called right after the underlying thread is stopped by setting the stopping flag to true.
     */
    void onStopThread() override;

    /**
     * Run method which will be executed by the underlying std::thread.
     * This method must respond to this->stopping becoming true.
     */
    void run() override;

private:
    Logger logger;

    pluginlib::ClassLoader<service::ServiceServer> serviceServerLoader;
    pluginlib::ClassLoader<service::ServiceClient> serviceClientLoader;

    pluginlib::ClassLoader<action::ActionServer> actionServerLoader;
    pluginlib::ClassLoader<action::ActionClient> actionClientLoader;

    std::map<const uint8_t, std::shared_ptr<service::ServiceServer> > serviceServers;
    std::map<const uint8_t, std::shared_ptr<service::ServiceClient> > serviceClients;

    std::map<const uint8_t, std::shared_ptr<action::ActionServer> > actionServers;
    std::map<const uint8_t, std::shared_ptr<action::ActionClient> > actionClients;

    rclcpp::Node::SharedPtr node;

    ConnectionBase &connection;

    rclcpp::CallbackGroup::SharedPtr serviceReentrantCallbackGroup;
    rclcpp::CallbackGroup::SharedPtr actionReentrantCallbackGroup;

    std::queue<std::unique_ptr<MessageBase> > toHandle;
    std::mutex toHandleMutex;
    std::condition_variable toHandleCV;
};

#endif //SERVICEACTION_MANAGER_HPP
