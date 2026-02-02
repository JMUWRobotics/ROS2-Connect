// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "serviceActionManager.hpp"
#include "global/globalConfig.hpp"
#include "connect/serviceActionMessage.hpp"

ServiceActionManager::ServiceActionManager(ConnectionBase &connection, const rclcpp::Node::SharedPtr &node) : logger("service-action-manager"), serviceServerLoader(PACKAGE_NAME, "service::ServiceServer"), serviceClientLoader(PACKAGE_NAME, "service::ServiceClient"), actionServerLoader(PACKAGE_NAME, "action::ActionServer"), actionClientLoader(PACKAGE_NAME, "action::ActionClient"), node(node), connection(connection) {
    this->serviceReentrantCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
    this->actionReentrantCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
}

ServiceActionManager::~ServiceActionManager() {
    // empty the toHandle queue
    while (!this->toHandle.empty()) this->toHandle.pop();
}

void ServiceActionManager::init() {
    if (!this->connection.getRemoteEndpoint().empty()) this->logger.setName("service-action-manager", this->connection.getRemoteEndpoint());

    const std::function<void(const std::shared_ptr<const std::vector<uint8_t>> &)> send = [weak = this->weak_from_this()](const std::shared_ptr<const std::vector<uint8_t>> &combinedVector) {
        if (const std::shared_ptr<ServiceActionManager> self = weak.lock()) {
            self->connection.send(combinedVector);
        }
    };

    for (Service_Action_t &server: GlobalConfig::serviceServer) {
        rclcpp::CallbackGroup::SharedPtr callbackGroup;
        if (server.useOwnThread) {
            if (server.allowSimultaneous) callbackGroup = this->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
            else callbackGroup = this->node->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
        } else callbackGroup = this->serviceReentrantCallbackGroup;

        const std::shared_ptr<service::ServiceServer> serverPlugin = this->serviceServerLoader.createSharedInstance(server.type);
        serverPlugin->init(server, this->node, callbackGroup, this->connection.getRemoteEndpoint(), send);
        this->serviceServers.insert(std::pair<const uint8_t, std::shared_ptr<service::ServiceServer> >(server.channel, serverPlugin));
    }

    for (Service_Action_t &client: GlobalConfig::serviceClient) {
        rclcpp::CallbackGroup::SharedPtr callbackGroup;
        if (client.useOwnThread) {
            if (client.allowSimultaneous) callbackGroup = this->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
            else callbackGroup = this->node->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
        } else callbackGroup = this->serviceReentrantCallbackGroup;

        const std::shared_ptr<service::ServiceClient> clientPlugin = this->serviceClientLoader.createSharedInstance(client.type);
        clientPlugin->init(client, this->node, callbackGroup, this->connection.getRemoteEndpoint(), send);
        this->serviceClients.insert(std::pair<const uint8_t, std::shared_ptr<service::ServiceClient> >(client.channel, clientPlugin));
    }

    for (Service_Action_t &server: GlobalConfig::actionServer) {
        rclcpp::CallbackGroup::SharedPtr callbackGroup;
        if (server.useOwnThread) {
            if (server.allowSimultaneous) callbackGroup = this->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
            else callbackGroup = this->node->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
        } else callbackGroup = this->actionReentrantCallbackGroup;

        const std::shared_ptr<action::ActionServer> serverPlugin = this->actionServerLoader.createSharedInstance(server.type);
        serverPlugin->init(server, this->node, callbackGroup, this->connection.getRemoteEndpoint(), send);
        this->actionServers.insert(std::pair<const uint8_t, std::shared_ptr<action::ActionServer> >(server.channel, serverPlugin));
    }

    for (Service_Action_t &client: GlobalConfig::actionClient) {
        rclcpp::CallbackGroup::SharedPtr callbackGroup;
        if (client.useOwnThread) {
            if (client.allowSimultaneous) callbackGroup = this->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
            else callbackGroup = this->node->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
        } else callbackGroup = this->actionReentrantCallbackGroup;

        const std::shared_ptr<action::ActionClient> clientPlugin = this->actionClientLoader.createSharedInstance(client.type);
        clientPlugin->init(client, this->node, callbackGroup, this->connection.getRemoteEndpoint(), send);
        this->actionClients.insert(std::pair<const uint8_t, std::shared_ptr<action::ActionClient> >(client.channel, clientPlugin));
    }
}

void ServiceActionManager::handle(std::unique_ptr<MessageBase> message) {
    std::unique_lock<std::mutex> lock(this->toHandleMutex);
    this->toHandle.push(std::move(message));
    this->toHandleCV.notify_all();
}

void ServiceActionManager::onStopThread() {
    // we need to wake up the thread so it can exit the run-loop
    this->toHandleCV.notify_all();

    // stop and destruct all clients
    while (!this->serviceClients.empty()) {
        const std::pair<const uint8_t, std::shared_ptr<service::ServiceClient> > &pair = *this->serviceClients.begin();
        pair.second->stop();
        this->serviceClients.erase(pair.first);
    }
    while (!this->actionClients.empty()) {
        const std::pair<const uint8_t, std::shared_ptr<action::ActionClient> > &pair = *this->actionClients.begin();
        pair.second->stop();
        this->actionClients.erase(pair.first);
    }
    // stop and destruct all servers
    while (!this->serviceServers.empty()) {
        const std::pair<const uint8_t, std::shared_ptr<service::ServiceServer> > &pair = *this->serviceServers.begin();
        pair.second->stop();
        this->serviceServers.erase(pair.first);
    }
    while (!this->actionServers.empty()) {
        const std::pair<const uint8_t, std::shared_ptr<action::ActionServer> > &pair = *this->actionServers.begin();
        pair.second->stop();
        this->actionServers.erase(pair.first);
    }
}

void ServiceActionManager::run() {
    while (!this->stopping) {
        // get a unique lock for the toHandle mutex
        std::unique_lock<std::mutex> lock(this->toHandleMutex);
        // release the lock and wait until notified. only wake up if:
        //  - the toHandle queue is not empty
        //  - or the thread is stopping
        this->toHandleCV.wait(lock, [this]() {
            return !this->toHandle.empty() || this->stopping;
        });
        // make sure to break early
        if (this->stopping) break;
        // make sure we have a message
        if (this->toHandle.empty()) continue;

        // get the next message to send and release the lock again
        std::unique_ptr<MessageBase> message = std::move(this->toHandle.front());
        this->toHandle.pop();
        lock.unlock();

        // try to convert the message into a ServiceActionMessage
        // this does implicitly ensure that we have a valid ServiceActionOpCode
        std::unique_ptr<ServiceActionMessage> serviceActionMessage = ServiceActionMessage::fromMessage(message);
        if (serviceActionMessage == nullptr) {
            RCLCPP_ERROR(this->logger.get(), "Received a message which does not hold valid service / action data, dropping it");
            // FIXME: dropping may not be the best idea, since the other party would either wait endlessly or until maxExecTime relapsed ... may be we should return an explicit error at this point
            continue;
        }

        // get the correct service / action server or client and move the message over
        // since the actual implementation of the server or client, this can do everything
        if (serviceActionMessage->isServiceActionOpCode(ServiceActionOpCode::SERVICE_TO_SERVER)) {
            // message to a service server
            const std::map<uint8_t, std::shared_ptr<service::ServiceServer> >::iterator it = this->serviceServers.find(serviceActionMessage->getServiceActionChannel());
            if (it == this->serviceServers.end() && !this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Received a message to handle for unknown service server channel %d, dropping it", serviceActionMessage->getServiceActionChannel());
            } else if (!this->stopping) {
                it->second->handle(std::move(serviceActionMessage));
            }
        } else if (serviceActionMessage->isServiceActionOpCode(ServiceActionOpCode::SERVICE_TO_CLIENT) || serviceActionMessage->isServiceActionOpCode(ServiceActionOpCode::SERVICE_TO_CLIENT_CANCEL)) {
            // message to a service client
            const std::map<uint8_t, std::shared_ptr<service::ServiceClient> >::iterator it = this->serviceClients.find(serviceActionMessage->getServiceActionChannel());
            if (it == this->serviceClients.end() && !this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Received a message to handle for unknown service client channel %d, dropping it", serviceActionMessage->getServiceActionChannel());
            } else if (!this->stopping) {
                it->second->handle(std::move(serviceActionMessage));
            }
        } else if (serviceActionMessage->isServiceActionOpCode(ServiceActionOpCode::ACTION_TO_CLIENT_GOAL) || serviceActionMessage->isServiceActionOpCode(ServiceActionOpCode::ACTION_TO_CLIENT_CANCEL)) {
            // message to an action client
            const std::map<uint8_t, std::shared_ptr<action::ActionClient> >::iterator it = this->actionClients.find(serviceActionMessage->getServiceActionChannel());
            if (it == this->actionClients.end() && !this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Received a message to handle for unknown action client channel %d, dropping it", serviceActionMessage->getServiceActionChannel());
            } else if (!this->stopping) {
                it->second->handle(std::move(serviceActionMessage));
            }
        } else {
            // message to an action server (since ServiceActionMessage::fromMessage(message) implicitly ensure that we have a valid ServiceActionOpCode)
            const std::map<uint8_t, std::shared_ptr<action::ActionServer> >::iterator it = this->actionServers.find(serviceActionMessage->getServiceActionChannel());
            if (it == this->actionServers.end() && !this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Received a message to handle for unknown action server channel %d, dropping it", serviceActionMessage->getServiceActionChannel());
            } else if (!this->stopping) {
                it->second->handle(std::move(serviceActionMessage));
            }
        }
    }
}
