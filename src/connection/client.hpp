// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef CLIENT_HPP
#define CLIENT_HPP

#include "common/thread.hpp"
#include "connect/logger.hpp"
#include "connectionManager.hpp"

#include <boost/asio/io_context.hpp>
#include <boost/asio/io_context_strand.hpp>

#include <rclcpp/rclcpp.hpp>

class Client final : public Thread, public std::enable_shared_from_this<Client> {
public:
    /**
     * Constructs a new Connect Client instance.
     * Can be parametrized e.g. by a WebsocketClientConnection.
     *
     * @param node the node for which the rclcpp::publisher and rclcpp::subscriber should be created
     * @param executor the executor which executes this
     */
    explicit Client(const rclcpp::Node::SharedPtr &node, rclcpp::executors::MultiThreadedExecutor &executor);

    ~Client() override;

    /**
     * Returns the connection manager which manages all connections of this Connect Server instance
     *
     * @return connection manager managing all (open) connections
     */
    ConnectionManager &getConnectionManager() {
        return this->connectionManager;
    }

private:
    Logger logger;

    rclcpp::Node::SharedPtr node;

    boost::asio::io_context ioc{};

    boost::asio::io_context::strand strand{ioc};

    ConnectionManager connectionManager{strand};

    bool isConnecting;

    rclcpp::executors::MultiThreadedExecutor &executor;

    /**
     * Opens the tcp connection to the given endpoint and passes the opened connection on to the connection manager
     *
     * @return true on success
     */
    bool connect();

    /**
     * Starts the io_context before the actual thread starts up
     *
     * @return true on success
     */
    bool onBeforeStartThread() override;

    /**
     * Stops the io_context, closes all connection and shuts down rclcpp
     */
    void onStopThread() override;

    /**
     * Spins the io_context
     */
    void run() override;
};

#endif //CLIENT_HPP
