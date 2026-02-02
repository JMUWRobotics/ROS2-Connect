// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SERVER_HPP_
#define SERVER_HPP_

#include "common/thread.hpp"
#include "connect/logger.hpp"
#include "connectionManager.hpp"

#include <memory>

#include <boost/asio/io_context.hpp>
#include <boost/asio/io_context_strand.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <rclcpp/rclcpp.hpp>

class Server final : public Thread {
public:
    /**
     * Constructs a new Connect Server instance.
     * Can be parametrized e.g. by WebsocketServerConnection.
     *
     * @param node the node for which the rclcpp::publisher and rclcpp::subscriber should be created
     * @param executor the executor which executes this
     */
    explicit Server(const rclcpp::Node::SharedPtr &node, rclcpp::executors::MultiThreadedExecutor &executor);

    ~Server() override;

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

    boost::asio::ip::tcp::acceptor acceptor{ioc};

    ConnectionManager connectionManager{strand};

    std::shared_ptr<boost::asio::io_context> newSockIOC;

    bool isListening;

    rclcpp::executors::MultiThreadedExecutor &executor;

    /**
     * Start listening for new connections.
     * This will resolve the listening address and start the acceptor.
     *
     * @return true on success
     */
    bool startListening();

    /**
     * Stops listening for new connections by stopping the acceptor.
     *
     * @return true on success
     */
    bool stopListening();

    /**
     * Performs the sync accept for incoming tcp connections.
     * Every accepted connection will then be pushed to the connection manager
     * which might initiate additional connection upgrades / handshakes (websocket connections).
     */
    void asyncAccept();

    /**
     * Starts the io_context before the actual thread starts up
     *
     * @return true on success
     */
    bool onBeforeStartThread() override;

    /**
     * Stops the io_context, stops listening and closes all open connections independent of state
     */
    void onStopThread() override;

    /**
     * Spins the io_context
     */
    void run() override;
};

#endif /* SERVER_HPP_ */
