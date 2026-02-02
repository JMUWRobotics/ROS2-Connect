// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "server.hpp"
#include "global/globalConfig.hpp"
#include "websocketConnection/server/websocketServerConnection.hpp"

#include <boost/asio/bind_executor.hpp>

#include <rclcpp/rclcpp.hpp>

Server::Server(const rclcpp::Node::SharedPtr &node, rclcpp::executors::MultiThreadedExecutor &executor) : node(node), isListening(false), executor(executor) {
}

Server::~Server() {
}

bool Server::startListening() {
    auto handleError = [=, this](const std::string &errorString, const boost::system::error_code &ec) {
        if (this->acceptor.is_open()) {
            boost::system::error_code err;
            this->acceptor.close(err);
        }
        RCLCPP_ERROR(this->logger.get(), "%s %s", errorString.c_str(), ec.message().c_str());
        return false;
    };

    boost::system::error_code ec;
    // Open the acceptor with the option to reuse the address (i.e. SO_REUSEADDR).
    boost::asio::ip::tcp::resolver resolver(this->ioc);
    const boost::asio::ip::tcp::resolver::results_type res = resolver.resolve(GlobalConfig::host, GlobalConfig::port, ec);
    if (ec)
        return handleError("Unable to resolve listening address:", ec);
    const boost::asio::ip::tcp::endpoint endpoint = *(res.begin());

    this->acceptor.open(endpoint.protocol(), ec);
    if (ec)
        return handleError("Unable to open acceptor:", ec);

    this->acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true), ec);
    if (ec)
        return handleError("Unable to set option 'reuse address':", ec);

    this->acceptor.bind(endpoint, ec);
    if (ec)
        return handleError("Unable to bind:", ec);

    this->acceptor.listen(boost::asio::socket_base::max_listen_connections, ec);
    if (ec)
        return handleError("Unable to listen:", ec);

    this->isListening = true;
    RCLCPP_INFO(this->logger.get(), "Listening on %s:%s", GlobalConfig::host.c_str(), GlobalConfig::port.c_str());

    return true;
}

bool Server::stopListening() {
    this->isListening = false;
    boost::system::error_code ec;
    this->acceptor.close(ec);
    if (ec)
        RCLCPP_ERROR(this->logger.get(), "Error on closing acceptor: %s", ec.message().c_str());
    return !ec;
}

void Server::asyncAccept() {
    // create a new io_context for the connection, such that all I/O for the connection can be done by an individual thread
    this->newSockIOC = std::make_shared<boost::asio::io_context>();

    this->acceptor.async_accept(*this->newSockIOC,
                                boost::asio::bind_executor(this->strand,
                                                           [this](const boost::system::error_code &ec, boost::asio::ip::tcp::socket socket) {
                                                               // Check whether the server was stopped by a signal before this
                                                               // completion handler had a chance to run.
                                                               if (!this->acceptor.is_open())
                                                                   return;

                                                               if (ec) {
                                                                   RCLCPP_ERROR(this->logger.get(), "Error on accept: %s", ec.message().c_str());

                                                                   // do not call async accept if the ioc stopped running TODO: make sure that ECANCELED is always the ec for when the ioc has stopped running
                                                                   // since otherwise this would call asyncAccept while the re-starting of the ioc also calls asyncAccept
                                                                   // this would result in two calls to this->acceptor.async_accept(...) which would run concurrently in the same ioc
                                                                   // which would result in undefined behaviour or even a deadlock
                                                                   if (ec != boost::system::errc::operation_canceled) this->asyncAccept();
                                                               } else {
                                                                   this->connectionManager.startConnection(std::make_shared<WebsocketServerConnection>(std::move(socket), this->connectionManager, std::move(this->newSockIOC), this->node));
                                                                   this->asyncAccept();
                                                               }
                                                           }
                                ));
}

bool Server::onBeforeStartThread() {
    this->ioc.restart();
    return this->startListening();
}

void Server::onStopThread() {
    this->ioc.stop();
    this->stopListening();
    this->connectionManager.stopAllConnections();
    // if this shuts down, e.g. after the (last) connection was disconnected
    // we want the entire application to shut down, hence we cancel the executor
    // we do not call rclcpp::shutdown() since we want the ros2 context to be kept alive
    this->executor.cancel();
}

void Server::run() {
    while (!this->stopping) {
        if (!this->isListening) {
            while (!this->stopping) {
                if (this->startListening())
                    break;
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }

            if (this->stopping)
                break;
        }

        this->asyncAccept();

        try {
            this->ioc.run();
            if (this->stopping)
                break;
        } catch (...) {
            const std::exception_ptr p = std::current_exception();
            RCLCPP_ERROR(this->logger.get(), "I/O context unexpectedly threw an exception: %s", p ? p.__cxa_exception_type()->name() : "null");
        }

        RCLCPP_ERROR(this->logger.get(), "Listening socket lost");
        this->stopListening();
        this->ioc.restart();
    }
}
