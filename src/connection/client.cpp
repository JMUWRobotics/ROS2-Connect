// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "client.hpp"

#include "global/globalConfig.hpp"
#include "websocketConnection/client/websocketClientConnection.hpp"

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl.hpp>

#include <rclcpp/rclcpp.hpp>

Client::Client(const rclcpp::Node::SharedPtr &node, rclcpp::executors::MultiThreadedExecutor &executor) : node(node), isConnecting(false), executor(executor) {
}

Client::~Client() {
}

bool Client::connect() {
    if (this->isConnecting || this->connectionManager.isClientConnected()) return true;
    this->isConnecting = true;

    auto handleError = [=, this](const std::string &errorString, const boost::system::error_code &ec) {
        RCLCPP_ERROR(this->logger.get(), "%s %s", errorString.c_str(), ec.message().c_str());
        this->isConnecting = false;
        return false;
    };

    // set the connection close callback of the connection manager
    // which will shutdown this thread and in consequence rclcpp
    // we need to use a weak reference since otherwise this cannot be destroyed
    std::function<void()> callback = [weak = this->weak_from_this()]() {
        if (const std::shared_ptr<Client> self = weak.lock()) {
            self->stopThread();
        }
    };
    this->connectionManager.setConnectionCloseCallback(std::move(callback));

    boost::system::error_code ec;

    // resolve the host:port
    boost::asio::ip::tcp::resolver resolver(this->ioc);
    const boost::asio::ip::tcp::resolver::results_type res = resolver.resolve(GlobalConfig::host, GlobalConfig::port, ec);
    if (ec)
        return handleError("Unable to resolve connection address:", ec);

    const boost::asio::ip::tcp::endpoint endpoint = *(res.begin());

    // create a new socket and connect to the resolved endpoint
    const std::shared_ptr<boost::asio::io_context> newSockIOC = std::make_shared<boost::asio::io_context>();
    boost::asio::ip::tcp::socket sock{*newSockIOC};
    sock.connect(endpoint, ec);
    if (ec)
        return handleError("Error connection:", ec);

    // if we want to run with ssl, we need to initialize the ssl context, set up an ssl-stream and perform a ssl handshake
    if (GlobalConfig::ssl) {
        // create the ssl context and use the systems certificates
        boost::asio::ssl::context ctx{boost::asio::ssl::context::tlsv12_client};
        ctx.set_default_verify_paths();

        // create the ssl stream which wraps the tcp-ip socket connection
        // let the stream verify the ssl certificates of the host using rfc2818 verification
        boost::asio::ssl::stream<boost::asio::ip::tcp::socket> ssl{std::move(sock), ctx};
        ssl.set_verify_mode(boost::asio::ssl::verify_peer);
        ssl.set_verify_callback(boost::asio::ssl::rfc2818_verification(GlobalConfig::host));

        // set the SNI Hostname extension
        if (!SSL_set_tlsext_host_name(ssl.native_handle(), GlobalConfig::host.c_str())) {
            return handleError("Failed to set SNI Hostname:", make_error_code(boost::system::errc::io_error));
        }

        // perform the ssl handshake
        ssl.handshake(boost::asio::ssl::stream_base::client, ec);
        if (ec)
            return handleError("SSH handshake failed:", ec);

        // start the connection -> websocket handshake
        RCLCPP_INFO(this->logger.get(), "Connected to %s:%s with SSL", GlobalConfig::host.c_str(), GlobalConfig::port.c_str());
        this->connectionManager.startConnection(std::make_shared<WebsocketClientConnection<boost::asio::ssl::stream<boost::asio::ip::tcp::socket> > >(std::move(ssl), this->connectionManager, std::move(newSockIOC), this->node));
    }
    // we run without ssl
    else {
        // start the connection -> websocket handshake
        RCLCPP_INFO(this->logger.get(), "Connected to %s:%s", GlobalConfig::host.c_str(), GlobalConfig::port.c_str());
        this->connectionManager.startConnection(std::make_shared<WebsocketClientConnection<boost::asio::ip::tcp::socket> >(std::move(sock), this->connectionManager, std::move(newSockIOC), this->node));
    }

    this->isConnecting = false;

    return true;
}

bool Client::onBeforeStartThread() {
    this->ioc.restart();
    return true; // we do not run connect() at this point since we want to evaluate its return value
}

void Client::onStopThread() {
    this->ioc.stop();
    this->connectionManager.stopAllConnections();
    // if this shuts down, e.g. after the (last) connection was disconnected
    // we want the entire application to shut down, hence we cancel the executor
    // we do not call rclcpp::shutdown() since we want the ros2 context to be kept alive
    this->executor.cancel();
}

void Client::run() {
    // try to connect
    if (!this->connect()) {
        // stop the ioc, close all connection, shutdown ros
        this->stopThread();
        return;
    }

    // create a work guard to keep the ioc alive without it waking up the cpu
    boost::asio::executor_work_guard<boost::asio::io_context::basic_executor_type<std::allocator<void>, 0> > workGuard = boost::asio::make_work_guard(this->ioc);

    try {
        this->ioc.run();
    } catch (...) {
        const std::exception_ptr p = std::current_exception();
        RCLCPP_ERROR(this->logger.get(), "I/O context unexpectedly threw an exception: %s", p ? p.__cxa_exception_type()->name() : "null");
    }

    RCLCPP_ERROR(this->logger.get(), "Connection lost");
    workGuard.reset();
    this->stopThread();
}
