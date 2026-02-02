// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef WEBSOCKETCONNECTIONBASE_HPP_
#define WEBSOCKETCONNECTIONBASE_HPP_

#include "websocketConnection/base/websocketConnectionBase.hpp"
#include "connection/connectionManager.hpp"
#include "connect/authentication.hpp"

#include <string>
#include <memory>
#include <regex>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/deadline_timer.hpp>

#include <pluginlib/class_loader.hpp>

// this regex matches any ip inside of a string
#define IP_REGEX R"(\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b)"

class WebsocketServerConnection final : public WebsocketConnectionBase<boost::asio::ip::tcp::socket> {
public:
    /**
     * Constructs a new websocket server connection.
     * The io_context should be a newly created io_context so this connection can run in an own thread.
     * The socket should be an already open tcp ip socket in need to be upgraded.
     *
     * @param socket the tcp ip socket which is already opened and needs to be upgraded (handshake) to establish the actual websocket connection
     * @param connectionManager the connection manager by which this connection is manager
     * @param ioc the io_context in which the connection runs
     * @param node the node for which the rclcpp::publisher and rclcpp::subscriber should be created
     */
    explicit WebsocketServerConnection(boost::asio::ip::tcp::socket socket, ConnectionManager &connectionManager, std::shared_ptr<boost::asio::io_context> ioc, const rclcpp::Node::SharedPtr &node);

    ~WebsocketServerConnection() override;

protected:
    /**
     * Run method of underlying thread.
     * This performs the websocket connection upgrade (handshake) and spins the io_context
     * to perform the sending and receiving.
     */
    void run() override;

    /**
     * Callback method which is called after
     * the websocket connection upgrade was performed
     *
     * Therefore this is the starting point which is called as soon as
     * this accepts data from the client as well as sends data to the client.
     */
    void onConnected() override;

    /**
     * Processes a message
     *
     * @param message the message
     */
    void processMessage(std::unique_ptr<MessageBase> message) override;

private:
    boost::beast::http::message<true, boost::beast::http::basic_string_body<char> > websocketHandshakeRequest{};

    boost::asio::steady_timer authenticationCheckTimer{*ioc};
    boost::asio::steady_timer authenticationEndTimer{*ioc};

    std::shared_ptr<authentication::Authentication> authentication{nullptr};

    pluginlib::ClassLoader<authentication::Authentication> authenticationLoader;

    /**
     * Validate a received user key for being linked to a valid authentication information
     */
    void validateUserKeyForAuthentication(const std::string &userKey);

    /**
     * Callback method which is called as soon as the authentication information check timer is due
     * and a valid authentication must exist.
     */
    void validAuthenticationCheckCallback();

    /**
     * Callback method which is called as soon as the authentication has ended.
     */
    void authenticationEndCallback();
};

#endif /* WEBSOCKETCONNECTIONBASE_HPP_ */
