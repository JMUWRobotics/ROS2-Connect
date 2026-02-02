// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef WEBSOCKETCLIENTCONNECTIONBASE_HPP
#define WEBSOCKETCLIENTCONNECTIONBASE_HPP

#include "websocketConnection/base/websocketConnectionBase.hpp"
#include "connection/connectionManager.hpp"
#include "global/globalConfig.hpp"
#include "message/vectorMessage.hpp"

#include <string>
#include <memory>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/date_time/posix_time/time_parsers.hpp>

#include <rclcpp/rclcpp.hpp>

template<class T>
class WebsocketClientConnection final : public WebsocketConnectionBase<T> {
public:
    /**
     * Constructs a new websocket client connection.
     * The io_context should be a newly created io_context so this connection can run in an own thread.
     * The socket should be an already open tcp ip socket in need to be upgraded.
     *
     * @param socket the tcp ip socket or ssl stream which is already opened and needs to be upgraded (handshake) to establish the actual websocket connection
     * @param connectionManager the connection manager by which this connection is manager
     * @param ioc the io_context in which the connection runs
     * @param node the node for which the rclcpp::publisher and rclcpp::subscriber should be created
     */
    explicit WebsocketClientConnection(T socket, ConnectionManager &connectionManager, std::shared_ptr<boost::asio::io_context> ioc, const rclcpp::Node::SharedPtr &node);

    ~WebsocketClientConnection() override;

protected:
    /**
     * Callback method which is called after
     * - the websocket connection upgrade was performed
     * - in case of server, only after the authentication was validated
     *
     * Therefore this is the starting point which is called as soon as
     * this accepts data from the client as well as sends data to the client.
     */
    void onConnected() override;

    /**
     * Run method of underlying thread.
     * This performs the websocket connection upgrade (handshake) and spins the io_context
     * to perform the sending and receiving.
     */
    void run() override;

    /**
     * Processes a message
     *
     * @param message the message
     */
    void processMessage(std::unique_ptr<MessageBase> message) override;
};

template<class T>
WebsocketClientConnection<T>::WebsocketClientConnection(T socket, ConnectionManager &connectionManager, std::shared_ptr<boost::asio::io_context> ioc, const rclcpp::Node::SharedPtr &node) : WebsocketConnectionBase<T>(std::move(socket), connectionManager, std::move(ioc), std::move(node)) {
    // the following lines configure the websocket::stream
    // it is important to do this before the stream is actually opened
    // since otherwise undefined behavior results

    // set how the timeouts of the websocket
    //  - the handshake should timeout after 30sec
    //  - the connection should never be considered idle
    //  - do not send automatic keep alive pings after the connection was considered idle
    this->ws.set_option(boost::beast::websocket::stream_base::timeout(
        std::chrono::seconds(30),
        boost::beast::websocket::stream_base::none(),
        false
    ));
}

template<class T>
WebsocketClientConnection<T>::~WebsocketClientConnection() {
}

template<class T>
void WebsocketClientConnection<T>::onConnected() {
    // call the base which inits the publisher manager
    WebsocketConnectionBase<T>::onConnected();

    // create an op-code AUTHENTICATION message holding the user key
    std::shared_ptr<const std::vector<uint8_t> > combinedVector = VectorMessage::forOpCodeAuthentication(GlobalConfig::userKey)->getCombinedVector();
    // and send it to the server
    this->send(combinedVector);
}

template<class T>
void WebsocketClientConnection<T>::processMessage(std::unique_ptr<MessageBase> message) {
    // if the message is of op-code AUTHENTICATION, the server has successfully authorized the connection
    if (message->isOpCode(OpCode::AUTHENTICATION)) {
        const std::string value = message->toString();
        if (value.empty()) RCLCPP_INFO(this->logger.get(), "Connection authenticated, authentication check was omitted");
        else {
            try {
                // get end time from message
                const boost::posix_time::ptime end = boost::posix_time::from_iso_string(value);
                // convert utc into local time
                const std::time_t tt = (end - boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1))).total_seconds();
                const std::tm *localTime = std::localtime(&tt);
                // format it
                std::stringstream ss;
                ss << std::put_time(localTime, "%Y-%m-%d %H:%M:%S");
                // log it
                RCLCPP_INFO(this->logger.get(), "Connection authenticated, authentication valid until %s", ss.str().c_str());
            } catch (...) {
                RCLCPP_INFO(this->logger.get(), "Connection authenticated, authentication end could not be pared");
            }

        }
        // every other message is handled by the base
    } else WebsocketConnectionBase<T>::processMessage(std::move(message));
}

template<class T>
void WebsocketClientConnection<T>::run() {
    this->ws.async_handshake(
        GlobalConfig::host,
        GlobalConfig::path,
        boost::asio::bind_executor(this->strand,
                                   [this](const boost::system::error_code &ec) {
                                       if (ec) {
                                           RCLCPP_ERROR(this->logger.get(), "Error while performing handshake: %s, closing connection", ec.message().c_str());
                                           this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::internal_error, "Websocket handshake failed");
                                       } else {
                                           RCLCPP_INFO(this->logger.get(), "Websocket connection successfully established to %s%s", GlobalConfig::host.c_str(), GlobalConfig::path.c_str());

                                           // we are connected
                                           this->onConnected();

                                           // we are also authorized and allow messages to be received
                                           this->onAuthorized();

                                           // begin to call async rea
                                           this->asyncRead();
                                       }
                                   }
        )
    );

    try {
        this->ioc->run();
    } catch (...) {
        const std::exception_ptr p = std::current_exception();
        RCLCPP_ERROR(this->logger.get(), "I/O context unexpectedly threw an exception: %s", p ? p.__cxa_exception_type()->name() : "null");
        // we terminate the connection at this point since this won't do anything anymore
        this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::internal_error, "I/O context unexpectedly threw an exception");
    }
}

#endif //WEBSOCKETCLIENTCONNECTIONBASE_HPP
