// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "websocketServerConnection.hpp"

#include <boost/asio/bind_executor.hpp>
#include <boost/beast/core.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <regex>

#include "global/globalConfig.hpp"
#include "message/vectorMessage.hpp"

WebsocketServerConnection::WebsocketServerConnection(boost::asio::ip::tcp::socket socket, ConnectionManager &connectionManager, std::shared_ptr<boost::asio::io_context> ioc, const rclcpp::Node::SharedPtr &node) : WebsocketConnectionBase<boost::asio::ip::tcp::socket>(std::move(socket), connectionManager, std::move(ioc), std::move(node)), authenticationCheckTimer((*this->ioc)), authenticationEndTimer((*this->ioc)), authentication(nullptr), authenticationLoader(PACKAGE_NAME, "authentication::Authentication") {
    // rename logger to use an empty remote-endpoint address
    this->logger.setName("websocket-connection", "");

    // the following lines configure the websocket::stream
    // it is important to do this before the stream is actually opened
    // since otherwise undefined behavior results

    // set how the timeouts of the websocket
    //  - the handshake should timeout after 30sec
    //  - the connection should be considered idle after 30sec
    //  - send automatic keep alive pings after the connection was considered idle
    this->ws.set_option(boost::beast::websocket::stream_base::timeout(
        std::chrono::seconds(30),
        std::chrono::seconds(30),
        true));

    if (!GlobalConfig::authenticationPlugin.empty()) {
        this->authentication = this->authenticationLoader.createSharedInstance(GlobalConfig::authenticationPlugin);
    }
}

WebsocketServerConnection::~WebsocketServerConnection() {
    this->authenticationCheckTimer.cancel();
    this->authenticationEndTimer.cancel();

    if (this->authentication != nullptr) {
        this->authentication.reset();
        this->authentication = nullptr;
    }
}

void WebsocketServerConnection::run() {
    // clear every field of the websocketHandshakeRequest container
    this->websocketHandshakeRequest.clear();

    // clearing the read buffer setting its readable data to zero
    this->readBuffer.clear();

    // we manually intercept the http request for the websocket handshake
    // if we do this, we can extract the X-Forwarded-For http header which is set
    // by the Apache Reverse Proxy and which contains the actual IP of the client
    boost::beast::http::async_read(this->ws.next_layer(), this->readBuffer, this->websocketHandshakeRequest,
                                   boost::asio::bind_executor(this->strand,
                                                              [this](const boost::beast::error_code &ec, std::size_t /* bytes_transferred */) {
                                                                  if (ec) {
                                                                      RCLCPP_ERROR(this->logger.get(), "Error while reading websocket handshake http request: %s, closing connection", ec.message().c_str());
                                                                      this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::internal_error, "Websocket handshake failed");
                                                                  } else {
                                                                      // no we try to accept the intercepted http request for the websocket handshake
                                                                      this->ws.async_accept(this->websocketHandshakeRequest,
                                                                                            boost::asio::bind_executor(this->strand,
                                                                                                                       [this](const boost::system::error_code &ecc) {
                                                                                                                           if (ecc) {
                                                                                                                               RCLCPP_ERROR(this->logger.get(), "Error while performing handshake: %s, closing connection", ecc.message().c_str());
                                                                                                                               this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::internal_error, "Websocket handshake failed");
                                                                                                                           } else {
                                                                                                                               // try to extract the X-Forwarded-For header
                                                                                                                               bool hasXForwardedForHeader = false;
                                                                                                                               const boost::beast::http::basic_fields<std::allocator<char> >::const_iterator it = this->websocketHandshakeRequest.find("X-Forwarded-For");
                                                                                                                               if (it != this->websocketHandshakeRequest.end()) {
                                                                                                                                   // for some reason, the header contains a line-break and non ascii symbols when print out
                                                                                                                                   // hence we need to extract actual ip using a regex
                                                                                                                                   const std::regex ipRegex(IP_REGEX);
                                                                                                                                   std::smatch match;
                                                                                                                                   const std::string xForwardedFor(it->value().data());

                                                                                                                                   if (std::regex_search(xForwardedFor, match, ipRegex)) {
                                                                                                                                       this->remoteEndpoint = match.str();
                                                                                                                                       this->logger.setName("websocket-connection", this->remoteEndpoint);
                                                                                                                                       RCLCPP_INFO(this->logger.get(), "Websocket connection successfully established from %s", this->remoteEndpoint.c_str());
                                                                                                                                       hasXForwardedForHeader = true;
                                                                                                                                   }
                                                                                                                               }
                                                                                                                               // if there is no X-Forwarded-For header, try to extract the remote_endpoint of the tcp ip socket
                                                                                                                               // if there is a reverse proxy or other relay inbetween, this may not return the actual client ip
                                                                                                                               if (!hasXForwardedForHeader) {
                                                                                                                                   boost::system::error_code err;
                                                                                                                                   const boost::asio::ip::tcp::endpoint endpoint = boost::beast::get_lowest_layer(this->ws).remote_endpoint(err);

                                                                                                                                   if (err)
                                                                                                                                       RCLCPP_WARN(this->logger.get(), "Websocket connection successfully established, however endpoint could not be determined because of the error: %s", err.message().c_str());
                                                                                                                                   else {
                                                                                                                                       this->remoteEndpoint = endpoint.address().to_string();
                                                                                                                                       this->logger.setName("websocket-connection", this->remoteEndpoint);
                                                                                                                                       RCLCPP_INFO(this->logger.get(), "Websocket connection successfully established from %s", this->remoteEndpoint.c_str());
                                                                                                                                   }
                                                                                                                               }

                                                                                                                               // we are connected
                                                                                                                               this->onConnected();

                                                                                                                               // we do not call onAuthorized at this point
                                                                                                                               // but call it either in onConnect OR after
                                                                                                                               // an authentication information was determined

                                                                                                                               // begin to call async read
                                                                                                                               this->asyncRead();
                                                                                                                           }
                                                                                                                       }));
                                                                  }
                                                              }));

    try {
        this->ioc->run();
    } catch (...) {
        const std::exception_ptr p = std::current_exception();
        RCLCPP_ERROR(this->logger.get(), "I/O context unexpectedly threw an exception: %s", p ? p.__cxa_exception_type()->name() : "null");
        // we terminate the connection at this point since this won't do anything anymore
        this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::internal_error, "I/O context unexpectedly threw an exception");
    }
}

void WebsocketServerConnection::onConnected() {
    // call the base which inits the publisher manager
    WebsocketConnectionBase<boost::asio::ip::tcp::socket>::onConnected();

    // we do not perform an authentication check therefore we directly can call onConnected
    if (GlobalConfig::authenticationOmit) {
        this->onAuthorized();
    }
    // start the timer to ensure the client sends its user key after a certain time
    // after the token was received and determined to be valid
    // the authenticationEndTimer must be started and onConnect must be called
    // before that any data received by the client must be discarded and no data send!
    else {
        this->authenticationCheckTimer.expires_after(std::chrono::seconds(GlobalConfig::authenticationTimeout));
        this->authenticationCheckTimer.async_wait(boost::bind(&WebsocketServerConnection::validAuthenticationCheckCallback, this));
    }
}

void WebsocketServerConnection::processMessage(std::unique_ptr<MessageBase> message) {
    // if the message is of op-code AUTHENTICATION, the message must be routed to the validation implementation
    if (message->isOpCode(OpCode::AUTHENTICATION)) return this->validateUserKeyForAuthentication(message->toString());
    // every other message is handled by the base
    else
        WebsocketConnectionBase::processMessage(std::move(message));
}

void WebsocketServerConnection::validateUserKeyForAuthentication(const std::string &userKey) {
    if (GlobalConfig::authenticationOmit) {
        // now construct a response to the client to notify it about the connection being authorized
        // the content should be an empty string
        const std::shared_ptr<const std::vector<uint8_t> > combinedVector = VectorMessage::forOpCodeAuthentication("")->getCombinedVector();
        this->send(combinedVector);

        RCLCPP_INFO(this->logger.get(), "Received message on channel AUTHENTICATION, dropping message");
        return;
    } else if (this->authentication == nullptr) {
        RCLCPP_ERROR(this->logger.get(), "Authentication should not be omitted, but no authentication plugin is available. Closing the connection");
        this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::going_away, "Authentication cannot be validated");
        return;
    }

    RCLCPP_INFO(this->logger.get(), "Received message on channel AUTHENTICATION, validating user key");
    this->authentication->getAuthenticationFromUserKey(
        this->logger,
        GlobalConfig::authenticationEndpoint,
        GlobalConfig::authenticationHost,
        GlobalConfig::authenticationPort,
        GlobalConfig::authenticationSsl,
        userKey,
        GlobalConfig::nodeNamespace,
        GlobalConfig::nodeDomainId);

    // the request did not resolve to an authentication information -> close to connection
    if (!this->authentication->isValid()) {
        RCLCPP_WARN(this->logger.get(), "No valid user key received. Closing the connection");
        this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::going_away, "No valid authentication");
    }
    // the request did resolve to an authentication information -> connection is authorized
    else {
        // now the connection is authorized
        this->onAuthorized();

        if (this->authentication->hasEnd()) {
            // start a timer to close the connection on authentication end
            const boost::posix_time::time_duration duration = this->authentication->getEnd() - boost::posix_time::second_clock::universal_time();
            this->authenticationEndTimer.expires_after(std::chrono::milliseconds(duration.total_milliseconds()));
            this->authenticationEndTimer.async_wait(boost::bind(&WebsocketServerConnection::authenticationEndCallback, this));

            // now construct a response to the client to notify it about the connection being authorized
            // the content should be the end time of the authentication
            const std::shared_ptr<const std::vector<uint8_t> > combinedVector = VectorMessage::forOpCodeAuthentication(boost::posix_time::to_iso_string(this->authentication->getEnd()))->getCombinedVector();
            this->send(combinedVector);

            RCLCPP_INFO(this->logger.get(), "Valid authentication for robot with namespace %s in domain %zu from %s until %s (UTC) received.", GlobalConfig::nodeNamespace.c_str(), GlobalConfig::nodeDomainId, this->authentication->getUser().c_str(), to_simple_string(this->authentication->getEnd()).c_str());
        } else {
            // now construct a response to the client to notify it about the connection being authorized
            // the content should be an empty string
            const std::shared_ptr<const std::vector<uint8_t> > combinedVector = VectorMessage::forOpCodeAuthentication("")->getCombinedVector();
            this->send(combinedVector);

            RCLCPP_INFO(this->logger.get(), "Valid authentication for robot with namespace %s in domain %zu from %s received.", GlobalConfig::nodeNamespace.c_str(), GlobalConfig::nodeDomainId, this->authentication->getUser().c_str());
        }
    }
}

void WebsocketServerConnection::authenticationEndCallback() {
    RCLCPP_INFO(this->logger.get(), "Authentication ended. Closing the connection");
    this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::going_away, "Authentication ended");
}

void WebsocketServerConnection::validAuthenticationCheckCallback() {
    if (!this->authentication->isValid()) {
        RCLCPP_WARN(this->logger.get(), "No valid user key received after %ds. Closing the connection", GlobalConfig::authenticationTimeout);
        this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::going_away, "No valid authentication");
    }
}
