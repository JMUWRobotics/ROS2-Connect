// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef WEBSOCKETCONNECTIONBASE_HPP
#define WEBSOCKETCONNECTIONBASE_HPP

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/websocket/stream.hpp>
#include <rclcpp/rclcpp.hpp>

#include "connect/logger.hpp"
#include "connect/messageBase.hpp"
#include "connection/base/connectionBase.hpp"
#include "connection/connectionManager.hpp"
#include "global/globalConfig.hpp"
#include "message/rclMessage.hpp"
#include "publisher/publisherManager.hpp"
#include "serviceAction/serviceActionManager.hpp"
#include "subscriber/subscriberManager.hpp"

// Helper trait to detect if a type is a boost::asio::ssl::stream specialization.
template <typename T>
struct is_boost_ssl_stream : std::false_type {};

template <typename Stream>
struct is_boost_ssl_stream<boost::asio::ssl::stream<Stream>> : std::true_type {};

// Helper alias for simple usage
template <typename T>
constexpr bool is_boost_ssl_stream_v = is_boost_ssl_stream<T>::value;

template <class T>
class WebsocketConnectionBase : public ConnectionBase {
   public:
    /**
     * Constructs a new websocket connection base.
     * The io_context should be a newly created io_context so this connection can run in an own thread.
     * The socket should be an already open tcp ip socket in need to be upgraded.
     *
     * @param socket the tcp ip socket which is already opened and needs to be upgraded (handshake) to establish the actual websocket connection
     * @param connectionManager the connection manager by which this connection is manager
     * @param ioc the io_context in which the connection runs
     * @param node the node for which the rclcpp::publisher and rclcpp::subscriber should be created
     */
    WebsocketConnectionBase(T socket, ConnectionManager &connectionManager, std::shared_ptr<boost::asio::io_context> ioc, const rclcpp::Node::SharedPtr &node);

    ~WebsocketConnectionBase() override;

   protected:
    Logger logger;

    rclcpp::Node::SharedPtr node;

    boost::beast::websocket::stream<T> ws;

    boost::beast::flat_buffer readBuffer;

    ConnectionManager &connectionManager;

    PublisherManager publisherManager;

    SubscriberManager subscriberManager;

    std::shared_ptr<ServiceActionManager> serviceActionManager;  // this must be wrapped inside a shared_ptr since we want to create weak_ptr on it for callbacks

    /**
     * Start the connection and therefore also the underlying thread.
     * Everything needed to be configured e.g. a boost::beast::websocket::stream must be done before this is called or while this called.
     *
     * @return true if start was successful
     */
    bool start() override;

    /**
     * Closes the connection and underlying socket
     */
    void close() override;

    /**
     * Closes the connection and underlying socket
     *
     * @param cc the close code
     * @param cr the close reason
     */
    void close(boost::beast::websocket::close_code cc, const std::string &cr) override;

    /**
     * Callback method which is called after
     * the websocket connection upgrade was performed
     *
     * Therefore this is the starting point which is called as soon as
     * this accepts data from the client as well as sends data to the client.
     */
    virtual void onConnected() {
    }

    /**
     * Callback method which is called after
     * the websocket connection was authorized.
     *
     * Therefore this is the starting point which is called as soon as this
     * accepts and sends data from or to *the other side*.
     */
    virtual void onAuthorized();

    /**
     * Called right after the underlying thread is stopped by setting the stopping flag to true.
     *
     * This can be used to clean up some stuff and stop other threads, services, etc.
     */
    void onStopThread() override;

    /**
     * Processes a message
     *
     * @param message the message
     */
    virtual void processMessage(std::unique_ptr<MessageBase> message);

    /**
     * Performs the async write of any data which is in the sendingQueue
     */
    void asyncWrite() override;

    /**
     * Performs the async read and processes them
     */
    void asyncRead() override;
};

template <class T>
WebsocketConnectionBase<T>::WebsocketConnectionBase(T socket, ConnectionManager &connectionManager, std::shared_ptr<boost::asio::io_context> ioc, const rclcpp::Node::SharedPtr &node) : ConnectionBase(std::move(ioc)), logger("websocket-connection"), node(node), ws(std::move(socket)), connectionManager(connectionManager), publisherManager(*this), subscriberManager(*this, node), serviceActionManager(std::make_shared<ServiceActionManager>(*this, node)) {
    // the following lines configure the websocket::stream
    // it is important to do this before the stream is actually opened
    // since otherwise undefined behavior results

    // configure websocket::stream to use binary protocol allowing to send arbitrary binary data
    this->ws.binary(true);

    // disable Nagle's algorithm (TCP_NO_DELAY)
    boost::beast::get_lowest_layer(this->ws).set_option(boost::asio::ip::tcp::no_delay(true));

    // configure fragmentation of boost beast
    // if fragmentation is off, each message is send as ONE message ... this enhances performance but may lead to more memory being used
    // if fragmentation is on, boost uses the write buffer so send messages ... if a message is larger than the write buffer, fragmentation will occur
    this->ws.auto_fragment(GlobalConfig::fragmentation);
    this->ws.write_buffer_bytes(GlobalConfig::fragmentationSize);

    // disable the per message deflate algorithm
    boost::beast::websocket::permessage_deflate perMessageDeflate;
    perMessageDeflate.client_enable = false;
    perMessageDeflate.server_enable = false;
    this->ws.set_option(perMessageDeflate);

    // configure the max message size which can be read during an async_read
    // this size is therefore the internal buffer size used by boost beast
    // this sice includes the header and body of a message
    // if a message exceeds the size of this, async_read fails with an ec indicating this error
    // fragmentation can not surpass this!
    // hence, this value should be set to a large enough size (default is 16MB)
    this->ws.read_message_max(GlobalConfig::maxMessageSize);

    // we do also resize the read buffer which is a boost::beast::flat_buffer to an initial size
    // the flat_buffer manages its own allocated memory and two pointers, one pointing to the start of read-able data,
    // one pointing to the end of read-able data
    // hence the flat_buffer only neads to re-allocate if it runs out of internal memory, but esp. not after a clear() which only reset the internal pointers
    this->readBuffer.reserve(4096);
}

template <class T>
WebsocketConnectionBase<T>::~WebsocketConnectionBase() {
}

template <class T>
bool WebsocketConnectionBase<T>::start() {
    const bool res = this->startThread("WSCON");
    if (!res)
        RCLCPP_ERROR(this->logger.get(), "Error while starting thread for websocket connection");
    return res;
}

template <class T>
void WebsocketConnectionBase<T>::close() {
    this->close(boost::beast::websocket::close_code::none, "");
}

template <class T>
void WebsocketConnectionBase<T>::close(const boost::beast::websocket::close_code cc, const std::string &cr) {
    boost::system::error_code ec;

    // send a notification about the closing
    // this is necessary since when ROS2 Connect is used with the server being placed behind a reverse proxy
    // with the client being connected using SSL/TLS to the reverse proxy and the server closing the connection
    // the ssl context will be destroyed before the closing frame reaches the client
    // in that case, the client will never see the closing reason
    // for this case, we se send a notification to make sure the client is informed about the closing reason
    const std::shared_ptr<const std::vector<uint8_t>> combinedVector = VectorMessage::forOpCodeNotification("Peer closed connection. Reason: " + cr)->getCombinedVector();
    this->ws.write(boost::asio::buffer(*combinedVector), ec);

    // send a websocket closing frame (ignoring any errors)
    boost::beast::websocket::close_reason close;
    close.code = cc;
    close.reason = cr;
    this->ws.close(close, ec);

    // perform a graceful socket shutdown
    if constexpr (is_boost_ssl_stream_v<T>) {
        // if this is an ssl stream, we want to shutdown the ssl context
        // hence we call shutdown on the next layer of the websocket connection
        this->ws.next_layer().shutdown(ec);
    } else {
        // if this is not an ssl stream, we can shutdown the lowest layer
        boost::beast::get_lowest_layer(this->ws).shutdown(boost::asio::socket_base::shutdown_both, ec);
    }

    // if an error occurred close the socket by force
    if (ec) boost::beast::get_lowest_layer(this->ws).close(ec);

    if (this->remoteEndpoint.empty())
        RCLCPP_INFO(this->logger.get(), "Websocket connection closed");
    else
        RCLCPP_INFO(this->logger.get(), "Websocket connection with %s closed", this->remoteEndpoint.c_str());
}

template <class T>
void WebsocketConnectionBase<T>::onAuthorized() {
    this->publisherManager.init(this->node);
    this->publisherManager.startThread("PUBMAN");

    this->subscriberManager.init();
    this->subscriberManager.startThread("SUBMAN");

    this->serviceActionManager->init();
    this->serviceActionManager->startThread("SVAMAN");
}

template <class T>
void WebsocketConnectionBase<T>::onStopThread() {
    ConnectionBase::onStopThread();

    this->publisherManager.stopThread();
    this->publisherManager.joinThread();

    this->subscriberManager.stopThread();
    this->subscriberManager.joinThread();

    this->serviceActionManager->stopThread();
    this->serviceActionManager->joinThread();
}

template <class T>
void WebsocketConnectionBase<T>::processMessage(std::unique_ptr<MessageBase> message) {
    // if the message is of op-code NOTIFICATION, the message must be logged
    if (message->isOpCode(OpCode::NOTIFICATION)) RCLCPP_INFO(this->logger.get(), "%s", message->toString().c_str());
    // if the message is of op-code SUBSCRIPTION, the message must be routed to the subscriber manager
    else if (message->isOpCode(OpCode::SUBSCRIPTION)) this->subscriberManager.subscribe(std::move(message));
    // if the message if of op-code SERVIVE_SERVER or SERVICE_CLIENT, the message must be route to the service manager
    else if (message->isOpCode(OpCode::SERVICE_ACTION))
        this->serviceActionManager->handle(std::move(message));
    // otherwise we publish it
    else
        this->publisherManager.publish(std::move(message));
}

template <class T>
void WebsocketConnectionBase<T>::asyncWrite() {
    // empty the sending queue if the actual websocket connection is not (yet) opened
    if (!this->ws.is_open()) {
        while (!this->sendingQueue.empty())
            this->sendingQueue.pop();
        return;
    }

    const std::shared_ptr<const std::vector<uint8_t>> combinedVector = this->sendingQueue.front();
    this->ws.async_write(boost::asio::buffer(*combinedVector),
                         boost::asio::bind_executor(this->strand,
                                                    [this, combinedVector](const boost::system::error_code &ec, std::size_t /* bytes_transferred */) {
                                                        this->sendingQueue.pop();

                                                        if (ec) {
                                                            RCLCPP_ERROR(this->logger.get(), "Error on write: %s, closing connection", ec.message().c_str());
                                                            this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::internal_error, "Error on write");
                                                        } else {
                                                            // send next message if there is any
                                                            if (!this->sendingQueue.empty()) this->asyncWrite();
                                                        }
                                                    }));
}

template <class T>
void WebsocketConnectionBase<T>::asyncRead() {
    if (!this->ws.is_open()) {
        RCLCPP_INFO(this->logger.get(), "asyncRead: Socket was already closed");
        return;
    }

    // clearing the read buffer setting its readable data to zero
    this->readBuffer.clear();

    this->ws.async_read(this->readBuffer,
                        boost::asio::bind_executor(this->strand,
                                                   [this](const boost::system::error_code &ec, std::size_t /* bytes_transferred */) {
                                                       if (ec) {
                                                           if (ec == boost::beast::websocket::error::closed) {
                                                               const boost::beast::websocket::close_reason cr = this->ws.reason();
                                                               if (!cr.reason.empty())
                                                                   RCLCPP_INFO(this->logger.get(), "Peer closed connection: %d. Reason: %s", cr.code, cr.reason.c_str());
                                                               else
                                                                   RCLCPP_INFO(this->logger.get(), "Peer closed connection: %d", cr.code);
                                                               this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::normal);
                                                           } else {
                                                               RCLCPP_ERROR(this->logger.get(), "Error on read: %s, closing connection", ec.message().c_str());
                                                               this->connectionManager.stopConnection(this->shared_from_this(), boost::beast::websocket::close_code::internal_error, "Error on read");
                                                           }
                                                       } else {
                                                           // extract the read from the flat-buffer and copy it into a Message
                                                           std::unique_ptr<RclMessage> message = RclMessage::fromFlatBuffer(this->readBuffer);

                                                           // move the data over to further process it
                                                           this->processMessage(std::move(message));

                                                           // read the next message
                                                           this->asyncRead();
                                                       }
                                                   }));
}

#endif  // WEBSOCKETCONNECTIONBASE_HPP
