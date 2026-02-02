// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef CONNECTIONBASE_HPP_
#define CONNECTIONBASE_HPP_

#include "common/thread.hpp"

#include <cstdint>
#include <string>
#include <queue>
#include <memory>

#include <boost/asio/io_context.hpp>
#include <boost/asio/io_context_strand.hpp>
#include <boost/beast/websocket/rfc6455.hpp>


class ConnectionBase : public Thread, public std::enable_shared_from_this<ConnectionBase> {
    friend class ConnectionManager;

public:
    /**
     * Creates a new connection.
     * The io_context should be a newly created io_context so this connection can run in an own thread.
     *
     * @param ioc the io_context in which the connection runs
     */
    explicit ConnectionBase(std::shared_ptr<boost::asio::io_context> ioc);

    ~ConnectionBase() override;

    /**
     * Sends the given message to the client connected or server connected to
     *
     * @param combinedVector the combined vector to send holding  channel followed by raw data
     */
    virtual void send(const std::shared_ptr<const std::vector<uint8_t> > &combinedVector);

    /**
     * @return the remote endpoint address
     */
    const std::string &getRemoteEndpoint();

protected:
    std::shared_ptr<boost::asio::io_context> ioc;
    boost::asio::io_context::strand strand;

    std::queue<std::shared_ptr<const std::vector<uint8_t> > > sendingQueue;

    std::string remoteEndpoint;

    /**
     * Start the connection and therefore also the underlying thread.
     * Everything needed to be configured e.g. a boost::beast::websocket::stream must be done before this is called or while this called.
     *
     * @return true if start was successful
     */
    virtual bool start() = 0;

    /**
     * Closes the connection and underlying socket
     */
    virtual void close() = 0;

    /**
     * Closes the connection and underlying socket
     *
     * @param cc the close code
     * @param cr the close reason
     */
    virtual void close(const boost::beast::websocket::close_code /* cc */, const std::string & /* cr */) {
        this->close();
    }

    /**
     * Called right after the underlying thread is stopped by setting the stopping flag to true.
     */
    void onStopThread() override;

    /**
     * Queues a message to be send.
     * This must be called from the io_context managing the queue (sync via strand)
     *
     * @param combinedVector the combined vector to send holding  channel followed by raw data
     */
    void queueMessage(const std::shared_ptr<const std::vector<uint8_t> > &combinedVector);

    /**
     * Performs the async write of any data which is in the sendingQueue
     */
    virtual void asyncWrite() = 0;

    /**
     * Performs the async read and processes them
     */
    virtual void asyncRead() = 0;
};

#endif /* CONNECTIONBASE_HPP_ */
