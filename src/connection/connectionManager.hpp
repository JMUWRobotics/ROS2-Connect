// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef CONNECTIONMANAGER_HPP_
#define CONNECTIONMANAGER_HPP_

#include "base/connectionBase.hpp"

#include <memory>

#include <boost/core/noncopyable.hpp>
#include <boost/bind/bind.hpp>
#include <boost/asio/post.hpp>
#include <boost/beast/websocket/rfc6455.hpp>

class ConnectionManager : boost::noncopyable {
public:
    /**
     * Construct a connection manager using a given strand for thread synchronisation
     *
     * @param serverStrand the server's strand
     */
    explicit ConnectionManager(boost::asio::io_context::strand &serverStrand) : serverStrand(serverStrand) {
    }

    /**
     * Adds the specified connection to the manager and starts it
     *
     * @param c the connection to add and start
     */
    void startConnection(const std::shared_ptr<ConnectionBase> &c) {
        if (c->start()) this->connections.push_back(c);
    }

    /**
     * Stops the specified connection and removes it from the manager
     *
     * @param connection the connection to stopp and remove
     */
    void stopConnection(const std::shared_ptr<ConnectionBase> &connection) {
        // Stopping a connection must be handled by the io_context of the server, such that a connection is able to stop itself.
        // Otherwise a dead lock would occur when joining the thread from within itself
        boost::asio::post(this->serverStrand, boost::bind(&ConnectionManager::stopConnectionInternal, this, connection, true));
    }

    /**
     * Stops the specified connection and removes it from the manager
     *
     * @param connection the connection to stopp and remove
     * @param cc close code
     * @param cr close reason
     */
    void stopConnection(const std::shared_ptr<ConnectionBase> &connection, const boost::beast::websocket::close_code cc, const std::string &cr = "") {
        // Stopping a connection must be handled by the io_context of the server, such that a connection is able to stop itself.
        // Otherwise a dead lock would occur when joining the thread from within itself
        boost::asio::post(this->serverStrand, boost::bind(&ConnectionManager::stopConnectionInternal, this, connection, cc, cr, true));
    }

    /**
     * Stops all connections and removes them from the manager
     * This is called when the server is shutting down AFTER the io_context was already stopped.
     */
    void stopAllConnections() {
        while (!this->connections.empty()) {
            const std::shared_ptr<ConnectionBase> connection = *connections.begin();
            this->stopConnectionInternal(connection, boost::beast::websocket::close_code::going_away, "Application shutdown", false);
        }
    }

    /**
     * Sets the connection close callback which is called whenever a connection is closed
     *
     * @param callback the callback function (should be moved to here)
     */
    void setConnectionCloseCallback(std::function<void()> callback) {
        this->connectionCloseCallback = std::move(callback);
    }

    /**
     * Returns if at least one connection is established / managed
     *
     * @return true if at least one connection is established / managed
     */
    bool isClientConnected() const {
        return !this->connections.empty();
    }

private:
    std::vector<std::shared_ptr<ConnectionBase> > connections;

    boost::asio::io_context::strand &serverStrand;

    std::function<void()> connectionCloseCallback = []() {
    };

    /**
     * Actually stopping and removing the given connection.
     * Can be executed by the server's io_context to mitigate a dead lock when joining a thread from within itself.
     *
     * @param connection the connection to stop and remove
     * @param callCallback if the connection close callback should be called
     * @see stopConnection
     */
    void stopConnectionInternal(const std::shared_ptr<ConnectionBase> &connection, const bool callCallback) {
        for (std::vector<std::shared_ptr<ConnectionBase> >::iterator it = this->connections.begin(); it != this->connections.end(); ++it) {
            if ((*it) == connection) {
                connections.erase(it);

                connection->stopThread();
                connection->joinThread();
                connection->close();

                if (callCallback) this->connectionCloseCallback();

                break;
            }
        }
    }

    /**
     * Actually stopping and removing the given connection.
     * Can be executed by the server's io_context to mitigate a dead lock when joining a thread from within itself.
     *
     * @param connection the connection to stop and remove
     * @param cc close code
     * @param cr close reason
     * @param callCallback if the connection close callback should be called
     * @see stopConnection
     */
    void stopConnectionInternal(const std::shared_ptr<ConnectionBase> &connection, const boost::beast::websocket::close_code cc, const std::string &cr, const bool callCallback) {
        for (std::vector<std::shared_ptr<ConnectionBase> >::iterator it = this->connections.begin(); it != this->connections.end(); ++it) {
            if ((*it) == connection) {
                connections.erase(it);

                connection->stopThread();
                connection->joinThread();
                connection->close(cc, cr);

                if (callCallback) this->connectionCloseCallback();

                break;
            }
        }
    }
};

#endif /* CONNECTIONMANAGER_HPP_ */
