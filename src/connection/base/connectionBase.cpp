// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "connectionBase.hpp"

#include <boost/bind/bind.hpp>
#include <boost/asio/post.hpp>


ConnectionBase::ConnectionBase(std::shared_ptr<boost::asio::io_context> ioc) : ioc(std::move(ioc)), strand(*this->ioc) {
}

ConnectionBase::~ConnectionBase() {
}

void ConnectionBase::onStopThread() {
    ioc->stop();
}

void ConnectionBase::send(const std::shared_ptr<const std::vector<uint8_t> > &combinedVector) {
    if (!this->strand.running_in_this_thread())
        boost::asio::post(this->strand, boost::bind(&ConnectionBase::queueMessage, this->shared_from_this(), combinedVector));
    else
        this->queueMessage(combinedVector);
}

const std::string &ConnectionBase::getRemoteEndpoint() {
    return this->remoteEndpoint;
}

void ConnectionBase::queueMessage(const std::shared_ptr<const std::vector<uint8_t> > &combinedVector) {
    this->sendingQueue.push(combinedVector);

    // do we need to start writing?
    if (this->sendingQueue.size() == 1) asyncWrite();
}
