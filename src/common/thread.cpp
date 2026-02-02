// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "thread.hpp"

#include <functional>
#include <system_error>
#include <pthread.h>

#include <rclcpp/rclcpp.hpp>

Thread::Thread() : stopping(false), thread(nullptr) {
}

Thread::~Thread() {
    this->stopThread();
    this->joinThread();
}

bool Thread::startThread(const std::string &name) {
    if (this->thread != nullptr) {
        delete this->thread;
        this->thread = nullptr;
    }

    this->stopping = false;

    if (!this->onBeforeStartThread()) return false;

    try {
        this->thread = new std::thread(std::bind(&Thread::run, this));
    } catch (std::system_error &) {
        this->thread = nullptr;
        return false;
    }

    if (name.length() > 0) this->setThreadName(name);

    this->onAfterStartThread();

    return true;
}

void Thread::stopThread() {
    this->onBeforeStopThread();
    this->stopping = true;
    this->onStopThread();
}

void Thread::joinThread() {
    if (this->thread != nullptr) {
        this->thread->join();
        this->onAfterJoinThread();
        delete this->thread;
        this->thread = nullptr;
    }
}

void Thread::setThreadName(const std::string &name) const {
    if (this->thread != nullptr) {
        // argument of pthread_setname_np must not exceed 16 chars incl. null termination, so we use at maximum 15 chars from the name
        const int err = pthread_setname_np(this->thread->native_handle(), name.substr(0, 15).c_str());
        if (err != 0)
            RCLCPP_ERROR(rclcpp::get_logger(name), "Thread name %s was not set, error number is %d", name.c_str(), err);
    }
}
