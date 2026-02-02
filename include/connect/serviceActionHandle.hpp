// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SERVICEACTIONHANDLE_HPP
#define SERVICEACTIONHANDLE_HPP

#include <condition_variable>
#include <connect/serviceActionMessage.hpp>
#include <mutex>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vector>

namespace service {
    class Service;
}

namespace action {
    class Action;
}

class ServiceActionHandle final {
    friend class action::Action;
    friend class service::Service;

public:
    // the goal id of this handle
    std::vector<uint8_t> gID;
    int64_t sequenceNumber{0};

    // the client goal handle
    std::shared_ptr<void> clientGoalHandle{nullptr};

    // these variables are used by the server to wait for a response on the initial goal request
    std::mutex goalMutex;
    std::condition_variable goalCV;
    std::unique_ptr<ServiceActionMessage> goalResponse{nullptr};
    std::atomic<bool> handlingGoal{false};

    // these variables are used by the server to wait for a response on a cancel request
    std::mutex cancelMutex;
    std::condition_variable cancelCV;
    std::unique_ptr<ServiceActionMessage> cancelResponse{nullptr};
    std::atomic<bool> handlingCancel{false};

    // these variables are used by
    //  - the server to wait for feedback and result
    //  - the client to wait for any message
    std::mutex waitMutex;
    std::condition_variable waitCV;
    std::queue<std::unique_ptr<ServiceActionMessage> > waitQueue;
    std::atomic<bool> handlingWait{false};

    // this thread is used by
    //  - the server to wait for feedback and result
    //  - the client to wait for any message
    std::thread thread;
    std::atomic<bool> done{false};

    /**
     * Converts a rclcpp_action::GoalUUID into a vector
     *
     * @param gid the rclcpp_action::GoalUUID
     * @returns the gid
     */
    static std::vector<uint8_t> convert(const rclcpp_action::GoalUUID &gid) {
        std::vector<uint8_t> gIDVector(gid.size());
        std::memcpy(gIDVector.data(), gid.data(), gid.size());
        return gIDVector;
    }

    /**
     * Creates a new ServiceActionHandle which collectively stores variables of an action call
     *
     * @param gid the gid
     */
    explicit ServiceActionHandle(const rclcpp_action::GoalUUID &gid) {
        std::vector<uint8_t> gIDVector(gid.size());
        std::memcpy(gIDVector.data(), gid.data(), gid.size());
        this->gID = std::move(gIDVector);
    }

    /**
     * Creates a new ServiceActionHandle which collectively stores variables of an action call
     *
     * @param gid the gid
     */
    explicit ServiceActionHandle(const std::span<uint8_t> &gid) {
        std::vector<uint8_t> gIDVector(gid.size());
        std::memcpy(gIDVector.data(), gid.data(), gid.size());
        this->gID = std::move(gIDVector);
    }

    /**
     * Creates a new ServiceActionHandle which collectively stores variables for an action call
     *
     * @param requestHeader the rmw request header
     */
    explicit ServiceActionHandle(const std::shared_ptr<rmw_request_id_t> &requestHeader) {
        std::vector<uint8_t> gIDVector(RMW_GID_STORAGE_SIZE);
        std::memcpy(gIDVector.data(), requestHeader->writer_guid, RMW_GID_STORAGE_SIZE);
        this->gID = std::move(gIDVector);
        this->sequenceNumber = requestHeader->sequence_number;
    }

    /**
     * @param gid goal id
     * @return goal id as string
     */
    static std::string gIDString(const rclcpp_action::GoalUUID &gid) {
        std::string gidString(reinterpret_cast<const char *>(gid.data()), gid.size());
        gidString.push_back('\0');
        return gidString;
    }

    /**
     * @return goal id as string
     */
    std::string gIDString() const {
        std::string gidString(reinterpret_cast<const char *>(this->gID.data()), this->gID.size());
        gidString.push_back('\0');
        return gidString;
    }

    /**
     * @return rmw request header
     */
    rmw_request_id_t requestHeader() const {
        rmw_request_id_t requestHeader;
        requestHeader.sequence_number = this->sequenceNumber;
        std::memcpy(requestHeader.writer_guid, this->gID.data(), RMW_GID_STORAGE_SIZE);
        return requestHeader;
    }

private:
    /**
     * Notifies all conditional variables
     */
    void notify_all() {
        this->goalCV.notify_all();
        this->cancelCV.notify_all();
        this->waitCV.notify_all();
    }
};

#endif  // SERVICEACTIONHANDLE_HPP
