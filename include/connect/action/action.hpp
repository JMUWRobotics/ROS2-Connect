// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef ACTION_HPP
#define ACTION_HPP

#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <queue>

#include "connect/serviceActionHandle.hpp"
#include "connect/logger.hpp"
#include "connect/serviceActionMessage.hpp"
#include "connect/types.hpp"

namespace action {
    class Action {
    public:
        virtual ~Action() = default;

        /**
         * Initializes a service server or client for the given service
         *
         * @param definition the service definition
         * @param node the node to create the service server for
         * @param callbackGroup the callback group to register the service in
         * @param remoteEndpoint the remote endpoint address if there is one
         * @param send a callback to send a combinedVector to "the other side" (wraps the send method of a ConnectionBase.hpp)
         */
        virtual void init(Service_Action_t &definition, const rclcpp::Node::SharedPtr &/*node*/, const rclcpp::CallbackGroup::SharedPtr &callbackGroup, const std::string &/*remoteEndpoint*/, std::function<void(const std::shared_ptr<const std::vector<uint8_t>> &)> send) {
            if (definition.compression->compressor != Compressor::NONE) this->compression = true;
            else this->compression = false;

            this->definition = &definition;
            this->callbackGroup = callbackGroup;

            this->send = [this, send](const ServiceActionOpCode serviceActionOpCode, const std::vector<uint8_t> &data, const std::span<uint8_t> &gid) {
                if (this->compression) send(ServiceActionMessage::forServiceAction(this->definition->channel, serviceActionOpCode, data, gid)->getCompressedCombinedVector(*this->definition->compression));
                else send(ServiceActionMessage::forServiceAction(this->definition->channel, serviceActionOpCode, data, gid)->getCombinedVector());
            };

            this->stopping = false;

            this->toRemoveThread = std::thread(&Action::executeRemove, this);
        }

        /**
         * This method should create and directly destruct a rclcpp client or service to test if it can successfully be created
         * This method is called during parameter initialization
         *
         * @param definition the action definition
         * @param nodeBaseInterface the nodes base interface
         * @param nodeGraphInterface the nodes graph interface
         * @param nodeClockInterface the nodes clock interface
         * @param nodeLoggingInterface the nodes logging interface
         * @param nodeWaitablesInterface the nodes waitables interface
         * @param callbackGroup the callback group to register the action in
         */
        virtual void test(Service_Action_t &definition, const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &nodeBaseInterface, const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr &nodeGraphInterface, const rclcpp::node_interfaces::NodeClockInterface::SharedPtr &nodeClockInterface, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr &nodeLoggingInterface, const rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr &nodeWaitablesInterface, const rclcpp::CallbackGroup::SharedPtr &callbackGroup) const = 0;

        /**
         * This callback handler handles a message from the other side.
         *
         * @param message the received message
         */
        virtual void handle(std::unique_ptr<ServiceActionMessage> message) = 0;

        /**
         * Stops the service by waking and joining every waiting thread.
         */
        virtual void stop() {
            // set the stopping value
            this->stopping = true;

            // join the toRemove thread
            this->toRemoveCV.notify_all();
            if (this->toRemoveThread.joinable()) this->toRemoveThread.join();

            // wake up every waiting process, join its thread, remove it
            // we do not need to acquire a lock at this point since:
            //  - no new handles will be inserted after stopping was set to true
            //  - the toRemoveThread was already stopped
            while (!this->handles.empty()) {
                const std::map<const std::string, std::shared_ptr<ServiceActionHandle> >::iterator it = this->handles.begin();
                it->second->notify_all();
                if (it->second->thread.joinable()) it->second->thread.join();
                this->handles.erase(it->first);
            }

            // finally clean up the toRemove queue
            while (!this->toRemove.empty()) {
                this->toRemove.pop();
            }
        }

    protected:
        Action() = default;

        Logger logger;

        Service_Action_t *definition = nullptr;

        rclcpp::CallbackGroup::SharedPtr callbackGroup;

        std::function<void(ServiceActionOpCode serviceActionOpCode, const std::vector<uint8_t> &data, const std::span<uint8_t> &gid)> send = [](const ServiceActionOpCode, const std::vector<uint8_t> &, const std::span<uint8_t> &) {
        };

        std::mutex handlesMutex;
        std::map<const std::string, std::shared_ptr<ServiceActionHandle> > handles;

        std::atomic<bool> stopping;

        /**
         * Marks a handle for remove.
         * This will asynchronous delete the action handle and also joining its thread.
         *
         * @param handle the handle to remove
         */
        void removeHandle(const std::shared_ptr<ServiceActionHandle> &handle) {
            std::unique_lock<std::mutex> lock(this->toRemoveMutex);
            this->toRemove.push(handle);
            this->toRemoveCV.notify_all();
        }

        /**
         * This should be executed in the toRemoveThread which does join all the threads of the
         * handles to remove
         */
        void executeRemove() {
            while (!this->stopping) {
                // acquire a unique lock on the mutex
                std::unique_lock<std::mutex> lock(this->toRemoveMutex);
                // wait on the conditional variable
                // only wake up if there is an handle in the queue or if stopping
                this->toRemoveCV.wait(lock, [this]() {
                    return !this->toRemove.empty() || this->stopping;
                });
                // if stopping, break
                if (this->stopping) break;
                // if there is nothing to do, continue
                if (this->toRemove.empty()) continue;

                // get the next handle to remove and release the lock
                const std::shared_ptr<ServiceActionHandle> handle = this->toRemove.front();
                this->toRemove.pop();
                lock.unlock();

                // wake the handle (to make absolutely sure)
                handle->notify_all();
                // join the action handles thread
                if (handle->thread.joinable()) handle->thread.join();
                // remove it from the handles queue
                std::unique_lock<std::mutex> handleLock(this->handlesMutex);
                this->handles.erase(handle->gIDString());
            }
        }

        std::thread toRemoveThread;
        std::queue<std::shared_ptr<ServiceActionHandle> > toRemove;
        std::mutex toRemoveMutex;
        std::condition_variable toRemoveCV;

    private:
        bool compression{false};
    };
} // namespace action

#endif  // ACTION_HPP
