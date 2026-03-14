// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SERVICECLIENT_HPP
#define SERVICECLIENT_HPP

#include <rclcpp/rclcpp.hpp>

#include "connect/serviceActionMessage.hpp"
#include "service.hpp"

namespace service {
    class ServiceClient : public Service {
    public:
        /**
          * Initializes a service server or client for the given service
          *
          * @param definition the service definition
          * @param node the node to create the service server for
          * @param callbackGroup the callback group to register the service in
          * @param remoteEndpoint the remote endpoint address if there is one
          * @param send a callback to send a combinedVector to "the other side" (wraps the send method of a ConnectionBase.hpp)
          */
        void init(Service_Action_t &definition, const rclcpp::Node::SharedPtr &node, const rclcpp::CallbackGroup::SharedPtr &callbackGroup, const std::string &remoteEndpoint, const std::function<void(const std::shared_ptr<const std::vector<uint8_t>> &)> send) override {
            if (!remoteEndpoint.empty())
                this->logger.setName(std::to_string(definition.channel) + "-service-client", remoteEndpoint);
            else
                this->logger.setName(std::to_string(definition.channel) + "-service-client");

            Service::init(definition, node, callbackGroup, remoteEndpoint, send);
        }

        ~ServiceClient() override {
            if (this->definition != nullptr) {
                RCLCPP_INFO(this->logger.get(), "Service Client for type %s on service-channel %d destroyed", this->definition->type.c_str(), this->definition->channel);
            }
        }

        /**
         * This callback handler handles a message from the other side.
         *
         * @param message the received message
         */
        void handle(std::unique_ptr<ServiceActionMessage> message) override {
            // convert the gid to a string and get the handle for it
            const std::string gidString = message->getGIDString();

            std::unique_lock<std::mutex> lock(this->handlesMutex);
            const std::map<const std::string, std::shared_ptr<ServiceActionHandle> >::iterator it = this->handles.find(gidString);

            if (message->isServiceActionOpCode(ServiceActionOpCode::SERVICE_TO_CLIENT)) {
                // there is a handle with the same gid
                if (it != this->handles.end()) {
                    RCLCPP_WARN(this->logger.get(), "Received a message for an already knowing goal id, responding with error");
                    this->send(ServiceActionOpCode::SERVICE_TO_SERVER, this->error, message->getGID());
                    return;
                }
                // if we are already stopping
                if (this->stopping) {
                    RCLCPP_WARN(this->logger.get(), "Stopped before starting to handle a new service request");
                    return;
                }
                // create a new handle, start its thread to execute the goal and store everything
                std::shared_ptr<ServiceActionHandle> handle = std::make_shared<ServiceActionHandle>(message->getGID());
                handle->thread = std::thread(&ServiceClient::executeHandle, this, std::move(message), handle);
                this->handles.insert(std::pair<const std::string, std::shared_ptr<ServiceActionHandle> >(gidString, handle));
            } else if (message->isServiceActionOpCode(ServiceActionOpCode::SERVICE_TO_CLIENT_CANCEL)) {
                // there is no handle with the gid
                if (it == this->handles.end()) {
                    RCLCPP_WARN(this->logger.get(), "Received a cancel request with an unknown goal id, dropping it");
                    return;
                }
                // store the response and notify the waiting thread
                std::unique_lock<std::mutex> waitLock(it->second->waitMutex);
                it->second->waitQueue.push(std::move(message));
                it->second->waitCV.notify_all();
            } else {
                RCLCPP_ERROR(this->logger.get(), "Received a message with unknown service action op-code, dropping it");
            }
        }

        /**
         * Stops the service by waking and joining every waiting thread.
         */
        void stop() override {
            // stop all pending requests since we cannot be sure that their callback will be called during a shutdown
            if (!this->handles.empty()) {
                RCLCPP_WARN(this->logger.get(), "Stopped before handling pending requests");
            }
            this->cancelAll();

            // call the underlying stop
            Service::stop();
        }

    protected:
        ServiceClient() = default;

        std::vector<uint8_t> error; // holds serialized data which represents an error
        std::size_t expectedSize{0}; // holds the expected size of an incoming request
        bool dynamicSize{false}; // if size of incoming request may be dynamic; expected size will be ignored

        /**
         * This should "cancel" the ongoing request.
         * By this it should
         *  - call remove_pending_request on the rclcpp::Client<...>::SharedPtr
         */
        virtual void cancel(int64_t requestId) = 0;

        /**
         * This should cancel all pending requests by
         *  - make sure the rclcpp::Client<...>::SharedPtr was initialized
         *  - call prune_pending_requests on the rclcpp::Client<...>::SharedPtr
         */
        virtual void cancelAll() = 0;

        /**
         * This should wait for the service (server) to call to become alive by
         *  - call wait_for_service on the rclcpp::Client<...>::SharedPtr
         */
        virtual bool waitForService() = 0;

        /**
         * This should perform the local request to the local service (server) by
         *  - crafting a std::shared_ptr<...::Request> holding the deserialized message data
         *  - call async_send_request on the rclcpp::Client<...>::SharedPtr
         *  - and return the requestId
         *
         * @throws this can throw a std::exception if the message data cannot be deserialized
         */
        virtual int64_t asyncSendRequest(const std::unique_ptr<ServiceActionMessage> &message, std::shared_ptr<ServiceActionHandle> handle) = 0;

        /**
         * Executes the handling of a message from the other side
         *
         * @param message the received message
         * @param handle the handle to execute
         */
        void executeHandle(std::unique_ptr<ServiceActionMessage> message, std::shared_ptr<ServiceActionHandle> handle) {
            // test for compression and uncompress if necessary
            if (message->isCompressed() && !message->decompress()) {
                RCLCPP_ERROR(this->logger.get(), "Received compressed message for service client %s on channel %d which cannot be decompressed, dropping message", this->definition->type.c_str(), this->definition->channel);
                this->send(ServiceActionOpCode::SERVICE_TO_SERVER, this->error, message->getGID());
                this->removeHandle(handle);
                return;
            }

            // check the size of the incoming request
            if (const size_t size = message->getData().size(); !this->dynamicSize && (size != this->expectedSize)) {
                RCLCPP_WARN(this->logger.get(), "Received request of unexpected size, expected %lu, got %lu, responding with error", expectedSize, size);
                this->send(ServiceActionOpCode::SERVICE_TO_SERVER, this->error, message->getGID());
                this->removeHandle(handle);
                return;
            }

            // wait for the service to come alive
            if (!this->waitForService()) {
                RCLCPP_WARN(this->logger.get(), "Server for service with type %s not available, responding with error", this->definition->type.c_str());
                this->send(ServiceActionOpCode::SERVICE_TO_SERVER, this->error, message->getGID());
                this->removeHandle(handle);
                return;
            }

            // test if stopping
            // we do not need to send a message to the other side, since if we are shutting down the connection, the other side will too and handle stopping case
            if (this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Stopped before handling the service request");
                this->removeHandle(handle);
                return;
            }

            // now async send the request
            // store the requestId which can be used to cancel this request
            int64_t requestId;
            try {
                requestId = this->asyncSendRequest(message, handle);
            } catch(std::exception &) {
                RCLCPP_WARN(this->logger.get(), "Received request holding invalid data, responding with error");
                this->send(ServiceActionOpCode::SERVICE_TO_SERVER, this->error, message->getGID());
                this->removeHandle(handle);
                return;
            }

            // now acquire a unique lock
            std::unique_lock<std::mutex> lock(handle->waitMutex);
            // and wait until notified. only wake up if
            //  - we have a response
            //  - we are done
            //  - we are stopping
            handle->waitCV.wait(lock, [this, handle]() {
                return !handle->waitQueue.empty() || handle->done || this->stopping;
            });

            // test if stopping or done -> if so return
            if (handle->done || this->stopping) {
                this->removeHandle(handle);
                return;
            } else if (handle->waitQueue.empty()) {
                RCLCPP_WARN(this->logger.get(), "Unexpectedly received no response");
                this->removeHandle(handle);
                return;
            }

            // if we reached this point we are not stopping, nor done and have received a message
            // in that case the message is always of op-code SERVICE_TO_CLIENT_CANCEL
            // which means we cancel the ongoing request
            RCLCPP_WARN(this->logger.get(), "Received a request to cancel the handling of the request");
            this->cancel(requestId);
            handle->done = true;
            this->removeHandle(handle);
        }

        /**
         * This functions sends the given serialized response data to the other side and
         * marks the handle as being done.
         * This should be the last call in the Service Client's callback function.
         */
        void sendAndDone(const std::vector<uint8_t> &data, const std::shared_ptr<ServiceActionHandle> &handle) {
            this->send(ServiceActionOpCode::SERVICE_TO_SERVER, data, handle->gID);
            handle->done = true;
            handle->waitCV.notify_all();
        }
    };
} // namespace service

#endif  // SERVICECLIENT_HPP
