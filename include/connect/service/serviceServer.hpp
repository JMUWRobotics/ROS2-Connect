// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SERVICESERVER_HPP
#define SERVICESERVER_HPP

#include "service.hpp"

namespace service {
    class ServiceServer : public Service {
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
                this->logger.setName(std::to_string(definition.channel) + "-service-server", remoteEndpoint);
            else
                this->logger.setName(std::to_string(definition.channel) + "-service-server");

            Service::init(definition, node, callbackGroup, remoteEndpoint, send);
        }

        ~ServiceServer() override {
            if (this->definition != nullptr) {
                RCLCPP_INFO(this->logger.get(), "Service Server for type %s on service-channel %d destroyed", this->definition->type.c_str(), this->definition->channel);
            }
        }

        /**
         * This callback handler handles a message from the other side.
         *
         * @param message the received message
         */
        void handle(std::unique_ptr<ServiceActionMessage> message) override {
            if (!message->isServiceActionOpCode(ServiceActionOpCode::SERVICE_TO_SERVER)) {
                RCLCPP_ERROR(this->logger.get(), "Received a message with unknown service action op-code, dropping it");
                return;
            }

            // convert the gid to a string and get the handle for it
            const std::string gidString = message->getGIDString();

            std::unique_lock<std::mutex> lock(this->handlesMutex);
            const std::map<const std::string, std::shared_ptr<ServiceActionHandle> >::iterator it = this->handles.find(gidString);

            // there is no handle with the gid
            if (it == this->handles.end()) {
                RCLCPP_WARN(this->logger.get(), "Received a message with an unknown goal id, dropping it");
                return;
            }

            // acquire a lock before checking the handlingGoal state
            std::unique_lock<std::mutex> goalLock(it->second->goalMutex);

            // we do not expect data
            if (!it->second->handlingGoal) {
                RCLCPP_WARN(this->logger.get(), "Received an unexpected response, dropping it");
                return;
            }

            // store the response and notify the cv
            it->second->goalResponse = std::move(message);
            it->second->goalCV.notify_all();
        }

        /**
         * Stops the service by waking and joining every waiting thread.
         */
        void stop() override {
            // if we have an ongoing request, send the response
            // we do this since we cannot trust ros2 in actually executing the callback while shutting down
            if (!this->handles.empty()) {
                RCLCPP_WARN(this->logger.get(), "Stopped before receiving a response while handling a local request");

                std::unique_lock<std::mutex> lock(this->handlesMutex);
                for (const std::pair<const std::string, std::shared_ptr<ServiceActionHandle> > &pair: this->handles) {
                    this->cancel(pair.second);
                }
            }

            // then call the underlying stop
            Service::stop();
        }

    protected:
        ServiceServer() = default;

        /**
         * This should "cancel" the ongoing callback of the given handle by sending a response to it.
         * By this it should
         *  - create a ServiceType::Response and write the variables indicating an error
         *  - get the request header of the handle
         *  - call send_response on the rclcpp::Service<...>::SharedPtr
         */
        virtual void cancel(const std::shared_ptr<ServiceActionHandle> &handle) = 0;

        /**
         * This sends the serialized data to the other side and waits for its response.
         * If this succeeds (a response was received and no interruption occurred), this will return a shared_ptr to an ServiceActionHandle holding the response.
         * If not, this will return a nullptr. In that case the callee (Service Server's callback function) must write the callbacks response and can then safely quit (no call to this->done!).
         *
         * @param data the serialized data to send to the other side
         * @param requestHeader the request header of the Service Server's callback function call
         * @param expectedSize the expected size of the response
         * @param dynamicSize if the response is allowed to be dynamic in size; expected size will be ignored
         * @returns a valid ServiceActionHandle holding the response or a nullptr
         */
        std::shared_ptr<ServiceActionHandle> sendAndWait(const std::vector<uint8_t> &data, const std::shared_ptr<rmw_request_id_t> &requestHeader, const size_t &expectedSize, const bool dynamicSize = false) {
            // if we are stopping we need to somehow handle the request anyway
            if (this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Stopped before sending the request while handling a local request");
                return nullptr;
            }

            // create a handle using the request header and store the handle
            std::shared_ptr<ServiceActionHandle> handle = std::make_shared<ServiceActionHandle>(requestHeader);
            const std::string gidString = handle->gIDString();

            std::unique_lock<std::mutex> handleLock(this->handlesMutex);
            this->handles.insert(std::pair<const std::string, std::shared_ptr<ServiceActionHandle> >(gidString, handle));
            handleLock.unlock();

            // send the message to the client on the other side
            handle->goalResponse = nullptr; // no lock needed as long handlingGoal is false
            handle->handlingGoal = true;

            this->send(ServiceActionOpCode::SERVICE_TO_CLIENT, data, handle->gID);

            // if we are stopping we need to somehow handle the request anyway
            // many requests hold a boolean variable "success" which can easily be used to communicate no success
            // however, many also don't hold it ... hence we need to agree on some way to communicate no success
            if (this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Stopped before receiving a response while handling a local request");
                handle->handlingGoal = false;
                this->removeHandle(handle);
                return nullptr;
            }

            // wait for the response ... max wait the definitions "maxExecTime" seconds to not block indefinitely
            std::unique_lock<std::mutex> lock(handle->goalMutex);
            handle->goalCV.wait_for(lock, std::chrono::seconds(this->definition->maxExecTime), [this, handle]() {
                return handle->goalResponse != nullptr || this->stopping;
            });

            // if we are stopping and have no response we can safely return
            // this is because we override stop() which handles this case for us (stopping can only be true if our override stop() was called)
            if (!rclcpp::ok() || this->stopping) {
                return nullptr;
            }
            // if there is unexpectedly no data we need to somehow handle the request
            // many requests hold a boolean variable "success" which can easily be used to communicate no success
            // however, many also don't hold it ... hence we need to agree on some way to communicate no success
            else if (handle->goalResponse == nullptr) {
                RCLCPP_WARN(this->logger.get(), "Unexpectedly received no response after %ds", this->definition->maxExecTime);
                this->send(ServiceActionOpCode::SERVICE_TO_CLIENT_CANCEL, {}, handle->gID);
                handle->handlingGoal = false;
                this->removeHandle(handle);
                return nullptr;
            }

            // test for compression and uncompress if necessary
            if (handle->goalResponse->isCompressed() && !handle->goalResponse->decompress()) {
                RCLCPP_ERROR(this->logger.get(), "Received compressed message for service server %s on channel %d which cannot be decompressed, dropping message", this->definition->type.c_str(), this->definition->channel);
                handle->handlingGoal = false;
                this->removeHandle(handle);
                return nullptr;
            }

            // test if the size of the response matches the expected size
            if (const size_t size = handle->goalResponse->getData().size(); !dynamicSize && (size != expectedSize)) {
                RCLCPP_WARN(this->logger.get(), "Unexpectedly received response of wrong size, expected %lu got %lu", expectedSize, size);
                handle->handlingGoal = false;
                this->removeHandle(handle);
                return nullptr;
            }

            // we do set handlingGoal to false, to ensure the goalResponse will not be overridden while we still hold a lock on it
            // with this we ensure that this->handle can not override the goalResponse while the local server's callback reads the response
            handle->handlingGoal = false;

            // success, we return the handle
            return handle;
        }

        /**
         * This should be the last call in a Service Server's callback function if
         * the call to this->sendAndWait was successful and the callbacks response was written.
         *
         * It should not be called when the call to this->sendAndWait was not successful.
         */
        void done(const std::shared_ptr<ServiceActionHandle> &handle) {
            handle->handlingGoal = false;
            this->removeHandle(handle);
        }
    };
} // namespace service

#endif  // SERVICESERVER_HPP
