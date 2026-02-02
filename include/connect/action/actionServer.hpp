// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef ACTIONSERVER_HPP
#define ACTIONSERVER_HPP

#include "action.hpp"

namespace action {
    class ActionServer : public Action {
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
                this->logger.setName(std::to_string(definition.channel) + "-action-server", remoteEndpoint);
            else
                this->logger.setName(std::to_string(definition.channel) + "-action-server");

            Action::init(definition, node, callbackGroup, remoteEndpoint, send);
        }

        ~ActionServer() override {
            if (this->definition != nullptr) {
                RCLCPP_INFO(this->logger.get(), "Action Server for type %s on action-channel %d destroyed", this->definition->type.c_str(), this->definition->channel);
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

            // there is no handle with the gid
            if (it == this->handles.end()) {
                RCLCPP_WARN(this->logger.get(), "Received a message with an unknown goal id, dropping it");
                return;
            }

            if (message->isServiceActionOpCode(ServiceActionOpCode::ACTION_TO_SERVER_GOAL_REJECT) || message->isServiceActionOpCode(ServiceActionOpCode::ACTION_TO_SERVER_GOAL_ACCEPT)) {
                // acquire a lock before checking the handlingGoal state
                std::unique_lock<std::mutex> goalLock(it->second->goalMutex);

                if (!it->second->handlingGoal) {
                    RCLCPP_WARN(this->logger.get(), "Received a goal response while not handling a goal, dropping it");
                } else {
                    it->second->goalResponse = std::move(message);
                    it->second->goalCV.notify_all();
                }
            } else if (message->isServiceActionOpCode(ServiceActionOpCode::ACTION_TO_SERVER_CANCEL_ACCEPT) || message->isServiceActionOpCode(ServiceActionOpCode::ACTION_TO_SERVER_CANCEL_REJECT)) {
                // acquire a lock before checking the handlingGoal state
                std::unique_lock<std::mutex> cancelLock(it->second->cancelMutex);

                if (!it->second->handlingCancel) {
                    RCLCPP_WARN(this->logger.get(), "Received a cancel response while not handling a cancel, dropping it");
                } else {
                    it->second->cancelResponse = std::move(message);
                    it->second->cancelCV.notify_all();
                }
            } 
            // this is either feedback or the result and handled by the spinning thread
            else { 
                std::unique_lock<std::mutex> waitLock(it->second->waitMutex);
                it->second->waitQueue.push(std::move(message));
                it->second->waitCV.notify_all();
            }
        }

        /**
         * Stops the service by waking and joining every waiting thread.
         */
        void stop() override {
            // we do this to cover the case where we stop while being in the callback
            // executed by a rclcpp::Executor thread
            if (!this->handles.empty()) {
                RCLCPP_WARN(this->logger.get(), "Stopped before handling all local requests");
            }

            // then call the underlying stop
            Action::stop();
        }

    protected:
        ActionServer() = default;

        /**
         * This should abort the goal. For this it should
         *  - static_cast goalHandle into a std::shared_ptr<rclcpp_action::ServerGoalHandle<...>>
         *  - create an appropriate std::shared_ptr<...::Result> using the data
         *  - abort the casted goalHandle
         *
         * @param goalHandle the goal to abort
         * @param data the data of the abort
         */
        virtual void abort(const std::shared_ptr<void> &goalHandle, const std::span<uint8_t> &data) = 0;

        /**
         * This should cancel the goal. For this it should
         *  - static_cast goalHandle into a std::shared_ptr<rclcpp_action::ServerGoalHandle<...>>
         *  - create an appropriate std::shared_ptr<...::Result> using the data
         *  - cancel the casted goalHandle
         *
         * @param goalHandle the goal to cancel
         * @param data the data of the cancel
         */
         virtual void cancel(const std::shared_ptr<void> &goalHandle, const std::span<uint8_t> &data) = 0;

         /**
         * This should succeed the goal. For this it should
         *  - static_cast goalHandle into a std::shared_ptr<rclcpp_action::ServerGoalHandle<...>>
         *  - create an appropriate std::shared_ptr<...::Result> using the data
         *  - succeed the casted goalHandle
         *
         * @param goalHandle the goal to succeed
         * @param data the data of the succeed
         */
         virtual void succeed(const std::shared_ptr<void> &goalHandle, const std::span<uint8_t> &data) = 0;

         /**
         * This should succeed emit feedback of the goal. For this it should
         *  - static_cast goalHandle into a std::shared_ptr<rclcpp_action::ServerGoalHandle<...>>
         *  - create an appropriate std::shared_ptr<...::Feedback> using the data
         *  - emit the feedback to the casted goalHandle
         *
         * @param goalHandle the goal to emit feedback of
         * @param data the data to emit
         */
         virtual void feedback(const std::shared_ptr<void> &goalHandle, const std::span<uint8_t> &data) = 0;

        /**
         * This sends the serialized data of the goal to the other side and waits for its response.
         * 
         * @param gid the GoalUUID issued by rclcpp_action
         * @param data the serialized data of the goal to send to the other side
         * @returns the goal response of the other side
         */
        rclcpp_action::GoalResponse sendGoalAndWait(const rclcpp_action::GoalUUID &gid, const std::vector<uint8_t> &data) {
            // check if stopping
            if (this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Stopped before sending the goal while handling a local request");
                return rclcpp_action::GoalResponse::REJECT;
            }

            // convert the gid into a string and store a handle
            std::shared_ptr<ServiceActionHandle> handle = std::make_shared<ServiceActionHandle>(gid);
            const std::string gidString = handle->gIDString();

            std::unique_lock<std::mutex> handleLock(this->handlesMutex);
            this->handles.insert(std::pair<const std::string, std::shared_ptr<ServiceActionHandle> >(gidString, handle));
            handleLock.unlock();

            // send the goal to the other side and wait for the response
            handle->goalResponse = nullptr; // no lock needed as long handlingGoal is false
            handle->handlingGoal = true;

            this->send(ServiceActionOpCode::ACTION_TO_CLIENT_GOAL, data, handle->gID);

            // check if stopping
            if (this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Stopped before receiving a goal response while handling a local request");
                handle->handlingGoal = false;
                this->removeHandle(handle);
                return rclcpp_action::GoalResponse::REJECT;
            }

            // wait for the response ... max wait 10 seconds for the other side to process the goal to not block indefinitely
            std::unique_lock<std::mutex> lock(handle->goalMutex);
            handle->goalCV.wait_for(lock, std::chrono::seconds(10), [this, handle]() {
                return handle->goalResponse != nullptr || this->stopping;
            });

            // if we are stopping and have no response we can safely return
            // this is because we override stop() which handles this case for us (stopping can only be true if our override stop() was called)
            if (!rclcpp::ok() || this->stopping) {
                return rclcpp_action::GoalResponse::REJECT;
            }
            // if there is unexpectedly no data
            else if (handle->goalResponse == nullptr) {
                RCLCPP_ERROR(this->logger.get(), "Unexpectedly received no goal response after 10s");
                this->send(ServiceActionOpCode::ACTION_TO_CLIENT_CANCEL, {}, handle->gID); // simply send a cancel without waiting for a response
                handle->handlingGoal = false;
                this->removeHandle(handle);
                return rclcpp_action::GoalResponse::REJECT;
            }

            // check the responses op code
            if (handle->goalResponse->isServiceActionOpCode(ServiceActionOpCode::ACTION_TO_SERVER_GOAL_ACCEPT)) {
                handle->handlingGoal = false;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            } else {
                handle->handlingGoal = false;
                this->removeHandle(handle);
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        /**
         * This sends the cancel to the other side and waits for its response.
         * 
         * @param gid the GoalUUID issued by rclcpp_action
         * @returns the goal response of the other side
         */
        rclcpp_action::CancelResponse sendCancelAndWait(const rclcpp_action::GoalUUID &gid) {
            // convert the gid into a string and find the handle
            const std::string gidString = ServiceActionHandle::gIDString(gid);

            std::unique_lock<std::mutex> handleLock(this->handlesMutex);
            const std::map<const std::string, std::shared_ptr<ServiceActionHandle> >::iterator it = this->handles.find(gidString);
            if (it == this->handles.end()) {
                RCLCPP_WARN(this->logger.get(), "Received cancel request for unknown goal id");
                return rclcpp_action::CancelResponse::REJECT;
            }
            const std::shared_ptr<ServiceActionHandle> &handle = it->second;
            handleLock.unlock();

            // check if stopping
            if (this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Stopped before sending the cancel while handling a local request");
                handle->handlingCancel = false;
                if (!handle->handlingWait) this->removeHandle(handle);
                return rclcpp_action::CancelResponse::REJECT;
            }

            // send the cancel to the other side and wait for the response
            handle->cancelResponse = nullptr; // no lock needed as long handlingCancel is false
            handle->handlingCancel = true;

            this->send(ServiceActionOpCode::ACTION_TO_CLIENT_CANCEL, {}, handle->gID);

            // check if stopping
            if (this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Stopped before receiving a cancel response while handling a local request");
                handle->handlingCancel = false;
                if (!handle->handlingWait) this->removeHandle(handle);
                return rclcpp_action::CancelResponse::REJECT;
            }

            // wait for the response ... max wait 10 seconds to not block indefinitely
            std::unique_lock<std::mutex> lock(handle->cancelMutex);
            handle->cancelCV.wait_for(lock, std::chrono::seconds(10), [this, handle]() {
                return handle->cancelResponse != nullptr || this->stopping;
            });

            // if we are stopping and have no response we can safely return
            // this is because we override stop() which handles this case for us (stopping can only be true if our override stop() was called)
            if (!rclcpp::ok() || this->stopping) {
                return rclcpp_action::CancelResponse::REJECT;
            }
            // if there is unexpectedly no data
            else if (handle->cancelResponse == nullptr) {
                RCLCPP_ERROR(this->logger.get(), "Unexpectedly received no cancel response after 10s");
                handle->handlingCancel = false;
                return rclcpp_action::CancelResponse::REJECT;
            }

            // check the responses op code
            if (handle->cancelResponse->isServiceActionOpCode(ServiceActionOpCode::ACTION_TO_SERVER_CANCEL_ACCEPT)) {
                handle->handlingCancel = false;
                if (!handle->handlingWait) this->removeHandle(handle);
                return rclcpp_action::CancelResponse::ACCEPT;
            } else {
                handle->handlingCancel = false;
                return rclcpp_action::CancelResponse::REJECT;
            }
        }

        bool handleAccepted(const std::shared_ptr<void> &goalHandle, const rclcpp_action::GoalUUID &gid) {
            // convert the gid into a string and find the action handle
            const std::string gidString = ServiceActionHandle::gIDString(gid);

            std::unique_lock<std::mutex> handleLock(this->handlesMutex);
            const std::map<const std::string, std::shared_ptr<ServiceActionHandle> >::iterator it = this->handles.find(gidString);
            if (it == this->handles.end()) {
                RCLCPP_WARN(this->logger.get(), "Received accepted request for unknown goal id");
                rclcpp_action::GoalUUID lGid = gid;
                this->send(ServiceActionOpCode::ACTION_TO_CLIENT_CANCEL, {}, lGid); // simply send a cancel without waiting for a response
                return false;
            }

            // test if we can start the thread (if the thread is not joinable)
            // this should always be the case since at this point no thread was started yet, but better safe than sorry
            if (it->second->thread.joinable() && !this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Unable to start the acceptor thread");
                this->send(ServiceActionOpCode::ACTION_TO_CLIENT_CANCEL, {}, it->second->gID); // simply send a cancel without waiting for a response
                this->removeHandle(it->second);
                return false;
            }

            // test if not stopping
            if (this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Stopped before accepting goal");
                this->removeHandle(it->second);
                return false;
            }

            // start the thread and done
            it->second->handlingWait = true;
            it->second->thread = std::thread(&ActionServer::executeAccepted, this, it->second, goalHandle);
            return true;
        }

        void executeAccepted(std::shared_ptr<ServiceActionHandle> handle, std::shared_ptr<void> goalHandle) {
            while(!this->stopping) {
                // wait for the next message to process it
                std::unique_lock<std::mutex> waitLock(handle->waitMutex);
                handle->waitCV.wait(waitLock, [this, handle]() {
                    return !handle->waitQueue.empty() || handle->done || this->stopping;
                });

                // if stopping
                if (this->stopping) {
                    RCLCPP_WARN(this->logger.get(), "Stopped before receiving a result response while handling a local request");
                    handle->handlingWait = false;
                    this->removeHandle(handle);
                    if (rclcpp::ok()) {
                        this->abort(goalHandle, {});
                    } else {
                        RCLCPP_WARN(this->logger.get(), "Could not abort local request");
                    }
                    break;
                }
                // if done
                else if (handle->done) {
                    handle->handlingWait = false;
                    this->removeHandle(handle);
                    break;
                }
                // no new message
                else if (handle->waitQueue.empty()) continue;

                // get the next message and release the lock
                const std::unique_ptr<ServiceActionMessage> message = std::move(handle->waitQueue.front());
                handle->waitQueue.pop();
                waitLock.unlock();

                // test for compression and uncompress if necessary
                if (message->isCompressed() && !message->decompress()) {
                    switch (message->getServiceActionOpCode()) {
                        // in case of feedback we can simply drop the message since it is not important
                        case ServiceActionOpCode::ACTION_TO_SERVER_FEEDBACK:
                            RCLCPP_ERROR(this->logger.get(), "Received a compressed feedback message for service client %s on channel %d which cannot be decompressed, dropping message", this->definition->type.c_str(), this->definition->channel);
                            continue;
                        // in case of result we need to abort with empty data
                        case ServiceActionOpCode::ACTION_TO_SERVER_RESULT_SUCCEEDED:
                        case ServiceActionOpCode::ACTION_TO_SERVER_RESULT_CANCELED:
                        case ServiceActionOpCode::ACTION_TO_SERVER_RESULT_ABORTED:
                        case ServiceActionOpCode::ACTION_TO_SERVER_RESULT_UNKNOWN:
                            RCLCPP_ERROR(this->logger.get(), "Received a compress result message for service client %s on channel %d which cannot be decompressed, aborting goal", this->definition->type.c_str(), this->definition->channel);
                            this->abort(goalHandle, {});
                            handle->done = true;
                            handle->handlingWait = false;
                            this->removeHandle(handle);
                            return;
                        default:
                            RCLCPP_WARN(this->logger.get(), "Received a message with unknown service action op-code, dropping it");
                    }                    
                }

                // process it
                switch (message->getServiceActionOpCode()) {
                    case ServiceActionOpCode::ACTION_TO_SERVER_FEEDBACK:
                        this->feedback(goalHandle, message->getData());
                        break;
                    case ServiceActionOpCode::ACTION_TO_SERVER_RESULT_SUCCEEDED:
                        this->succeed(goalHandle, message->getData());
                        handle->done = true;
                        handle->handlingWait = false;
                        this->removeHandle(handle);
                        return;
                    case ServiceActionOpCode::ACTION_TO_SERVER_RESULT_CANCELED:
                        this->cancel(goalHandle, message->getData());
                        handle->done = true;
                        handle->handlingWait = false;
                        this->removeHandle(handle);
                        return;
                    case ServiceActionOpCode::ACTION_TO_SERVER_RESULT_ABORTED:
                    case ServiceActionOpCode::ACTION_TO_SERVER_RESULT_UNKNOWN:
                        this->abort(goalHandle, message->getData());
                        handle->done = true;
                        handle->handlingWait = false;
                        this->removeHandle(handle);
                        return;
                    default:
                        RCLCPP_WARN(this->logger.get(), "Received a message with unknown service action op-code, dropping it");
                }
            }
        }
    };
} // namespace action

#endif  // ACTIONSERVER_HPP
