// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef ACTIONCLIENT_HPP
#define ACTIONCLIENT_HPP

#include "action.hpp"

namespace action {
class ActionClient : public Action {
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
            this->logger.setName(std::to_string(definition.channel) + "-action-client", remoteEndpoint);
        else
            this->logger.setName(std::to_string(definition.channel) + "-action-client");

        Action::init(definition, node, callbackGroup, remoteEndpoint, send);
    }

    ~ActionClient() override {
        if (this->definition != nullptr) {
            RCLCPP_INFO(this->logger.get(), "Action Client for type %s on action-channel %d destroyed", this->definition->type.c_str(), this->definition->channel);
        }
    }

    /**
     * This callback handler handles a message from the other side.
     *
     * @param message the received message
     */
    void handle(std::unique_ptr<ServiceActionMessage> message) override {
        // convert the gid to a string and get the action handle for it
        const std::string gidString = message->getGIDString();

        std::unique_lock<std::mutex> lock(this->handlesMutex);
        const std::map<const std::string, std::shared_ptr<ServiceActionHandle>>::iterator it = this->handles.find(gidString);

        if (message->isServiceActionOpCode(ServiceActionOpCode::ACTION_TO_CLIENT_GOAL)) {
            // there is a handle with the same gid
            if (it != this->handles.end()) {
                RCLCPP_WARN(this->logger.get(), "Received a message for an already knowing goal it, responding with reject");
                this->send(ServiceActionOpCode::ACTION_TO_SERVER_GOAL_REJECT, {}, message->getGID());
                return;
            }
            // if we are already stopping
            if (this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Stopped before starting to handle a new action request");
                return;
            }
            // create a new handle, start its thread to execute the goal and store everything
            std::shared_ptr<ServiceActionHandle> handle = std::make_shared<ServiceActionHandle>(message->getGID());
            handle->waitQueue.push(std::move(message));
            handle->thread = std::thread(&ActionClient::executeHandle, this, handle);
            this->handles.insert(std::pair<const std::string, std::shared_ptr<ServiceActionHandle>>(gidString, handle));
        } else if (message->isServiceActionOpCode(ServiceActionOpCode::ACTION_TO_CLIENT_CANCEL)) {
            // there is no handle with the gid
            if (it == this->handles.end()) {
                RCLCPP_WARN(this->logger.get(), "Received a cancel request with an unknown goal id, responding with reject");
                this->send(ServiceActionOpCode::ACTION_TO_SERVER_GOAL_REJECT, {}, message->getGID());
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

  protected:
    ActionClient() = default;

    std::size_t expectedSize{0}; // holds the expected size of an incoming request

    /**
     * This should wait for the action (server) to call to become alive by
     *  - call wait_for_action_server on the rclcpp_action::Client<...>::SharedPtr
     */
    virtual bool waitForAction() = 0;

    /**
     * This should send the local goal to the local action (server) by
     *  - crafting a ...::Goal holding the deserialized message
     *  - crafting a rclcpp_action::Client<...>::SendGoalOptions which defined all the action client callback methods
     *  - call async_send_goal on the rclcpp_action::Client<...>::SharedPtr
     */
    virtual void asyncSendGoal(const std::unique_ptr<ServiceActionMessage> &message, std::shared_ptr<ServiceActionHandle> handle) = 0;

    /**
     * This should cancel the local goal by
     *  - making sure the handle has a stored clientGoalHandle
     *  - casting the stored clientGoalHandle into a rclcpp_action::ClientGoalHandle<...>
     *  - calling async_cancel_goal on the rclcpp_action::Client<...>::SharedPtr
     */
    virtual void asyncCancelGoal(std::shared_ptr<ServiceActionHandle> handle) = 0;

    /**
     * Callback method which should be called when cancelling the local action call was successful / the cancelling was accepted
     *
     * @param handle the handle of which the local action call was canceled
     */
    void cancelAccepted(const std::shared_ptr<ServiceActionHandle> &handle) {
        this->send(ServiceActionOpCode::ACTION_TO_SERVER_CANCEL_ACCEPT, {}, handle->gID);
        handle->done = true;
        handle->handlingCancel = false;
        handle->waitCV.notify_all();
    }

    /**
     * Callback method which should be called when cancelling the local action call failed / the cancelling was rejected
     *
     * @param handle the handle of which the local action call failed to cancel
     */
    void cancelRejected(const std::shared_ptr<ServiceActionHandle> &handle) {
        this->send(ServiceActionOpCode::ACTION_TO_SERVER_CANCEL_REJECT, {}, handle->gID);
        handle->handlingCancel = false;
    }

    /**
     * Callback method which should be called when the local goal was accepted
     *
     * @param handle the handle of which the local goal was accepted
     * @param goalHandle the goal handle of the accepted goal
     */
    void goalAccepted(const std::shared_ptr<ServiceActionHandle> &handle, const std::shared_ptr<void> &goalHandle) {
        this->send(ServiceActionOpCode::ACTION_TO_SERVER_GOAL_ACCEPT, {}, handle->gID);
        handle->clientGoalHandle = goalHandle;
        handle->handlingGoal = false;
        handle->waitCV.notify_all();
    }

    /**
     * Callback method which should be called when the local goal was rejected
     *
     * @param handle the handle of which the local goal was rejected
     */
    void goalRejected(const std::shared_ptr<ServiceActionHandle> &handle) {
        this->send(ServiceActionOpCode::ACTION_TO_SERVER_GOAL_ACCEPT, {}, handle->gID);
        handle->done = true;
        handle->handlingGoal = false;
        handle->waitCV.notify_all();
    }

    /**
     * Callback method which should be called when the local goal result was successful
     *
     * @param data the serialized data of the result
     * @param handle the handle of which the local result was successful
     */
    void resultSucceeded(const std::vector<uint8_t> &data, const std::shared_ptr<ServiceActionHandle> &handle) {
        this->send(ServiceActionOpCode::ACTION_TO_SERVER_RESULT_SUCCEEDED, data, handle->gID);
        handle->done = true;
        handle->waitCV.notify_all();
    }

    /**
     * Callback method which should be called when the local goal result was canceled
     *
     * @param data the serialized data of the result
     * @param handle the handle of which the local result was canceled
     */
    void resultCanceled(const std::vector<uint8_t> &data, const std::shared_ptr<ServiceActionHandle> &handle) {
        this->send(ServiceActionOpCode::ACTION_TO_SERVER_RESULT_CANCELED, data, handle->gID);
        handle->done = true;
        handle->waitCV.notify_all();
    }

    /**
     * Callback method which should be called when the local goal result was aborted
     *
     * @param data the serialized data of the result
     * @param handle the handle of which the local result was aborted
     */
    void resultAborted(const std::vector<uint8_t> &data, const std::shared_ptr<ServiceActionHandle> &handle) {
        this->send(ServiceActionOpCode::ACTION_TO_SERVER_RESULT_ABORTED, data, handle->gID);
        handle->done = true;
        handle->waitCV.notify_all();
    }

    /**
     * Callback method which should be called when the local action goal generated feedback
     *
     * @param data the serialized data of the feedback
     * @param handle the handle of which the local action goal generated feedback
     */
    void feedback(const std::vector<uint8_t> &data, const std::shared_ptr<ServiceActionHandle> &handle) {
        this->send(ServiceActionOpCode::ACTION_TO_SERVER_FEEDBACK, data, handle->gID);
    }

  private:
    /**
     * This executes in a std::thread and processes incoming messages
     */
    void executeHandle(std::shared_ptr<ServiceActionHandle> handle) {
        while (!this->stopping) {
            // wait for the next message to process it
            std::unique_lock<std::mutex> waitLock(handle->waitMutex);
            handle->waitCV.wait(waitLock, [this, handle]() {
                return !handle->waitQueue.empty() || handle->done || this->stopping;
            });

            // if stopping
            if (this->stopping) {
                RCLCPP_WARN(this->logger.get(), "Stopped while handling a local request, trying to cancel local request");
                this->executeCancel(handle);
                this->removeHandle(handle);
                break;
            }
            // if done
            else if (handle->done) {
                this->removeHandle(handle);
                break;
            }
            // no new message
            else if (handle->waitQueue.empty())
                continue;

            // get the next message and release the lock
            // we do not decompress it right away but later
            std::unique_ptr<ServiceActionMessage> message = std::move(handle->waitQueue.front());
            handle->waitQueue.pop();
            waitLock.unlock();

            // process it
            switch (message->getServiceActionOpCode()) {
            case ServiceActionOpCode::ACTION_TO_CLIENT_GOAL:
                // this would mean, that we received a second goal which should never happen
                if (handle->handlingGoal || handle->handlingCancel) {
                    RCLCPP_WARN(this->logger.get(), "Received an unexpected goal while already handling another goal or cancel, dropping it");
                    continue;
                }

                // execute the goal
                this->executeGoal(message, handle);
                break;
            case ServiceActionOpCode::ACTION_TO_CLIENT_CANCEL:
                // this would mean, that we have not yet accepted or rejected the goal
                // but the other side has already canceled
                // however, we need to wait till the goal was rejected or accepted before the cancel request can be processed
                if (handle->handlingGoal) {
                    waitLock.lock();
                    handle->waitQueue.push(std::move(message));
                    waitLock.unlock();
                    continue;
                }
                // this would we are already processing a cancel request, just drop it
                else if (handle->handlingCancel) {
                    RCLCPP_WARN(this->logger.get(), "Received a cancel while already handling another cancel, dropping it");
                    continue;
                }

                // execute the cancel
                this->executeCancel(handle);
                break;
            default:
                RCLCPP_WARN(this->logger.get(), "Received a message with unknown service action op-code, dropping it");
            }
        }
    }

    /**
     * This is called by executeHandle and processes a message which is of type ServiceActionOpCode::ACTION_TO_CLIENT_GOAL
     */
    void executeGoal(const std::unique_ptr<ServiceActionMessage> &message, std::shared_ptr<ServiceActionHandle> handle) {
        handle->handlingGoal = true;

        // test for compression and uncompress if necessary
        // on failure, reject the goal
        if (message->isCompressed() && !message->decompress()) {
            this->send(ServiceActionOpCode::ACTION_TO_SERVER_GOAL_REJECT, {}, handle->gID);
            handle->handlingGoal = false;
            handle->done = true;
            return;
        }

        // check the size of the incoming request
        if (const size_t size = message->getData().size(); size != this->expectedSize) {
            RCLCPP_WARN(this->logger.get(), "Received goal of unexpected size, expected %lu, got %lu, responding with reject", expectedSize, size);
            this->send(ServiceActionOpCode::ACTION_TO_SERVER_GOAL_REJECT, {}, handle->gID);
            handle->handlingGoal = false;
            handle->done = true;
            return;
        }

        // wait for the action to come alive
        if (!this->waitForAction()) {
            RCLCPP_WARN(this->logger.get(), "Server for action with type %s not available, responding with reject", this->definition->type.c_str());
            this->send(ServiceActionOpCode::ACTION_TO_SERVER_GOAL_REJECT, {}, handle->gID);
            handle->handlingGoal = false;
            handle->done = true;
            return;
        }

        // test if stopping
        // we do not need to send a message to the other side, since if we are shutting down the connection, the other side will too and handle stopping case
        if (this->stopping) {
            RCLCPP_WARN(this->logger.get(), "Stopped before handling the action goal");
            handle->handlingGoal = false;
            handle->done = true;
            return;
        }

        // send the async goal
        this->asyncSendGoal(message, handle);
    }

    /**
     * This is called by executeHandle and processes a message which is of type ServiceActionOpCode::ACTION_TO_CLIENT_CANCEL
     */
    void executeCancel(const std::shared_ptr<ServiceActionHandle> &handle) {
        handle->handlingCancel = true;
        // we do not check for stopping here since we want to also execute this when stopping to try to cancel the local goal
        this->asyncCancelGoal(handle);
    }
};
} // namespace action

#endif // ACTIONCLIENT_HPP
