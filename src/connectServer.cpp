// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <rclcpp/rclcpp.hpp>

#include "connectBase.hpp"
#include "connection/server.hpp"
#include "status/statusPublisher.hpp"

class ConnectServer final : public ConnectBase {
   public:
    ConnectServer() : ConnectBase("connect_server") {
    }

    ~ConnectServer() override {
        this->halt();
    }

    /**
     * Spins the node by creating either a websocket server or a websocket client
     *
     * @param executor the executor in which executes this
     */
    void spin(rclcpp::executors::MultiThreadedExecutor &executor) override {
        try {
            // start the websocket server
            RCLCPP_INFO(this->get_logger(), "Starting");
            this->webSocketConnection = std::make_shared<Server>(this->shared_from_this(), executor);
            if (!this->webSocketConnection->startThread(GlobalConfig::nodeName)) {
                RCLCPP_FATAL(this->get_logger(), "Websocket server thread could not be started");
                rclcpp::shutdown(nullptr, "Websocket server thread could not be started");
            }
            // start the connect status publisher
            std::function<void()> callback = [this]() {
                if (this->webSocketConnection != nullptr) {
                    this->statusPublisher->publishStatus(this->webSocketConnection->getConnectionManager().isClientConnected());
                } else {
                    this->statusPublisher->publishStatus(false);
                }
            };
            this->statusPublisher = std::make_unique<StatusPublisher>(this->shared_from_this(), std::move(callback));
        } catch (std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "Unable to create websocket server, error was: %s", e.what());
            rclcpp::shutdown(nullptr, "Unable to create websocket server");
        }
    }

    /**
     * Hals the node by stopping the websocket server os client
     */
    void halt() override {
        if (this->statusPublisher != nullptr) {
            this->statusPublisher.reset();
            this->statusPublisher = nullptr;
        }

        if (this->webSocketConnection != nullptr) {
            this->webSocketConnection->stopThread();
            this->webSocketConnection->joinThread();

            this->webSocketConnection.reset();
            this->webSocketConnection = nullptr;
        }
    }

   private:
    std::shared_ptr<Server> webSocketConnection = nullptr;

    std::unique_ptr<StatusPublisher> statusPublisher;
};

std::shared_ptr<ConnectServer> node = nullptr;

void signal(const int value) {
    // if sigint and we (still) have a node, try to halt it before shutting down the ros2 context
    if (value == SIGINT && node != nullptr) node->halt();
    // shutdown the ros2 context (this is also what the ros2 signal handlers would do)
    else
        rclcpp::shutdown();
}

int main(const int argc, char *argv[]) {
    // init ros2 context
    rclcpp::init(argc, argv);

    // remove the default ros2 signal handlers
    rclcpp::uninstall_signal_handlers();

    // register signal handlers
    std::signal(SIGINT, signal);
    std::signal(SIGTERM, signal);

    // create a temporary node which is used to
    // evaluate the number of needed threads for the multi-threaded-executor
    int numberOfThreads;
    {
        const std::shared_ptr<ConnectServer> tmp = std::make_shared<ConnectServer>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(tmp);
        executor.spin_some();
        numberOfThreads = tmp->evalNrOfMultiThreadedExecutorThreads() + 1;  // 1 additional thread for the connect status timer callback
        if (numberOfThreads < 1) {
            rclcpp::shutdown();
            return -1;
        }
    }

    // create a multi threaded executor and let it execute the node
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), numberOfThreads);

    node = std::make_shared<ConnectServer>();
    if (node->init(true)) {
        node->spin(executor);

        executor.add_node(node);
        executor.spin();
    }

    // shut down gracefully
    node->halt();
    rclcpp::shutdown();

    // reset the node to ensure its context is destroyed before leaving main
    node.reset();
    node = nullptr;

    return 0;
}
