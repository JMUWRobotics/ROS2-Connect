// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <rclcpp/rclcpp.hpp>

#include "connectBase.hpp"
#include "connection/client.hpp"

class ConnectClient final : public ConnectBase {
   public:
    ConnectClient() : ConnectBase("connect_client") {
    }

    ~ConnectClient() override {
        this->halt();
    }

    /**
     * Spins the node by creating either a websocket server or a websocket client
     *
     * @param executor the executor in which executes this
     */
    void spin(rclcpp::executors::MultiThreadedExecutor &executor) override {
        try {
            this->webSocketConnection = std::make_shared<Client>(this->shared_from_this(), executor);
            if (!this->webSocketConnection->startThread(GlobalConfig::nodeName)) {
                RCLCPP_FATAL(this->get_logger(), "Websocket client thread could not be started");
                rclcpp::shutdown(nullptr, "Websocket client thread could not be started");
            }
        } catch (std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "Unable to create websocket client, error was: %s", e.what());
            rclcpp::shutdown(nullptr, "Unable to create websocket client");
        }
    }

    /**
     * Hals the node by stopping the websocket server os client
     */
    void halt() override {
        if (this->webSocketConnection != nullptr) {
            this->webSocketConnection->stopThread();
            this->webSocketConnection->joinThread();

            this->webSocketConnection.reset();
            this->webSocketConnection = nullptr;
        }
    }

   private:
    std::shared_ptr<Client> webSocketConnection = nullptr;
};

std::shared_ptr<ConnectClient> node = nullptr;

void signal(const int value) {
    // if sigint and we (still) have a node, try to halt it before shutting down the ros2 context
    if (value == SIGINT && node != nullptr) node->halt();
    // shutdown the ros2 context (this is also what the ros2 signal handlers would do)
    else rclcpp::shutdown();
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
        const std::shared_ptr<ConnectClient> tmp = std::make_shared<ConnectClient>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(tmp);
        executor.spin_some();
        numberOfThreads = tmp->evalNrOfMultiThreadedExecutorThreads();
        if (numberOfThreads < 1) {
            rclcpp::shutdown();
            return -1;
        }
    }

    // create a multi threaded executor and let it execute the node
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), numberOfThreads);

    node = std::make_shared<ConnectClient>();
    if (node->init(false)) {
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
