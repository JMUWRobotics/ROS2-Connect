cmake_minimum_required(VERSION 3.28)
project(connect)

# as soon as we reach cmake version 3.30 we should change this to NEW
# and drop the COMPONENTS ... from the find_package(BOOST REQUIRED ...)
if (POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 23)

set(THREADS_PREFER_PTHREAD_FLAG ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs)
find_package(rosidl_default_generators REQUIRED)

find_package(Threads REQUIRED)
find_package(Boost 1.75 REQUIRED COMPONENTS json)
find_package(OpenSSL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(lz4 REQUIRED IMPORTED_TARGET liblz4)

find_package(ZLIB REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/ConnectStatus.msg"
        DEPENDENCIES
        std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

# add libraries
add_library(${PROJECT_NAME}_headers INTERFACE)

target_include_directories(${PROJECT_NAME}_headers
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# add executables
add_executable(
        server
        include/connect/action/action.hpp
        include/connect/action/actionClient.hpp
        include/connect/action/actionServer.hpp
        include/connect/service/service.hpp
        include/connect/service/serviceClient.hpp
        include/connect/service/serviceServer.hpp
        include/connect/authentication.hpp
        include/connect/logger.hpp
        include/connect/messageBase.hpp
        include/connect/serviceActionHandle.hpp
        include/connect/serviceActionMessage.hpp
        include/connect/types.hpp
        src/common/logger.cpp
        src/common/enumParser.hpp
        src/common/thread.cpp
        src/common/thread.hpp
        src/connection/base/connectionBase.cpp
        src/connection/base/connectionBase.hpp
        src/connection/connectionManager.hpp
        src/connection/server.cpp
        src/connection/server.hpp
        src/global/globalConfig.cpp
        src/global/globalConfig.hpp
        src/message/messageBase.cpp
        src/message/rclMessage.cpp
        src/message/rclMessage.hpp
        src/message/serviceActionMessage.cpp
        src/message/vectorMessage.cpp
        src/message/vectorMessage.hpp
        src/publisher/publisherBase.hpp
        src/publisher/publisherManager.cpp
        src/publisher/publisherManager.hpp
        src/publisher/sharedPublisher.cpp
        src/publisher/sharedPublisher.hpp
        src/publisher/threadedPublisher.cpp
        src/publisher/threadedPublisher.hpp
        src/serviceAction/serviceActionManager.cpp
        src/serviceAction/serviceActionManager.hpp
        src/subscriber/clockSubscriber.cpp
        src/subscriber/clockSubscriber.hpp
        src/subscriber/sharedSubscriber.cpp
        src/subscriber/sharedSubscriber.hpp
        src/subscriber/subscriberBase.hpp
        src/subscriber/subscriberManager.cpp
        src/subscriber/subscriberManager.hpp
        src/subscriber/threadedSubscriber.cpp
        src/subscriber/threadedSubscriber.hpp
        src/websocketConnection/base/websocketConnectionBase.hpp
        src/websocketConnection/server/websocketServerConnection.cpp
        src/websocketConnection/server/websocketServerConnection.hpp
        src/status/statusPublisher.hpp
        src/status/statusPublisher.cpp
        src/connectBase.hpp
        src/connectServer.cpp
)
target_include_directories(server PRIVATE ${Boost_INCLUDE_DIRS} src include)
target_link_libraries(server PUBLIC
        ${cpp_typesupport_target}
        Threads::Threads
        Boost::json
        OpenSSL::SSL
        ZLIB::ZLIB
        PkgConfig::lz4
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_action::rclcpp_action
)
target_compile_definitions(server PUBLIC PACKAGE_NAME="${PROJECT_NAME}")

add_executable(
        client
        include/connect/action/action.hpp
        include/connect/action/actionClient.hpp
        include/connect/action/actionServer.hpp
        include/connect/service/service.hpp
        include/connect/service/serviceClient.hpp
        include/connect/service/serviceServer.hpp
        include/connect/authentication.hpp
        include/connect/logger.hpp
        include/connect/messageBase.hpp
        include/connect/serviceActionHandle.hpp
        include/connect/serviceActionMessage.hpp
        include/connect/types.hpp
        src/common/logger.cpp
        src/common/enumParser.hpp
        src/common/thread.cpp
        src/common/thread.hpp
        src/connection/client.cpp
        src/connection/client.hpp
        src/connection/base/connectionBase.cpp
        src/connection/base/connectionBase.hpp
        src/connection/connectionManager.hpp
        src/global/globalConfig.cpp
        src/global/globalConfig.hpp
        src/message/messageBase.cpp
        src/message/rclMessage.cpp
        src/message/rclMessage.hpp
        src/message/serviceActionMessage.cpp
        src/message/vectorMessage.cpp
        src/message/vectorMessage.hpp
        src/publisher/publisherBase.hpp
        src/publisher/publisherManager.cpp
        src/publisher/publisherManager.hpp
        src/publisher/sharedPublisher.cpp
        src/publisher/sharedPublisher.hpp
        src/publisher/threadedPublisher.cpp
        src/publisher/threadedPublisher.hpp
        src/serviceAction/serviceActionManager.cpp
        src/serviceAction/serviceActionManager.hpp
        src/subscriber/clockSubscriber.cpp
        src/subscriber/clockSubscriber.hpp
        src/subscriber/sharedSubscriber.cpp
        src/subscriber/sharedSubscriber.hpp
        src/subscriber/subscriberBase.hpp
        src/subscriber/subscriberManager.cpp
        src/subscriber/subscriberManager.hpp
        src/subscriber/threadedSubscriber.cpp
        src/subscriber/threadedSubscriber.hpp
        src/websocketConnection/base/websocketConnectionBase.hpp
        src/websocketConnection/client/websocketClientConnection.hpp
        src/connectBase.hpp
        src/connectClient.cpp
)
target_include_directories(client PRIVATE ${Boost_INCLUDE_DIRS} src include)
target_link_libraries(client PUBLIC
        ${cpp_typesupport_target}
        Threads::Threads
        Boost::json
        OpenSSL::SSL
        ZLIB::ZLIB
        PkgConfig::lz4
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_action::rclcpp_action
)
target_compile_definitions(client PUBLIC PACKAGE_NAME="${PROJECT_NAME}")

# install node targets
install(TARGETS
        server
        client
        DESTINATION lib/${PROJECT_NAME}
)
# install header targets
install(TARGETS ${PROJECT_NAME}_headers
        EXPORT ${PROJECT_NAME}_headers_export
        INCLUDES DESTINATION include
)
# install launch files
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)
# install and export includes
install(DIRECTORY
        include/
        DESTINATION include
)

ament_export_targets(${PROJECT_NAME}_headers_export HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action pluginlib std_msgs rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
