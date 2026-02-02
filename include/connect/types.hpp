// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef TYPES_HPP
#define TYPES_HPP

#include <string>
#include <cstdint>

#include <lz4hc.h>
#include <zlib.h>

#include <rclcpp/rclcpp.hpp>

#define OP_CODE_MIN 249                     // inclusive
#define OP_CODE_MAX 255                     // inclusive

#define SERVICE_ACTION_OP_CODE_MIN 0        // inclusive
#define SERVICE_ACTION_OP_CODE_MAX 13       // inclusive

#define COMPRESSOR_MIN 0                    // inclusive
#define COMPRESSOR_MAX 3                    // inclusive

#define LZ4_DEFAULT_MIN_RATE 1              // inclusive
#define LZ4_DEFAULT_MAX_RATE 65537          // inclusive
#define LZ4_HC_MIN_RATE 1                   // inclusive
#define LZ4_HC_MAX_RATE LZ4HC_CLEVEL_MAX    // inclusive
#define ZLIB_MIN_RATE Z_BEST_SPEED          // inclusive
#define ZLIB_MAX_RATE Z_BEST_COMPRESSION    // inclusive

typedef enum Compressor : uint8_t {
    NONE = 0,            // extremely fast but no compression at all
    LZ4_DEFAULT = 1,     // very fast but low compression
    LZ4_HC = 2,          // faster than ZLIB for low compression rates, slower than ZLIB for high compression rates, always larger compressed size than ZLIB
    ZLIB = 3             // lowest compressed size overall
} compressor_t;

typedef struct {
    Compressor compressor;
    uint32_t rate;
} compression_t;

typedef enum OpCode : uint8_t {
    NOTIFICATION = 249,
    SUBSCRIPTION = 250,
    SERVICE_ACTION = 251,
    TF = 252,
    TF_STATIC = 253,
    CLOCK = 254,
    AUTHENTICATION = 255
} opcode_t;

typedef enum ServiceActionOpCode : uint8_t {
    SERVICE_TO_CLIENT = 0,
    SERVICE_TO_CLIENT_CANCEL = 1,
    SERVICE_TO_SERVER = 2,
    
    ACTION_TO_CLIENT_GOAL = 3,
    ACTION_TO_SERVER_GOAL_REJECT = 4,
    ACTION_TO_SERVER_GOAL_ACCEPT = 5,
    
    ACTION_TO_CLIENT_CANCEL = 6,
    ACTION_TO_SERVER_CANCEL_REJECT = 7,
    ACTION_TO_SERVER_CANCEL_ACCEPT = 8,

    ACTION_TO_SERVER_FEEDBACK = 9,
    ACTION_TO_SERVER_RESULT_SUCCEEDED = 10,
    ACTION_TO_SERVER_RESULT_ABORTED = 11,
    ACTION_TO_SERVER_RESULT_CANCELED = 12,
    ACTION_TO_SERVER_RESULT_UNKNOWN = 13
} service_action_opcode_t;

typedef struct {
    uint8_t channel;                      // channel to match subscriber and publisher; needed for communication protocol; unique in regard of publisher and subscriber
    std::string topic;                    // ros2 topic name, must start with '/'
    std::string type;                     // ros2 topic type
    bool useOwnThread;                    // if subscriber / publisher should use an own thread (ThreadedSubscriber, ThreadedPublisher) or use a shared thread
    bool permanent;                       // if a subscriber should not be destroyed if there is no local subscriber for the publisher on the other side [ignored for publisher]
    const rmw_qos_profile_t *qos;         // a qos profile
    const compression_t *compression;     // a compression profile [nullptr for publisher]
} Topic_t;

typedef struct {
    uint8_t channel;                      // channel to match service / action client and server; needed for communication protocol; unique in regard of services or actions
    std::string type;                     // the type of the service / action as defined in the plugins.xml e.g. connect_plugins::AddTwoIntsService
    bool useOwnThread;                    // if an exclusive callback group backed by an own thread should be used
    bool allowSimultaneous;               // only in combination with useOwnThread: if the exclusive callback group should be of type reentrant
    const rmw_qos_profile_t *serviceQos;  // the service qos profile
    const rmw_qos_profile_t *feedbackQos; // the feedback qos profile                                       [nullptr for service]
    const rmw_qos_profile_t *statusQos;   // the status qos profile                                         [nullptr for service]
    int32_t resultTimeout;                // the result timeout                                             [ignored for service; ignored got action client]
    uint32_t maxExecTime;                 // max execution time for a service server until being "canceled" [ignored for service client; ignored for action]
    const compression_t *compression;     // a compression profile
} Service_Action_t;

/**
 * QoS profile to be used for pre-defined publisher & subscriber for /clock and /tf
 */
constexpr rmw_qos_profile_t CLOCK_AND_TF_QOS = {
    .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    .depth = 100,
    .reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    . durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
    .deadline = RMW_QOS_DEADLINE_DEFAULT,
    .lifespan = RMW_QOS_LIFESPAN_DEFAULT,
    .liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    .liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    .avoid_ros_namespace_conventions = false
};

/**
 * QoS profile to be used for pre-defined publisher & subscriber for /tf_static
 */
constexpr rmw_qos_profile_t TF_STATIC_QOS = {
    .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    .depth = 100,
    .reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    . durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    .deadline = RMW_QOS_DEADLINE_DEFAULT,
    .lifespan = RMW_QOS_LIFESPAN_DEFAULT,
    .liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    .liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    .avoid_ros_namespace_conventions = false
};

/**
 * Default compression profile with no compression
 */
constexpr compression_t DEFAULT_COMPRESSION_PROFILE = {
    .compressor = Compressor::NONE,
    .rate = 0
};

#endif //TYPES_HPP
