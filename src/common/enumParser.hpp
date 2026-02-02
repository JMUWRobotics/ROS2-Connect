// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef ENUMPARSER_HPP
#define ENUMPARSER_HPP

#include "connect/types.hpp"

#include <regex>

#include <rclcpp/rclcpp.hpp>

#define RMW_TIME_REGEX R"(\{\s*(-?\d+)(?:[uU]*[lL]*)?\s*,\s*(\d+)(?:[uU]*[lL]*)?\s*\})"

class EnumParser {
public:
    /**
     * Parses a compressor of a Compression Profile into typed
     *
     * @param compressor string holding compressor
     * @return type holding compressor
     * @throws std::invalid_argument if compressor is not a known compressor
     */
    static Compressor compressor(const std::string &compressor) {
        if (compressor == "NONE") {
            return Compressor::NONE;
        } else if (compressor == "LZ4_DEFAULT") {
            return Compressor::LZ4_DEFAULT;
        } else if (compressor == "LZ4_HC") {
            return Compressor::LZ4_HC;
        } else if (compressor == "ZLIB") {
            return Compressor::ZLIB;
        } else {
            throw std::invalid_argument("Not a compressor: \"" + compressor + "\"");
        }
    }

    /**
     * Parses the history policy of a QoS definition into typed
     *
     * @param policy string holding history policy
     * @return type holding history policy
     * @throws std::invalid_argument if policy is not a known history policy
     */
    static rmw_qos_history_policy_t historyPolicy(const std::string &policy) {
        if (policy == "RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT") {
            return RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
        } else if (policy == "RMW_QOS_POLICY_HISTORY_KEEP_LAST") {
            return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        } else if (policy == "RMW_QOS_POLICY_HISTORY_KEEP_ALL") {
            return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        } else {
            throw std::invalid_argument("Not a history policy: \"" + policy + "\"");
        }
    }

    /**
     * Parses the reliability policy of a QoS definition into typed
     *
     * @param policy string holding reliability policy
     * @return type holding reliability policy
     * @throws std::invalid_argument if policy is not a known reliability policy
     */
    static rmw_qos_reliability_policy_t reliabilityPolicy(const std::string &policy) {
        if (policy == "RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT") {
            return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
        } else if (policy == "RMW_QOS_POLICY_RELIABILITY_RELIABLE") {
            return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        } else if (policy == "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT") {
            return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        } else {
            throw std::invalid_argument("Not a reliability policy: \"" + policy + "\"");
        }
    }

    /**
     * Parses the durability policy of a QoS definition into typed
     *
     * @param policy string holding durability policy
     * @return type holding durability policy
     * @throws std::invalid_argument if policy is not a known durability policy
     */
    static rmw_qos_durability_policy_t durabilityPolicy(const std::string &policy) {
        if (policy == "RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT") {
            return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
        } else if (policy == "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL") {
            return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        } else if (policy == "RMW_QOS_POLICY_DURABILITY_VOLATILE") {
            return RMW_QOS_POLICY_DURABILITY_VOLATILE;
        } else {
            throw std::invalid_argument("Not a durability policy: \"" + policy + "\"");
        }
    }

    /**
     * Parses the deadline policy of a QoS definition into typed
     *
     * @param policy string holding deadline policy
     * @return type holding deadline policy
     * @throws std::invalid_argument if policy is not a known deadline policy
     */
    static rmw_time_t deadlinePolicy(const std::string &policy) {
        if (policy == "RMW_QOS_DEADLINE_DEFAULT") {
            return RMW_QOS_DEADLINE_DEFAULT;
        } else if (policy == "RMW_QOS_DEADLINE_BEST_AVAILABLE") {
            return RMW_QOS_DEADLINE_BEST_AVAILABLE;
        } else {
            return EnumParser::parseRmwTime(policy, "deadline");
        }
    }

    /**
     * Parses the lifespan policy of a QoS definition into typed
     *
     * @param policy string holding lifespan policy
     * @return type holding lifespan policy
     * @throws std::invalid_argument if policy is not a known lifespan policy
     */
    static rmw_time_t lifespanPolicy(const std::string &policy) {
        if (policy == "RMW_QOS_LIFESPAN_DEFAULT") {
            return RMW_QOS_LIFESPAN_DEFAULT;
        } else {
            return EnumParser::parseRmwTime(policy, "lifespan");
        }
    }

    /**
     * Parses the liveliness policy of a QoS definition into typed
     *
     * @param policy string holding liveliness policy
     * @return type holding liveliness policy
     * @throws std::invalid_argument if policy is not a known liveliness policy
     */
    static rmw_qos_liveliness_policy_t livelinessPolicy(const std::string &policy) {
        if (policy == "RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT") {
            return RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
        } else if (policy == "RMW_QOS_POLICY_LIVELINESS_AUTOMATIC") {
            return RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
        } else if (policy == "RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC") {
            return RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
        } else if (policy == "RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE") {
            return RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE;
        } else {
            throw std::invalid_argument("Not a liveliness policy: \"" + policy + "\"");
        }
    }

    /**
     * Parses the livelinessLeaseDuration policy of a QoS definition into typed
     *
     * @param policy string holding livelinessLeaseDuration policy
     * @return type holding livelinessLeaseDuration policy
     * @throws std::invalid_argument if policy is not a known livelinessLeaseDuration policy
     */
    static rmw_time_t livelinessLeaseDurationPolicy(const std::string &policy) {
        if (policy == "RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT") {
            return RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
        } else if (policy == "RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE") {
            return RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE;
        } else {
            return EnumParser::parseRmwTime(policy, "livelinessLeaseDuration");
        }
    }

private:
    /**
     * Parses a value holding a inline rmw_time_t definition into a rmw_time t
     *
     * @param value inline rmw_time_t definition
     * @param policy name of policy to parse rmw_time_t for
     * @return parsed rmw_time_t
     * @throws std::invalid_argument if value does not hold a inline rmw_time_t definition
     */
    static rmw_time_t parseRmwTime(const std::string &value, const std::string &policy) {
        const std::regex timeRegex(RMW_TIME_REGEX);
        std::smatch match;
        if (std::regex_match(value, match, timeRegex)) {
            rmw_time_t parsedTime;
            parsedTime.sec = std::stoll(match[1].str());
            parsedTime.nsec = std::stoull(match[2].str());
            return parsedTime;
        } else {
            throw std::invalid_argument("Not a " + policy + " policy: \"" + value + "\"");
        }
    }
};

#endif //ENUMPARSER_HPP
