// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef CONNECTBASE_HPP
#define CONNECTBASE_HPP

#include <boost/json.hpp>
#include <limits>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/enumParser.hpp"
#include "connect/action/actionClient.hpp"
#include "connect/action/actionServer.hpp"
#include "connect/authentication.hpp"
#include "connect/service/serviceClient.hpp"
#include "connect/service/serviceServer.hpp"
#include "connect/types.hpp"
#include "global/globalConfig.hpp"

class ConnectBase : public rclcpp::Node {
   public:
    explicit ConnectBase(const std::string &node_name) : Node(node_name) {
    }

    /**
     * Evaluates the number of threads needed for the MultiThreadedExecutor.
     * This evaluation is based on the number of callbacks which need to be executed in parallel.
     *
     * This will never not return a number below 4 on success.
     *  - 1 thread for the mutually exclusive callback group of shared subscriber
     *  - 1 thread for the mutually exclusive callback group of services
     *  - 1 thread for the mutually exclusive callback group of actions
     *  - 1 thread for the default callback group (used for callbacks of publisher)
     *
     * @return number of threads needed, -1 on failure
     */
    int evalNrOfMultiThreadedExecutorThreads() {
        int nrOfThreads = 4;

        // test for useOwnThread flag
        const std::vector<std::string> useOwnThread = {"subscriber", "service/server", "service/client", "action/server", "action/client"};
        for (const std::string &parameter : useOwnThread) {
            this->declare_parameter<std::vector<std::string> >(parameter, std::vector<std::string>{});
            const std::vector<std::string> values = this->get_parameter(parameter).as_string_array();
            for (const std::string &parameterValue : values) {
                boost::json::object json;
                try {
                    boost::json::value value = boost::json::parse(parameterValue);

                    if (value.is_object())
                        json = value.as_object();
                    else
                        throw std::invalid_argument(parameter + " is not a JSON object");

                    if (!json["useOwnThread"].is_bool()) throw std::invalid_argument(parameter + " does not hold \"useOwnThread\" as bool");
                } catch (std::exception &e) {
                    RCLCPP_FATAL(this->get_logger(), "%s '%s' is not well formatted. %s", parameter.c_str(), parameterValue.c_str(), e.what());
                    rclcpp::shutdown(nullptr, parameter + " are not well formatted");
                    return -1;
                }

                if (json["useOwnThread"].as_bool()) nrOfThreads += 1;
            }
        }

        // test for clock and tf
        // "clock/subscribe" uses explicit thread and executed a boost io context
        this->declare_parameter<bool>("clock/publish", false);
        this->declare_parameter<bool>("tf/publish", false);
        this->declare_parameter<bool>("tf/subscribe", false);

        nrOfThreads += (this->get_parameter("clock/publish").as_bool() ? 1 : 0);
        nrOfThreads += (this->get_parameter("tf/publish").as_bool() ? 1 : 0);
        nrOfThreads += (this->get_parameter("tf/subscribe").as_bool() ? 1 : 0);

        return nrOfThreads;
    }

    /**
     * Initialize the environment
     *
     * @param authentication if the authentication parameter should be validated (typically true for server, false for client)
     * @return if initializing was successful
     */
    bool init(const bool authentication) {
        // the actual name may differ when started using ros2 launch
        GlobalConfig::nodeName = this->get_name();
        GlobalConfig::nodeNamespace = this->get_namespace();
        
        // first resolve and print out the domain id
        // then initialize all the parameters and therefore the global config
        return this->resolveDomain() && this->initializeParameters(authentication);
    }

    /**
     * Spins the node by creating either a websocket server or client
     *
     * @param executor the executor in which executes this
     */
    virtual void spin(rclcpp::executors::MultiThreadedExecutor &executor) = 0;

    /**
     * Hals the node by stopping the websocket server os client
     */
    virtual void halt() = 0;

   private:
    /**
     * Resolves and prints out the DDS domain ID with which this node runs
     *
     * @returns true on success, false on failure
     */
    bool resolveDomain() {
        const rcl_node_t *rclNodeHandle = this->get_node_base_interface()->get_rcl_node_handle();
        size_t domain;
        const rcl_ret_t ret = rcl_node_get_domain_id(rclNodeHandle, &domain);

        if (ret != RCL_RET_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error on resolve domain ID: %s", rcl_get_error_string().str);
            return false;
        } else {
            GlobalConfig::nodeDomainId = domain;
            RCLCPP_INFO(this->get_logger(), "Running on domain ID: %lu", domain);
            return true;
        }
    }

    /**
     * Initializes all ros2 parameters by first declaring them with their respective
     * default value, retrieving them and printing them out for the user to verify
     *
     * @param authentication if the authentication parameter should be validated (typically true for server, false for client)
     * @returns true on success, false on failure
     */
    bool initializeParameters(const bool authentication) {
        // declare parameters with default values
        this->declare_parameter<std::string>("host", "127.0.0.1");
        this->declare_parameter<std::string>("port", "9090");

        this->declare_parameter<std::string>("path", "/");  // has no effect on server
        this->declare_parameter<bool>("ssl", false);        // has no effect on server (not implemented)

        this->declare_parameter<std::string>("authentication/host", "127.0.0.1");
        this->declare_parameter<std::string>("authentication/port", "8080");
        this->declare_parameter<bool>("authentication/ssl", false);

        this->declare_parameter<std::string>("authentication/plugin", "");
        this->declare_parameter<std::string>("authentication/endpoint", "/");
        this->declare_parameter<bool>("authentication/omit", false);
        this->declare_parameter<int>("authentication/timeout", 10);

        this->declare_parameter<bool>("fragmentation/enable", true);
        this->declare_parameter<int>("fragmentation/size", 4096);  // default value according to boost beast documentation

        this->declare_parameter<int>("max_message_size", 16 * 1024 * 1024);  // default value according to boost beast documentation

        this->declare_parameter<std::string>("user_key", "");

        this->declare_parameter<bool>("clock/subscribe", false);  // creates a ClockSubscriber
        this->declare_parameter<bool>("clock/publish", false);    // creates a ThreadedPublisher on /clock

        this->declare_parameter<bool>("tf/subscribe", false);  // creates two threaded subscriber
        this->declare_parameter<bool>("tf/publish", false);    // creates two threaded publisher

        this->declare_parameter<std::vector<std::string> >("subscriber", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string> >("publisher", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string> >("service/server", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string> >("service/client", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string> >("action/server", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string> >("action/client", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string> >("qos", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string> >("compression", std::vector<std::string>{});

        // retrieve the parameter values
        this->get_parameter("host", GlobalConfig::host);
        this->get_parameter("port", GlobalConfig::port);

        this->get_parameter("path", GlobalConfig::path);
        this->get_parameter("ssl", GlobalConfig::ssl);

        this->get_parameter("authentication/host", GlobalConfig::authenticationHost);
        this->get_parameter("authentication/port", GlobalConfig::authenticationPort);
        this->get_parameter("authentication/ssl", GlobalConfig::authenticationSsl);

        this->get_parameter("authentication/plugin", GlobalConfig::authenticationPlugin);
        this->get_parameter("authentication/endpoint", GlobalConfig::authenticationEndpoint);
        this->get_parameter("authentication/omit", GlobalConfig::authenticationOmit);
        this->get_parameter("authentication/timeout", GlobalConfig::authenticationTimeout);

        this->get_parameter("fragmentation/enable", GlobalConfig::fragmentation);
        this->get_parameter("fragmentation/size", GlobalConfig::fragmentationSize);

        this->get_parameter("max_message_size", GlobalConfig::maxMessageSize);

        this->get_parameter("user_key", GlobalConfig::userKey);

        this->get_parameter("clock/subscribe", GlobalConfig::subscribeClock);
        this->get_parameter("clock/publish", GlobalConfig::publishClock);

        this->get_parameter("tf/subscribe", GlobalConfig::subscribeTf);
        this->get_parameter("tf/publish", GlobalConfig::publishTf);

        // retrieve and parse the qos definitions
        const std::vector<std::string> qosDefinitions = this->get_parameter("qos").as_string_array();
        std::vector<uint8_t> known;
        for (const std::string &qos : qosDefinitions) {
            int64_t id;
            rmw_qos_history_policy_t history;
            size_t depth;
            rmw_qos_reliability_policy_t reliability;
            rmw_qos_durability_policy_t durability;
            rmw_time_t deadline;
            rmw_time_t lifespan;
            rmw_qos_liveliness_policy_t liveliness;
            rmw_time_t livelinessLeaseDuration;

            try {
                boost::json::value value = boost::json::parse(qos);
                boost::json::object json;

                if (value.is_object())
                    json = value.as_object();
                else
                    throw std::invalid_argument("QoS Definition is not a JSON object");

                if (!json["id"].is_int64()) throw std::invalid_argument("QoS Definition does not hold \"id\" as int");
                if (!json["history"].is_string()) throw std::invalid_argument("QoS Definition does not hold \"history\" as string");
                if (!json["depth"].is_int64()) throw std::invalid_argument("QoS Definition does not hold \"depth\" as int");
                if (!json["reliability"].is_string()) throw std::invalid_argument("QoS Definition does not hold \"reliability\" as string");
                if (!json["durability"].is_string()) throw std::invalid_argument("QoS Definition does not hold \"durability\" as string");

                history = EnumParser::historyPolicy(boost::json::value_to<std::string>(json["history"]));
                depth = json["depth"].as_int64();
                reliability = EnumParser::reliabilityPolicy(boost::json::value_to<std::string>(json["reliability"]));
                durability = EnumParser::durabilityPolicy(boost::json::value_to<std::string>(json["durability"]));

                if (depth < 1) throw std::invalid_argument("QoS Definition holds \"depth\" smaller than 1");

                if (boost::json::value *val = json.if_contains("deadline")) {
                    if (val->is_string())
                        deadline = EnumParser::deadlinePolicy(boost::json::value_to<std::string>(*val));
                    else
                        throw std::invalid_argument("QoS Definition does not hold \"deadline\" as string");
                } else
                    deadline = RMW_QOS_DEADLINE_DEFAULT;

                if (boost::json::value *val = json.if_contains("lifespan")) {
                    if (val->is_string())
                        lifespan = EnumParser::lifespanPolicy(boost::json::value_to<std::string>(*val));
                    else
                        throw std::invalid_argument("QoS Definition does not hold \"lifespan\" as string");
                } else
                    lifespan = RMW_QOS_LIFESPAN_DEFAULT;

                if (boost::json::value *val = json.if_contains("liveliness")) {
                    if (val->is_string())
                        liveliness = EnumParser::livelinessPolicy(boost::json::value_to<std::string>(*val));
                    else
                        throw std::invalid_argument("QoS Definition does not hold \"liveliness\" as string");
                } else
                    liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;

                if (boost::json::value *val = json.if_contains("liveliness_lease_duration")) {
                    if (val->is_string())
                        livelinessLeaseDuration = EnumParser::livelinessLeaseDurationPolicy(boost::json::value_to<std::string>(*val));
                    else
                        throw std::invalid_argument("QoS Definition does not hold \"liveliness_lease_duration\" as string");
                } else
                    livelinessLeaseDuration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;

                id = json["id"].as_int64();
                if (id < std::numeric_limits<uint8_t>::min() || id > std::numeric_limits<uint8_t>::max()) throw std::invalid_argument("QoS Definition does not hold \"id\" as uint8_t");

                for (const uint8_t &val : known) {
                    if (val == id) throw std::invalid_argument("There are at least two QoS definitions which hold the same \"id\"");
                }
                known.push_back(id);
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "QoS Definition '%s' is not well formatted. %s", qos.c_str(), e.what());
                rclcpp::shutdown(nullptr, "QoS Definitions are not well formatted");
                return false;
            }

            rmw_qos_profile_t profile = {
                history,
                depth,
                reliability,
                durability,
                deadline,
                lifespan,
                liveliness,
                livelinessLeaseDuration,
                false};

            GlobalConfig::qos.insert(std::pair<uint8_t, rmw_qos_profile_t>(id, profile));
        }

        // retrieve and parse the compression profiles
        const std::vector<std::string> compressionProfiles = this->get_parameter("compression").as_string_array();
        known.clear();
        for (const std::string &compression : compressionProfiles) {
            int64_t id;
            Compressor compressor;
            int64_t rate;

            try {
                boost::json::value value = boost::json::parse(compression);
                boost::json::object json;

                if (value.is_object())
                    json = value.as_object();
                else
                    throw std::invalid_argument("Compression Profile is not a JSON object");

                if (!json["id"].is_int64())
                    throw std::invalid_argument("Compression Profile does not hold \"id\" as int");
                else if (!json["compressor"].is_string())
                    throw std::invalid_argument("Compression Profile does not hold \"compressor\" as string");
                else if (!json["rate"].is_int64())
                    throw std::invalid_argument("Compression Profile does not hold \"rate\" as int");

                compressor = EnumParser::compressor(boost::json::value_to<std::string>(json["compressor"]));

                rate = json["rate"].as_int64();
                if (compressor == Compressor::LZ4_DEFAULT && (rate < LZ4_DEFAULT_MIN_RATE || rate > LZ4_DEFAULT_MAX_RATE))
                    throw std::invalid_argument("Compression Profile does hold \"rate\" out of range");
                else if (compressor == Compressor::LZ4_HC && (rate < LZ4_HC_MIN_RATE || rate > LZ4_HC_MAX_RATE))
                    throw std::invalid_argument("Compression Profile does hold \"rate\" out of range");
                else if (compressor == Compressor::ZLIB && (rate < ZLIB_MIN_RATE || rate > ZLIB_MAX_RATE))
                    throw std::invalid_argument("Compression Profile does hold \"rate\" out of range");
                else if (rate < std::numeric_limits<uint32_t>::min() || rate > std::numeric_limits<uint32_t>::max())
                    throw std::invalid_argument("Compression Profile does hold \"rate\" out of range");

                id = json["id"].as_int64();
                if (id < std::numeric_limits<uint8_t>::min() || id > std::numeric_limits<uint8_t>::max()) throw std::invalid_argument("Compression Profile does not hold \"id\" as uint8_t");

                for (const uint8_t &val : known) {
                    if (val == id) throw std::invalid_argument("There are at least two Compression Profiles which hold the same \"id\"");
                }
                known.push_back(id);
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Compression Profile '%s' is not well formatted. %s", compression.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Compression Profiles are not well formatted");
                return false;
            }

            compression_t profile = {
                .compressor = compressor,
                .rate = static_cast<uint32_t>(rate)};

            GlobalConfig::compression.insert(std::pair<uint8_t, compression_t>(id, profile));
        }

        // retrieve and parse the subscriber
        const std::vector<std::string> subscriber = this->get_parameter("subscriber").as_string_array();
        known.clear();
        for (const std::string &sub : subscriber) {
            boost::json::object json;

            rmw_qos_profile_t *qos;
            compression_t *compression;

            try {
                boost::json::value value = boost::json::parse(sub);

                if (value.is_object())
                    json = value.as_object();
                else
                    throw std::invalid_argument("Subscriber is not a JSON object");

                if (!json["channel"].is_int64())
                    throw std::invalid_argument("Subscriber does not hold \"channel\" as int");
                else if (!json["topic"].is_string())
                    throw std::invalid_argument("Subscriber does not hold \"topic\" as string");
                else if (!json["type"].is_string())
                    throw std::invalid_argument("Subscriber does not hold \"type\" as string");
                else if (!json["useOwnThread"].is_bool())
                    throw std::invalid_argument("Subscriber does not hold \"useOwnThread\" as bool");
                else if (!json["permanent"].is_bool())
                    throw std::invalid_argument("Subscriber does not hold \"permanent\" as bool");
                else if (!json["qos"].is_int64())
                    throw std::invalid_argument("Subscriber does not hold \"qos\" as int");
                else if (!json["compression"].is_int64())
                    throw std::invalid_argument("Subscriber does not hold \"compression\" as int");

                // test the topic itself
                const std::string topic = boost::json::value_to<std::string>(json["topic"]);
                if (topic == "/tf") {
                    throw std::invalid_argument("Subscriber for topic \"/tf\" cannot be created manually. Use parameter \"tf/subscribe\"");
                } else if (topic == "/tf_static") {
                    throw std::invalid_argument("Subscriber for topic \"/tf_static\" cannot be created manually. Use parameter \"tf/subscribe\"");
                } else if (topic == "/clock") {
                    throw std::invalid_argument("Subscriber for topic \"/clock\" cannot be created manually. Use parameter \"clock/subscribe\"");
                }

                // get the qos
                if (int64_t *val = json["qos"].if_int64()) {
                    const std::map<uint8_t, rmw_qos_profile_t>::iterator it = GlobalConfig::qos.find(*val);
                    if (it == GlobalConfig::qos.end())
                        throw std::invalid_argument("Subscriber holds unknown \"qos\" id");
                    else
                        qos = &it->second;
                } else
                    throw std::invalid_argument("Subscriber holds unknown \"qos\" id");

                // get the compression profile
                if (int64_t *val = json["compression"].if_int64()) {
                    const std::map<uint8_t, compression_t>::iterator itt = GlobalConfig::compression.find(*val);
                    if (itt == GlobalConfig::compression.end())
                        throw std::invalid_argument("Subscriber holds unknown \"compression\" id");
                    else
                        compression = &itt->second;
                } else
                    throw std::invalid_argument("Subscriber holds unknown \"compression\" id");

                // test for compression
                if (compression->compressor != Compressor::NONE && !json["useOwnThread"].as_bool()) throw std::invalid_argument("Subscriber uses compression but not own thread");

                const int64_t channel = json["channel"].as_int64();
                if (channel < std::numeric_limits<uint8_t>::min() || channel > std::numeric_limits<uint8_t>::max()) throw std::invalid_argument("Subscriber does not hold \"channel\" as uint8_t");
                if (channel >= OP_CODE_MIN && channel <= OP_CODE_MAX) throw std::invalid_argument("Subscriber does hold a reserved \"channel\"");

                for (const uint8_t &val : known) {
                    if (val == channel) throw std::invalid_argument("There are at least two subscribers which hold the same \"channel\"");
                }
                known.push_back(channel);
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Subscriber '%s' is not well formatted. %s", sub.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Subscribers are not well formatted");
                return false;
            }

            const Topic_t topic = {
                .channel = static_cast<uint8_t>(json["channel"].as_int64()),
                .topic = boost::json::value_to<std::string>(json["topic"]),
                .type = boost::json::value_to<std::string>(json["type"]),
                .useOwnThread = json["useOwnThread"].as_bool(),
                .permanent = json["permanent"].as_bool(),
                .qos = qos,
                .compression = compression};

            try {
                std::shared_ptr<rclcpp::GenericSubscription> _subscription = this->create_generic_subscription(topic.topic, topic.type, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*topic.qos), *topic.qos), [](const std::shared_ptr<const rclcpp::SerializedMessage> & /* serializedMessage */) {
                });
                _subscription.reset();
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Subscriber '%s' cannot be created. %s", sub.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Subscribers cannot be created");
                return false;
            }

            GlobalConfig::subscriber.push_back(topic);
        }

        // retrieve and parse the publisher
        const std::vector<std::string> publisher = this->get_parameter("publisher").as_string_array();
        known.clear();
        for (const std::string &pub : publisher) {
            boost::json::object json;

            rmw_qos_profile_t *qos;

            try {
                boost::json::value value = boost::json::parse(pub);

                if (value.is_object())
                    json = value.as_object();
                else
                    throw std::invalid_argument("Publisher is not a JSON object");

                if (!json["channel"].is_int64())
                    throw std::invalid_argument("Publisher does not hold \"channel\" as int");
                else if (!json["topic"].is_string())
                    throw std::invalid_argument("Publisher does not hold \"topic\" as string");
                else if (!json["type"].is_string())
                    throw std::invalid_argument("Publisher does not hold \"type\" as string");
                else if (!json["useOwnThread"].is_bool())
                    throw std::invalid_argument("Publisher does not hold \"useOwnThread\" as bool");
                else if (!json["qos"].is_int64())
                    throw std::invalid_argument("Publisher does not hold \"qos\" as int");

                // test the topic itself
                const std::string topic = boost::json::value_to<std::string>(json["topic"]);
                if (topic == "/tf") {
                    throw std::invalid_argument("Publisher for topic \"/tf\" cannot be created manually. Use parameter \"tf/publish\"");
                } else if (topic == "/tf_static") {
                    throw std::invalid_argument("Publisher for topic \"/tf_static\" cannot be created manually. Use parameter \"tf/publish\"");
                } else if (topic == "/clock") {
                    throw std::invalid_argument("Publisher for topic \"/clock\" cannot be created manually. Use parameter \"clock/publish\"");
                }

                // get the qos
                if (int64_t *val = json["qos"].if_int64()) {
                    const std::map<uint8_t, rmw_qos_profile_t>::iterator it = GlobalConfig::qos.find(*val);
                    if (it == GlobalConfig::qos.end())
                        throw std::invalid_argument("Publisher holds unknown \"qos\" id");
                    else
                        qos = &it->second;
                } else
                    throw std::invalid_argument("Publisher holds unknown \"qos\" id");

                const int64_t channel = json["channel"].as_int64();
                if (channel < std::numeric_limits<uint8_t>::min() || channel > std::numeric_limits<uint8_t>::max()) throw std::invalid_argument("Publisher does not hold \"channel\" as uint8_t");
                if (channel >= OP_CODE_MIN && channel <= OP_CODE_MAX) throw std::invalid_argument("Publisher does hold a reserved \"channel\"");

                for (const uint8_t &val : known) {
                    if (val == channel) throw std::invalid_argument("There are at least two publisher which hold the same \"channel\"");
                }
                for (const Topic_t &subscriber : GlobalConfig::subscriber) {
                    if (subscriber.channel == channel) throw std::invalid_argument("There is a publisher and subscriber which hold both the same \"channel\"");
                }
                known.push_back(channel);
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Publisher '%s' is not well formatted. %s", pub.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Publishers are not well formatted");
                return false;
            }

            const Topic_t topic = {
                .channel = static_cast<uint8_t>(json["channel"].as_int64()),
                .topic = boost::json::value_to<std::string>(json["topic"]),
                .type = boost::json::value_to<std::string>(json["type"]),
                .useOwnThread = json["useOwnThread"].as_bool(),
                .permanent = true,  // not relevant for publisher
                .qos = qos,
                .compression = &DEFAULT_COMPRESSION_PROFILE  // not relevant for publisher
            };

            try {
                std::shared_ptr<rclcpp::GenericPublisher> _publisher = this->create_generic_publisher(topic.topic, topic.type, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(*topic.qos), *topic.qos));
                _publisher.reset();
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Publisher '%s' cannot be created. %s", pub.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Publishers cannot be created");
                return false;
            }

            GlobalConfig::publisher.push_back(topic);
        }

        // retrieve and parse the service servers
        const std::vector<std::string> serviceServer = this->get_parameter("service/server").as_string_array();
        known.clear();
        pluginlib::ClassLoader<service::ServiceServer> serviceServerLoader(PACKAGE_NAME, "service::ServiceServer");
        for (const std::string &serv : serviceServer) {
            boost::json::object json;

            rmw_qos_profile_t *qos;
            compression_t *compression;

            try {
                boost::json::value value = boost::json::parse(serv);

                if (value.is_object())
                    json = value.as_object();
                else
                    throw std::invalid_argument("Service Server is not a JSON object");

                if (!json["channel"].is_int64())
                    throw std::invalid_argument("Service Server does not hold \"channel\" as int");
                else if (!json["type"].is_string())
                    throw std::invalid_argument("Service Server does not hold \"type\" as string");
                else if (!json["maxExecTime"].is_int64())
                    throw std::invalid_argument("Service Server does not hold \"maxExecTime\" as int");
                else if (!json["useOwnThread"].is_bool())
                    throw std::invalid_argument("Service Server does not hold \"useOwnThread\" as bool");
                else if (!json["allowSimultaneous"].is_bool())
                    throw std::invalid_argument("Service Server does not hold \"allowSimultaneous\" as bool");

                // test for simultaneous execution
                if (json["allowSimultaneous"].as_bool() && !json["useOwnThread"].as_bool()) throw std::invalid_argument("Service Server defines simultaneous execution but not usage of own thread");

                // get the qos
                if (int64_t *val = json["service-qos"].if_int64()) {
                    const std::map<uint8_t, rmw_qos_profile_t>::iterator it = GlobalConfig::qos.find(*val);
                    if (it == GlobalConfig::qos.end())
                        throw std::invalid_argument("Service Server holds unknown \"service-qos\" id");
                    else
                        qos = &it->second;
                } else
                    throw std::invalid_argument("Service Server holds unknown \"service-qos\" id");

                // get the compression profile
                if (int64_t *val = json["compression"].if_int64()) {
                    const std::map<uint8_t, compression_t>::iterator itt = GlobalConfig::compression.find(*val);
                    if (itt == GlobalConfig::compression.end())
                        throw std::invalid_argument("Service Server holds unknown \"compression\" id");
                    else
                        compression = &itt->second;
                } else
                    throw std::invalid_argument("Service Server holds unknown \"compression\" id");

                // test for compression
                if (compression->compressor != Compressor::NONE && !json["useOwnThread"].as_bool()) throw std::invalid_argument("Service Server uses compression but not own thread");

                const int64_t maxExecTime = json["maxExecTime"].as_int64();
                if (maxExecTime < std::numeric_limits<uint32_t>::min() || maxExecTime > std::numeric_limits<uint32_t>::max()) throw std::invalid_argument("Service Server does not hold \"maxExecTime\" as uint32_t");

                const int64_t channel = json["channel"].as_int64();
                if (channel < std::numeric_limits<uint8_t>::min() || channel > std::numeric_limits<uint8_t>::max()) throw std::invalid_argument("Service Server does not hold \"channel\" as uint8_t");

                for (const uint8_t &val : known) {
                    if (val == channel) throw std::invalid_argument("There are at least two service server which hold the same \"channel\"");
                }
                known.push_back(channel);
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Service Server '%s' is not well formatted. %s", serv.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Service Server is not well formatted");
                return false;
            }

            Service_Action_t service = {
                .channel = static_cast<uint8_t>(json["channel"].as_int64()),
                .type = boost::json::value_to<std::string>(json["type"]),
                .useOwnThread = json["useOwnThread"].as_bool(),
                .allowSimultaneous = json["allowSimultaneous"].as_bool(),
                .serviceQos = qos,
                .feedbackQos = qos,  // not relevant for service
                .statusQos = qos,    // not relevant for service
                .resultTimeout = 0,
                .maxExecTime = static_cast<uint32_t>(json["maxExecTime"].as_int64()),
                .compression = compression};

            try {
                std::shared_ptr<service::ServiceServer> _service = serviceServerLoader.createSharedInstance(service.type);
                _service->test(service, this->get_node_base_interface(), this->get_node_graph_interface(), this->get_node_services_interface(), this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
                _service.reset();
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Service Server '%s' cannot be created. %s", serv.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Service Servers cannot be created");
                return false;
            }

            GlobalConfig::serviceServer.push_back(service);
        }

        // retrieve and parse the service clients
        const std::vector<std::string> serviceClients = this->get_parameter("service/client").as_string_array();
        known.clear();
        pluginlib::ClassLoader<service::ServiceClient> serviceClientLoader(PACKAGE_NAME, "service::ServiceClient");
        for (const std::string &clie : serviceClients) {
            boost::json::object json;

            rmw_qos_profile_t *qos;
            compression_t *compression;

            try {
                boost::json::value value = boost::json::parse(clie);

                if (value.is_object())
                    json = value.as_object();
                else
                    throw std::invalid_argument("Service Client is not a JSON object");

                if (!json["channel"].is_int64())
                    throw std::invalid_argument("Service Client does not hold \"channel\" as int");
                else if (!json["type"].is_string())
                    throw std::invalid_argument("Service Client does not hold \"type\" as string");
                else if (!json["useOwnThread"].is_bool())
                    throw std::invalid_argument("Service Client does not hold \"useOwnThread\" as bool");
                else if (!json["allowSimultaneous"].is_bool())
                    throw std::invalid_argument("Service Client does not hold \"allowSimultaneous\" as bool");

                // test for simultaneous execution
                if (json["allowSimultaneous"].as_bool() && !json["useOwnThread"].as_bool()) throw std::invalid_argument("Service Client defines simultaneous execution but not usage of own thread");

                // get the qos
                if (int64_t *val = json["service-qos"].if_int64()) {
                    const std::map<uint8_t, rmw_qos_profile_t>::iterator it = GlobalConfig::qos.find(*val);
                    if (it == GlobalConfig::qos.end())
                        throw std::invalid_argument("Service Client holds unknown \"service-qos\" id");
                    else
                        qos = &it->second;
                } else
                    throw std::invalid_argument("Service Client holds unknown \"service-qos\" id");

                // get the compression profile
                if (int64_t *val = json["compression"].if_int64()) {
                    const std::map<uint8_t, compression_t>::iterator itt = GlobalConfig::compression.find(*val);
                    if (itt == GlobalConfig::compression.end())
                        throw std::invalid_argument("Service Client holds unknown \"compression\" id");
                    else
                        compression = &itt->second;
                } else
                    throw std::invalid_argument("Service Client holds unknown \"compression\" id");

                // test for compression
                if (compression->compressor != Compressor::NONE && !json["useOwnThread"].as_bool()) throw std::invalid_argument("Service Client uses compression but not own thread");

                const int64_t channel = json["channel"].as_int64();
                if (channel < std::numeric_limits<uint8_t>::min() || channel > std::numeric_limits<uint8_t>::max()) throw std::invalid_argument("Service Client does not hold \"channel\" as uint8_t");

                for (const uint8_t &val : known) {
                    if (val == channel) throw std::invalid_argument("There are at least two service clients which hold the same \"channel\"");
                }
                for (const Service_Action_t &server : GlobalConfig::serviceServer) {
                    if (server.channel == channel) throw std::invalid_argument("There is a service client and server which hold both the same \"channel\"");
                }
                known.push_back(channel);
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Service Client '%s' is not well formatted. %s", clie.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Service Client is not well formatted");
                return false;
            }

            Service_Action_t service = {
                .channel = static_cast<uint8_t>(json["channel"].as_int64()),
                .type = boost::json::value_to<std::string>(json["type"]),
                .useOwnThread = json["useOwnThread"].as_bool(),
                .allowSimultaneous = json["allowSimultaneous"].as_bool(),
                .serviceQos = qos,
                .feedbackQos = qos,  // not relevant for service
                .statusQos = qos,    // not relevant for service
                .resultTimeout = 0,
                .maxExecTime = 0,
                .compression = compression};

            try {
                std::shared_ptr<service::ServiceClient> _service = serviceClientLoader.createSharedInstance(service.type);
                _service->test(service, this->get_node_base_interface(), this->get_node_graph_interface(), this->get_node_services_interface(), this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
                _service.reset();
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Service Client '%s' cannot be created. %s", clie.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Service Clients cannot be created");
                return false;
            }

            GlobalConfig::serviceClient.push_back(service);
        }

        // retrieve and parse the action servers
        const std::vector<std::string> actionServer = this->get_parameter("action/server").as_string_array();
        known.clear();
        pluginlib::ClassLoader<action::ActionServer> actionServerLoader(PACKAGE_NAME, "action::ActionServer");
        for (const std::string &serv : actionServer) {
            boost::json::object json;

            rmw_qos_profile_t *serviceQos;
            rmw_qos_profile_t *feedbackQos;
            rmw_qos_profile_t *statusQos;
            compression_t *compression;

            try {
                boost::json::value value = boost::json::parse(serv);

                if (value.is_object())
                    json = value.as_object();
                else
                    throw std::invalid_argument("Action Server is not a JSON object");

                if (!json["channel"].is_int64())
                    throw std::invalid_argument("Action Server does not hold \"channel\" as int");
                else if (!json["type"].is_string())
                    throw std::invalid_argument("Action Server does not hold \"type\" as string");
                else if (!json["resultTimeout"].is_int64())
                    throw std::invalid_argument("Action Server does not hold \"resultTimeout\" as int");
                else if (!json["useOwnThread"].is_bool())
                    throw std::invalid_argument("Action Server does not hold \"useOwnThread\" as bool");
                else if (!json["allowSimultaneous"].is_bool())
                    throw std::invalid_argument("Action Server does not hold \"allowSimultaneous\" as bool");

                // test for simultaneous execution
                if (json["allowSimultaneous"].as_bool() && !json["useOwnThread"].as_bool()) throw std::invalid_argument("Action Server defines simultaneous execution but not usage of own thread");

                // get the qos
                if (int64_t *val = json["service-qos"].if_int64()) {
                    const std::map<uint8_t, rmw_qos_profile_t>::iterator serviceIt = GlobalConfig::qos.find(*val);
                    if (serviceIt == GlobalConfig::qos.end())
                        throw std::invalid_argument("Action Server holds unknown \"service-qos\" id");
                    else
                        serviceQos = &serviceIt->second;
                } else
                    throw std::invalid_argument("Action Server holds unknown \"service-qos\" id");

                if (int64_t *val = json["feedback-qos"].if_int64()) {
                    const std::map<uint8_t, rmw_qos_profile_t>::iterator feedbackIt = GlobalConfig::qos.find(*val);
                    if (feedbackIt == GlobalConfig::qos.end())
                        throw std::invalid_argument("Action Server holds unknown \"feedback-qos\" id");
                    else
                        feedbackQos = &feedbackIt->second;
                } else
                    throw std::invalid_argument("Action Server holds unknown \"feedback-qos\" id");

                if (int64_t *val = json["status-qos"].if_int64()) {
                    const std::map<uint8_t, rmw_qos_profile_t>::iterator statusIt = GlobalConfig::qos.find(*val);
                    if (statusIt == GlobalConfig::qos.end())
                        throw std::invalid_argument("Action Server holds unknown \"status-qos\" id");
                    else
                        statusQos = &statusIt->second;
                } else
                    throw std::invalid_argument("Action Server holds unknown \"status-qos\" id");

                // get the compression profile
                if (int64_t *val = json["compression"].if_int64()) {
                    const std::map<uint8_t, compression_t>::iterator itt = GlobalConfig::compression.find(*val);
                    if (itt == GlobalConfig::compression.end())
                        throw std::invalid_argument("Action Server holds unknown \"compression\" id");
                    else
                        compression = &itt->second;
                } else
                    throw std::invalid_argument("Action Server holds unknown \"compression\" id");

                // test for compression
                if (compression->compressor != Compressor::NONE && !json["useOwnThread"].as_bool()) throw std::invalid_argument("Action Server uses compression but not own thread");

                const int64_t resultTimeout = json["resultTimeout"].as_int64();
                if (resultTimeout < std::numeric_limits<int32_t>::min() || resultTimeout > std::numeric_limits<int32_t>::max()) throw std::invalid_argument("Action Server does not hold \"resultTimeout\" as int32_t");

                const int64_t channel = json["channel"].as_int64();
                if (channel < std::numeric_limits<uint8_t>::min() || channel > std::numeric_limits<uint8_t>::max()) throw std::invalid_argument("Action Server does not hold \"channel\" as uint8_t");

                for (const uint8_t &val : known) {
                    if (val == channel) throw std::invalid_argument("There are at least two action server which hold the same \"channel\"");
                }
                known.push_back(channel);
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Action Server '%s' is not well formatted. %s", serv.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Action Server is not well formatted");
                return false;
            }

            Service_Action_t action = {
                .channel = static_cast<uint8_t>(json["channel"].as_int64()),
                .type = boost::json::value_to<std::string>(json["type"]),
                .useOwnThread = json["useOwnThread"].as_bool(),
                .allowSimultaneous = json["allowSimultaneous"].as_bool(),
                .serviceQos = serviceQos,
                .feedbackQos = feedbackQos,
                .statusQos = statusQos,
                .resultTimeout = static_cast<int32_t>(json["resultTimeout"].as_int64()),
                .maxExecTime = 0,
                .compression = compression};

            try {
                std::shared_ptr<action::ActionServer> _action = actionServerLoader.createSharedInstance(action.type);
                _action->test(action, this->get_node_base_interface(), this->get_node_graph_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(), this->get_node_waitables_interface(), this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
                _action.reset();
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Action Server '%s' cannot be created. %s", serv.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Action Servers cannot be created");
                return false;
            }

            GlobalConfig::actionServer.push_back(action);
        }

        // retrieve and parse the action clients
        const std::vector<std::string> actionClients = this->get_parameter("action/client").as_string_array();
        known.clear();
        pluginlib::ClassLoader<action::ActionClient> actionClientLoader(PACKAGE_NAME, "action::ActionClient");
        for (const std::string &clie : actionClients) {
            boost::json::object json;

            rmw_qos_profile_t *serviceQos;
            rmw_qos_profile_t *feedbackQos;
            rmw_qos_profile_t *statusQos;
            compression_t *compression;

            try {
                boost::json::value value = boost::json::parse(clie);

                if (value.is_object())
                    json = value.as_object();
                else
                    throw std::invalid_argument("Action Client is not a JSON object");

                if (!json["channel"].is_int64())
                    throw std::invalid_argument("Action Client does not hold \"channel\" as int");
                else if (!json["type"].is_string())
                    throw std::invalid_argument("Action Client does not hold \"type\" as string");
                else if (!json["useOwnThread"].is_bool())
                    throw std::invalid_argument("Action Client does not hold \"useOwnThread\" as bool");
                else if (!json["allowSimultaneous"].is_bool())
                    throw std::invalid_argument("Action Client does not hold \"allowSimultaneous\" as bool");

                // test for simultaneous execution
                if (json["allowSimultaneous"].as_bool() && !json["useOwnThread"].as_bool()) throw std::invalid_argument("Action Client defines simultaneous execution but not usage of own thread");

                // get the qos
                if (int64_t *val = json["service-qos"].if_int64()) {
                    const std::map<uint8_t, rmw_qos_profile_t>::iterator serviceIt = GlobalConfig::qos.find(*val);
                    if (serviceIt == GlobalConfig::qos.end())
                        throw std::invalid_argument("Action Client holds unknown \"service-qos\" id");
                    else
                        serviceQos = &serviceIt->second;
                } else
                    throw std::invalid_argument("Action Client holds unknown \"service-qos\" id");

                if (int64_t *val = json["feedback-qos"].if_int64()) {
                    const std::map<uint8_t, rmw_qos_profile_t>::iterator feedbackIt = GlobalConfig::qos.find(*val);
                    if (feedbackIt == GlobalConfig::qos.end())
                        throw std::invalid_argument("Action Client holds unknown \"feedback-qos\" id");
                    else
                        feedbackQos = &feedbackIt->second;
                } else
                    throw std::invalid_argument("Action Client holds unknown \"feedback-qos\" id");

                if (int64_t *val = json["status-qos"].if_int64()) {
                    const std::map<uint8_t, rmw_qos_profile_t>::iterator statusIt = GlobalConfig::qos.find(*val);
                    if (statusIt == GlobalConfig::qos.end())
                        throw std::invalid_argument("Action Client holds unknown \"status-qos\" id");
                    else
                        statusQos = &statusIt->second;
                } else
                    throw std::invalid_argument("Action Client holds unknown \"status-qos\" id");

                // get the compression profile
                if (int64_t *val = json["compression"].if_int64()) {
                    const std::map<uint8_t, compression_t>::iterator itt = GlobalConfig::compression.find(*val);
                    if (itt == GlobalConfig::compression.end())
                        throw std::invalid_argument("Action Client holds unknown \"compression\" id");
                    else
                        compression = &itt->second;
                } else
                    throw std::invalid_argument("Action Client holds unknown \"compression\" id");

                // test for compression
                if (compression->compressor != Compressor::NONE && !json["useOwnThread"].as_bool()) throw std::invalid_argument("Action Client uses compression but not own thread");

                const int64_t channel = json["channel"].as_int64();
                if (channel < std::numeric_limits<uint8_t>::min() || channel > std::numeric_limits<uint8_t>::max()) throw std::invalid_argument("Action Client does not hold \"channel\" as uint8_t");

                for (const uint8_t &val : known) {
                    if (val == channel) throw std::invalid_argument("There are at least two action clients which hold the same \"channel\"");
                }
                for (const Service_Action_t &server : GlobalConfig::actionServer) {
                    if (server.channel == channel) throw std::invalid_argument("There is a action client and server which hold both the same \"channel\"");
                }
                known.push_back(channel);
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Action Client '%s' is not well formatted. %s", clie.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Action Client is not well formatted");
                return false;
            }

            Service_Action_t action = {
                .channel = static_cast<uint8_t>(json["channel"].as_int64()),
                .type = boost::json::value_to<std::string>(json["type"]),
                .useOwnThread = json["useOwnThread"].as_bool(),
                .allowSimultaneous = json["allowSimultaneous"].as_bool(),
                .serviceQos = serviceQos,
                .feedbackQos = feedbackQos,
                .statusQos = statusQos,
                .resultTimeout = 0,
                .maxExecTime = 0,
                .compression = compression};

            try {
                std::shared_ptr<action::ActionClient> _action = actionClientLoader.createSharedInstance(action.type);
                _action->test(action, this->get_node_base_interface(), this->get_node_graph_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(), this->get_node_waitables_interface(), this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
                _action.reset();
            } catch (std::exception &e) {
                RCLCPP_FATAL(this->get_logger(), "Action Client '%s' cannot be created. %s", clie.c_str(), e.what());
                rclcpp::shutdown(nullptr, "Action Clients cannot be created");
                return false;
            }

            GlobalConfig::actionClient.push_back(action);
        }

        // check the pre-defined publisher and subscriber flags for conflicts
        if (GlobalConfig::publishTf && GlobalConfig::subscribeTf) {
            RCLCPP_FATAL(this->get_logger(), "Cannot publish and subscribe to tf2 topics at the same time");
            rclcpp::shutdown(nullptr, "Cannot publish and subscribe to tf2 topics at the same time");
            return false;
        }

        // if "clock/publish" is true
        // we need to add a pre-defined publisher to publish the servers time on /clock
        if (GlobalConfig::publishClock) {
            GlobalConfig::publisher.push_back({
                .channel = OpCode::CLOCK,
                .topic = "/clock",
                .type = "rosgraph_msgs/msg/Clock",
                .useOwnThread = true,
                .permanent = true,  // not relevant for publisher
                .qos = &CLOCK_AND_TF_QOS,
                .compression = &DEFAULT_COMPRESSION_PROFILE  // not relevant for publisher
            });
        }

        // if "tf/publish" is true
        // we need to add a pre-defined publisher to publish on /tf and /tf_static
        if (GlobalConfig::publishTf) {
            GlobalConfig::publisher.push_back({
                .channel = OpCode::TF,
                .topic = "/tf",
                .type = "tf2_msgs/msg/TFMessage",
                .useOwnThread = true,
                .permanent = true,  // not relevant for publisher
                .qos = &CLOCK_AND_TF_QOS,
                .compression = &DEFAULT_COMPRESSION_PROFILE  // not relevant for publisher
            });
            GlobalConfig::publisher.push_back({
                .channel = OpCode::TF_STATIC,
                .topic = "/tf_static",
                .type = "tf2_msgs/msg/TFMessage",
                .useOwnThread = true,
                .permanent = true,  // not relevant for publisher
                .qos = &TF_STATIC_QOS,
                .compression = &DEFAULT_COMPRESSION_PROFILE  // not relevant for publisher
            });
        }

        // if "tf/subscribe" is true
        // we need to add a pre-defined subscriber to subscribe to /tf and /tf_static
        if (GlobalConfig::subscribeTf) {
            GlobalConfig::subscriber.push_back({.channel = OpCode::TF,
                                                .topic = "/tf",
                                                .type = "tf2_msgs/msg/TFMessage",
                                                .useOwnThread = true,
                                                .permanent = true,
                                                .qos = &CLOCK_AND_TF_QOS,
                                                .compression = &DEFAULT_COMPRESSION_PROFILE});
            GlobalConfig::subscriber.push_back({.channel = OpCode::TF_STATIC,
                                                .topic = "/tf_static",
                                                .type = "tf2_msgs/msg/TFMessage",
                                                .useOwnThread = true,
                                                .permanent = true,
                                                .qos = &TF_STATIC_QOS,
                                                .compression = &DEFAULT_COMPRESSION_PROFILE});
        }

        // checking if the ranges of the provided values
        if (GlobalConfig::fragmentationSize < 8) {
            RCLCPP_FATAL(this->get_logger(), "Value for websocket_write_buffer_bytes cannot be lower than 8");
            rclcpp::shutdown(nullptr, "Value for websocket_write_buffer_bytes cannot be lower than 8");
            return false;
        }

        if (authentication) {
            // checking if the ranges of the provided values
            if (GlobalConfig::authenticationTimeout <= 0) {
                RCLCPP_FATAL(this->get_logger(), "Value for authentication/timeout cannot be lower than 1");
                rclcpp::shutdown(nullptr, "Value for authentication/timeout cannot be lower than 1");
                return false;
            }

            // check if the authentication plugin is available
            if (!GlobalConfig::authenticationPlugin.empty()) {
                pluginlib::ClassLoader<authentication::Authentication> authenticationLoader(PACKAGE_NAME, "authentication::Authentication");
                try {
                    std::shared_ptr<authentication::Authentication> _authentication = authenticationLoader.createSharedInstance(GlobalConfig::authenticationPlugin);
                    _authentication.reset();
                } catch (std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Authentication plugin cannot be loaded. Error was: %s", e.what());
                    rclcpp::shutdown(nullptr, "Authentication plugin cannot be loaded");
                    return false;
                }
            }

            // test if a plugin is available if needed
            if (!GlobalConfig::authenticationOmit && GlobalConfig::authenticationPlugin.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Authentication check should not be omitted but no authentication plugin is provided");
                rclcpp::shutdown(nullptr, "Authentication check should not be omitted but no authentication plugin is provided");
                return false;
            }

            // print out certain parameters or warning
            if (GlobalConfig::authenticationOmit)
                RCLCPP_WARN(this->get_logger(), "Running with omitting the authentication validation check");
        }

        return true;
    }
};

#endif  // CONNECTBASE_HPP
