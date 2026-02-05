# ROS2 Connect

A performant ROS2 over WAN Solution based on WebSockets.

ROS2 Connect enables:
  - communication between two ROS2 (DDS) domains
  - communication between two host systems running ROS2 nodes which are connected via a WAN

In this context, ROS2 Connect enables the transfer of topic data, the calling of services and actions,
the transfer of tf2 transformations, and time synchronization mechanisms.

ROS2 Connect uses a WebSocket connection between the ROS2 Connect server and the ROS2 Connect client as its communication
medium, which is inherently NAT-friendly. The data to be transferred can be selectively compressed using three different algorithms (deflate, lz4-default, lz4-hc).

Furthermore, ROS2 Connect offers options for authenticating and authorizing incoming client connections.

## Build 

ROS2 Connect is implemented as a ROS2 package and organized as a colcon project.
It can therefore be built using colcon and compiles (tested) with ROS2 Jazzy Jalisco and later version.

Dependencies:
  - `colcon`
  - `ament_cmake`
  - `rclcpp`
  - `rclcpp_action`
  - `pluginlib`
  - `std_msgs`
  - `rosidl_default_generators`
  - `boost` >= 1.75
  - `OpenSSL`
  - `lz4`
  - `ZLIB`

Since the implementation of authentication and services and actions requires user-specific or type-specific code, this has been moved to the additional [ROS2-Connect-Plugins](https://github.com/JMUWRobotics/ROS2-Connect) package (see also [Authentication](#authentication) and [Services & Actions](#services--actions)). Therefore, it may also be necessary to build ROS2-Connect-Plugins.

## Usage

ROS2 Connect compiles into a server node and a client node, which correspond to a WebSocket server and a client. The server node implements authentication mechanisms, whereas the client does not.
<br>
The client node additionally implements SSL/TLS to establish a secure WebSocket connection. The server node, on the other hand, does not currently implement SSL/TLS and should be operated behind a reverse proxy for SSL/TLS. 
<br>
Apart from these differences, the two nodes behave identically, meaning that both can receive and send topics, services, actions, and tf2.

The source code published here includes a launch file for the [server](/launch/server.py) and [client](/launch//client.py) with associated parameter
files for the [server](/launch/server.yaml) and [client](/launch/client.yaml).

### Parameters

|      **Parameter**      |                                                                         **Function**                                                                         | **Applicable to** |       **Type**      | **Default value** |
|:-----------------------:|:------------------------------------------------------------------------------------------------------------------------------------------------------------:|:-----------------:|:-------------------:|:-----------------:|
| host                    | Server: websocket server address to bind to<br>Client: websocket server address to connect to                                                                |  Server<br>Client |        string       |    "127.0.0.1"    |
| port                    | Server: websocket server port to bind to<br>Client: websocket server port to connect to                                                                      |  Server<br>Client |        string       |       "9090"      |
| path                    | Path of websocket server to connect to                                                                                                                       |       Client      |        string       |        "/"        |
| ssl                     | If ssl should be used to connect to websocket server                                                                                                         |       Client      |         bool        |       false       |
| authentication/host     | Authentication server address                                                                                                                                |       Server      |        string       |    "127.0.0.1"    |
| authentication/port     | Authentication server port                                                                                                                                   |       Server      |        string       |       "8080"      |
| authentication/ssl      | If ssl should be used to connect to authentication server                                                                                                    |       Server      |         bool        |       false       |
| authentication/plugin   | Implementation of authentication mechanism, see [Authentication](#authentication)                                                                            |       Server      |        string       |         ""        |
| authentication/endpoint | Authentication server endpoint                                                                                                                               |       Server      |        string       |        "/"        |
| authentication/omit     | If authentication check should be omitted                                                                                                                    |       Server      |         bool        |       false       |
| authentication/timeout  | Timeout in seconds after which the connection is closed if the client did not send its user_key                                                              |       Server      |         int         |         10        |
| user_key                | Key which identifies a user / client                                                                                                                         |       Client      |        string       |         ""        |
| clock/subscribe         | If the `/clock` topic should be send to the client                                                                                                           |  Server<br>Client |         bool        |       false       |
| clock/publish           | If the `/clock` topic should be published (this will only publish data if the server sends data on this topic)                                               |  Server<br>Client |         bool        |       false       |
| tf/subscribe            | If a subscription to the topics `/tf` and `/tf_static` should be established with the data send to the connection partner                                    |  Server<br>Client |         bool        |       false       |
| tf/publish              | If publishers for the topics `/tf` and `/tf_static` should be established (this will only publish data if the connection partner sends data on these topics) |  Server<br>Client |         bool        |       false       |
| fragmentation/enable    | If fragmentation is allowed on the websocket frames, see [RFC 6455](https://datatracker.ietf.org/doc/html/rfc6455)                                           |  Server<br>Client |         bool        |        true       |
| fragmentation/size      | Fragmentation size in bytes for websocket frames if fragmentation is enabled                                                                                 |  Server<br>Client |         int         |        4096       |
| max_message_size        | Maximum size of a message in bytes which is accepted to receive                                                                                              |  Server<br>Client |         int         |      16777216     |
| qos                     | Definition of QoS profiles, see [QoS](#qos)                                                                                                                  |  Server<br>Client | std::vector<string> |         {}        |
| compression             | Definition of compression profiles, see [Compression](#compression)                                                                                          |  Server<br>Client | std::vector<string> |         {}        |
| publisher               | Definition of publishers to create, see [Topics](#topics)                                                                                                    |  Server<br>Client | std::vector<string> |         {}        |
| subscriber              | Definition of subscriptions to create, see [Topics](#topics)                                                                                                 |  Server<br>Client | std::vector<string> |         {}        |
| service/server          | Definition of service server, see [Services & Actions](#services--actions)                                                                                   |  Server<br>Client | std::vector<string> |         {}        |
| service/client          | Definition of service clients, see [Services & Actions](#services--actions)                                                                                  |  Server<br>Client | std::vector<string> |         {}        |
| action/server           | Definition of action server, see [Services & Actions](#services--actions)                                                                                    |  Server<br>Client | std::vector<string> |         {}        |
| action/client           | Definition of action clients, see [Services & Actions](#services--actions)                                                                                   |  Server<br>Client | std::vector<string> |         {}        |

### Authentication

Authentication takes place after the client node successfully connects and transmits its `user_key`, which happens automatically. The server node then validates this key and either approves the data exchange or closes the WebSocket connection. Before successful authentication, the server node will not send any data and will drop any data it receives.
However, in order for the server node to know how to authenticate the `user_key`, the actual authentication must be implemented via [ROS2 Connect Plugins](https://github.com/JMUWRobotics/ROS2-Connect-Plugins). This is done by implementing the `authentication::Authentication` header exported by ROS2 Connect. The implementation is loaded according to the `authentication/plugin` parameter.

The published [ROS2 Connect Plugins](https://github.com/JMUWRobotics/ROS2-Connect-Plugins) source contains an exemplary authentication implementation for reference.

### Topics

Topics can be transmitted via ROS2 Connect by specifying publishers and subscriptions.

The basic principle here is that, for example, the server defines a subscription and the client defines a publisher for a common topic.<br>
The server can then collect published messages on the topic and transfer them via the WebSocket connection to the client, which republishes them via its publisher.

Since rclcpp defines *GenericSubscription* and *GenericPublisher*, the subscription and publisher of the ROS2 Connect nodes can be defined via parameters, as the topic type does not need to be known at compile time.

However, since such definitions of publishers and subscriptions are complex type definitions, the parameters `publisher` and `subscriber` expect an array of strings, where each string is a JSON string that defines a publisher or a subscription.

Each publisher / subscriber has the following key-value pairs:

|    **Key**   | **Possible Values** |                                                                                                                                                                                                                                                                                                                                                                                                                      **Explanation**                                                                                                                                                                                                                                                                                                                                                                                                                      | **Applicable to**       |
|:------------:|:-------------------:|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|-------------------------|
| channel      | 0 - 248             | Every publisher and subscriber combination has a unique id.<br>This id is used for identifying data during websocket communication.<br>The id between a publisher and a subscriber for a common topic must be identical.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | subsciber<br>publisher  |
| topic        | any                 | The topic name                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | subsciber<br>publisher  |
| type         | any                 | The topic type<br>The topic type between a publisher and a subscriber for a common topic must be identical.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | subscriber<br>publisher |
| useOwnThread | true \| false       | If the subscription or publisher should us an own thread, or use a shared thread.<br>Using an own thread may be necessary for topics with large messages (e.g. images) or<br>topics with high frequency data.<br><br>This must be true, if compression is used!                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | subscriber<br>publisher |
| qos          | 0 - 255             | The id of a QoS profile, see [QoS](#qos)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | subscriber<br>publisher |
| permanent    | true \| false       | If false, the subscription is destroyed if the communication partner has no consumer for the data.<br>This means that if, for example, the server has a subscription and the client has a corresponding publisher, the client permanently monitors its publisher to see if there are any local subscriptions for it.<br>If not, the client informs the server, whereupon the server destroys its subscription for the common topic. As soon as the client has a local subscription for the common topic again, it informs the server,<br>which then creates a subscription for the common topic again and begins to transfer data.<br>This greatly reduces the load on the WebSocket connection, as data is only collected and transferred when it is actually needed.<br><br>If yes, the subscription is never destroyed and remains permanently intact. | subscriber              |
| compression  | 0 - 255             | The id of a compression profile, see [Compression](#compression)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | subscriber              |

For the transmission of topics, it is therefore important that the channel and type between the publisher and subscription are identical for a shared topic. The topic name may differ. Different QoS profiles can also be applied.<br>
Thus, 249 individual topics can be transmitted in the current implementation.

An example specification of publisher and subscription for a common topic would therefore be:
<br>
Publisher: `'{"channel": 3, "topic": "imu", "type": "sensor_msgs/msg/Imu", "qos": 3, "useOwnThread": false}'`
<br>
Subscription: `'{"channel": 3, "topic": "imu", "type": "sensor_msgs/msg/Imu", "qos": 3, "compression": 1, "useOwnThread": false, "permanent": false}'`

### QoS

As already mentioned in Topics, each publisher and each subscription maintains a QoS profile that bundles QoS policies, assigns them a unique ID, and applies them at runtime.

Like Topics, QoS profiles are JSON strings that define the individual policies.<br> 
The policies *history*, *depth*, *reliability*, and *durability* must always be defined.<br>
The policies *deadline*, *lifespan*, *liveliness*, and *liveliness_lease_duration* are optional and are initialized with default parameters.

The possible values for the individual policies correspond to those defined by rmw.<br>
The file [enumParser.hpp](/src/common/enumParser.hpp) implements the parsing of the QoS profiles and serves as a reference for all possible policy-value pairs.

An exemplary QoS profile would therefore be:<br>
`'{"id": 1, "history": "RMW_QOS_POLICY_HISTORY_KEEP_LAST", "depth": 10, "reliability": "RMW_QOS_POLICY_RELIABILITY_RELIABLE", "durability": "RMW_QOS_POLICY_DURABILITY_VOLATILE"}'`

### Compression

As already mentioned in Topics, each publisher and each subscription maintains a Compression profile that bundles compression algorithm and rate, assigns them a unique ID, and applies them at runtime.

Like Topics, Compression profiles are JSON strings that define the individual options.<br>
A compression profile defines whether and, if so, which of the three possible compression algorithms should be used and at what compression rate.<br>
The following options are available:

| **Compressor** |  **Rate** | **Default Rate of Compressor** |                                       **Note**                                       |
|:--------------:|:---------:|:------------------------------:|:------------------------------------------------------------------------------------:|
| None           | any       | any                            |                                                                                      |
| LZ4_DEFAULT    | 1 - 65537 | 1                              | Rate is understood as accelerator -> The higher the rate, the faster the compression |
| LZ4_HC         | 1 - 12    | 9                              |                                                                                      |
| ZLIB           | 1 - 9     | 6                              |                                                                                      |

An exemplary Compression profile would therefore be:<br>
`'{"id": 4, "compressor": "ZLIB",        "rate": 6}'`

### Services & Actions

Compared to topics, for which the type does not need to be known at compile time, the type must be known at compile time for services and actions.
This is because there is no complete generic implementation for service and action servers and clients in rclcpp yet.
<br>
However, in order to keep the ROS2 Connect code as generic as possible, the type-specific implementation of services & action servers & clients is outsourced to [ROS2 Connect Plugins](https://github.com/JMUWRobotics/ROS2-Connect-Plugins).
These plugins are then loaded dynamically at runtime by ROS2 Connect via `pluginlib`.

The basic principle for transferring service calls and action goals can be described as follows:
<br>
Assumption: A ROS2 node running on the host system of the ROS2 Connect Server provides a service server that is to be called by the host system of the ROS Connect Client.
<br>
To make this possible, the ROS2 Connect Server must implement a corresponding service client and the ROS2 Connect Client must implement a corresponding service server.
<br>
The goal is to capture the call by the ROS2 Connect Client, serialize it, transfer it via the WebSocket connection to the ROS2 Connect Server, which then repeats the original call to the actual service, receives the response, serializes it again, and transfers it back to the ROS2 Connect Client, which can then respond to the call with the response.

This is achieved by implementing the `action::ActionSever`, `action::ActionClient`, `service::ServiceServer`, `service::ServiceClient` header exported by ROS2 Connect.
<br>
This header declares methods to be implemented that have clear tasks to fulfill. The basic process and transport logic has already been implemented.

The published [ROS2 Connect Plugins](https://github.com/JMUWRobotics/ROS2-Connect-Plugins) source contains an exemplary implementation for reference.

The plugins implemented in this way are then loaded using the parameters `action/client`, `action/server`, `service/client`, and `service/server`.
<br>
Each has the following key-value pairs:

|      **Key**      | **Possible Values** |                                                                                                                                                                                    **Explanation**                                                                                                                                                                                    | **Applicable to**                                                  |
|:-----------------:|:-------------------:|:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|--------------------------------------------------------------------|
| channel           | 0 - 255             | Every server and client combination has a unique id.<br>This id is used for identifying data during websocket communication.<br>The id between a server and a client for a common action or service must be identical.<br>Both, actions and services, have the range from 0 - 255.                                                                                                    | action/server<br>action/client<br>service/server<br>service/client |
| type              | any                 | The service or action server or client type as defined in plugins.xml.<br>E.g. `connect_plugins::AddTwoIntsServiceClient`                                                                                                                                                                                                                                                             | action/server<br>action/client<br>service/server<br>service/client |
| useOwnThread      | true \| false       | If the server or client should us an own thread, or use a shared thread.<br>Using an own thread may be necessary for actions or services transport large data or<br>when they expected to be called frequently.<br><br>This must be true, if allowSimultaneous is true!<br><br>This must not be true, if compression is used, but is highly recommended to enable it for compression. | action/server<br>action/client<br>service/server<br>service/client |
| allowSimultaneous | true \| false       | If true, calls / goals to the service or action are processed simultaneously.<br>If false, calls / goals to the service or action are processed sequentially.<br><br>Use this with caution, since this may lead to thread starvation.                                                                                                                                                 | action/server<br>action/client<br>service/server<br>service/client |
| service-qos       | 0 - 255             | The id of a QoS profile which is applied to the Service, see [QoS](#qos)                                                                                                                                                                                                                                                                                                              | action/server<br>action/client<br>service/server<br>service/client |
| feedback-qos      | 0 - 255             | The id of a QoS profile which is applied to the Feedback Topic, see [QoS](#qos)                                                                                                                                                                                                                                                                                                       | action/server<br>action/client                                     |
| status-qos        | 0 - 255             | The id of a QoS profile which is applied the the Status Topic, see [QoS](#ros)                                                                                                                                                                                                                                                                                                        | action/server<br>action/client                                     |
| maxExecTime       | 0 - 4294967295      | The max execution time in seconds after which a call is interrupted and stopped.                                                                                                                                                                                                                                                                                                      | service/server                                                     |
| resultTimeout     | -1 - 2147483647     | Timeout in seconds after which the goal result will no longer be available.<br>See [ROS2 Design](https://design.ros2.org/articles/actions.html) for more information.                                                                                                                                                                                                                 | action/server                                                      |
| compression       | 0 - 255             | The id of a compression profile, see [Compression](#compression)                                                                                                                                                                                                                                                                                                                      | action/server<br>action/client<br>service/server<br>service/client |

Thus, 256 individual actions and 256 individual services can be transmitted in the current implementation.

An example specification of service server and client for a common service would therefore be:
<br>
Service Server: `'{"channel": 0, "type": "connect_plugins::AddTwoIntsServiceServer", "service-qos": 1, "useOwnThread": false, "allowSimultaneous": false, "maxExecTime": 10, "compression": 1}'`
<br>
Service Client: `'{"channel": 0, "type": "connect_plugins::AddTwoIntsServiceClient", "service-qos": 1, "useOwnThread": false, "allowSimultaneous": false, "compression": 1}'`

An example specification of action server and client for a common service would therefore be:
<br>
Action Server: `'{"channel": 0, "type": "connect_plugins::FibonacciActionServer", "service-qos": 1, "feedback-qos": 1, "status-qos": 2, "useOwnThread": false, "allowSimultaneous": false, "compression": 1, "resultTimeout": 10}'`
<br>
Action Client: `'{"channel": 0, "type": "connect_plugins::FibonacciActionClient", "service-qos": 1, "feedback-qos": 1, "status-qos": 2, "useOwnThread": false, "allowSimultaneous": false, "compression": 1}'`

## Limitations

Since ROS2 Connect cannot distinguish between self-published data and data from other nodes, there must never be a configuration in which both nodes, server and client, have both a subscription and a publisher for a common topic. In this case, an endless loop would occur.

The same applies to tf2 and clock, but here the parameters tf/subscribe, tf/publish, clock/subscribe, and clock/publish explicitly prevent this situation.

ROS2 Connect does not currently implement any options for ROS2 parameters.

## License

This project is licensed under the **Mozilla Public License 2.0 (MPL-2.0)**. 
The MPL-2.0 is a "file-level copyleft" license that balances the needs of open-source and proprietary software.

Please see the [LICENSE](/LICENSE) file for the full license text.

[![License: MPL 2.0](https://img.shields.io/badge/License-MPL_2.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)