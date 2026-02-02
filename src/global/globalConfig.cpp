// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "globalConfig.hpp"

std::string GlobalConfig::nodeName = "";
std::string GlobalConfig::nodeNamespace = "";
size_t GlobalConfig::nodeDomainId = 0;

std::string GlobalConfig::host = "";
std::string GlobalConfig::path = "";
std::string GlobalConfig::port = "";

bool GlobalConfig::ssl = false;

std::string GlobalConfig::authenticationHost = "";
std::string GlobalConfig::authenticationPort = "";
bool GlobalConfig::authenticationSsl = false;

std::string GlobalConfig::authenticationPlugin = "";
std::string GlobalConfig::authenticationEndpoint = "";
bool GlobalConfig::authenticationOmit = false;
uint16_t GlobalConfig::authenticationTimeout = 10;

bool GlobalConfig::fragmentation = true;
size_t GlobalConfig::fragmentationSize = 4096; // default value according to boost beast documentation

size_t GlobalConfig::maxMessageSize = 16 * 1024 * 1024; // default value according to boost beast documentation

std::string GlobalConfig::userKey = "";

bool GlobalConfig::subscribeClock = false;
bool GlobalConfig::publishClock = false;

bool GlobalConfig::subscribeTf = false;
bool GlobalConfig::publishTf = false;

std::map<uint8_t, rmw_qos_profile_t> GlobalConfig::qos = std::map<uint8_t, rmw_qos_profile_t>{};
std::map<uint8_t, compression_t> GlobalConfig::compression = std::map<uint8_t, compression_t>{};

std::vector<Topic_t> GlobalConfig::subscriber = std::vector<Topic_t>{};
std::vector<Topic_t> GlobalConfig::publisher = std::vector<Topic_t>{};

std::vector<Service_Action_t> GlobalConfig::serviceServer = std::vector<Service_Action_t>{};
std::vector<Service_Action_t> GlobalConfig::serviceClient = std::vector<Service_Action_t>{};

std::vector<Service_Action_t> GlobalConfig::actionServer = std::vector<Service_Action_t>{};
std::vector<Service_Action_t> GlobalConfig::actionClient = std::vector<Service_Action_t>{};
