# Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

std_msgs/Header header

# if at least one client is connect
bool client_connected
# if the instance is configured to omit the reservation check
bool authentication_omit
# the robot is defined by its domain_id and namespace combination
# since there is only one robot for each combination
string name_space
uint8 domain_id
