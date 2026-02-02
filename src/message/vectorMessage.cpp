// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "vectorMessage.hpp"

VectorMessage::VectorMessage(const uint8_t channel, const Compressor compressor, const uint32_t uncompressedSize, std::vector<uint8_t> &&combinedVector) : MessageBase(channel, compressor, uncompressedSize), combinedVector(std::make_shared<std::vector<uint8_t> >(std::move(combinedVector))) {
}

VectorMessage::VectorMessage(const uint8_t channel, const Compressor compressor, const uint32_t uncompressedSize, const std::shared_ptr<std::vector<uint8_t> > &combinedVector) : MessageBase(channel, compressor, uncompressedSize), combinedVector(combinedVector) {
}

std::unique_ptr<VectorMessage> VectorMessage::fromSerializedMessage(uint8_t channel, const rclcpp::SerializedMessage &serializedMessage) {
    std::vector<uint8_t> combinedVector(2 + serializedMessage.get_rcl_serialized_message().buffer_length);
    combinedVector[0] = channel;
    combinedVector[1] = Compressor::NONE;
    std::memcpy(combinedVector.data() + 2, serializedMessage.get_rcl_serialized_message().buffer, serializedMessage.get_rcl_serialized_message().buffer_length);
    return std::make_unique<VectorMessage>(channel, Compressor::NONE, combinedVector.size() - 2, std::move(combinedVector));
}

std::unique_ptr<VectorMessage> VectorMessage::fromSerializedMessage(uint8_t channel, const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMessage) {
    std::vector<uint8_t> combinedVector(2 + serializedMessage->get_rcl_serialized_message().buffer_length);
    combinedVector[0] = channel;
    combinedVector[1] = Compressor::NONE;
    std::memcpy(combinedVector.data() + 2, serializedMessage->get_rcl_serialized_message().buffer, serializedMessage->get_rcl_serialized_message().buffer_length);
    return std::make_unique<VectorMessage>(channel, Compressor::NONE, combinedVector.size() - 2, std::move(combinedVector));
}

std::unique_ptr<VectorMessage> VectorMessage::forOpCodeSubscription(const uint8_t channel, const bool value) {
    std::vector<uint8_t> combinedVector(4);
    combinedVector[0] = OpCode::SUBSCRIPTION;
    combinedVector[1] = Compressor::NONE;
    combinedVector[2] = channel;
    combinedVector[3] = value ? 1 : 0;
    return std::make_unique<VectorMessage>(OpCode::SUBSCRIPTION, Compressor::NONE, 2, std::move(combinedVector));
}

std::unique_ptr<VectorMessage> VectorMessage::forOpCodeAuthentication(const std::string &value) {
    std::vector<uint8_t> combinedVector(3 + value.size());
    combinedVector[0] = OpCode::AUTHENTICATION;
    combinedVector[1] = Compressor::NONE;
    std::ranges::copy(value, combinedVector.begin() + 2);
    combinedVector.push_back('\0'); // string termination
    return std::make_unique<VectorMessage>(OpCode::AUTHENTICATION, Compressor::NONE, value.size() + 1, std::move(combinedVector));
}

std::unique_ptr<VectorMessage> VectorMessage::forOpCodeNotification(const std::string &message) {
    std::vector<uint8_t> combinedVector(3 + message.size());
    combinedVector[0] = OpCode::NOTIFICATION;
    combinedVector[1] = Compressor::NONE;
    std::ranges::copy(message, combinedVector.begin() + 2);
    combinedVector.push_back('\0'); // string termination
    return std::make_unique<VectorMessage>(OpCode::NOTIFICATION, Compressor::NONE, message.size() + 1, std::move(combinedVector));
}

std::shared_ptr<const rclcpp::SerializedMessage> VectorMessage::getSerializedMessage() const {
    if (this->compressor == Compressor::NONE) {
        rclcpp::SerializedMessage serializedMessage;
        serializedMessage.reserve(this->combinedVector->size() - 2);
        std::memcpy(serializedMessage.get_rcl_serialized_message().buffer, this->combinedVector->data() + 2, this->combinedVector->size() - 2);
        serializedMessage.get_rcl_serialized_message().buffer_length = this->combinedVector->size() - 2;
        return std::make_shared<const rclcpp::SerializedMessage>(serializedMessage);
    } else {
        rclcpp::SerializedMessage serializedMessage;
        serializedMessage.reserve(this->combinedVector->size() - 6);
        std::memcpy(serializedMessage.get_rcl_serialized_message().buffer, this->combinedVector->data() + 6, this->combinedVector->size() - 6);
        serializedMessage.get_rcl_serialized_message().buffer_length = this->combinedVector->size() - 6;
        return std::make_shared<const rclcpp::SerializedMessage>(serializedMessage);
    }
}

std::shared_ptr<const std::vector<uint8_t>> VectorMessage::getCombinedVector() const {
    return this->combinedVector;
}

std::shared_ptr<std::vector<uint8_t> > VectorMessage::getCopiedCombinedVector() const {
    std::vector<uint8_t> combinedVector(this->combinedVector->size());
    std::memcpy(combinedVector.data(), this->combinedVector->data(), this->combinedVector->size());
    return std::make_shared<std::vector<uint8_t> >(std::move(combinedVector));
}

std::string VectorMessage::toString() const {
    // skip the first two byte since they hold the channel and compressor
    const std::string string = reinterpret_cast<char *>(this->combinedVector.get()) + 2;
    return string;
}

std::pair<const uint8_t, const bool> VectorMessage::toSubscription() const {
    const uint8_t channel = (*this->combinedVector)[2];
    const uint8_t value = (*this->combinedVector)[3];
    return std::pair<const uint8_t, const bool>(channel, value == 1);
}

size_t VectorMessage::getSrcSize() const {
    if (this->compressor == Compressor::NONE) return this->combinedVector->size() - 2; // channel and compressor
    else return this->combinedVector->size() - 6; // channel, compressor and uncompressed size
}

uint8_t *VectorMessage::getSrc() const {
    if (this->compressor == Compressor::NONE) return this->combinedVector->data() + 2;
    else return this->combinedVector->data() + 6;
}

void VectorMessage::store(const std::vector<uint8_t> &data) {
    this->combinedVector->resize(2 + data.size()); // channel and compressor
    (*this->combinedVector)[1] = Compressor::NONE; // no longer compressed
    std::memcpy(this->combinedVector->data() + 2, data.data(), data.size());
}
