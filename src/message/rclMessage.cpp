// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "rclMessage.hpp"

RclMessage::RclMessage(const uint8_t channel, const Compressor compressor, const uint32_t uncompressedSize, rclcpp::SerializedMessage &&serializedMessage) : MessageBase(channel, compressor, uncompressedSize), serializedMessage(std::make_shared<rclcpp::SerializedMessage>(std::move(serializedMessage))) {
}

std::unique_ptr<RclMessage> RclMessage::fromFlatBuffer(const boost::beast::flat_buffer &flatBuffer) {
    // extract a pointer to the underlying data of the flatbuffer
    const boost::beast::flat_buffer::const_buffers_type readableData = flatBuffer.data();
    const uint8_t *dataPtr = static_cast<const uint8_t *>(readableData.data());

    // extract the channel as the first byte
    // extract the compressor as the second byte
    const uint8_t channel = dataPtr[0];
    const uint8_t compressor = dataPtr[1];

    // if uncompressed
    //  - read the data into a rclcpp::SerializedMessage
    //  - return a Message instance
    if (compressor == Compressor::NONE) {
        // remaining amount of bytes
        const size_t dataSize = boost::asio::buffer_size(readableData) - 2;

        // copy data into serialized message
        rclcpp::SerializedMessage serializedMessage;
        if (dataSize > 0) {
            serializedMessage.reserve(dataSize);
            std::memcpy(serializedMessage.get_rcl_serialized_message().buffer, dataPtr + 2, dataSize);
            serializedMessage.get_rcl_serialized_message().buffer_length = dataSize;
        }

        return std::make_unique<RclMessage>(channel, Compressor::NONE, dataSize, std::move(serializedMessage));
    }
    // if compressed
    //  - extract the uncompressed size
    //  - read the data into a vector
    //  - return a CompressedMessage instance
    else if (compressor >= COMPRESSOR_MIN && compressor <= COMPRESSOR_MAX) {
        // uncompressed size
        const uint32_t uncompressedSize =
                (static_cast<uint32_t>(dataPtr[2])) |
                (static_cast<uint32_t>(dataPtr[3]) << 8) |
                (static_cast<uint32_t>(dataPtr[4]) << 16) |
                (static_cast<uint32_t>(dataPtr[5]) << 24);

        // remaining amount of bytes
        const size_t dataSize = boost::asio::buffer_size(readableData) - 6;

        // copy data into serialized message
        rclcpp::SerializedMessage serializedMessage;
        if (dataSize > 0) {
            serializedMessage.reserve(dataSize);
            std::memcpy(serializedMessage.get_rcl_serialized_message().buffer, dataPtr + 6, dataSize);
            serializedMessage.get_rcl_serialized_message().buffer_length = dataSize;
        }

        return std::make_unique<RclMessage>(channel, static_cast<Compressor>(compressor), uncompressedSize, std::move(serializedMessage));
    }
    // unknown compressor value
    //  - return an empty message
    else {
        rclcpp::SerializedMessage serializedMessage;
        return std::make_unique<RclMessage>(channel, Compressor::NONE, 0, std::move(serializedMessage));
    }
}

std::shared_ptr<const rclcpp::SerializedMessage> RclMessage::getSerializedMessage() const {
    return this->serializedMessage;
}

std::shared_ptr<const std::vector<uint8_t>> RclMessage::getCombinedVector() const {
    if (this->compressor == Compressor::NONE) {
        std::vector<uint8_t> combinedVector(2 + this->serializedMessage->get_rcl_serialized_message().buffer_length);
        combinedVector[0] = this->channel;
        combinedVector[1] = Compressor::NONE;
        std::memcpy(combinedVector.data() + 2, this->serializedMessage->get_rcl_serialized_message().buffer, this->serializedMessage->get_rcl_serialized_message().buffer_length);
        return std::make_shared<const std::vector<uint8_t>>(std::move(combinedVector));
    } else {
        std::vector<uint8_t> combinedVector(6 + this->serializedMessage->get_rcl_serialized_message().buffer_length);
        combinedVector[0] = this->channel;
        combinedVector[1] = this->compressor;
        combinedVector[2] = static_cast<uint8_t>(this->uncompressedSize & 0xFF);
        combinedVector[3] = static_cast<uint8_t>((this->uncompressedSize >> 8) & 0xFF);
        combinedVector[4] = static_cast<uint8_t>((this->uncompressedSize >> 16) & 0xFF);
        combinedVector[5] = static_cast<uint8_t>((this->uncompressedSize >> 24) & 0xFF);
        std::memcpy(combinedVector.data() + 6, this->serializedMessage->get_rcl_serialized_message().buffer, this->serializedMessage->get_rcl_serialized_message().buffer_length);
        return std::make_shared<const std::vector<uint8_t>>(std::move(combinedVector));
    }
}

std::shared_ptr<std::vector<uint8_t> > RclMessage::getCopiedCombinedVector() const {
    if (this->compressor == Compressor::NONE) {
        std::vector<uint8_t> combinedVector(2 + this->serializedMessage->get_rcl_serialized_message().buffer_length);
        combinedVector[0] = this->channel;
        combinedVector[1] = Compressor::NONE;
        std::memcpy(combinedVector.data() + 2, this->serializedMessage->get_rcl_serialized_message().buffer, this->serializedMessage->get_rcl_serialized_message().buffer_length);
        return std::make_shared<std::vector<uint8_t> >(std::move(combinedVector));
    } else {
        std::vector<uint8_t> combinedVector(6 + this->serializedMessage->get_rcl_serialized_message().buffer_length);
        combinedVector[0] = this->channel;
        combinedVector[1] = this->compressor;
        combinedVector[2] = static_cast<uint8_t>(this->uncompressedSize & 0xFF);
        combinedVector[3] = static_cast<uint8_t>((this->uncompressedSize >> 8) & 0xFF);
        combinedVector[4] = static_cast<uint8_t>((this->uncompressedSize >> 16) & 0xFF);
        combinedVector[5] = static_cast<uint8_t>((this->uncompressedSize >> 24) & 0xFF);
        std::memcpy(combinedVector.data() + 6, this->serializedMessage->get_rcl_serialized_message().buffer, this->serializedMessage->get_rcl_serialized_message().buffer_length);
        return std::make_shared<std::vector<uint8_t> >(std::move(combinedVector));
    }
}

std::string RclMessage::toString() const {
    const std::string string = reinterpret_cast<char *>(this->serializedMessage->get_rcl_serialized_message().buffer);
    return string;
}

std::pair<const uint8_t, const bool> RclMessage::toSubscription() const {
    const uint8_t channel = this->serializedMessage->get_rcl_serialized_message().buffer[0];
    const uint8_t value = this->serializedMessage->get_rcl_serialized_message().buffer[1];
    return std::pair<const uint8_t, const bool>(channel, value == 1);
}

size_t RclMessage::getSrcSize() const {
    return this->serializedMessage->get_rcl_serialized_message().buffer_length;
}

uint8_t *RclMessage::getSrc() const {
    return this->serializedMessage->get_rcl_serialized_message().buffer;
}

void RclMessage::store(const std::vector<uint8_t> &data) {
    this->serializedMessage->reserve(data.size());
    this->serializedMessage->get_rcl_serialized_message().buffer_length = data.size();
    std::memcpy(this->serializedMessage->get_rcl_serialized_message().buffer, data.data(), data.size());
}
