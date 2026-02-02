// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "connect/serviceActionMessage.hpp"

ServiceActionMessage::ServiceActionMessage(const uint8_t channel, const Compressor compressor, const uint32_t uncompressedSize, const uint8_t serviceActionChannel, const ServiceActionOpCode serviceActionOpCode, std::vector<uint8_t> &&combinedVector) : MessageBase(channel, compressor, uncompressedSize), combinedVector(std::make_shared<std::vector<uint8_t> >(std::move(combinedVector))), serviceActionChannel(serviceActionChannel), serviceActionOpCode(serviceActionOpCode) {
    if (this->compressor == Compressor::NONE) {
        const uint8_t gidSize = (*this->combinedVector)[4];
        this->gID = std::span<uint8_t>(this->combinedVector->begin() + 5, gidSize);
        this->data = std::span<uint8_t>(this->combinedVector->begin() + 5 + gidSize, this->combinedVector->size() - 5 - gidSize);
    } else {
        const uint8_t gidSize = (*this->combinedVector)[8];
        this->gID = std::span<uint8_t>(this->combinedVector->begin() + 9, gidSize);
        this->data = std::span<uint8_t>(this->combinedVector->begin() + 9 + gidSize, this->combinedVector->size() - 9 - gidSize);
    }
}

ServiceActionMessage::ServiceActionMessage(const uint8_t channel, const Compressor compressor, const uint32_t uncompressedSize, const uint8_t serviceActionChannel, const ServiceActionOpCode serviceActionOpCode, const std::shared_ptr<std::vector<uint8_t> > &combinedVector) : MessageBase(channel, compressor, uncompressedSize), combinedVector(combinedVector), serviceActionChannel(serviceActionChannel), serviceActionOpCode(serviceActionOpCode) {
    if (this->compressor == Compressor::NONE) {
        const uint8_t gidSize = (*this->combinedVector)[4];
        this->gID = std::span<uint8_t>(this->combinedVector->begin() + 5, gidSize);
        this->data = std::span<uint8_t>(this->combinedVector->begin() + 5 + gidSize, this->combinedVector->size() - 5 - gidSize);
    } else {
        const uint8_t gidSize = (*this->combinedVector)[8];
        this->gID = std::span<uint8_t>(this->combinedVector->begin() + 9, gidSize);
        this->data = std::span<uint8_t>(this->combinedVector->begin() + 9 + gidSize, this->combinedVector->size() - 9 - gidSize);
    }
}

std::unique_ptr<ServiceActionMessage> ServiceActionMessage::fromMessage(const std::unique_ptr<MessageBase> &message) {
    // get a true copy of the combined vector so we can directly move it into this message
    const std::shared_ptr<std::vector<uint8_t> > combinedVector = message->getCopiedCombinedVector();

    if (message->compressor == Compressor::NONE) {
        // check if we seem to have the appropriate size
        // 0: channel, 1: compressor, 2: service/action channel, 3: service/action op-code, 4: gidSize, 5: first byte of gid
        if (combinedVector->size() < 6) return nullptr;

        const uint8_t serviceActionChannel = (*combinedVector)[2];
        const uint8_t serviceActionOpCode = (*combinedVector)[3];
        const uint8_t gidSize = (*combinedVector)[4];

        // we do not have a valid service action opcode, return a nullptr
        if (serviceActionOpCode < SERVICE_ACTION_OP_CODE_MIN || serviceActionOpCode > SERVICE_ACTION_OP_CODE_MAX) return nullptr;

        // check the size again
        // we check if the gid is fully there
        if (combinedVector->size() < static_cast<size_t>(5 + gidSize)) return nullptr;

        return std::make_unique<ServiceActionMessage>(message->channel, Compressor::NONE, combinedVector->size() - 5 - gidSize, serviceActionChannel, static_cast<ServiceActionOpCode>(serviceActionOpCode), combinedVector); // uncompressed size is data size since we do not compress the extended header
    } else {
        // check if we seem to have the appropriate size
        // 0: channel, 1: compressor, 2-5: uncompressed size, 6: service/action channel, 7: service/action op-code, 8: gidSize, 9: first byte of gid
        if (combinedVector->size() < 10) return nullptr;

        const uint8_t serviceActionChannel = (*combinedVector)[6];
        const uint8_t serviceActionOpCode = (*combinedVector)[7];
        const uint8_t gidSize = (*combinedVector)[8];

        // we do not have a valid service action opcode, return a nullptr
        if (serviceActionOpCode < SERVICE_ACTION_OP_CODE_MIN || serviceActionOpCode > SERVICE_ACTION_OP_CODE_MAX) return nullptr;

        // check the size again
        // we check if the gid is fully there
        if (combinedVector->size() < static_cast<size_t>(9 + gidSize)) return nullptr;

        return std::make_unique<ServiceActionMessage>(message->channel, message->compressor, message->uncompressedSize, serviceActionChannel, static_cast<ServiceActionOpCode>(serviceActionOpCode), combinedVector); // uncompressed size is data size since we do not compress the extended header
    }
}

std::unique_ptr<ServiceActionMessage> ServiceActionMessage::forServiceAction(uint8_t serviceActionChannel, ServiceActionOpCode serviceActionOpCode, const std::vector<uint8_t> &data, const std::span<uint8_t> &gID) {
    std::vector<uint8_t> combinedVector(5 + gID.size() + data.size());
    // header
    combinedVector[0] = OpCode::SERVICE_ACTION;
    combinedVector[1] = Compressor::NONE;
    // extended header
    combinedVector[2] = serviceActionChannel;
    combinedVector[3] = serviceActionOpCode;
    combinedVector[4] = gID.size(); // FIXME: we do not protect, that this is in the range of an uint8_t
    std::memcpy(combinedVector.data() + 5, gID.data(), gID.size());
    // body
    std::memcpy(combinedVector.data() + 5 + gID.size(), data.data(), data.size());
    return std::make_unique<ServiceActionMessage>(OpCode::SERVICE_ACTION, Compressor::NONE, data.size(), serviceActionChannel, serviceActionOpCode, std::move(combinedVector)); // uncompressed size is data size since we do not compress the extended header
}

std::shared_ptr<const rclcpp::SerializedMessage> ServiceActionMessage::getSerializedMessage() const {
    // this is equivalent to vectorMessage.hpp

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

std::shared_ptr<const std::vector<uint8_t>> ServiceActionMessage::getCombinedVector() const {
    return this->combinedVector;
}

std::shared_ptr<std::vector<uint8_t>> ServiceActionMessage::getCopiedCombinedVector() const {
    std::vector<uint8_t> combinedVector(this->combinedVector->size());
    std::memcpy(combinedVector.data(), this->combinedVector->data(), this->combinedVector->size());
    return std::make_shared<std::vector<uint8_t> >(std::move(combinedVector));
}

std::string ServiceActionMessage::toString() const {
    // skip the first two byte since they hold the channel and compressor
    const std::string string = reinterpret_cast<char *>(this->combinedVector.get()) + 5 + this->gID.size();
    return string;
}

std::pair<const uint8_t, const bool> ServiceActionMessage::toSubscription() const {
    // this is actually pure garbage but we do need to implement this

    const uint8_t channel = (*this->combinedVector)[2];
    const uint8_t value = (*this->combinedVector)[3];
    return std::pair<const uint8_t, const bool>(channel, value == 1);
}

const std::span<uint8_t> & ServiceActionMessage::getData() const {
    return this->data;
}

uint8_t ServiceActionMessage::getServiceActionChannel() const {
    return this->serviceActionChannel;
}

const std::span<uint8_t> & ServiceActionMessage::getGID() const {
    return this->gID;
}

std::string ServiceActionMessage::getGIDString() const {
    std::string gid(reinterpret_cast<const char *>(this->gID.data()), this->gID.size());
    gid.push_back('\0');
    return gid;
}

ServiceActionOpCode ServiceActionMessage::getServiceActionOpCode() const {
    return this->serviceActionOpCode;
}

bool ServiceActionMessage::isServiceActionOpCode(const ServiceActionOpCode serviceActionOpCode) const {
    return this->serviceActionOpCode == serviceActionOpCode;
}

size_t ServiceActionMessage::getSrcSize() const {
    return this->data.size();
}

uint8_t *ServiceActionMessage::getSrc() const {
    return this->data.data();
}

void ServiceActionMessage::store(const std::vector<uint8_t> &data) {
    const uint8_t gidSize = this->gID.size();

    // first move the additional header
    (*this->combinedVector)[1] = Compressor::NONE;
    (*this->combinedVector)[2] = this->serviceActionChannel;
    (*this->combinedVector)[3] = this->serviceActionOpCode;
    (*this->combinedVector)[4] = gidSize;
    std::memcpy(this->combinedVector->data() + 5, this->gID.data(), gidSize);

    // then resize the combined vector
    this->combinedVector->resize(5 + data.size() + gidSize);

    // then copy the data
    std::memcpy(this->combinedVector->data() + 5 + gidSize, data.data(), data.size());

    // then recompute the gid and data span
    this->gID = std::span<uint8_t>(this->combinedVector->begin() + 5, gidSize);
    this->data = std::span<uint8_t>(this->combinedVector->begin() + 5 + gidSize, data.size());
}

std::shared_ptr<const std::vector<uint8_t>> ServiceActionMessage::lz4DefaultCompression(const uint32_t rate) const {
    // check the size of the data to compress ... if it is too big we do not compress
    if (this->getSrcSize() > LZ4_MAX_SRC_SIZE) return this->getCombinedVector();

    // access the raw data
    const char *src = reinterpret_cast<const char *>(this->getSrc());
    const uint32_t srcSize = static_cast<uint32_t>(this->getSrcSize());

    // compute the worst-case compressed size
    const int maxDstSize = LZ4_compressBound(srcSize);

    // allocate space which is the worst-case compressed size plus 9 + gidSize additional bytes
    // [0]: channel, [1]: compressor, [2-5]: uncompressed size, 6: service action channel, 7: service action op-code, 8: gid size, [9->9+gidSize]: gid, [9+gidSize+1->]: compressed data
    std::vector<uint8_t> combinedVector = std::vector<uint8_t>(maxDstSize + 9 + this->gID.size());

    // write the channel, compression flat, uncompressed size and service action op-code (not compressed!)
    combinedVector[0] = this->channel;
    combinedVector[1] = Compressor::LZ4_DEFAULT;
    combinedVector[2] = static_cast<uint8_t>(srcSize & 0xFF);
    combinedVector[3] = static_cast<uint8_t>((srcSize >> 8) & 0xFF);
    combinedVector[4] = static_cast<uint8_t>((srcSize >> 16) & 0xFF);
    combinedVector[5] = static_cast<uint8_t>((srcSize >> 24) & 0xFF);
    combinedVector[6] = this->serviceActionChannel;
    combinedVector[7] = this->serviceActionOpCode;
    combinedVector[8] = static_cast<uint8_t>(this->gID.size());
    std::memcpy(combinedVector.data() + 9, this->gID.data(), this->gID.size());

    // compress the raw data
    char *dst = reinterpret_cast<char *>(&combinedVector[9 + this->gID.size()]);
    const int compressedSize = LZ4_compress_fast(
        src,
        dst,
        srcSize,
        maxDstSize,
        rate
    );

    // this would either indicate a compression error OR an emtpy input to begin with
    if (compressedSize <= 0) return this->getCombinedVector();

    // resize the vector to the actual size which is the compressed size plus the 9 + gidSize additional bytes
    combinedVector.resize(compressedSize + 9 + this->gID.size());
    return std::make_shared<const std::vector<uint8_t>>(combinedVector);
}

std::shared_ptr<const std::vector<uint8_t>> ServiceActionMessage::lz4HCCompression(const uint32_t rate) const {
    // check the size of the data to compress ... if it is too big we do not compress
    if (this->getSrcSize() > LZ4_MAX_SRC_SIZE) return this->getCombinedVector();

    // access the raw data
    const char *src = reinterpret_cast<const char *>(this->getSrc());
    const uint32_t srcSize = static_cast<uint32_t>(this->getSrcSize());

    // compute the worst-case compressed size
    const int maxDstSize = LZ4_compressBound(srcSize);

    // allocate space which is the worst-case compressed size plus 9 + gidSize additional bytes
    // [0]: channel, [1]: compressor, [2-5]: uncompressed size, 6: service action channel, 7: service action op-code, 8: gid size, [9->9+gidSize]: gid, [9+gidSize+1->]: compressed data
    std::vector<uint8_t> combinedVector = std::vector<uint8_t>(maxDstSize + 9 + this->gID.size());

    // write the channel, compression flat, uncompressed size and service action op-code (not compressed!)
    combinedVector[0] = this->channel;
    combinedVector[1] = Compressor::LZ4_HC;
    combinedVector[2] = static_cast<uint8_t>(srcSize & 0xFF);
    combinedVector[3] = static_cast<uint8_t>((srcSize >> 8) & 0xFF);
    combinedVector[4] = static_cast<uint8_t>((srcSize >> 16) & 0xFF);
    combinedVector[5] = static_cast<uint8_t>((srcSize >> 24) & 0xFF);
    combinedVector[6] = this->serviceActionChannel;
    combinedVector[7] = this->serviceActionOpCode;
    combinedVector[8] = static_cast<uint8_t>(this->gID.size());
    std::memcpy(combinedVector.data() + 9, this->gID.data(), this->gID.size());

    // compress the raw data
    char *dst = reinterpret_cast<char *>(&combinedVector[9 + this->gID.size()]);
    const int compressedSize = LZ4_compress_HC(
        src,
        dst,
        srcSize,
        maxDstSize,
        rate
    );

    // this would either indicate a compression error OR an emtpy input to begin with
    if (compressedSize <= 0) return this->getCombinedVector();

    // resize the vector to the actual size which is the compressed size plus the 9 + gidSize additional bytes
    combinedVector.resize(compressedSize + 9 + this->gID.size());
    return std::make_shared<const std::vector<uint8_t>>(combinedVector);
}

std::shared_ptr<const std::vector<uint8_t>> ServiceActionMessage::zlibCompression(const uint32_t rate) const {
    // check the size of the data to compress ... if it is too big we do not compress
    if (this->getSrcSize() > ZLIB_MAX_SRC_SIZE) return this->getCombinedVector();

    // access the raw data
    const Bytef *src = reinterpret_cast<const Bytef *>(this->getSrc());
    const uLongf srcSize = static_cast<uLongf>(this->getSrcSize());

    // compute the worst-case compressed size
    uLongf dstSize = compressBound(srcSize);

    // allocate space which is the worst-case compressed size plus 9 + gidSize additional bytes
    // [0]: channel, [1]: compressor, [2-5]: uncompressed size, 6: service action channel, 7: service action op-code, 8: gid size, [9->9+gidSize]: gid, [9+gidSize+1->]: compressed data
    std::vector<uint8_t> combinedVector = std::vector<uint8_t>(dstSize + 9 + this->gID.size());

    // write the channel, compression flat, uncompressed size and service action op-code (not compressed!)
    combinedVector[0] = this->channel;
    combinedVector[1] = Compressor::ZLIB;
    combinedVector[2] = static_cast<uint8_t>(srcSize & 0xFF);
    combinedVector[3] = static_cast<uint8_t>((srcSize >> 8) & 0xFF);
    combinedVector[4] = static_cast<uint8_t>((srcSize >> 16) & 0xFF);
    combinedVector[5] = static_cast<uint8_t>((srcSize >> 24) & 0xFF);
    combinedVector[6] = this->serviceActionChannel;
    combinedVector[7] = this->serviceActionOpCode;
    combinedVector[8] = static_cast<uint8_t>(this->gID.size());
    std::memcpy(combinedVector.data() + 9, this->gID.data(), this->gID.size());

    // compress the raw data
    Bytef *dst = reinterpret_cast<Bytef *>(&combinedVector[9 + this->gID.size()]);
    const int result = compress2(
        dst,
        &dstSize,
        src,
        srcSize,
        rate
    );

    // this would either indicate a compression error
    if (result != Z_OK) return this->getCombinedVector();

    // resize the vector to the actual size which is the compressed size plus the 9 + gidSize additional bytes
    combinedVector.resize(dstSize + 9 + this->gID.size());
    return std::make_shared<const std::vector<uint8_t>>(combinedVector);
}
