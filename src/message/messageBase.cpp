// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "connect/messageBase.hpp"

MessageBase::MessageBase(const uint8_t channel, const Compressor compressor, const uint32_t uncompressedSize) : channel(channel), compressor(compressor), uncompressedSize(uncompressedSize) {
}

std::shared_ptr<const std::vector<uint8_t>> MessageBase::getCompressedCombinedVector(const compression_t &compression) const {
    if (compression.compressor == Compressor::LZ4_DEFAULT) return this->lz4DefaultCompression(compression.rate);
    else if (compression.compressor == Compressor::LZ4_HC) return this->lz4HCCompression(compression.rate);
    else if (compression.compressor == Compressor::ZLIB) return this->zlibCompression(compression.rate);
    else return this->getCombinedVector();
}

bool MessageBase::decompress() {
    if (this->compressor == Compressor::LZ4_HC || this->compressor == Compressor::LZ4_DEFAULT) return this->lz4Decompression();
    else if (this->compressor == Compressor::ZLIB) return zlibDecompression();
    else return false;
}

bool MessageBase::isOpCode(const OpCode opCode) const {
    return this->channel == opCode;
}

uint8_t MessageBase::getChannel() const {
    return this->channel;
}

bool MessageBase::isCompressed() const {
    return this->compressor != Compressor::NONE;
}

std::shared_ptr<const std::vector<uint8_t>> MessageBase::lz4DefaultCompression(const uint32_t rate) const {
    // check the size of the data to compress ... if it is too big we do not compress
    if (this->getSrcSize() > LZ4_MAX_SRC_SIZE) return this->getCombinedVector();

    // access the raw data
    const char *src = reinterpret_cast<const char *>(this->getSrc());
    const uint32_t srcSize = static_cast<uint32_t>(this->getSrcSize());

    // compute the worst-case compressed size
    const int maxDstSize = LZ4_compressBound(srcSize);

    // allocate space which is the worst-case compressed size plus 6 additional bytes
    // [0]: channel, [1]: compressor, [2-5]: uncompressed size, [6->]: compressed data
    std::vector<uint8_t> combinedVector = std::vector<uint8_t>(maxDstSize + 6);

    // write the channel, compression flat and uncompressed size (not compressed!)
    combinedVector[0] = this->channel;
    combinedVector[1] = Compressor::LZ4_DEFAULT;
    combinedVector[2] = static_cast<uint8_t>(srcSize & 0xFF);
    combinedVector[3] = static_cast<uint8_t>((srcSize >> 8) & 0xFF);
    combinedVector[4] = static_cast<uint8_t>((srcSize >> 16) & 0xFF);
    combinedVector[5] = static_cast<uint8_t>((srcSize >> 24) & 0xFF);

    // compress the raw data
    char *dst = reinterpret_cast<char *>(&combinedVector[6]);
    const int compressedSize = LZ4_compress_fast(
        src,
        dst,
        srcSize,
        maxDstSize,
        rate
    );

    // this would either indicate a compression error OR an emtpy input to begin with
    if (compressedSize <= 0) return this->getCombinedVector();

    // resize the vector to the actual size which is the compressed size plus the 6 additional bytes
    combinedVector.resize(compressedSize + 6);
    return std::make_shared<const std::vector<uint8_t>>(combinedVector);
}

std::shared_ptr<const std::vector<uint8_t>> MessageBase::lz4HCCompression(const uint32_t rate) const {
    // check the size of the data to compress ... if it is too big we do not compress
    if (this->getSrcSize() > LZ4_MAX_SRC_SIZE) return this->getCombinedVector();

    // access the raw data
    const char *src = reinterpret_cast<const char *>(this->getSrc());
    const uint32_t srcSize = static_cast<uint32_t>(this->getSrcSize());

    // compute the worst-case compressed size
    const int maxDstSize = LZ4_compressBound(srcSize);

    // allocate space which is the worst-case compressed size plus 6 additional bytes
    // [0]: channel, [1]: compressor, [2-5]: uncompressed size, [6->]: compressed data
    std::vector<uint8_t> combinedVector = std::vector<uint8_t>(maxDstSize + 6);

    // write the channel, compression flat and uncompressed size (not compressed!)
    combinedVector[0] = this->channel;
    combinedVector[1] = Compressor::LZ4_HC;
    combinedVector[2] = static_cast<uint8_t>(srcSize & 0xFF);
    combinedVector[3] = static_cast<uint8_t>((srcSize >> 8) & 0xFF);
    combinedVector[4] = static_cast<uint8_t>((srcSize >> 16) & 0xFF);
    combinedVector[5] = static_cast<uint8_t>((srcSize >> 24) & 0xFF);

    // compress the raw data
    char *dst = reinterpret_cast<char *>(&combinedVector[6]);
    const int compressedSize = LZ4_compress_HC(
        src,
        dst,
        srcSize,
        maxDstSize,
        rate
    );

    // this would either indicate a compression error OR an emtpy input to begin with
    if (compressedSize <= 0) return this->getCombinedVector();

    // resize the vector to the actual size which is the compressed size plus the 6 additional bytes
    combinedVector.resize(compressedSize + 6);
    return std::make_shared<const std::vector<uint8_t>>(combinedVector);
}

std::shared_ptr<const std::vector<uint8_t>> MessageBase::zlibCompression(const uint32_t rate) const {
    // check the size of the data to compress ... if it is too big we do not compress
    if (this->getSrcSize() > ZLIB_MAX_SRC_SIZE) return this->getCombinedVector();

    // access the raw data
    const Bytef *src = reinterpret_cast<const Bytef *>(this->getSrc());
    const uLongf srcSize = static_cast<uLongf>(this->getSrcSize());

    // compute the worst-case compressed size
    uLongf dstSize = compressBound(srcSize);

    // allocate space which is the worst-case compressed size plus 6 additional bytes
    // [0]: channel, [1]: compressor, [2-5]: uncompressed size, [6->]: compressed data
    std::vector<uint8_t> combinedVector = std::vector<uint8_t>(dstSize + 6);

    // write the channel, compression flat and uncompressed size (not compressed!)
    combinedVector[0] = this->channel;
    combinedVector[1] = Compressor::ZLIB;
    combinedVector[2] = static_cast<uint8_t>(srcSize & 0xFF);
    combinedVector[3] = static_cast<uint8_t>((srcSize >> 8) & 0xFF);
    combinedVector[4] = static_cast<uint8_t>((srcSize >> 16) & 0xFF);
    combinedVector[5] = static_cast<uint8_t>((srcSize >> 24) & 0xFF);

    // compress the raw data
    Bytef *dst = reinterpret_cast<Bytef *>(&combinedVector[6]);
    const int result = compress2(
        dst,
        &dstSize,
        src,
        srcSize,
        rate
    );

    // this would either indicate a compression error
    if (result != Z_OK) return this->getCombinedVector();

    // resize the vector to the actual size which is the compressed size plus the 6 additional bytes
    combinedVector.resize(dstSize + 6);
    return std::make_shared<const std::vector<uint8_t>>(combinedVector);
}

bool MessageBase::lz4Decompression() {
    // check the size of the data to compress ... if it is too big we do not decompress
    if (this->uncompressedSize > ZLIB_MAX_SRC_SIZE) return false;

    // access the raw data
    const char *src = reinterpret_cast<const char *>(this->getSrc());
    const uint32_t srcSize = static_cast<uint32_t>(this->getSrcSize());

    // allocate space and access it
    std::vector<uint8_t> uncompressed(this->uncompressedSize);
    char *dst = reinterpret_cast<char *>(uncompressed.data());

    // decompress the raw data
    const int result = LZ4_decompress_safe(
        src,
        dst,
        srcSize,
        this->uncompressedSize
    );

    // this would indicate a compression error
    if (result != static_cast<int>(this->uncompressedSize)) {
        return false;
    }
    // on success, we update the compressor and store the uncompressed data
    else {
        this->compressor = Compressor::NONE;
        this->store(uncompressed);
        return true;
    }
}

bool MessageBase::zlibDecompression() {
    // check the size of the data to compress ... if it is too big we do not decompress
    if (this->uncompressedSize > ZLIB_MAX_SRC_SIZE) return false;

    // access the raw data
    const Bytef *src = reinterpret_cast<const Bytef *>(this->getSrc());
    const uLongf srcSize = static_cast<uLongf>(this->getSrcSize());

    // allocate space and access it
    std::vector<uint8_t> uncompressed(this->uncompressedSize);
    Bytef *dst = reinterpret_cast<Bytef *>(uncompressed.data());
    uLongf dstSize = static_cast<uLongf>(this->uncompressedSize);

    // decompress the raw data
    const int result = uncompress(
        dst,
        &dstSize,
        src,
        srcSize
    );

    // this would indicate a compression error
    if (result != Z_OK || dstSize != static_cast<uLongf>(this->uncompressedSize)) {
        return false;
    }
    // on success, we update the compressor and store the uncompressed data
    else {
        this->compressor = Compressor::NONE;
        this->store(uncompressed);
        return true;
    }
}
