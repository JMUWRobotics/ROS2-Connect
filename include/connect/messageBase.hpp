// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef MESSAGEBASE_HPP
#define MESSAGEBASE_HPP

#include "types.hpp"

#include <cstdint>

#define LZ4_MAX_SRC_SIZE static_cast<size_t>(std::min({static_cast<size_t>(LZ4_MAX_INPUT_SIZE), static_cast<size_t>(std::numeric_limits<int>::max() - 1), static_cast<size_t>(std::numeric_limits<uint32_t>::max() - 1)}))
#define ZLIB_MAX_SRC_SIZE static_cast<size_t>(std::min({static_cast<size_t>(std::numeric_limits<uLong>::max() - 1), static_cast<size_t>(std::numeric_limits<uint32_t>::max() - 1)}))

class MessageBase {
    friend class TimeMeasurement;
    friend class ServiceActionMessage;

public:
    /**
     * Constructs a new MessageBase
     *
     * @param channel channel of the message
     * @param compressor compressor the message
     * @param uncompressedSize size of the data if uncompressed (does not include meta-data like channel and compressor)
     */
    explicit MessageBase(uint8_t channel, Compressor compressor, uint32_t uncompressedSize);

    virtual ~MessageBase() = default;

    /**
     * Transforms the here stored data into a rclcpp::SerializedMessage
     *
     * @return rclcpp::SerializedMessage holding the data
     */
    virtual std::shared_ptr<const rclcpp::SerializedMessage> getSerializedMessage() const = 0;

    /**
     * Transforms the here stored data and channel into a combined vector
     *
     * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5 if data is compressed, followed by the data
     */
    virtual std::shared_ptr<const std::vector<uint8_t>> getCombinedVector() const = 0;

    /**
     * Transforms the here stored data and channel into a combined vector which is a true copy of the stored data owned by the callee
     *
     * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5 if data is compressed, followed by the data
     */
    virtual std::shared_ptr<std::vector<uint8_t> > getCopiedCombinedVector() const = 0;

    /**
     * Transforms the here stored data and the channel into a compressed combined vector
     *
     * @param compression the compression profile holding a compressor and rate to apply
     * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5, followed by the data
     */
    std::shared_ptr<const std::vector<uint8_t>> getCompressedCombinedVector(const compression_t &compression) const;

    /**
     * Decompresses the here stored compressed data
     *
     * @return true on success, false on failure
     */
    bool decompress();

    /**
     * Transforms the actual content of the stored data into a std::string.
     * This does not perform any sanity checks. The callee should be sure, that the actual stored
     * data is a readable string and not compressed!
     *
     * @return stored data as std::string
     */
    virtual std::string toString() const = 0;

    /**
     * Transforms the actual content of the stored data into a std::pair which holds the channel and value of an op-code SUBSCRIPTION message.
     * This does not perform any sanity checks. The callee should be sure, that the actual stored data is
     * a channel and value of an op-code SUBSCRIPTION and not compressed!
     *
     * @return std::pair holding first: channel, second: value of an op-code SUBSCRIPTION message
     */
    virtual std::pair<const uint8_t, const bool> toSubscription() const = 0;

    /**
     * Tests if this message is a op-code message
     *
     * @param opCode the op-code to test for
     * @return true if channel matched op-code
     */
    bool isOpCode(OpCode opCode) const;

    /**
     * @return channel of this message
     */
    uint8_t getChannel() const;

    /**
     * @return if this is compressed and needs a call to decompress
     */
    bool isCompressed() const;

protected:
    uint8_t channel;

    Compressor compressor;
    uint32_t uncompressedSize;

    /**
     * @return size of the source for (de-)compression -> size of the actual data without meta data
     */
    virtual size_t getSrcSize() const = 0;

    /**
     * @return pointer to the source for (de-)compression -> pointer to where the actual data starts
     */
    virtual uint8_t *getSrc() const = 0;

    /**
     * Stores the given uncompressed data into the message
     * This is called by decompress on success and should override the stored data inside of the message
     * to now hold the uncompressed data.
     *
     * @param data uncompressed data
     */
    virtual void store(const std::vector<uint8_t> &data) = 0;

    /**
     * Compresses using LZ4_compress_fast
     *
     * @param rate the compression rate (in this case the compression acceleration)
     * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5, followed by the data
     */
    virtual std::shared_ptr<const std::vector<uint8_t>> lz4DefaultCompression(uint32_t rate) const;

    /**
     * Compresses using LZ4_compress_HC
     *
     * @param rate the compression rate
     * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5, followed by the data
     */
    virtual std::shared_ptr<const std::vector<uint8_t>> lz4HCCompression(uint32_t rate) const;

    /**
     * Compresses using compress2
     *
     * @param rate the compression rate
     * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5, followed by the data
     */
    virtual std::shared_ptr<const std::vector<uint8_t>> zlibCompression(uint32_t rate) const;

    /**
    * Decompresses using LZ4_decompress_safe
    *
    * @return true on success, false on failure
    */
    bool lz4Decompression();

    /**
     * Decompresses using ZLIB
     *
     * @return true on success, false on failure
     */
    bool zlibDecompression();
};


#endif //MESSAGEBASE_HPP
