// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SERVICEACTIONMESSAGE_HPP
#define SERVICEACTIONMESSAGE_HPP

#include "messageBase.hpp"

class ServiceActionMessage final : public MessageBase {
public:
    /**
     * Constructs a new ServiceActionMessage
     *
     * A ServiceActionMessage keeps the data stored inside of std::vector.
     * In contrast to a VectorMessage, the stored data is not a combinedVector but the pure data.
     * It holds several additional fields which hold meta-data for service and action call routing.
     *
     * @param channel the channel of the message
     * @param compressor the compressor of the message
     * @param uncompressedSize size of the data if uncompressed (does not include meta-data like channel and compressor)
     * @param serviceActionChannel the service action channel
     * @param serviceActionOpCode the service action op-code
     * @param combinedVector combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5 if data is compressed, followed by the data
     */
    explicit ServiceActionMessage(uint8_t channel, Compressor compressor, uint32_t uncompressedSize, uint8_t serviceActionChannel, ServiceActionOpCode serviceActionOpCode, std::vector<uint8_t> &&combinedVector);

    /**
     * Constructs a new ServiceActionMessage
     *
     * A ServiceActionMessage keeps the data stored inside of std::vector.
     * In contrast to a VectorMessage, the stored data is not a combinedVector but the pure data.
     * It holds several additional fields which hold meta-data for service and action call routing.
     *
     * @param channel the channel of the message
     * @param compressor the compressor of the message
     * @param uncompressedSize size of the data if uncompressed (does not include meta-data like channel and compressor)
     * @param serviceActionChannel the service action channel
     * @param serviceActionOpCode the service action op-code
     * @param combinedVector combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5 if data is compressed, followed by the data
     */
    explicit ServiceActionMessage(uint8_t channel, Compressor compressor, uint32_t uncompressedSize, uint8_t serviceActionChannel, ServiceActionOpCode serviceActionOpCode, const std::shared_ptr<std::vector<uint8_t> > &combinedVector);


    /**
     * Converts the given message into a ServiceActionMessage if possible
     *
     * @param message the message to convert
     * @return the converted message or nullptr
     */
    static std::unique_ptr<ServiceActionMessage> fromMessage(const std::unique_ptr<MessageBase> &message);

    /**
     * Creates a ServiceActionMessage for a service
     * This will create a new and hopefully unique gID
     *
     * @param serviceActionChannel the channel of the service or action
     * @param serviceActionOpCode the op-code of the service or action routing
     * @param data the pure serialized data
     * @param gID the gID to use
     * @return the created ServiceActionMessage
     */
    static std::unique_ptr<ServiceActionMessage> forServiceAction(uint8_t serviceActionChannel, ServiceActionOpCode serviceActionOpCode, const std::vector<uint8_t> &data, const std::span<uint8_t> &gID);


    /**
     * Transforms the here stored data into a rclcpp::SerializedMessage
     *
     * @return rclcpp::SerializedMessage holding the data
     */
    std::shared_ptr<const rclcpp::SerializedMessage> getSerializedMessage() const override;

    /**
     * Transforms the here stored data and channel into a combined vector
     *
     * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5 if data is compressed, followed by the data
     */
    std::shared_ptr<const std::vector<uint8_t>> getCombinedVector() const override;

    /**
     * Transforms the here stored data and channel into a combined vector which is a true copy of the stored data owned by the callee
     *
     * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5 if data is compressed, followed by the data
     */
    std::shared_ptr<std::vector<uint8_t>> getCopiedCombinedVector() const override;

    /**
     * Transforms the actual content of the stored data into a std::string.
     * This does not perform any sanity checks. The callee should be sure, that the actual stored
     * data is a readable string and not compressed!
     *
     * @return stored data as std::string
     */
    std::string toString() const override;

    /**
     * Transforms the actual content of the stored data into a std::pair which holds the channel and value of an op-code SUBSCRIPTION message.
     * This does not perform any sanity checks. The callee should be sure, that the actual stored data is
     * a channel and value of an op-code SUBSCRIPTION and not compressed!
     *
     * @return std::pair holding first: channel, second: value of an op-code SUBSCRIPTION message
     */
    std::pair<const uint8_t, const bool> toSubscription() const override;

    /**
     * @return returns a span which spans the pure serialized data of this service / action message
     */
    const std::span<uint8_t> &getData() const;

    /**
     * @return returns the service / action channel
     */
    uint8_t getServiceActionChannel() const;

    /**
     * @return returns the goal id
     */
    const std::span<uint8_t> &getGID() const;

    /**
     * @return returns the goal id as string
     */
    std::string getGIDString() const;

    /**
     * @return returns the service action op-code
     */
    ServiceActionOpCode getServiceActionOpCode() const;

    /**
     * Tests if this message is a service / action op-code message
     *
     * @param serviceActionOpCode the service / action op-code to test for
     * @return true if match
     */
    bool isServiceActionOpCode(ServiceActionOpCode serviceActionOpCode) const;

protected:
    /**
     * @return size of the source for (de-)compression -> size of the actual data without meta data
     */
    size_t getSrcSize() const override;

    /**
     * @return pointer to the source for (de-)compression -> pointer to where the actual data starts
     */
    uint8_t * getSrc() const override;

    /**
     * Stores the given uncompressed data into the message
     * This is called by decompress on success and should override the stored data inside of the message
     * to now hold the uncompressed data.
     *
     * @param data uncompressed data
     */
    void store(const std::vector<uint8_t> &data) override;

    /**
    * Compresses using LZ4_compress_fast
    *
    * @param rate the compression rate (in this case the compression acceleration)
    * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5, followed by the data
    */
    std::shared_ptr<const std::vector<uint8_t>> lz4DefaultCompression(uint32_t rate) const override;

    /**
     * Compresses using LZ4_compress_HC
     *
     * @param rate the compression rate
     * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5, followed by the data
     */
    std::shared_ptr<const std::vector<uint8_t>> lz4HCCompression(uint32_t rate) const override;

    /**
     * Compresses using compress2
     *
     * @param rate the compression rate
     * @return combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5, followed by the data
     */
    std::shared_ptr<const std::vector<uint8_t>> zlibCompression(uint32_t rate) const override;

private:
    std::shared_ptr<std::vector<uint8_t> > combinedVector;

    uint8_t serviceActionChannel;
    ServiceActionOpCode serviceActionOpCode;

    std::span<uint8_t> gID;
    std::span<uint8_t> data;
};

#endif //SERVICEACTIONMESSAGE_HPP
