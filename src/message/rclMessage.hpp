// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef RCLMESSAGE_HPP
#define RCLMESSAGE_HPP

#include "connect/messageBase.hpp"

#include <boost/beast/websocket/stream.hpp>
#include <rclcpp/rclcpp.hpp>

class RclMessage final : public MessageBase {
public:
    /**
     * Constructs a new RclMessage
     *
     * A RclMessage keeps the data stored inside of rclcpp::SerializedMessage.
     * The rclcpp::SerializedMessage only holds the data.
     * The RclMessage is optimal for data which should be published by a rclcpp::GenericPublisher.
     *
     * @param channel the channel of the message
     * @param compressor the compressor of the message
     * @param uncompressedSize size of the data if uncompressed (does not include meta-data like channel and compressor)
     * @param serializedMessage rclcpp::SerializedMessage holding the data
     */
    explicit RclMessage(uint8_t channel, Compressor compressor, uint32_t uncompressedSize, rclcpp::SerializedMessage &&serializedMessage);

    /**
     * Constructs a new RclMessage out of the content in a boost::beast::flat_buffer
     *
     * @param flatBuffer flat buffer to extract the data from
     * @return a RclMessage holding either compressed or uncompressed data
     */
    static std::unique_ptr<RclMessage> fromFlatBuffer(const boost::beast::flat_buffer &flatBuffer);

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
    std::shared_ptr<std::vector<uint8_t> > getCopiedCombinedVector() const override;

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

protected:
    /**
     * @return size of the source for (de-)compression -> size of the actual data without meta data
     */
    size_t getSrcSize() const override;

    /**
     * @return pointer to the source for (de-)compression -> pointer to where the actual data starts
     */
    uint8_t *getSrc() const override;

    /**
     * Stores the given uncompressed data into the message
     * This is called by decompress on success and should override the stored data inside of the message
     * to now hold the uncompressed data.
     *
     * @param data uncompressed data
     */
    void store(const std::vector<uint8_t> &data) override;

private:
    std::shared_ptr<rclcpp::SerializedMessage> serializedMessage;
};


#endif //RCLMESSAGE_HPP
