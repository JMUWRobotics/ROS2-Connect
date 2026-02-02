// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef VECTORMESSAGE_HPP
#define VECTORMESSAGE_HPP

#include "connect/messageBase.hpp"

class VectorMessage final : public MessageBase {
public:
    /**
     * Constructs a new VectorMessage
     *
     * A VectorMessage keeps the data stored inside of std::vector.
     * The vector always is a combinedVector, meaning it always holds the channel, compressor, optional uncompressedSize followed by the data.
     * The VectorMessage is optimal for data which should be send over the websocket connection since it expects a combinedVector.
     *
     * @param channel the channel of the message
     * @param compressor the compressor of the message
     * @param uncompressedSize size of the data if uncompressed (does not include meta-data like channel and compressor)
     * @param combinedVector combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5 if data is compressed, followed by the data
     */
    explicit VectorMessage(uint8_t channel, Compressor compressor, uint32_t uncompressedSize, std::vector<uint8_t> &&combinedVector);

    /**
     * Constructs a new VectorMessage
     *
     * A VectorMessage keeps the data stored inside of std::vector.
     * The vector always is a combinedVector, meaning it always holds the channel, compressor, optional uncompressedSize followed by the data.
     * The VectorMessage is optimal for data which should be send over the websocket connection since it expects a combinedVector.
     *
     * @param channel the channel of the message
     * @param compressor the compressor of the message
     * @param uncompressedSize size of the data if uncompressed (does not include meta-data like channel and compressor)
     * @param combinedVector combined vector holding a channel at byte pos 0, the compressor at byte pos 1, the uncompressed size from byte pos 3 to 5 if data is compressed, followed by the data
     */
    explicit VectorMessage(uint8_t channel, Compressor compressor, uint32_t uncompressedSize, const std::shared_ptr<std::vector<uint8_t> > &combinedVector);

    /**
     * Constructs a new VectorMessage out of a serialized message holding uncompressed data!
     *
     * @param channel the channel of the message
     * @param serializedMessage the serialized message
     * @return the created vector message holding uncompressed data
     */
    static std::unique_ptr<VectorMessage> fromSerializedMessage(uint8_t channel, const rclcpp::SerializedMessage &serializedMessage);

    /**
     * Constructs a new VectorMessage out of a serialized message holding uncompressed data!
     *
     * @param channel the channel of the message
     * @param serializedMessage the serialized message
     * @return the created vector message holding uncompressed data
     */
    static std::unique_ptr<VectorMessage> fromSerializedMessage(uint8_t channel, const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMessage);

    /**
     * Constructs a new VectorMessage for the op-code SUBSCRIPTION
     *
     * @param channel the channel for which the subscription state should be changed
     * @param value if a subscription should me made (true) or not (false)
     * @return the created vector message holding uncompressed data
     */
    static std::unique_ptr<VectorMessage> forOpCodeSubscription(uint8_t channel, bool value);

    /**
     * Constructs a new VectorMessage for the op-code AUTHENTICATION
     * Possible values of value are:
     *  - [CLIENT->SERVER] the user-key which should be authenticated
     *  - [SERVER->CLIENT] the end time of the authentication as ptime->iso_string
     *  - [SERVER->CLIENT] an empty string (if the authentication check is omitted)
     *
     * @param value the user key to authenticate with OR the end time of the authentication OR an empty string
     * @return the created vector message holding uncompressed data
     */
    static std::unique_ptr<VectorMessage> forOpCodeAuthentication(const std::string &value);

    /**
     * Constructs a new VectorMessage for the op-code NOTIFICATION
     * 
     * @param message the notification message
     * @return the created vector message holding uncompressed data
     */
    static std::unique_ptr<VectorMessage> forOpCodeNotification(const std::string &message);

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
    std::shared_ptr<std::vector<uint8_t> > combinedVector;

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
};


#endif //VECTORMESSAGE_HPP
