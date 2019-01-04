/*
 * This file is part of the rc_visard_opencv_example package.
 *
 * Copyright (c) 2018 Roboception GmbH
 * All rights reserved
 *
 * Author: Raphael Schaller
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RC_VISARD_OPENCV_EXAMPLE_GC_RECEIVER_H
#define RC_VISARD_OPENCV_EXAMPLE_GC_RECEIVER_H

#include "image.h"
#include "common.h"
#include "image_receiver_factory.h"

#include <rc_genicam_api/genicam/GenICamFwd.h>

#include <string>
#include <set>
#include <unordered_map>

/**
 * @brief Comparison function for shared_ptr to ImageReceiverFactory.
 *
 * It compares the std::type_info::hash_code of the factories.
 * It is used to ensure that each concrete ImageReceiverFactory is passed to
 * GcReceiver only once.
 */
struct ImageReceiverFactoryCmp
{
  bool operator()(const ImageReceiverFactory::ConstPtr &lhs,
                  const ImageReceiverFactory::ConstPtr &rhs)
  {
    if (!lhs && rhs) { return true; }
    if (!rhs) { return false; }
    return typeid(*lhs).hash_code() < typeid(*rhs).hash_code();
  }
};

/**
 * @brief A wrapper around rc_genicam_api for automatic bootstrapping and
 * shutdowning genicam, and conversion of the images to cv::Mat.
 *
 * The class methods must be called in a specific order:
 * 1. open(): it will open the connection to the genicam device.
 * 2. initializeStreams(): it will initialize the image streams and
 *    start streaming.
 *
 * After that, receive() can be called in a loop to grab the images.
 */
class GcReceiver
{
  public:
    using ReceiverFactories =
        std::set<ImageReceiverFactory::ConstPtr, ImageReceiverFactoryCmp>;

  public:
    /**
     * @brief Constructor.
     *
     * Will not set up the connection. Use open() for that.
     *
     * @param device_name ID of the device
     * @param synchronize_data whether to synchronize the received data
     */
    GcReceiver(std::string device_name, bool synchronize_data);

    ~GcReceiver();

    GcReceiver(const GcReceiver &) = delete;
    GcReceiver &operator=(const GcReceiver &) = delete;
    GcReceiver &operator=(GcReceiver &&other) = delete;

    GcReceiver(GcReceiver &&other);

    /**
     * @brief Checks if open() has been called successfully.
     */
    bool isOpened() const noexcept
    { return device_.operator bool(); }

    /**
     * @brief Checks if initializeStreams() has been called successfully.
     */
    bool isInitialized() const noexcept
    {
      return nodemap_.operator bool() && stream_.operator bool();
    }

    /**
     * @brief Open the connection.
     * @return true of successful, otherwise false
     */
    bool open();

    /**
     * @brief Enables image streams and starts streaming.
     * @param image_receiver_factories set of image streams to enable
     * @return true of successful, otherwise false
     */
    bool initializeStreams(const ReceiverFactories &image_receiver_factories);

    /**
     * @brief Waits for a new image to be received.
     *
     * Depending on whether synchonization is enabled (see constructor),
     * the ImageSet will contain only one image (if synchronization is
     * disabled) or all images for which streaming is enabled, all with
     * the same timestamp. Intermediate images that don't form a synchronized
     * set, are dropped.
     *
     * @param timeout maximum time to wait for a new result
     * @return the image set containing the image(s), or nullptr in case of
     * a timeout or error
     */
    std::unique_ptr<ImageSet> receive(std::chrono::milliseconds timeout);

    const std::shared_ptr<GENAPI_NAMESPACE::CNodeMapRef> getNodeMap() const;

  private:
    const std::string device_name_;
    const bool synchronize_data_;

    std::shared_ptr<rcg::Device> device_;
    std::shared_ptr<GENAPI_NAMESPACE::CNodeMapRef> nodemap_;
    std::shared_ptr<rcg::Stream> stream_;

    std::vector<std::unique_ptr<ImageReceiver>> image_receivers_;
};


#endif //RC_VISARD_OPENCV_EXAMPLE_GC_RECEIVER_H
