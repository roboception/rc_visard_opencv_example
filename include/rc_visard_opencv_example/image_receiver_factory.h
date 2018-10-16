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

#ifndef RC_VISARD_OPENCV_EXAMPLE_IMAGE_RECEIVER_FACTORY_H
#define RC_VISARD_OPENCV_EXAMPLE_IMAGE_RECEIVER_FACTORY_H

#include <rc_visard_opencv_example/common.h>

#include <memory>

class ImageReceiver;

/**
 * @brief Base class of all ImageReceiver factories.
 *
 * The factories are passed to GcReceiver for indicating which image streams
 * to listen to.
 */
class ImageReceiverFactory
{
  public:
    using Ptr = std::shared_ptr<ImageReceiverFactory>;
    using ConstPtr = std::shared_ptr<const ImageReceiverFactory>;

    virtual ~ImageReceiverFactory() = default;

    /**
     * @brief Enable streaming of this image type and make the
     * approprate receiver.
     *
     * This method will internally be called by GcReceiver.
     *
     * @return The created ImageReceiver of successful, nullptr if not.
     */
    virtual std::unique_ptr<ImageReceiver> enableAndMake(
        const NodeMap &node_map) const = 0;
};

/**
 * @brief Factory for an intensity image receiver.
 */
class IntensityReceiverFactory : public ImageReceiverFactory
{
  public:
    /**
     * @brief Constructor.
     * @param left_right whether to listen to the left or right image, or both
     * @param enable_color_if_available whether to enable streaming of color
     * image if it's a color device
     */
    IntensityReceiverFactory(LeftRight left_right,
                             bool enable_color_if_available);

    virtual ~IntensityReceiverFactory() = default;

    virtual std::unique_ptr<ImageReceiver> enableAndMake(
        const NodeMap &node_map) const;

  private:
    const LeftRight left_right_;
    const bool enable_color_if_available_;
};

/**
 * @brief Factory for a disparity image receiver.
 */
class DisparityReceiverFactory : public ImageReceiverFactory
{
  public:
    virtual ~DisparityReceiverFactory() = default;

    virtual std::unique_ptr<ImageReceiver> enableAndMake(
        const NodeMap &node_map) const;
};

/**
 * @brief Factory for a confidence image receiver.
 */
class ConfidenceReceiverFactory : public ImageReceiverFactory
{
  public:
    virtual ~ConfidenceReceiverFactory() = default;

    virtual std::unique_ptr<ImageReceiver> enableAndMake(
        const NodeMap &node_map) const;
};

/**
 * @brief Factory for an error image receiver.
 */
class ErrorReceiverFactory : public ImageReceiverFactory
{
  public:
    virtual ~ErrorReceiverFactory() = default;

    virtual std::unique_ptr<ImageReceiver> enableAndMake(
        const NodeMap &node_map) const;
};


#endif //RC_VISARD_OPENCV_EXAMPLE_IMAGE_RECEIVER_FACTORY_H
