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

#ifndef RC_VISARD_OPENCV_EXAMPLE_IMAGE_RECEIVER_H
#define RC_VISARD_OPENCV_EXAMPLE_IMAGE_RECEIVER_H

#include <rc_visard_opencv_example/image.h>
#include <rc_visard_opencv_example/common.h>

#include <rc_genicam_api/imagelist.h>
#include <rc_genicam_api/genicam/GenICamFwd.h>

#include <deque>


/**
 * @brief Base class for image receivers.
 *
 * Each concrete image receiver is responsible for enabling and receiving
 * one component from the device. This includes conversion of the received
 * image to a cv::Mat.
 *
 * Each ImageReceiver is constructed by an ImageReceiverFactory. ImageReceiver
 * is to be used only internally.
 *
 * Each ImageReceiver contains an image queue that is required for the case
 * that synchronization is turned on.
 * In process(), the image is put into the queue.
 * grab() then gets the image from the queue and converts it to a cv::Mat.
 *
 */
class ImageReceiver
{
  public:
    ImageReceiver(const ImageReceiver &) = delete;
    ImageReceiver &operator=(const ImageReceiver &) = delete;
    ImageReceiver(ImageReceiver &&other) = delete;
    ImageReceiver &operator=(ImageReceiver &&) = delete;

    virtual ~ImageReceiver() = default;

    /**
     * @brief Processes a Genicam buffer in case this ImageReceiver is
     * responsible for this kind of buffer.
     *
     * Responsibility is determined by the pixel format of the buffer.
     *
     * @param buffer Genicam buffer
     * @param part_idx part index for multipart support
     * @return true, if the buffer was processed; false, otherwise
     */
    bool process(const rcg::Buffer &buffer, std::size_t part_idx);

    /**
     * @brief Checks if the image queue contains an image with a certain
     * timestamp.
     */
    bool contains(std::uint64_t timestamp_ns) const;

    /**
     * @brief Gets a previously processed image with a certain timestamp.
     *
     * This method will get an image from the image queue, convert to to a
     * cv::Mat and copy it to the appropriate member of ImageSet.
     *
     * @param timestamp_ns timestamp of the image
     * @param set this ImageSet will contain the grabbed image
     */
    void grab(std::uint64_t timestamp_ns, ImageSet &set);

  protected:
    explicit ImageReceiver(std::uint64_t pixel_format_, size_t buffer_size);

  private:
    /**
     * @brief Method for converting a Genicam buffer to a cv::Mat.
     */
    virtual void copyToImageSet(const rcg::Image &buffer, ImageSet &set) = 0;

  private:
    const std::uint64_t pixel_format_;
    rcg::ImageList image_list_;
};


/**
 * @brief Image receiver responsible for intensity images.
 */
class IntensityReceiver : public ImageReceiver
{
  public:
    virtual ~IntensityReceiver() = default;

    /**
     * @brief Enables the intensity component and creates a new instance of
     * an IntensityReceiver.
     *
     * @param node_map Genicam node map
     * @param left_right whether to receive the left or right image, or both
     * @param enable_color_if_available whether to enable streaming of color
     * images if it is a color camera
     * @return the created IntensityReceiver instance is successful, otherwise
     * nullptr
     */
    static std::unique_ptr<IntensityReceiver> enableAndMake(
        const NodeMap &node_map,
        LeftRight left_right,
        bool enable_color_if_available);

  protected:
    IntensityReceiver(LeftRight left_right, bool color);

  private:
    virtual void
    copyToImageSet(const rcg::Image &buffer, ImageSet &set) override;

  private:
    LeftRight left_right_;
    bool color_;
};

/**
 * @brief Image receiver responsible for disparity images.
 */
class DisparityReceiver : public ImageReceiver
{
  public:
    virtual ~DisparityReceiver() = default;

    /**
     * @brief Enables the disparity component and creates a new instance of
     * a DisparityReceiver.
     *
     * @param node_map Genicam node map
     * @return the created DisparityReceiver instance is successful, otherwise
     * nullptr
     */
    static std::unique_ptr<DisparityReceiver> enableAndMake(
        const NodeMap &node_map);

  protected:
    DisparityReceiver(float coord_scale, int coord_inv,
                      DisparityImage image_template);

  private:
    virtual void copyToImageSet(const rcg::Image &buffer,
                                ImageSet &set) override;

  private:
    float coord_scale_;
    int coord_inv_;
    DisparityImage image_template_;
};

/**
 * @brief Image receiver responsible for confidence images.
 */
class ConfidenceReceiver : public ImageReceiver
{
  public:
    virtual ~ConfidenceReceiver() = default;

    /**
     * @brief Enables the confidence component and creates a new instance of
     * a ConfidenceReceiver.
     *
     * @param node_map Genicam node map
     * @return the created ConfidenceReceiver instance is successful, otherwise
     * nullptr
     */
    static std::unique_ptr<ConfidenceReceiver>
    enableAndMake(const NodeMap &node_map);

  private:
    virtual void copyToImageSet(const rcg::Image &buffer,
                                ImageSet &set) override;

  protected:
    ConfidenceReceiver(float coord_scale);

  private:
    float coord_scale_;
};

/**
 * @brief Image receiver responsible for error images.
 */
class ErrorReceiver : public ImageReceiver
{
  public:
    virtual ~ErrorReceiver() = default;

    /**
     * @brief Enables the confidence component and creates a new instance of
     * a ErrorReceiver.
     *
     * @param node_map Genicam node map
     * @return the created ErrorReceiver instance is successful, otherwise
     * nullptr
     */
    static std::unique_ptr<ErrorReceiver>
    enableAndMake(const NodeMap &node_map);

  private:
    virtual void
    copyToImageSet(const rcg::Image &buffer, ImageSet &set) override;

  protected:
    ErrorReceiver(float coord_scale);

  private:
    float coord_scale_;
};

#endif //RC_VISARD_OPENCV_EXAMPLE_IMAGE_RECEIVER_H
