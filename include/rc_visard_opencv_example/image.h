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

#ifndef RC_VISARD_OPENCV_EXAMPLE_IMAGE_H
#define RC_VISARD_OPENCV_EXAMPLE_IMAGE_H

#include "common.h"

#include <chrono>
#include <memory>

/**
 * @brief Base class for all the image containers.
 */
struct Image
{
    enum class Type
    {
        LEFT,
        RIGHT,
        DISPARITY,
        ERROR_IMG,
        CONFIDENCE
    };

    using Timestamp = std::chrono::system_clock::time_point;

    /**
     * @brief Type of the contained image.
     */
    Type type_;
    /**
     * @brief The image data.
     */
    cv::Mat data_;
    /**
     * @brief Timestamp of the image.
     */
    Timestamp timestamp_;

  protected:
    Image(Type type, cv::Mat data, Timestamp timestamp) :
        type_(type), data_(std::move(data)), timestamp_(timestamp)
    {}
};

template<Image::Type type>
struct ImageBase : Image
{
  ImageBase(cv::Mat data, Timestamp timestamp) :
      Image{type, std::move(data), timestamp}
  {}
};

/**
 * @brief Image container representing a left image.
 *
 * The type of the contained cv::Mat will be CV_8UC1 or CV_8UC3, depending
 * on whether a monochrome or color image is contained, resp.
 */
struct LeftImage : ImageBase<Image::Type::LEFT>
{
  using Ptr = std::shared_ptr<LeftImage>;
  using ConstPtr = std::shared_ptr<const LeftImage>;

  using ImageBase::ImageBase;
};

/**
 * @brief Image container representing a right image.
 *
 * The type of the contained cv::Mat will be CV_8UC1 or CV_8UC3, depending
 * on whether a monochrome or color image is contained, resp.
 */
struct RightImage : ImageBase<Image::Type::RIGHT>
{
  using Ptr = std::shared_ptr<RightImage>;
  using ConstPtr = std::shared_ptr<const RightImage>;

  using ImageBase::ImageBase;
};

/**
 * @brief Image container representing a confidence image.
 *
 * The type of the contained cv::Mat will be CV_32FC1.
 */
struct ConfidenceImage : ImageBase<Image::Type::CONFIDENCE>
{
  using Ptr = std::shared_ptr<ConfidenceImage>;
  using ConstPtr = std::shared_ptr<const ConfidenceImage>;

  using ImageBase::ImageBase;
};

/**
 * @brief Image container representing an error image.
 *
 * The type of the contained cv::Mat will be CV_32FC1.
 */
struct ErrorImage : ImageBase<Image::Type::ERROR_IMG>
{
  using Ptr = std::shared_ptr<ErrorImage>;
  using ConstPtr = std::shared_ptr<const ErrorImage>;

  using ImageBase::ImageBase;
};

/**
 * @brief Image container representing a disparity image.
 *
 * The type of the contained cv::Mat will be CV_32FC1.
 */
struct DisparityImage : ImageBase<Image::Type::DISPARITY>
{
  using Ptr = std::shared_ptr<DisparityImage>;
  using ConstPtr = std::shared_ptr<const DisparityImage>;

  DisparityImage(cv::Mat data, Timestamp timestamp,
                 double baseline, double focal_length, double cx, double cy) :
      ImageBase{std::move(data), timestamp},
      baseline_(baseline),
      focal_length_(focal_length),
      cx_(cx), cy_(cy)
  {};

  /**
   * @brief Baseline of the device.
   */
  double baseline_;
  /**
   * @brief Focal length of the device.
   */
  double focal_length_;
  /**
   * @brief Principal point in x direction of the device.
   */
  double cx_;
  /**
   * @brief Principal point in y direction of the device.
   */
  double cy_;
};

/**
 * @brief A set of all images potentially contained in a synchronized data set.
 */
struct ImageSet
{
  LeftImage::Ptr left_img_;
  RightImage::Ptr right_img_;
  DisparityImage::Ptr disparity_img_;
  ConfidenceImage::Ptr confidence_img_;
  ErrorImage::Ptr error_img_;
};

#endif //RC_VISARD_OPENCV_EXAMPLE_IMAGE_H
