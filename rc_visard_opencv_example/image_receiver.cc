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

#include "image_receiver.h"

#include <rc_genicam_api/device.h>
#include <rc_genicam_api/config.h>
#include <rc_genicam_api/stream.h>
#include <rc_genicam_api/image.h>
#include <rc_genicam_api/pixel_formats.h>
#include <rc_genicam_api/imagelist.h>
#include <rc_genicam_api/buffer.h>

#include <iostream>

constexpr char GC_INTENSITY[] = "Intensity";
constexpr char GC_INTENSITY_COMBINED[] = "IntensityCombined";
constexpr char GC_DISPARITY[] = "Disparity";
constexpr char GC_CONFIDENCE[] = "Confidence";
constexpr char GC_ERROR[] = "Error";
constexpr char GC_COMPONENT_SELECTOR[] = "ComponentSelector";
constexpr char GC_COMPONENT_ENABLE[] = "ComponentEnable";
constexpr char GC_PIXEL_FORMAT[] = "PixelFormat";
constexpr char GC_MONO8[] = "Mono8";
constexpr char GC_YCBCR411_8[] = "YCbCr411_8";
constexpr char GC_RGB8[] = "RGB8";
constexpr char GC_FOCAL_LENGTH_FACTOR[] = "FocalLengthFactor";
constexpr char GC_BASELINE[] = "Baseline";
constexpr char GC_SCAN_3D_PRINCIPAL_POINT_U[] = "Scan3dPrincipalPointU";
constexpr char GC_SCAN_3D_PRINCIPAL_POINT_V[] = "Scan3dPrincipalPointV";
constexpr char GC_SCAN_3D_COORDINATE_SCALE[] = "Scan3dCoordinateScale";
constexpr char GC_SCAN_3D_INVALID_DATA_VALUE[] = "Scan3dInvalidDataValue";

/**
 * @brief Selects a Genicam component and enables it.
 * @param node_map Genicam node map
 * @param component component to enable
 * @return success
 */
static bool selectAndEnableComponent(
    const NodeMap &node_map,
    const char *component)
{
  try
  {
    rcg::setEnum(node_map, GC_COMPONENT_SELECTOR, component, true);
    rcg::setBoolean(node_map, GC_COMPONENT_ENABLE, true, true);
  }
  catch (const std::exception &ex)
  {
    std::cerr << "Could not enable " << component << ": " <<
              ex.what() << std::endl;
    return false;
  }
  catch (const GENICAM_NAMESPACE::GenericException &ex)
  {
    std::cerr << "Could not enable " << component << ": " <<
              ex.what() << std::endl;
    return false;
  }
  catch (...)
  {
    std::cerr << "Could not enable " << component << std::endl;
    return false;
  }

  return true;
}

/**
 * @brief Converts the timestamp contained in the Genicam image buffer
 * to the timestamp contained in Image.
 */
static Image::Timestamp getTimestamp(const rcg::Image &buffer)
{
  return Image::Timestamp{
      std::chrono::duration_cast<Image::Timestamp::duration>(
          std::chrono::nanoseconds(buffer.getTimestampNS()))};
}

ImageReceiver::ImageReceiver(std::uint64_t pixel_format, size_t buffer_size) :
    pixel_format_(pixel_format),
    image_list_(buffer_size)
{}

bool ImageReceiver::process(const rcg::Buffer &buffer, std::size_t part_idx)
{
  // check if this receiver is responsible for this kind of Genicam buffer
  if (buffer.getPixelFormat(part_idx) == pixel_format_)
  {
    image_list_.add(&buffer, part_idx);
    return true;
  }

  return false;
}

bool ImageReceiver::contains(std::uint64_t timestamp_ns) const
{
  return static_cast<bool>(image_list_.find(timestamp_ns));
}

void ImageReceiver::grab(std::uint64_t timestamp_ns, ImageSet &set)
{
  // get the image from the queue
  const auto image = image_list_.find(timestamp_ns);
  if (!image)
  {
    throw std::out_of_range("timestamp_ns");
  }

  // convert the image to a cv::Mat
  copyToImageSet(*image, set);

  // remove the grabbed image and older ones from the image queue
  image_list_.removeOld(timestamp_ns);
}

IntensityReceiver::IntensityReceiver(LeftRight left_right, std::uint64_t format) :
    ImageReceiver(format, 50),
    left_right_(left_right),
    format_(format)
{}

std::unique_ptr<IntensityReceiver> IntensityReceiver::enableAndMake(
    const NodeMap &node_map, LeftRight left_right,
    bool enable_color_if_available)
{
  assert(node_map);

  try
  {
    // Check if we should enable the Intensity or IntensityCombined component.
    // The Intensity only contains the left image. IntensityCombined contains
    // left and right image.
    if (left_right == LeftRight::LEFT)
    {
      if (!selectAndEnableComponent(node_map, GC_INTENSITY))
      { return {}; }
    }
    else if (left_right == LeftRight::RIGHT ||
             left_right == LeftRight::LEFT_RIGHT)
    {
      if (!selectAndEnableComponent(node_map, GC_INTENSITY_COMBINED))
      { return {}; }
    }
    else
    { assert(false); }

    // enable streaming of color image if it is a color sensor
    std::uint64_t format=Mono8;

    if (enable_color_if_available)
    {
      // Get available pixel formats. The Intensity(Combined) component is
      // still selected due to the previous call to selectAndEnableComponent().
      std::vector<std::string> pixel_formats;
      rcg::getEnum(node_map, GC_PIXEL_FORMAT, pixel_formats, true);
      // Check if the available pixel formats contains YCbCr411_8. Then it's a
      // color sensor.
      if (std::find(pixel_formats.begin(), pixel_formats.end(),
                    GC_YCBCR411_8) != pixel_formats.end())
      {
        rcg::setEnum(node_map, GC_PIXEL_FORMAT, GC_YCBCR411_8, true);
        format = YCbCr411_8;
      }
      else if (std::find(pixel_formats.begin(), pixel_formats.end(),
               GC_RGB8) != pixel_formats.end())
      {
        rcg::setEnum(node_map, GC_PIXEL_FORMAT, GC_RGB8, true);
        format = RGB8;
      }
      else
      {
        rcg::setEnum(node_map, GC_PIXEL_FORMAT, GC_MONO8, true);
        format = Mono8;
      }
    }
    else
    {
      // Set pixel format to Mono8. This format is available for both
      // monochrome and color sensors.
      rcg::setEnum(node_map, GC_PIXEL_FORMAT, GC_MONO8, true);
      format = Mono8;
    }

    return std::unique_ptr<IntensityReceiver>(
        new IntensityReceiver(left_right, format));
  }
  catch (const std::exception &ex)
  {
    std::cerr << "Could not enable intensity: " << ex.what() << std::endl;
  }
  catch (const GENICAM_NAMESPACE::GenericException &ex)
  {
    std::cerr << "Could not enable intensity: " << ex.what() << std::endl;
  }
  catch (...)
  {
    std::cerr << "Could not enable intensity" << std::endl;
  }

  return {};
}

void IntensityReceiver::copyToImageSet(const rcg::Image &buffer,
                                       ImageSet &set)
{
  const auto width = static_cast<int>(buffer.getWidth());
  const auto height = static_cast<int>(buffer.getHeight());

  cv::Mat img;

  if (format_ == YCbCr411_8)
  {
    // convert a YCbCr411_8 buffer to a BGR cv::Mat

    assert(buffer.getPixelFormat() == YCbCr411_8);

    img = cv::Mat(height, width, CV_8UC3);

    // Width of a row in the Genicam buffer in bytes including padding.
    // Due to the encoding, 4 pixels are descibed by 6 bytes.
    const int buffer_step =
        (width / 4) * 6 + static_cast<int>(buffer.getXPadding());

    // data start pointer
    auto buffer_row_ptr = buffer.getPixels();

    for (int row_idx = 0; row_idx < height; ++row_idx)
    {
      const auto img_row = img.ptr<cv::Vec3b>(row_idx);
      auto buffer_cell_ptr = buffer_row_ptr;
      for (int col_idx = 0; col_idx < width; col_idx += 4)
      {
        // convert YCbCr411_8 to RGB
        rcg::convYCbCr411toQuadRGB(img_row[col_idx].val, buffer_cell_ptr,
                                   col_idx);
      }

      buffer_row_ptr += buffer_step;
    }

    // convert RGB to OpenCV specific BGR
    for (int row_idx = 0; row_idx < height; ++row_idx)
    {
      const auto img_row = img.ptr<cv::Vec3b>(row_idx);
      for (int col_idx = 0; col_idx < width; ++col_idx)
      {
        std::swap(img_row[col_idx].val[0], img_row[col_idx].val[2]);
      }
    }
  }
  else if (format_ == RGB8)
  {
    // convert a YCbCr411_8 buffer to a BGR cv::Mat

    assert(buffer.getPixelFormat() == RGB8);

    img = cv::Mat(height, width, CV_8UC3);

    // Width of a row in the Genicam buffer in bytes including padding.
    // Due to the encoding, 4 pixels are descibed by 6 bytes.
    const int buffer_step =
        3 * width + static_cast<int>(buffer.getXPadding());

    // data start pointer
    auto buffer_row_ptr = buffer.getPixels();

    for (int row_idx = 0; row_idx < height; ++row_idx)
    {
      const auto img_row = img.ptr<cv::Vec3b>(row_idx);
      auto buffer_cell_ptr = buffer_row_ptr;
      for (int col_idx = 0; col_idx < width; ++col_idx)
      {
        img_row[col_idx].val[0]=*buffer_cell_ptr++;
        img_row[col_idx].val[1]=*buffer_cell_ptr++;
        img_row[col_idx].val[2]=*buffer_cell_ptr++;
      }

      buffer_row_ptr += buffer_step;
    }

    // convert RGB to OpenCV specific BGR
    for (int row_idx = 0; row_idx < height; ++row_idx)
    {
      const auto img_row = img.ptr<cv::Vec3b>(row_idx);
      for (int col_idx = 0; col_idx < width; ++col_idx)
      {
        std::swap(img_row[col_idx].val[0], img_row[col_idx].val[2]);
      }
    }
  }
  else
  {
    // convert a Mono8 buffer to a mono cv::Mat

    assert(buffer.getPixelFormat() == Mono8);

    img = cv::Mat(height, width, CV_8UC1);

    // width of a row in the Genicam buffer including padding
    const int buffer_step = width + static_cast<int>(buffer.getXPadding());

    // data start pointer
    auto buffer_row_ptr = buffer.getPixels();

    for (int row_idx = 0; row_idx < height; ++row_idx)
    {
      const auto img_row = img.ptr<std::uint8_t>(row_idx);
      // plain byte-wise copy from the buffer to the cv::Mat
      std::copy(buffer_row_ptr, buffer_row_ptr + width, img_row);
      buffer_row_ptr += buffer_step;
    }
  }

  // The Intensity component contains only one image. The IntensityCombined
  // component contains the left and right image, stacked on top of each other.
  if (left_right_ == LeftRight::LEFT)
  {
    set.left_img_ = std::make_shared<LeftImage>(img, getTimestamp(buffer));
  }
  else
  {
    // extract right image
    cv::Mat right = img(cv::Rect(0, img.rows / 2, img.cols, img.rows / 2));
    set.right_img_ = std::make_shared<RightImage>(right, getTimestamp(buffer));
    if (left_right_ == LeftRight::LEFT_RIGHT)
    {
      // extract left image
      cv::Mat left = img(cv::Rect(0, 0, img.cols, img.rows / 2));
      set.left_img_ = std::make_shared<LeftImage>(left, getTimestamp(buffer));
    }
  }
}

DisparityReceiver::DisparityReceiver(float coord_scale, int coord_inv,
                                     DisparityImage image_template) :
    ImageReceiver(Coord3D_C16, 5),
    coord_scale_(coord_scale),
    coord_inv_(coord_inv),
    image_template_(std::move(image_template))
{}

std::unique_ptr<DisparityReceiver> DisparityReceiver::enableAndMake(
    const NodeMap &node_map)
{
  assert(node_map);

  if (!selectAndEnableComponent(node_map, GC_DISPARITY))
  { return {}; }

  double f, t, c_x, c_y, coord_scale;
  int coord_inv;

  try
  {
    // get important parameter for disparity such as the
    // stereo camera system parameters
    f = rcg::getFloat(node_map, GC_FOCAL_LENGTH_FACTOR, 0, 0, true);
    t = rcg::getFloat(node_map, GC_BASELINE, 0, 0, true);
    c_x = rcg::getFloat(node_map, GC_SCAN_3D_PRINCIPAL_POINT_U, 0, 0, true);
    c_y = rcg::getFloat(node_map, GC_SCAN_3D_PRINCIPAL_POINT_V, 0, 0, true);

    // The disparity is transmitted as uint16. This scaling parameter converts
    // it to a float.
    coord_scale =
        rcg::getFloat(node_map, GC_SCAN_3D_COORDINATE_SCALE, 0, 0, true);

    // This value represents invalid pixels in the Genicam buffer
    coord_inv = static_cast<int>(
        rcg::getFloat(node_map, GC_SCAN_3D_INVALID_DATA_VALUE, 0, 0, true));
  }
  catch (const std::exception &ex)
  {
    std::cerr << "Cannot read parameters for disparity: " << ex.what()
              << std::endl;
    return {};
  }
  catch (const GENICAM_NAMESPACE::GenericException &ex)
  {
    std::cerr << "Cannot read parameters for disparity: " << ex.what()
              << std::endl;
    return {};
  }
  catch (...)
  {
    std::cerr << "Cannot read parameters for disparity" << std::endl;
    return {};
  }

  // construct a template disparity image containing the camera parameters
  DisparityImage image_template({}, {}, t, f, c_x, c_y);

  return std::unique_ptr<DisparityReceiver>(
      new DisparityReceiver(coord_scale, coord_inv, std::move(image_template)));
}

/**
 * @brief Append two uint8 to a uint16 while taking endianess into account.
 * @tparam big_endian whether the uint8 data is big endian
 * @param d point to the first of the two uint8
 * @return resulting uint16
 */
template<bool big_endian>
static inline std::uint16_t appendUInt8(const std::uint8_t *const d)
{
  return (static_cast<std::uint16_t>(d[1 - big_endian]) << 8) | d[big_endian];
}

void DisparityReceiver::copyToImageSet(const rcg::Image &buffer,
                                       ImageSet &set)
{
  // convert disparity received from the device to cv::Mat
  //
  // The disparity in the Genicam buffer is encoded as uint16, scaled by a
  // constant factor. Therefore, we need to convert it to a float and scale
  // it accordingly to get the actual pixel value.

  const auto width = static_cast<int>(buffer.getWidth());
  const auto height = static_cast<int>(buffer.getHeight());
  const bool bigendian = buffer.isBigEndian();

  cv::Mat img(height, width, CV_32FC1);

  const int buffer_x_padding = static_cast<int>(buffer.getXPadding());
  const std::uint8_t *buffer_row_ptr = buffer.getPixels();

  for (int row_idx = 0; row_idx < height; ++row_idx)
  {
    const auto img_row_ptr = img.ptr<float>(row_idx);
    if (bigendian)
    {
      for (int col_idx = 0; col_idx < width; ++col_idx)
      {
        // append to consecutive uint8 to a uint16
        const auto v = appendUInt8<true>(buffer_row_ptr);
        if (v != coord_inv_)
        {
          // scale uint16 to get the actual disparity in pixel
          img_row_ptr[col_idx] = coord_scale_ * v;
        }
        else
        {
          // invalid pixel
          img_row_ptr[col_idx] = std::numeric_limits<float>::infinity();
        }
        buffer_row_ptr += 2;
      }
    }
    else
    {
      for (int col_idx = 0; col_idx < width; ++col_idx)
      {
        // append to consecutive uint8 to a uint16
        const auto v = appendUInt8<false>(buffer_row_ptr);
        if (v != coord_inv_)
        {
          // scale uint16 to get the actual disparity in pixel
          img_row_ptr[col_idx] = coord_scale_ * v;
        }
        else
        {
          // invalid pixel
          img_row_ptr[col_idx] = std::numeric_limits<float>::infinity();
        }
        buffer_row_ptr += 2;
      }
    }

    buffer_row_ptr += buffer_x_padding;
  }

  set.disparity_img_ = std::make_shared<DisparityImage>(image_template_);
  set.disparity_img_->timestamp_ = getTimestamp(buffer);
  set.disparity_img_->data_ = img;
  // the focal length stored in the template is the "focal length factor",
  // we therefore need to multiply it with the image width to get the
  // actual focal length
  set.disparity_img_->focal_length_ *= width;
}

/**
 * @brief Reads the coordinate scale factor of the device.
 * @param node_map Genicam node map
 * @param coord_scale read coordinate scale
 * @return success
 */
static bool getCoordScaleParam(const NodeMap &node_map, float &coord_scale)
{
  try
  {
    coord_scale = static_cast<float>(
        rcg::getFloat(node_map, GC_SCAN_3D_COORDINATE_SCALE, 0, 0, true));
  }
  catch (const std::exception &ex)
  {
    std::cerr << "Cannot read " << GC_SCAN_3D_COORDINATE_SCALE << ": "
              << ex.what() << std::endl;
    return false;
  }
  catch (const GENICAM_NAMESPACE::GenericException &ex)
  {
    std::cerr << "Cannot read " << GC_SCAN_3D_COORDINATE_SCALE << ": "
              << ex.what() << std::endl;
    return false;
  }
  catch (...)
  {
    std::cerr << "Cannot read " << GC_SCAN_3D_COORDINATE_SCALE << std::endl;
    return false;
  }
  return true;
}

/**
 * @brief Copies a uint8 Genicam buffer to a CV_32FC1 cv::Mat while
 * taking into account a scale factor.
 * @param buffer Genicam buffer
 * @param scale factor for conversion from uint8 to float
 */
static cv::Mat copy8bitBufferToFloat(const rcg::Image &buffer,
                                     const float scale)
{
  const auto width = static_cast<int>(buffer.getWidth());
  const auto height = static_cast<int>(buffer.getHeight());
  cv::Mat img(height, width, CV_32FC1);

  const int buffer_step = width + static_cast<int>(buffer.getXPadding());
  auto buffer_row_ptr = buffer.getPixels();

  for (int row_idx = 0; row_idx < height; ++row_idx)
  {
    const auto img_row = img.ptr<float>(row_idx);
    for (int col_idx = 0; col_idx < width; ++col_idx)
    {
      img_row[col_idx] = scale * buffer_row_ptr[col_idx];
    }
    buffer_row_ptr += buffer_step;
  }

  return img;
}

ConfidenceReceiver::ConfidenceReceiver(float coord_scale) :
    ImageReceiver(Confidence8, 5),
    coord_scale_(coord_scale)
{}

std::unique_ptr<ConfidenceReceiver> ConfidenceReceiver::enableAndMake(
    const NodeMap &node_map)
{
  assert(node_map);

  if (!selectAndEnableComponent(node_map, GC_CONFIDENCE))
  { return {}; }

  float coord_scale;
  if (!getCoordScaleParam(node_map, coord_scale))
  { return {}; }

  return std::unique_ptr<ConfidenceReceiver>(
      new ConfidenceReceiver(coord_scale));
}

void ConfidenceReceiver::copyToImageSet(const rcg::Image &buffer,
                                        ImageSet &set)
{
  cv::Mat img = copy8bitBufferToFloat(buffer, coord_scale_);
  set.confidence_img_ = std::make_shared<ConfidenceImage>(img,
                                                          getTimestamp(buffer));
}

ErrorReceiver::ErrorReceiver(float coord_scale) :
    ImageReceiver(Error8, 5),
    coord_scale_(coord_scale)
{}

std::unique_ptr<ErrorReceiver> ErrorReceiver::enableAndMake(
    const NodeMap &node_map)
{
  assert(node_map);

  float coord_scale;
  if (!getCoordScaleParam(node_map, coord_scale))
  { return {}; }

  if (!selectAndEnableComponent(node_map, GC_ERROR))
  { return {}; }
  return std::unique_ptr<ErrorReceiver>(new ErrorReceiver(coord_scale));
}

void
ErrorReceiver::copyToImageSet(const rcg::Image &buffer, ImageSet &set)
{
  cv::Mat img = copy8bitBufferToFloat(buffer, coord_scale_);
  set.error_img_ = std::make_shared<ErrorImage>(img, getTimestamp(buffer));
}
