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

#include <rc_visard_opencv_example/gc_receiver.h>
#include "image_receiver.h"

#include <rc_genicam_api/device.h>
#include <rc_genicam_api/config.h>
#include <rc_genicam_api/stream.h>
#include <rc_genicam_api/image.h>
#include <rc_genicam_api/pixel_formats.h>
#include <rc_genicam_api/imagelist.h>

#include <unordered_set>

constexpr auto *GC_COMPONENT_SELECTOR = "ComponentSelector";
constexpr auto *GC_COMPONENT_ENABLE = "ComponentEnable";

GcReceiver::GcReceiver(std::string device_name, bool synchronize_data) :
    device_name_(device_name),
    synchronize_data_(synchronize_data),
    device_(nullptr)
{}

GcReceiver::~GcReceiver()
{
  try
  {
    // close streams and connection to device
    if (stream_)
    { stream_->close(); }
    if (device_)
    { device_->close(); }
  }
  catch (const std::exception &ex)
  {
    std::cerr << "Could not close GcReceiver: " << ex.what() << std::endl;
  }
  catch (const GENICAM_NAMESPACE::GenericException &ex)
  {
    std::cerr << "Could not close GcReceiver: " << ex.what() << std::endl;
  }
  catch (...)
  {
    std::cerr << "Could not close GcReceiver" << std::endl;
  }
}

GcReceiver::GcReceiver(GcReceiver &&other) :
    device_name_(other.device_name_),
    synchronize_data_(other.synchronize_data_),
    device_(std::move(other.device_)),
    nodemap_(std::move(other.nodemap_)),
    stream_(std::move(other.stream_)),
    image_receivers_(std::move(image_receivers_))
{
  other.image_receivers_.clear();
  other.stream_.reset();
  other.nodemap_.reset();
  other.device_.reset();
}

bool GcReceiver::open()
{
  if (isOpened())
  {
    throw std::runtime_error("Device is already opened");
  }

  // Search for device. device_name can be either serial number, GenTL ID
  // or user defined ID
  auto device = rcg::getDevice(device_name_.c_str());
  if (!device)
  {
    std::cerr << "Could not get device" << std::endl;
    return false;
  }

  // open connection to device
  try
  {
    device->open(rcg::Device::CONTROL);
  }
  catch (const std::exception &ex)
  {
    std::cerr << "Could not open device: " << ex.what() << std::endl;
    return false;
  }
  catch (const GENICAM_NAMESPACE::GenericException &ex)
  {
    std::cerr << "Could not open device: " << ex.what() << std::endl;
    return false;
  }
  catch (...)
  {
    std::cerr << "Could not open device" << std::endl;
    return false;
  }

  device_ = std::move(device);
  assert(device_);
  return true;
}

bool GcReceiver::initializeStreams(
    const ReceiverFactories &image_receiver_factories)
{
  if (!isOpened())
  {
    throw std::runtime_error("Device has not been opened");
  }

  try
  {
    // get Genicam node map
    auto nodemap = device_->getRemoteNodeMap();
    if (!nodemap)
    {
      std::cerr << "Could not get nodemap" << std::endl;
      return false;
    }

    // first, get all available components and disable all
    std::vector<std::string> available_components;
    rcg::getEnum(nodemap, "ComponentSelector", available_components, true);
    for (const auto &c : available_components)
    {
      rcg::setEnum(nodemap, GC_COMPONENT_SELECTOR, c.c_str(), true);
      rcg::setBoolean(nodemap, GC_COMPONENT_ENABLE, false, true);
    }

    // second, enable the requested components again
    std::vector<std::unique_ptr<ImageReceiver>> image_receivers;
    for (const auto &receiver_factory : image_receiver_factories)
    {
      image_receivers.emplace_back(
          receiver_factory->enableAndMake(nodemap));
    }

    // start streaming
    auto streams = device_->getStreams();
    if (streams.empty())
    {
      std::cerr << "Could not start streaming" << std::endl;
      return false;
    }
    streams[0]->open();
    streams[0]->startStreaming();

    nodemap_ = std::move(nodemap);
    stream_ = std::move(streams[0]);
    image_receivers_ = std::move(image_receivers);
  }
  catch (const std::exception &ex)
  {
    std::cerr << "Could not initialize GcReceiver" << ex.what() << std::endl;
    return false;
  }
  catch (const GENICAM_NAMESPACE::GenericException &ex)
  {
    std::cerr << "Could not initialize GcReceiver" << ex.what() << std::endl;
    return false;
  }
  catch (...)
  {
    std::cerr << "Could not initialize GcReceiver" << std::endl;
    return false;
  }

  assert(nodemap_);
  assert(stream_);
  return true;
}

std::unique_ptr<ImageSet> GcReceiver::receive(std::chrono::milliseconds timeout)
{
  if (!isInitialized())
  {
    throw std::runtime_error("GcReceiver has not been initialized");
  }

  auto timeout_time = std::chrono::steady_clock::now() + timeout;

  // loop until timeout
  while (std::chrono::steady_clock::now() < timeout_time)
  {
    // Grab next image from Genicam.
    // The passed timeout is the remaining time for this loop.
    const auto *buffer = stream_->grab(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            timeout_time - std::chrono::steady_clock::now()).count());
    if (buffer == nullptr)
    { return {}; }

    if (buffer->getIsIncomplete())
    {
      std::cerr << "Received incomplete buffer" << std::endl;
      continue;
    }

    // for multipart support, iterate all Genicam buffer parts
    const auto n_parts = buffer->getNumberOfParts();
    for (std::size_t part_idx = 0; part_idx < n_parts; ++part_idx)
    {
      if (!buffer->getImagePresent(part_idx))
      {
        std::cerr << "Buffer does not contain an image" << std::endl;
        continue;
      }

      // Iterate all receivers and check if one of them is responsible for
      // receiving the image contained in the buffer
      for (const auto &receiver : image_receivers_)
      {
        if (receiver->process(*buffer, 0))
        {
          // this receiver was responsible
          break;
        }
      }
    }

    const auto timestamp = buffer->getTimestampNS();
    if (synchronize_data_)
    {
      // client-side synchronization is turned on, so check if all receivers
      // have received an image with the timestamp of the current image buffer
      if (std::all_of(image_receivers_.begin(), image_receivers_.end(),
                      [timestamp](
                          const std::unique_ptr<ImageReceiver> &receiver)
                      {
                        return receiver->contains(timestamp);
                      }))
      {
        // grab all the images and return the synchronized image set
        //
        // XXX checking must be done before grabbing, since grab() will
        // remove the image from the receiver's queue
        auto image_set = std::unique_ptr<ImageSet>(new ImageSet());
        for (const auto &receiver : image_receivers_)
        {
          receiver->grab(timestamp, *image_set);
        }
        return image_set;
      }
    }
    else
    {
      // client-side synchronization is off, so we grab the images
      // from all buffers that contain an image with the timestamp if the
      // current image buffer
      auto image_set = std::unique_ptr<ImageSet>(new ImageSet());
      bool got_image = false;
      for (const auto &receiver : image_receivers_)
      {
        if (receiver->contains(timestamp))
        {
          receiver->grab(timestamp, *image_set);
          got_image = true;
        }
      }
      if (got_image)
      { return image_set; }
      else
      {
        std::cerr << "Received image buffer was not processed by any receiver"
                  << std::endl;
      }
    }
  }

  return {};
}
