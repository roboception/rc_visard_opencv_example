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

#include <rc_visard_opencv_example/gc_cleaner.h>
#include <rc_visard_opencv_example/gc_receiver.h>

#include <rc_genicam_api/config.h>

#if CV_MAJOR_VERSION == 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <iostream>
#include <csignal>
#include <atomic>
#include <map>

namespace
{
std::atomic_bool stop;

void signal_handler(int sig)
{
  signal(sig, signal_handler);
  stop = true;
}
}

bool left = false;
bool right = false;
bool disparity = false;
bool confidence = false;
bool error = false;

bool synchronize_data = false;

int frame_rate = 0;

// command line flags
std::map<std::string, bool *> flags
    {
        {"--left",        &left},
        {"--right",       &right},
        {"--disparity",   &disparity},
        {"--confidence",  &confidence},
        {"--error",       &error},
        {"--synchronize", &synchronize_data}
    };

std::string device;

// parse command line arguments
bool parseArguments(int argc, char **argv)
{
  if (argc < 2)
  { return false; }

  // iterate command line flags and check if each is contained in the flags map
  for (int i = 1; i < argc - 1; ++i)
  {
    const auto flag = flags.find(argv[i]);
    if (flag != flags.end())
    {
      *flag->second = true;
    }
    else if (std::string(argv[i]).find("--frame-rate=") == 0)
    {
      const std::string f_str = std::string(argv[i]).substr(13);
      try
      {
        frame_rate = std::stoi(f_str);
      }
      catch (const std::exception &)
      {
        std::cerr << "Value of frame rate is no number" << std::endl;
        return false;
      }
    }
    else
    {
      std::cerr << "Argument " << argv[i] << " not known" << std::endl;
      return false;
    }
  }

  // read device id, it should be at the last position
  const std::string dev = argv[argc - 1];
  if (dev.find("--") == 0)
  {
    std::cerr << "Final argument must be device ID" << std::endl;
    return false;
  }

  device = dev;
  return true;
}

// name of the OpenCV window
static const std::string cv_win_name = "stream";

int main(int argc, char **argv)
{
  // install signal handler to catch Ctrl+C
  signal(SIGINT, signal_handler);

  if (!parseArguments(argc, argv))
  {
    std::cerr << "Usage: " << argv[0] << " [options] <device id>\n";
    for (const auto &s : flags)
    {
      std::cerr << '\t' << s.first << '\n';
    }
    std::cerr << '\t' << "--frame-rate=<n>" << '\n';
    return 1;
  }

  // RAII genicam resource cleaner
  GcCleaner gc_cleaner;

  // wrapper around rc_genicam_api
  GcReceiver gc_receiver(device, synchronize_data);

  // open connection to device
  if (!gc_receiver.open())
  {
    std::cerr << "Could not open device '" << device << '\'' << std::endl;
    return 1;
  }

  // depending on command line arguments,
  // create respective image receiver factories
  GcReceiver::ReceiverFactories receiver_factories;

  if (left || right)
  {
    LeftRight left_right;
    if (left && right)
    { left_right = LeftRight::LEFT_RIGHT; }
    else if (left)
    { left_right = LeftRight::LEFT; }
    else
    { left_right = LeftRight::RIGHT; }
    receiver_factories.insert(std::make_shared<IntensityReceiverFactory>(
        left_right, true));
  }
  if (disparity)
  {
    receiver_factories.insert(std::make_shared<DisparityReceiverFactory>());
  }
  if (confidence)
  {
    receiver_factories.insert(std::make_shared<ConfidenceReceiverFactory>());
  }
  if (error)
  {
    receiver_factories.insert(std::make_shared<ErrorReceiverFactory>());
  }

  if (receiver_factories.empty())
  {
    std::cerr << "At least one stream must be enabled" << std::endl;
    return 1;
  }

  // enable streams and start streaming
  if (!gc_receiver.initializeStreams(receiver_factories))
  {
    std::cerr << "Could not initialize flags" << std::endl;
    return 1;
  }

  // set frame rate if requested
  if (frame_rate > 0)
  {
    if (!rcg::setFloat(gc_receiver.getNodeMap(), "AcquisitionFrameRate",
                       frame_rate, false))
    {
      std::cerr << "Could not set frame rate" << std::endl;
      return 1;
    }
  }

  std::cout
      << "Press 'n'(ext) or 'p'(revious) to cycle through streams, 'q' to exit"
      << std::endl;

  // Put all enabled streams in a list to create a mapping from some continuous
  // index to the streams. This index will be used to cycle through the streams.
  std::vector<bool *> show;
  if (left)
  { show.push_back(&left); }
  if (right)
  { show.push_back(&right); }
  if (disparity)
  { show.push_back(&disparity); }
  if (confidence)
  { show.push_back(&confidence); }
  if (error)
  { show.push_back(&error); }

  int current_stream = 0;

  // loop until Ctrl+C is hit
  while (!stop)
  {
    // receive images with a 3 s timeout
    const auto image_set = gc_receiver.receive(std::chrono::seconds(3));
    if (!image_set)
    {
      std::cerr << "Did not receive data before timeout" << std::endl;
      continue;
    }

    // Method for setting the OpenCV window title.
    // This is only available beginning with OpenCV 3.0
    auto setWindowTitle = [](const std::string &title)
    {
#if CV_MAJOR_VERSION == 3
      cv::setWindowTitle(cv_win_name, title);
#endif
    };

    // check which stream to show and whether we got data for that stream
    if (show[current_stream] == &left && image_set->left_img_)
    {
      cv::imshow(cv_win_name, image_set->left_img_->data_);
      setWindowTitle("left");
    }
    if (show[current_stream] == &right && image_set->right_img_)
    {
      cv::imshow(cv_win_name, image_set->right_img_->data_);
      setWindowTitle("right");
    }
    if (show[current_stream] == &disparity && image_set->disparity_img_)
    {
      cv::Mat disp = image_set->disparity_img_->data_;
      cv::threshold(disp, disp, 1000, 0, CV_THRESH_TOZERO_INV);
      cv::normalize(disp, disp, 0.0, 1.0, cv::NORM_MINMAX, CV_32FC1);
      cv::imshow(cv_win_name, disp);
      setWindowTitle("disparity");
    }
    if (show[current_stream] == &confidence && image_set->confidence_img_)
    {
      cv::Mat conf = image_set->confidence_img_->data_;
      cv::normalize(conf, conf, 0.0, 1.0, cv::NORM_MINMAX, CV_32FC1);
      cv::imshow(cv_win_name, conf);
      setWindowTitle("confidence");
    }
    if (show[current_stream] == &error && image_set->error_img_)
    {
      cv::Mat err = image_set->error_img_->data_;
      cv::normalize(err, err, 0.0, 1.0, cv::NORM_MINMAX, CV_32FC1);
      cv::imshow(cv_win_name, err);
      setWindowTitle("error");
    }

    // read key presses
    const int key = cv::waitKey(1);
    if (key > 0)
    {
      const char ascii_key = static_cast<char>(key);
      if (ascii_key == 'n')
      {
        // go to next stream
        current_stream = (current_stream + 1) % static_cast<int>(show.size());
      }
      if (ascii_key == 'p')
      {
        // go to previous stream
        current_stream = (current_stream - 1) % static_cast<int>(show.size());
      }
      if (ascii_key == 'q')
      {
        // quit
        break;
      }
    }
  }
}
