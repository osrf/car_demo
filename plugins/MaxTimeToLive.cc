/*
 * Copyright 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <cstdlib>
#include <cstdint>
#include <string>
#include <regex>
#include <chrono>
#include <thread>
#include "MaxTimeToLive.hh"

#include <ignition/msgs/int64.pb.h>
#include <ignition/transport.hh>
#include <ignition/common/Console.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{

GZ_REGISTER_SYSTEM_PLUGIN(MaxTimeToLive)

class MaxTimeToLiveImpl
{
  public:
    enum EXIT_CODE {
      BAD_ARGUMENTS = 26,
      LIFESPAN_REACHED = 27,
    };

    // \brief given command line arguments, returns an implementation pointer
    static MaxTimeToLiveImpl* ParseArgs(int _argc, char **_argv);

    // \brief destructor
    ~MaxTimeToLiveImpl();

  protected:
    // \brief How long to wait in seconds before exiting gzserver
    unsigned int seconds_to_live_;

    // \brief thread in which countdown is happening
    std::thread countdown_thread_;

    // \brief constructor
    // \param seconds_to_live seconds to wait before calling exit()
    MaxTimeToLiveImpl(unsigned int _seconds_to_live);

    // checks lifespan at a regular interval
    void CountdownLifespan();
};


void MaxTimeToLive::Load(int _argc, char **_argv)
{
  // ignition::common::Console::SetQuiet(false);
  igndbg << "Loading MaxTimeToLiveImpl" << std::endl;
  this->impl_ = std::unique_ptr<MaxTimeToLiveImpl>(
      MaxTimeToLiveImpl::ParseArgs(_argc, _argv));
}


MaxTimeToLiveImpl* MaxTimeToLiveImpl::ParseArgs( int _argc, char **_argv)
{
  const auto lifespan_regex = std::regex("--lifespan=([0-9]+)",
      std::regex_constants::ECMAScript);

  for (int i = 0; i < _argc; i++)
  {
    std::cmatch results;
    if (std::regex_match(_argv[i], results, lifespan_regex))
    {
      //regex match guarantees atoi conversion can be performed
      std::string lifespan_str = results[1];
      unsigned int lifespan = std::atoi(lifespan_str.c_str());
      igndbg << "Parsed argument --lifespan=" << lifespan << std::endl;
      return new MaxTimeToLiveImpl(lifespan);
    }
    else
    {
      igndbg << "Ignoring argument '" << _argv[i] << "'" << std::endl;
    }
  }

  ignerr << "Did not find --lifespan (example --lifespan=42)" << std::endl;
  std::exit(BAD_ARGUMENTS);
}


MaxTimeToLiveImpl::MaxTimeToLiveImpl(unsigned int _seconds_to_live) :
  seconds_to_live_(_seconds_to_live)
{
  //Start thread to do publishing
  countdown_thread_ = std::thread(&MaxTimeToLiveImpl::CountdownLifespan, this);
  // Must detach or join thread before std::thread gets destructed
  countdown_thread_.detach();
}


MaxTimeToLiveImpl::~MaxTimeToLiveImpl()
{
}


void MaxTimeToLiveImpl::CountdownLifespan()
{
  auto time_to_live = std::chrono::seconds(this->seconds_to_live_);
  const auto start_time = std::chrono::steady_clock::now();
  const auto death_time = start_time + time_to_live;
  ignition::transport::Node node;
  auto doomsday_pub = node.Advertise<ignition::msgs::Int64>("time_remaining");

  while (std::chrono::steady_clock::now() < death_time)
  {
    ignition::msgs::Int64 msg;
    int64_t remainder = std::chrono::duration_cast<std::chrono::seconds>(
        death_time - std::chrono::steady_clock::now()).count();
    msg.set_data(remainder);
    doomsday_pub.Publish(msg);
    // 1Hz
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  //last message out, no time remaining
  ignition::msgs::Int64 msg;
  msg.set_data(0);
  doomsday_pub.Publish(msg);

  ignmsg << "Lifespan reached, exiting" << std::endl;
  gazebo::shutdown();

  // Note: actual exit code unlikely to be LIFESPAN_REACHED
  // It depends on what other threads do while terminating
  std::exit(LIFESPAN_REACHED);
}

}  // namespace gazebo
