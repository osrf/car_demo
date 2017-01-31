/*
 * Copyright 2017 Open Source Robotics Foundation
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

#include <fstream>
#include "PriusLogger.hh"

namespace priuscup
{
  // \brief Contruct the logger and starts writing to the file
  PriusLogger::PriusLogger(std::string _filename, double _rate)
  {
    this->filename_ = _filename;
    this->period_ = 1.0 / _rate;
  }

  bool PriusLogger::Start()
  {
    this->numDataWritten_ = 0;
    this->terminate_ = false;
    this->isGood_ = true;
    this->thread_.reset(new std::thread(std::bind(
            &PriusLogger::DoLogging, this)));
  }

  void PriusLogger::Stop()
  {
    // Signal running thread that it's time to close, and block until it does
    if (this->IsLogging())
    {
      this->terminate_ = true;
      this->thread_->join();
      this->isGood_ = false;
    }
  }

  PriusLogger::~PriusLogger()
  {
    this->Stop();
  }

  bool PriusLogger::IsHungry(const gazebo::common::Time &_curTime) const
  {
    bool hungry = false;
    if (!IsLogging())
      hungry = false;  // No thread to consume data
    else if ((_curTime - this->lastMealTime_).Double() >= this->period_)
      hungry = true;  // logger hasn't been fed in a while
    else if (this->lastMealTime_ > _curTime)
      hungry = true;  // logger was restarted
    return hungry;
  }

  void PriusLogger::Feed(const gazebo::common::Time &_curTime,
      const CarData &_data)
  {
    std::lock_guard<std::mutex> scope_lock(this->mutex_);
    this->lastMealTime_ = _curTime;
    this->data_.push_back(_data);
  }

  void PriusLogger::DoLogging()
  {
    std::ofstream stream;
    stream.open(this->filename_.c_str(), std::ofstream::trunc);
    stream << "[";
    while (!this->terminate_)
    {
      // Sleep in sim time, not wall clock
      gazebo::common::Time::MSleep(200);
      std::lock_guard<std::mutex> scope_lock(this->mutex_);

      while (!this->data_.empty())
      {
        auto data = this->data_.front();
        this->data_.pop_front();

        // data needs to be worked a bit to match the log format
        //
        // convert seconds (float) to milliseconds (int)
        int timestamp = data.timestamp * 1000.0;
        // accel pedal is an integer (0-100)
        int accelPedal = data.accelPedal * 100.0;
        // steering appears to be steering angle (radians) * 21
        double steerWheel = data.steerAngle * 21.0;
        // speed as an integer in k/h
        int speed_kph = 60.0 * 60.0 * data.mps / 1000.0;
        // convert gallons to mL
        double gasConsumed = 3785.41 * data.gasConsumed;
        // odom appears to be meters moved * 0.85
        // 1609.34 meters to a mile
        int odom = 1609.34 * data.odom * 0.85;
        // \brief Battery SoC one of [0, 12.5, 25, 37.5, 50, 62.5, 75, 87.5, 100]
        std::string soc = "0.0";
        if (data.soc > 0.125 && data.soc < 0.25)
          soc = "12.5";
        else if (data.soc < 0.375)
          soc = "25.0";
        else if (data.soc < 0.5)
          soc = "37.5";
        else if (data.soc < 0.625)
          soc = "50.0";
        else if (data.soc < 0.75)
          soc = "62.5";
        else if (data.soc < 0.875)
          soc = "75.0";
        else if (data.soc < 0.95)
          soc = "87.5";
        else
          soc = "100.0";

        //Example output (no newline)
        // [1481657121650, 0, false, "D", 0.0, 1010, 56, -5.7, 5.0, 0.0,
        // 185.823, 3790.0, 38.164, -122.46351, -2613.6, false, 50.0]
        stream << std::fixed
            << (this->numDataWritten_ ? ", [" : "[")
            << timestamp << ", "
            << accelPedal << ", "
            << (data.brake ? "true" : "false") << ", "
            << '"' << data.gear << '"' << ", "
            << steerWheel << ", "
            << data.rpm << ", "
            << speed_kph << ", "
            << 0.0 /* accel_long */ << ", "
            << 0.0 /* accel_lat */ << ", "
            << data.yawRate << ", "
            << data.gasConsumed << ", "
            << odom << ".0" /* int with .0 appended */ << ", "
            << 0.0 /* lat_deg */ << ", "
            << 0.0 /* lon_deg */ << ", "
            << 0.0 /* altitude */ << ", "
            << (data.evMode ? "true" : "false") << ", "
            << soc << "]\n";
        this->numDataWritten_++;
      }
      stream.flush();
    }
    stream << "]";
    stream.flush();
    stream.close();
  }
}  // namespace priuscup
