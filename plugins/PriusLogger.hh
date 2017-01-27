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

#ifndef PRIUSCUP_PLUGINS_PRIUSLOGGER_HH_
#define PRIUSCUP_PLUGINS_PRIUSLOGGER_HH_

#include <string>
#include <mutex>
#include <thread>
#include <list>
#include <gazebo/common/Time.hh>


namespace priuscup
{
  struct CarData
  {
    // \brief seconds since epoch
    double timestamp = 0.0;
    // \brief How much is the accel pedal pressed [0.0,1.0]
    double accelPedal = 0.0;
    // \brief Angle of steering in radians [-0.6,+0.6]
    double steerAngle = 0.0;
    // \brief distance travelled in miles
    double odom = 0.0;
    // \brief speed in m/s
    double mps = 0.0;
    // \brief gas consumed so far in gallons
    double gasConsumed = 0.0;
    // \brief gear car is in as a char 'D', 'R', or 'N'
    char gear = 'D';
    // \brief true if the brake is applied
    bool brake = false;
    // \brief true if in ev mode
    bool evMode = false;
    // \brief engine rpm
    int rpm = 0.0;
    // \brief angular z velocity (rad/s)
    double yawRate = 0.0;
    // \brief Battery SoC [0.0, 1.0]
    double soc = 0.0;
  };


  // \brief Class handles formatting data and writing it to a log file
  class PriusLogger
  {
    public:
      // \brief Contruct the logger and starts writing to the file
      PriusLogger(std::string filename, double rate);

      // \brief stop logger thread, finish writing data_, close log
      ~PriusLogger();

      // \brief Starts the logger
      bool Start();

      // \brief Stops the logger
      void Stop();

      // \brief return true iff logger expects data
      bool IsHungry(const gazebo::common::Time &_curTime) const;

      // \brief adds a datapoint to be logged
      void Feed(const gazebo::common::Time &_curTime, const CarData &_data);

      // \brief returns True iff the logger is able to log
      bool IsLogging() const {return isGood_;}

    protected:
      // This is a single use class, not using PIMPL for simplicity

      // \brief True iff PriusLogger is being destroyed and thread must die
      bool terminate_ = false;
      // \brief True iff logger is able to log
      bool isGood_ = false;
      // \brief counts the number of entries that have been written to file
      unsigned int numDataWritten_ = 0;
      // \brief anticipated time between calls to Feed
      double period_;
      // \brief name of the file to log to
      std::string filename_;
      // \brief list of data points that have not yet been written to log file
      std::list<CarData> data_;
      // \brief mutex to guard data_
      std::mutex mutex_;
      // \brief thread in which log file gets written to
      std::unique_ptr<std::thread> thread_;
      // \brief last time the logger was fed
      gazebo::common::Time lastMealTime_;

      // \brief Thread that does the logging
      void DoLogging();
  };
}  // namespace priuscup
#endif  // PRIUSCUP_PLUGINS_PRIUSLOGGER_HH_
