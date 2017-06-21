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

#ifndef PRIUSCUP_PLUGINS_MAX_TIME_TO_LIVE_HH_
#define PRIUSCUP_PLUGINS_MAX_TIME_TO_LIVE_HH_

#include "gazebo/gazebo.hh"


namespace gazebo
{
// PIMPL pattern
class MaxTimeToLiveImpl;

// \brief Plugin stops the gzserver process after a set timeperiod
class MaxTimeToLive : public SystemPlugin
{
  public:
    // \brief called when the controller is first loaded
    void Load(int _argc, char ** _argv);

  protected:
    std::unique_ptr<MaxTimeToLiveImpl> impl_;
};
}

#endif  // PRIUSCUP_PLUGINS_MAX_TIME_TO_LIVE_HH_
