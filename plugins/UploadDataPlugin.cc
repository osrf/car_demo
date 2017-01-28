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

#include <string>
#include "UploadDataPlugin.hh"

#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/transport.hh>
#include <ignition/common/Console.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class UploadDataPluginPrivate
  {
    /// \brief Ign transport node
    public: ignition::transport::Node node;

    /// \brief Flag to indicate if file has been uploaded
    public: bool uploaded = false;

    /// \brief Service callback to upload data
    /// \param[in] _req Request message
    /// \param[in] _rep Response message
    /// \param[in] _result Result of the service call
    public: void Upload(const ignition::msgs::StringMsg &_req,
        ignition::msgs::StringMsg &_rep, bool &_result);
  };
}

using namespace gazebo;

GZ_REGISTER_SYSTEM_PLUGIN(UploadDataPlugin)

/////////////////////////////////////////////////
std::string custom_exec(std::string _cmd)
{
  _cmd += " 2>/dev/null";

#ifdef _WIN32
  FILE *pipe = _popen(_cmd.c_str(), "r");
#else
  FILE *pipe = popen(_cmd.c_str(), "r");
#endif

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

#ifdef _WIN32
  _pclose(pipe);
#else
  pclose(pipe);
#endif

  return result;
}

/////////////////////////////////////////////////
UploadDataPlugin::UploadDataPlugin()
	: dataPtr(new UploadDataPluginPrivate)
{
}

/////////////////////////////////////////////////
UploadDataPlugin::~UploadDataPlugin()
{
  if (this->dataPtr->uploaded)
    return;

  auto pub = this->dataPtr->node.Advertise<ignition::msgs::Any>("/prius/stop");
  ignition::msgs::Any msg;
  pub.Publish(msg);
}

/////////////////////////////////////////////////
void UploadDataPlugin::Load(int _argc, char **_argv)
{
  // Advertise a service call.
  std::string service = "/priuscup/upload";
  if (!this->dataPtr->node.Advertise(service,
      &UploadDataPluginPrivate::Upload, this->dataPtr.get()))
  {
    std::cerr << "Error advertising service [" << service << "]" << std::endl;
    return;
  }
}

/////////////////////////////////////////////////
void UploadDataPluginPrivate::Upload(const ignition::msgs::StringMsg &_req,
      ignition::msgs::StringMsg &_rep, bool &_result)
{
  // only allow one upload for now.
  if (this->uploaded)
    return;

  std::string filename = _req.data();
  if (filename.empty())
  {
     std::string errorMsg = "Missing filename";
    _rep.set_data(errorMsg);
    _result = false;
    return;
  }
  const char *id = common::getEnv("AWS_ACCESS_KEY_ID");
  const char *secret = common::getEnv("AWS_SECRET_ACCESS_KEY");
  if (!id || !secret)
  {
    std::string errorMsg = "Missing AWS credentials";
    _rep.set_data(errorMsg);
    _result = false;
    return;
  }
  std::string targetname = "prius_data.txt";
  const char *target = common::getEnv("PRIUS_USER_ID");
  if (target)
  {
    targetname = std::string(target) + "_" + targetname;
  }
  std::string scriptPath = common::find_file("upload.py");
  if (scriptPath.empty())
  {
     std::string errorMsg = "Unable to find upload script";
    _rep.set_data(errorMsg);
    _result = false;
    return;
  }
  std::string uploadCmdStr = "python " + scriptPath + " "
      + filename + " " + targetname + " " +  id + " " + secret;
  std::string out = custom_exec(uploadCmdStr);
  _rep.set_data(out);
  _result = true;
  this->uploaded = true;
  return;
}

