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

#include <fstream>
#include <mutex>
#include <thread>

#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "PriusHybridPlugin.hh"
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>

namespace gazebo
{
  class PriusData
  {
    public: double timestamp = 0.0;
    public: ignition::math::Pose3d pose;
    public: double fuelEfficiency = 0.0;
  };

  class PriusHybridPluginPrivate
  {
    /// \enum DirectionType
    /// \brief Direction selector switch type.
    public: enum DirectionType {
              /// \brief Reverse
              REVERSE = -1,
              /// \brief Neutral
              NEUTRAL = 0,
              /// \brief Forward
              FORWARD = 1
            };

    /// \brief Pointer to the world
    public: physics::WorldPtr world;

    /// \brief Pointer to the parent model
    public: physics::ModelPtr model;

    /// \brief Transport node
    public: transport::NodePtr gznode;

    /// \brief Ignition transport node
    public: ignition::transport::Node node;

    /// \brief Physics update event connection
    public: event::ConnectionPtr updateConnection;

    /// brief Front left wheel joint
    public: physics::JointPtr flWheelJoint;

    /// brief Front right wheel joint
    public: physics::JointPtr frWheelJoint;

    /// brief Rear left wheel joint
    public: physics::JointPtr blWheelJoint;

    /// brief Rear right wheel joint
    public: physics::JointPtr brWheelJoint;

    /// \brief Front left wheel steering joint
    public: physics::JointPtr flWheelSteeringJoint;

    /// \brief Front right wheel steering joint
    public: physics::JointPtr frWheelSteeringJoint;

    /// \brief Steering wheel joint
    public: physics::JointPtr handWheelJoint;

    /// \brief PID control for the front left wheel steering joint
    public: common::PID flWheelSteeringPID;

    /// \brief PID control for the front right wheel steering joint
    public: common::PID frWheelSteeringPID;

    /// \brief PID control for steering wheel joint
    public: common::PID handWheelPID;

    /// \brief Last sim time received
    public: common::Time lastSimTime;

    /// \brief Last sim time when a gas command is received
    public: common::Time lastGasCmdTime;

    /// \brief Last sim time when a steering command is received
    public: common::Time lastSteeringCmdTime;

    /// \brief Current direction of the vehicle: FORWARD, NEUTRAL, REVERSE.
    public: DirectionType directionState;

    /// \brief Minimum brake percentage
    public: double minBrakePercent = 0;

    /// \brief Max torque that can be applied to the front wheels
    public: double frontTorque = 0;

    /// \brief Max torque that can be applied to the back wheels
    public: double backTorque = 0;

    /// \brief Max speed (m/s) of the car
    public: double maxSpeed = 0;

    /// \brief Max steering angle
    public: double maxSteer = 0;

    /// \brief Max torque that can be applied to the front brakes
    public: double frontBrakeTorque = 0;

    /// \brief Max torque that can be applied to the rear brakes
    public: double backBrakeTorque = 0;

    /// \brief Angle ratio between the steering wheel and the front wheels
    public: double steeringRatio = 0;

    /// \brief Max range of hand steering wheel
    public: double handWheelHigh = 0;

    /// \brief Min range of hand steering wheel
    public: double handWheelLow = 0;

    /// \brief Max force that can be applied to hand steering wheel
    public: double handWheelForce = 0;

    /// \brief Max force that can be applied to wheel steering joints
    public: double steeredWheelForce= 0;

    /// \brief Front left wheel steering joint P gain
    public: double fLwheelSteeringPgain = 0;

    /// \brief Front right wheel steering joint P gain
    public: double fRwheelSteeringPgain = 0;

    /// \brief Front left wheel steering joint I gain
    public: double fLwheelSteeringIgain = 0;

    /// \brief Front right wheel steering joint I gain
    public: double fRwheelSteeringIgain = 0;

    /// \brief Front left wheel steering joint D gain
    public: double fLwheelSteeringDgain = 0;

    /// \brief Front right wheel steering joint D gain
    public: double fRwheelSteeringDgain = 0;

    /// \brief Front left wheel steering command
    public: double flWheelSteeringCmd = 0;

    /// \brief Front right wheel steering command
    public: double frWheelSteeringCmd = 0;

    /// \brief Steering wheel command
    public: double handWheelCmd = 0;

    /// \brief Front left wheel radius
    public: double flWheelRadius = 0;

    /// \brief Front right wheel radius
    public: double frWheelRadius = 0;

    /// \brief Rear left wheel radius
    public: double blWheelRadius = 0;

    /// \brief Rear right wheel radius
    public: double brWheelRadius = 0;

    /// \brief Distance distance between front and rear axles
    public: double wheelbaseLength = 0;

    /// \brief Distance distance between front left and right wheels
    public: double frontTrackWidth = 0;

    /// \brief Distance distance between rear left and right wheels
    public: double backTrackWidth = 0;

    /// \brief Gas pedal position in percentage. 1.0 = Fully accelerated.
    public: double gasPedalPercent = 0;

    /// \brief Brake pedal position in percentage. 1.0 =
    public: double brakePedalPercent = 0;

    /// \brief Angle state of hand steering wheel joint
    public: double handWheelState = 0;

    /// \brief Angle state of front left wheel steering joint
    public: double flSteeringState = 0;

    /// \brief Angle state of front right wheel steering joint
    public: double frSteeringState = 0;

    /// \brief Angle state of front left wheel joint
    public: double flWheelState = 0;

    /// \brief Angle state of front right wheel joint
    public: double frWheelState = 0;

    /// \brief Angle state of rear left wheel joint
    public: double blWheelState = 0;

    /// \brief Angle state of rear right wheel joint
    public: double brWheelState = 0;

    /// \brief Subscriber to the keyboard topic
    public: transport::SubscriberPtr keyboardSub;

    /// \brief Mutex to protect updates
    public: std::mutex mutex;

    /// \brief Mutex to protect logger writes
    public: std::mutex loggerMutex;

    /// \brief Thread to log data
    public: std::unique_ptr<std::thread> loggerThread;

    /// \brief Time last data were pushed to logger
    public: common::Time lastLoggerWriteTime;

    /// \brief List of data to write to file
    public: std::list<PriusData> dataPoints;

    /// \brief Total distance traveled.
    public: double totalDistance = 0;

    /// \brief Flag used to determine when to quit the logger thread
    public: bool quit = false;

    /// \brief Logger stream that writes to file
    public: std::ofstream loggerStream;

    /// \brief Rate (hz) at which data are logged.
    public: double logRate = 1;

    /// \brief Last model world pose written to log
    public: ignition::math::Pose3d lastModelWorldPose;
  };
}

using namespace gazebo;

/////////////////////////////////////////////////
PriusHybridPlugin::PriusHybridPlugin()
    : dataPtr(new PriusHybridPluginPrivate)
{
  this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
  this->dataPtr->steeredWheelForce = 5000;
  this->dataPtr->handWheelForce = 1;
  this->dataPtr->flWheelRadius = 0.3;
  this->dataPtr->frWheelRadius = 0.3;
  this->dataPtr->blWheelRadius = 0.3;
  this->dataPtr->brWheelRadius = 0.3;

  // hz
  this->dataPtr->logRate = 20;
}

/////////////////////////////////////////////////
PriusHybridPlugin::~PriusHybridPlugin()
{
  this->dataPtr->updateConnection.reset();
  this->dataPtr->quit = true;
  this->dataPtr->loggerThread->join();
  this->dataPtr->loggerThread.reset();
}

/////////////////////////////////////////////////
void PriusHybridPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;
  this->dataPtr->world = this->dataPtr->model->GetWorld();

  this->dataPtr->gznode = transport::NodePtr(new transport::Node());
  this->dataPtr->gznode->Init();

  this->dataPtr->node.Subscribe("/cmd_vel", &PriusHybridPlugin::OnCmdVel, this);

  std::string handWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("steering_wheel");
  this->dataPtr->handWheelJoint =
    this->dataPtr->model->GetJoint(handWheelJointName);
  if (!this->dataPtr->handWheelJoint)
  {
    std::cerr << "could not find steering wheel joint" <<std::endl;
    return;
  }

  std::string flWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_left_wheel");
  this->dataPtr->flWheelJoint =
    this->dataPtr->model->GetJoint(flWheelJointName);
  if (!this->dataPtr->flWheelJoint)
  {
    std::cerr << "could not find front left wheel joint" <<std::endl;
    return;
  }

  std::string frWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_right_wheel");
  this->dataPtr->frWheelJoint =
    this->dataPtr->model->GetJoint(frWheelJointName);
  if (!this->dataPtr->frWheelJoint)
  {
    std::cerr << "could not find front right wheel joint" <<std::endl;
    return;
  }

  std::string blWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("back_left_wheel");
  this->dataPtr->blWheelJoint =
    this->dataPtr->model->GetJoint(blWheelJointName);
  if (!this->dataPtr->blWheelJoint)
  {
    std::cerr << "could not find back left wheel joint" <<std::endl;
    return;
  }

  std::string brWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("back_right_wheel");
  this->dataPtr->brWheelJoint =
    this->dataPtr->model->GetJoint(brWheelJointName);
  if (!this->dataPtr->brWheelJoint)
  {
    std::cerr << "could not find back right wheel joint" <<std::endl;
    return;
  }

  std::string flWheelSteeringJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_left_wheel_steering");
  this->dataPtr->flWheelSteeringJoint =
    this->dataPtr->model->GetJoint(flWheelSteeringJointName);
  if (!this->dataPtr->flWheelSteeringJoint)
  {
    std::cerr << "could not find front left steering joint" <<std::endl;
    return;
  }

  std::string frWheelSteeringJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_right_wheel_steering");
  this->dataPtr->frWheelSteeringJoint =
    this->dataPtr->model->GetJoint(frWheelSteeringJointName);
  if (!this->dataPtr->frWheelSteeringJoint)
  {
    std::cerr << "could not find front right steering joint" <<std::endl;
    return;
  }

  std::string paramName;
  double paramDefault;

  paramName = "front_torque";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frontTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->frontTorque = paramDefault;

  paramName = "back_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->backTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->backTorque = paramDefault;

  paramName = "front_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frontBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->frontBrakeTorque = paramDefault;

  paramName = "back_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->backBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->backBrakeTorque = paramDefault;

  paramName = "max_speed";
  paramDefault = 10;
  if (_sdf->HasElement(paramName))
    this->dataPtr->maxSpeed = _sdf->Get<double>(paramName);
  else
    this->dataPtr->maxSpeed = paramDefault;

  paramName = "max_steer";
  paramDefault = 0.6;
  if (_sdf->HasElement(paramName))
    this->dataPtr->maxSteer = _sdf->Get<double>(paramName);
  else
    this->dataPtr->maxSteer = paramDefault;

  paramName = "min_brake_percent";
  paramDefault = 0.02;
  if (_sdf->HasElement(paramName))
    this->dataPtr->minBrakePercent = _sdf->Get<double>(paramName);
  else
    this->dataPtr->minBrakePercent = paramDefault;

  paramName = "flwheel_steering_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->fLwheelSteeringPgain = _sdf->Get<double>(paramName);
  else
    this->dataPtr->fLwheelSteeringPgain = paramDefault;

  paramName = "frwheel_steering_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->fRwheelSteeringPgain = _sdf->Get<double>(paramName);
  else
    this->dataPtr->fRwheelSteeringPgain = paramDefault;

  paramName = "flwheel_steering_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->fLwheelSteeringIgain = _sdf->Get<double>(paramName);
  else
    this->dataPtr->fLwheelSteeringIgain = paramDefault;

  paramName = "frwheel_steering_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->fRwheelSteeringIgain = _sdf->Get<double>(paramName);
  else
    this->dataPtr->fRwheelSteeringIgain = paramDefault;

  paramName = "flwheel_steering_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->fLwheelSteeringDgain = _sdf->Get<double>(paramName);
  else
    this->dataPtr->fLwheelSteeringDgain = paramDefault;

  paramName = "frwheel_steering_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->fRwheelSteeringDgain = _sdf->Get<double>(paramName);
  else
    this->dataPtr->fRwheelSteeringDgain = paramDefault;

  this->UpdateHandWheelRatio();

  // Update wheel radius for each wheel from SDF collision objects
  //  assumes that wheel link is child of joint (and not parent of joint)
  //  assumes that wheel link has only one collision
  unsigned int id = 0;
  this->dataPtr->flWheelRadius = this->CollisionRadius(
      this->dataPtr->flWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->frWheelRadius = this->CollisionRadius(
      this->dataPtr->frWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->blWheelRadius = this->CollisionRadius(
      this->dataPtr->blWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->brWheelRadius = this->CollisionRadius(
      this->dataPtr->brWheelJoint->GetChild()->GetCollision(id));

  /* gzerr << "wheel radius: "
     << this->dataPtr->flWheelRadius << " "
     << this->dataPtr->frWheelRadius << " "
     << this->dataPtr->blWheelRadius << " "
     << this->dataPtr->brWheelRadius << std::endl;*/

  // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  //  first compute the positions of the 4 wheel centers
  //  again assumes wheel link is child of joint and has only one collision
  ignition::math::Vector3d flCenterPos =
    this->dataPtr->flWheelJoint->GetChild()->GetCollision(id)
    ->GetWorldPose().pos.Ign();
  ignition::math::Vector3d frCenterPos =
    this->dataPtr->frWheelJoint->GetChild()->GetCollision(id)
    ->GetWorldPose().pos.Ign();
  ignition::math::Vector3d blCenterPos =
    this->dataPtr->blWheelJoint->GetChild()->GetCollision(id)
    ->GetWorldPose().pos.Ign();
  ignition::math::Vector3d brCenterPos =
    this->dataPtr->brWheelJoint->GetChild()->GetCollision(id)
    ->GetWorldPose().pos.Ign();

  // track widths are computed first
  ignition::math::Vector3d vec3 = flCenterPos - frCenterPos;
  this->dataPtr->frontTrackWidth = vec3.Length();
  vec3 = flCenterPos - frCenterPos;
  this->dataPtr->backTrackWidth = vec3.Length();
  // to compute wheelbase, first position of axle centers are computed
  ignition::math::Vector3d frontAxlePos = (flCenterPos + frCenterPos) / 2;
  ignition::math::Vector3d backAxlePos = (blCenterPos + brCenterPos) / 2;
  // then the wheelbase is the distance between the axle centers
  vec3 = frontAxlePos - backAxlePos;
  this->dataPtr->wheelbaseLength = vec3.Length();

  /* gzerr << "wheel base length and track width: "
     << this->dataPtr->wheelbaseLength << " "
     << this->dataPtr->frontTrackWidth
     << " " << this->dataPtr->backTrackWidth << std::endl; */



  this->dataPtr->handWheelPID.Init(100, 0, 0, 0, 0,
      this->dataPtr->handWheelForce, -this->dataPtr->handWheelForce);

  this->dataPtr->flWheelSteeringPID.Init(this->dataPtr->fLwheelSteeringPgain,
      this->dataPtr->fLwheelSteeringIgain,
      this->dataPtr->fLwheelSteeringDgain,
      0, 0, this->dataPtr->steeredWheelForce,
      -this->dataPtr->steeredWheelForce);
  this->dataPtr->frWheelSteeringPID.Init(this->dataPtr->fRwheelSteeringPgain,
      this->dataPtr->fRwheelSteeringIgain,
      this->dataPtr->fRwheelSteeringDgain,
      0, 0, this->dataPtr->steeredWheelForce,
      -this->dataPtr->steeredWheelForce);


  this->dataPtr->loggerThread.reset(new std::thread(
      std::bind(&PriusHybridPlugin::RunLogger, this)));

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PriusHybridPlugin::Update, this));

  this->dataPtr->keyboardSub =
    this->dataPtr->gznode->Subscribe("~/keyboard/keypress",
        &PriusHybridPlugin::OnKeyPress, this, true);


  this->dataPtr->node.Subscribe("/keypress", &PriusHybridPlugin::OnKeyPressIgn,
      this);
}

/////////////////////////////////////////////////
void PriusHybridPlugin::RunLogger()
{
  this->dataPtr->loggerStream.open("prius_data.txt");

  while (!this->dataPtr->quit)
  {
    common::Time::MSleep(200);
    std::lock_guard<std::mutex> loggerLock(this->dataPtr->loggerMutex);

    // write to file
    while (!this->dataPtr->dataPoints.empty())
    {
      auto data = this->dataPtr->dataPoints.front();
      this->dataPtr->dataPoints.pop_front();
      double distance = (data.pose.Pos() -
          this->dataPtr->lastModelWorldPose.Pos()).Length();
      this->dataPtr->totalDistance += distance;
      this->dataPtr->loggerStream
          << data.timestamp << ", " << this->dataPtr->totalDistance << ", "
          << data.fuelEfficiency << "\n";

      this->dataPtr->lastModelWorldPose = data.pose;
    }
    this->dataPtr->loggerStream.flush();
  }

  this->dataPtr->loggerStream.close();
}

/////////////////////////////////////////////////
/*void PriusHybridPlugin::OnCmdVel(const ignition::msgs::CmdVel2D &_msg)
{
  this->dataPtr->gasPedalPercent = std::min(_msg.velocity(), 1.0);
  this->dataPtr->handWheelCmd = this->dataPtr->handWheelState + _msg.theta();

  this->dataPtr->lastGasCmdTime = this->dataPtr->world->SimTime();
  this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
}*/

/////////////////////////////////////////////////
void PriusHybridPlugin::OnCmdVel(const ignition::msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->gasPedalPercent = std::min(_msg.position().x(), 1.0);
  this->dataPtr->handWheelCmd = _msg.position().y();
  this->dataPtr->brakePedalPercent = _msg.position().z();

  this->dataPtr->lastGasCmdTime = this->dataPtr->world->SimTime();
  this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->brakePedalPercent = 0;
  switch (_msg->int_value())
  {
    // w - accelerate forward
    case 87:
    case 119:
    {
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
      this->dataPtr->lastGasCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // a - steer left
    case 65:
    case 97:
    {
      this->dataPtr->handWheelCmd = this->dataPtr->handWheelState + 0.1;
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // s - reverse
    case 83:
    case 115:
    {
      if (this->dataPtr->directionState != PriusHybridPluginPrivate::REVERSE)
        this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->directionState = PriusHybridPluginPrivate::REVERSE;
      this->dataPtr->lastGasCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // d - steer right
    case 68:
    case 100:
    {
      this->dataPtr->handWheelCmd = this->dataPtr->handWheelState - 0.1;
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // e brake
    case 69:
    case 101:
    {
      this->dataPtr->brakePedalPercent = 1.0;
      this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->lastGasCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    default:
    {
      break;
    }
  }
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnKeyPressIgn(const ignition::msgs::Any &_msg)
{
}

/////////////////////////////////////////////////
void PriusHybridPlugin::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  common::Time curTime = this->dataPtr->world->SimTime();
  double dt = (curTime - this->dataPtr->lastSimTime).Double();
  if (dt < 0)
  {
    // has time been reset?
    this->dataPtr->lastSimTime = curTime;

    std::lock_guard<std::mutex> loggerLock(this->dataPtr->loggerMutex);
    this->dataPtr->totalDistance = 0;
    this->dataPtr->lastModelWorldPose = ignition::math::Pose3d::Zero;
    return;
  }
  else if (ignition::math::equal(dt, 0.0))
  {
    return;
  }

  this->dataPtr->handWheelState =
      this->dataPtr->handWheelJoint->GetAngle(0).Radian();
  this->dataPtr->flSteeringState =
      this->dataPtr->flWheelSteeringJoint->GetAngle(0).Radian();
  this->dataPtr->frSteeringState =
      this->dataPtr->frWheelSteeringJoint->GetAngle(0).Radian();

  this->dataPtr->flWheelState = this->dataPtr->flWheelJoint->GetVelocity(0);
  this->dataPtr->frWheelState = this->dataPtr->frWheelJoint->GetVelocity(0);
  this->dataPtr->blWheelState = this->dataPtr->blWheelJoint->GetVelocity(0);
  this->dataPtr->brWheelState = this->dataPtr->brWheelJoint->GetVelocity(0);

  this->dataPtr->lastSimTime = curTime;

  // PID (position) steering
  double steerError =
      this->dataPtr->handWheelState - this->dataPtr->handWheelCmd;
  double steerCmd = this->dataPtr->handWheelPID.Update(steerError, dt);
  this->dataPtr->handWheelJoint->SetForce(0, steerCmd);

  // PID (position) steering joints based on steering position
  // Ackermann steering geometry here
  //  \TODO provide documentation for these equations
  double tanSteer =
      tan(this->dataPtr->handWheelState * this->dataPtr->steeringRatio);
  this->dataPtr->flWheelSteeringCmd = atan2(tanSteer,
      1 - this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength *
      tanSteer);
  this->dataPtr->frWheelSteeringCmd = atan2(tanSteer,
      1 + this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength *
      tanSteer);
  // this->flWheelSteeringCmd = this->handWheelState * this->steeringRatio;
  // this->frWheelSteeringCmd = this->handWheelState * this->steeringRatio;

  double flwsError =
      this->dataPtr->flSteeringState - this->dataPtr->flWheelSteeringCmd;
  double flwsCmd = this->dataPtr->flWheelSteeringPID.Update(flwsError, dt);
  this->dataPtr->flWheelSteeringJoint->SetForce(0, flwsCmd);

  double frwsError =
      this->dataPtr->frSteeringState - this->dataPtr->frWheelSteeringCmd;
  double frwsCmd = this->dataPtr->frWheelSteeringPID.Update(frwsError, dt);
  this->dataPtr->frWheelSteeringJoint->SetForce(0, frwsCmd);

  // Gas pedal torque.
  // Map gas torques to individual wheels.
  // Cut off gas torque at a given wheel if max speed is exceeded.
  // Use directionState to determine direction of that can be applied torque.
  // Note that definition of DirectionType allows multiplication to determine
  // torque direction.
  // double gasPercent = this->GasPedalPercent();
  double gasPercent = this->dataPtr->gasPedalPercent;
  double gasMultiplier = this->GasTorqueMultiplier();
  double flGasTorque = 0, frGasTorque = 0, blGasTorque = 0, brGasTorque = 0;
  // Apply equal torque at left and right wheels, which is an implicit model
  // of the differential.
  if ((fabs(this->dataPtr->flWheelState * this->dataPtr->flWheelRadius) <
      this->dataPtr->maxSpeed)
      && (fabs(this->dataPtr->frWheelState * this->dataPtr->frWheelRadius) <
      this->dataPtr->maxSpeed))
  {
    flGasTorque = gasPercent*this->dataPtr->frontTorque * gasMultiplier;
    frGasTorque = gasPercent*this->dataPtr->frontTorque * gasMultiplier;
  }
  if ((fabs(this->dataPtr->blWheelState * this->dataPtr->blWheelRadius) <
      this->dataPtr->maxSpeed)
      && (fabs(this->dataPtr->brWheelState * this->dataPtr->brWheelRadius) <
      this->dataPtr->maxSpeed))
  {
    blGasTorque = gasPercent * this->dataPtr->backTorque * gasMultiplier;
    brGasTorque = gasPercent * this->dataPtr->backTorque * gasMultiplier;
  }

  // Brake pedal, hand-brake torque.
  // Compute percents and add together, saturating at 100%
  // double brakePercent = this->BrakePedalPercent()
  //   + this->HandBrakePercent();
  double brakePercent = this->dataPtr->brakePedalPercent;

  brakePercent = ignition::math::clamp(
      brakePercent, this->dataPtr->minBrakePercent, 1.0);
  // Map brake torques to individual wheels.
  // Apply brake torque in opposition to wheel spin direction.
  double flBrakeTorque, frBrakeTorque, blBrakeTorque, brBrakeTorque;
  // Below the smoothing speed in rad/s, reduce that can be applied brake torque
  double smoothingSpeed = 0.5;
  flBrakeTorque = -brakePercent*this->dataPtr->frontBrakeTorque *
      ignition::math::clamp(
      this->dataPtr->flWheelState / smoothingSpeed, -1.0, 1.0);
  frBrakeTorque = -brakePercent*this->dataPtr->frontBrakeTorque *
      ignition::math::clamp(
      this->dataPtr->frWheelState / smoothingSpeed, -1.0, 1.0);
  blBrakeTorque = -brakePercent*this->dataPtr->backBrakeTorque *
      ignition::math::clamp(
      this->dataPtr->blWheelState / smoothingSpeed, -1.0, 1.0);
  brBrakeTorque = -brakePercent*this->dataPtr->backBrakeTorque *
      ignition::math::clamp(
      this->dataPtr->brWheelState / smoothingSpeed, -1.0, 1.0);

  // Lock wheels if high braking that can be applied at low speed
  if (brakePercent > 0.7 && fabs(this->dataPtr->flWheelState) < smoothingSpeed)
    this->dataPtr->flWheelJoint->SetParam("stop_cfm", 0, 0.0);
  else
    this->dataPtr->flWheelJoint->SetParam("stop_cfm", 0, 1.0);

  if (brakePercent > 0.7 && fabs(this->dataPtr->frWheelState) < smoothingSpeed)
    this->dataPtr->frWheelJoint->SetParam("stop_cfm", 0, 0.0);
  else
    this->dataPtr->frWheelJoint->SetParam("stop_cfm", 0, 1.0);

  if (brakePercent > 0.7 && fabs(this->dataPtr->blWheelState) < smoothingSpeed)
    this->dataPtr->blWheelJoint->SetParam("stop_cfm", 0, 0.0);
  else
    this->dataPtr->blWheelJoint->SetParam("stop_cfm", 0, 1.0);

  if (brakePercent > 0.7 && fabs(this->dataPtr->brWheelState) < smoothingSpeed)
    this->dataPtr->brWheelJoint->SetParam("stop_cfm", 0, 0.0);
  else
    this->dataPtr->brWheelJoint->SetParam("stop_cfm", 0, 1.0);

  this->dataPtr->flWheelJoint->SetForce(0, flGasTorque + flBrakeTorque);
  this->dataPtr->frWheelJoint->SetForce(0, frGasTorque + frBrakeTorque);
  this->dataPtr->blWheelJoint->SetForce(0, blGasTorque + blBrakeTorque);
  this->dataPtr->brWheelJoint->SetForce(0, brGasTorque + brBrakeTorque);

  // gzerr << "gas and brake torque " << flGasTorque << " "
  //       << flBrakeTorque << std::endl;

  // reset if last command is more than x sec ago
  if ((curTime - this->dataPtr->lastGasCmdTime).Double() > 0.3)
  {
    this->dataPtr->gasPedalPercent = 0.0;
    this->dataPtr->brakePedalPercent = 0.0;
  }
  if ((curTime - this->dataPtr->lastSteeringCmdTime).Double() > 0.3)
  {
    this->dataPtr->handWheelCmd = 0;
  }


  // push to logger list
  std::lock_guard<std::mutex> loggerLock(this->dataPtr->loggerMutex);
  if ((curTime - this->dataPtr->lastLoggerWriteTime).Double() >=
      1.0/this->dataPtr->logRate)
  {
    this->dataPtr->lastLoggerWriteTime = curTime;

    PriusData data;
    data.timestamp = curTime.Double();
    data.pose = this->dataPtr->model->GetWorldPose().Ign();
    data.fuelEfficiency= 1;
    this->dataPtr->dataPoints.push_back(data);
  }
}

/////////////////////////////////////////////////
void PriusHybridPlugin::UpdateHandWheelRatio()
{
  // The total range the steering wheel can rotate
  this->dataPtr->handWheelHigh =
      this->dataPtr->handWheelJoint->GetHighStop(0).Radian();
  this->dataPtr->handWheelLow =
      this->dataPtr->handWheelJoint->GetLowStop(0).Radian();
  double handWheelRange =
      this->dataPtr->handWheelHigh - this->dataPtr->handWheelLow;
  double high = std::min(
      this->dataPtr->flWheelSteeringJoint->GetHighStop(0).Radian(),
      this->dataPtr->frWheelSteeringJoint->GetHighStop(0).Radian());
  high = std::min(high, this->dataPtr->maxSteer);
  double low = std::max(
      this->dataPtr->flWheelSteeringJoint->GetLowStop(0).Radian(),
      this->dataPtr->frWheelSteeringJoint->GetLowStop(0).Radian());
  low = std::max(low, -this->dataPtr->maxSteer);
  double tireAngleRange = high - low;

  // Compute the angle ratio between the steering wheel and the tires
  this->dataPtr->steeringRatio = tireAngleRange / handWheelRange;
}

/////////////////////////////////////////////////
// function that extracts the radius of a cylinder or sphere collision shape
// the function returns zero otherwise
double PriusHybridPlugin::CollisionRadius(physics::CollisionPtr _coll)
{
  if (!_coll || !(_coll->GetShape()))
    return 0;
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
  {
    physics::CylinderShape *cyl =
        static_cast<physics::CylinderShape*>(_coll->GetShape().get());
    return cyl->GetRadius();
  }
  else if (_coll->GetShape()->HasType(physics::Base::SPHERE_SHAPE))
  {
    physics::SphereShape *sph =
        static_cast<physics::SphereShape*>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

/////////////////////////////////////////////////
double PriusHybridPlugin::GasTorqueMultiplier()
{
  // if (this->dataPtr->keyState == ON)
  {
    if (this->dataPtr->directionState == PriusHybridPluginPrivate::FORWARD)
      return 1.0;
    else if (this->dataPtr->directionState == PriusHybridPluginPrivate::REVERSE)
      return -1.0;
  }
  return 0;
}

GZ_REGISTER_MODEL_PLUGIN(PriusHybridPlugin)
