/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <functional>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/sensors/Noise.hh"

#include "WindPlugin.hh"

// added for printing on console
#include <gazebo/gazebo.hh>
#include <string>
#include <iostream>
using namespace std;


// / \brief Private class for WindPlugin

class gazebo::WindPluginPrivate
{
  /*
  /// \brief World pointer.
  public: physics::WorldPtr world;

  /// \brief Connection to World Update events.
  public: event::ConnectionPtr updateConnection;
  */
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  std::string namespace_;

  std::string frame_id_;
  std::string link_name_;
  std::string wind_pub_topic_;

  double wind_velocity_mean_;
  double wind_velocity_variance_;
  double wind_gust_velocity_mean_;
  double wind_gust_velocity_variance_;

  double wind_azimuth_;
  double wind_gust_azimuth_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr wind_pub_;

};


using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(WindPlugin)

/////////////////////////////////////////////////
/* WindPlugin::WindPlugin()
    : dataPtr(new WindPluginPrivate)
{
}
*/

/////////////////////////////////////////////////
void WindPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) //void WindPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  printf("Print something 3");
  GZ_ASSERT(_world, "WindPlugin world pointer is NULL");
  world_ = _world;


  //physics::Wind &wind = this->dataPtr->world->Wind();
  //model_ = _model;
  // todo is this needed: world_ = model_->GetWorld();
  double wind_gust_start = kDefaultWindGustStart; //todo where is this defined/declared?
  double wind_gust_duration = kDefaultWindGustDuration; //todo where is this defined/declared?

/*  if (_sdf->HasElement("horizontal"))
  {
    sdf::ElementPtr sdfHoriz = _sdf->GetElement("horizontal");

    if (sdfHoriz->HasElement("magnitude"))
    {
      sdf::ElementPtr sdfMag = sdfHoriz->GetElement("magnitude");

      if (sdfMag->HasElement("time_for_rise"))
      {
        this->dataPtr->characteristicTimeForWindRise =
          sdfMag->Get<double>("time_for_rise");
      }

      if (sdfMag->HasElement("sin"))
      {
        sdf::ElementPtr sdfMagSin = sdfMag->GetElement("sin");

        if (sdfMagSin->HasElement("amplitude_percent"))
        {
          this->dataPtr->magnitudeSinAmplitudePercent =
            sdfMagSin->Get<double>("amplitude_percent");
        }

        if (sdfMagSin->HasElement("period"))
        {
          this->dataPtr->magnitudeSinPeriod = sdfMagSin->Get<double>("period");
        }
      }

      if (sdfMag->HasElement("noise"))
      {
        this->dataPtr->noiseMagnitude = sensors::NoiseFactory::NewNoiseModel(
              sdfMag->GetElement("noise"));
      }
    }

    if (sdfHoriz->HasElement("direction"))
    {
      sdf::ElementPtr sdfDir = sdfHoriz->GetElement("direction");

      if (sdfDir->HasElement("time_for_rise"))
      {
        this->dataPtr->characteristicTimeForWindOrientationChange =
          sdfDir->Get<double>("time_for_rise");
      }

      if (sdfDir->HasElement("sin"))
      {
        sdf::ElementPtr sdfDirSin = sdfDir->GetElement("sin");

        if (sdfDirSin->HasElement("amplitude"))
        {
          this->dataPtr->orientationSinAmplitude =
            sdfDirSin->Get<double>("amplitude");
        }

        if (sdfDirSin->HasElement("period"))
        {
          this->dataPtr->orientationSinPeriod =
            sdfDirSin->Get<double>("period");
        }
      }

      if (sdfDir->HasElement("noise"))
      {
        this->dataPtr->noiseDirection = sensors::NoiseFactory::NewNoiseModel(
            sdfDir->GetElement("noise"));
      }
    }
  }*/
  printf("Print something 4");
  getSdfParam<std::string>(_sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
  // Get the wind params from SDF.
  getSdfParam<double>(_sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
  getSdfParam<double>(_sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
  getSdfParam<double>(_sdf, "windAzimuth", wind_azimuth_, wind_azimuth_);
  // Get the wind gust params from SDF.
  getSdfParam<double>(_sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
  getSdfParam<double>(_sdf, "windGustVelocityMean", wind_gust_velocity_mean_, wind_gust_velocity_mean_);
  getSdfParam<double>(_sdf, "windGustVelocityVariance", wind_gust_velocity_variance_, wind_gust_velocity_variance_);
  getSdfParam<double>(_sdf, "windGustAzimuth", wind_gust_azimuth_, wind_gust_azimuth_);

  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("Couldn't find specified link \"" << link_name_ << "\".");

  printf("Print something 5");

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WindPlugin::OnUpdate, this, _1)); //TODO: FIX THIS PART HERE: NOTHING BEING BINDED
  printf("Print something 6");

  wind_pub_ = node_handle_->Advertise<wind_field_msgs::msgs::WindField>(wind_pub_topic_, 1);
  printf("Print something 7");



/////////////////////////////////////////////////
void WindPlugin::OnUpdate(const common::UpdateInfo& _info)
{

  // Get the current simulation time.
  // common::Time now = this->dataPtr->world->SimTime().Double();

  //TODO: get the time DONE
  common::Time now = world_->GetSimTime();

  // Calculate the wind velocity.
  double wind_velocity = wind_velocity_mean_; // todo need to add where this is defined

  ignition::math::Vector3d wind_gust(0, 0, 0);
  // Calculate the wind gust velocity.
  double wind_gust_velocity = 0;
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    wind_gust_velocity = wind_gust_velocity_mean_;
  }
  // Add the wind gust to the default wind
  double wind_vel_x = sin(wind_azimuth_)*wind_velocity + sin(wind_gust_azimuth_)*wind_gust_velocity;
  double wind_vel_y = cos(wind_azimuth_)*wind_velocity + cos(wind_gust_azimuth_)*wind_gust_velocity;

  double wind_total_velocity = sqrt(wind_vel_x*wind_vel_x + wind_vel_y*wind_vel_y);
  double wind_total_azimuth  = atan2(wind_vel_x,wind_vel_y);

  wind_field_msgs::msgs::WindField wind_msg;

  wind_msg.set_frame_id(frame_id_);
  Set(wind_msg.mutable_stamp(), now);
  wind_msg.set_azimuth(wind_total_azimuth);
  wind_msg.set_velocity(wind_total_velocity);

  wind_pub_->Publish(wind_msg);
  printf("Print something");
  cout << wind_total_velocity << endl;;
// todo: this can be removed too, right?
/*  // Update loop for using the force on mass approximation
  // This is not recommended. Please use the LiftDragPlugin instead.

  // Get all the models
  physics::Model_V models = this->dataPtr->world->Models();

  // Process each model.
  for (auto const &model : models)
  {
    // Get all the links
    physics::Link_V links = model->GetLinks();

    // Process each link.
    for (auto const &link : links)
    {
      // Skip links for which the wind is disabled
      if (!link->WindMode())
        continue;

    }
  } */

}
