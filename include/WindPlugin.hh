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
#ifndef GAZEBO_PLUGINS_WINDPLUGIN_HH_
#define GAZEBO_PLUGINS_WINDPLUGIN_HH_

#include <memory>

#include <ignition/math/Vector3.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

/*
 * taken from gazebo_wind_field_plugin.h
 */
#include <string>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// KITEPOWER (Xander)
#include "WindField.pb.h"
#include "common.h"
/*
 * end
 */

namespace gazebo
{
  // Forward declaration
  class WindPluginPrivate;

  /// \brief A plugin that simulates a simple wind model.
  // The wind is described as a uniform worldwide model. So it is independant
  // from model position for simple computations. Its components are computed
  // separately:
  // - Horizontal amplitude:
  //      Low pass filtering on user input (complementary gain)
  //      + small local fluctuations
  //      + noise on value (noise amplitude is a factor of wind magnitude)
  //
  // - Horizontal direction:
  //      Low pass filtering on user input (complementary gain)
  //      + small local fluctuations
  //      + noise on value
  //
  // - Vertical amplitude:
  //      Noise proportionnal to wind magnitude.

/*
 * taken from gazebo_wind_field_plugin.h
 */
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "world";

static constexpr double kDefaultWindVelocityMean = 0.0;
static constexpr double kDefaultWindVelocityVariance = 0.0;
static constexpr double kDefaultWindGustVelocityMean = 0.0;
static constexpr double kDefaultWindGustVelocityVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static constexpr double kDefaultWindAzimuth = 0.0;
static constexpr double kDefaultWindGustAzimuth = M_PI/4;

// KITEPOWER (Xander)
static const std::string kDefaultWindFieldPubTopic= "/wind_field";
/*
 * end
 */

  class GAZEBO_VISIBLE WindPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: WindPlugin();

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Get the global wind velocity.
    /// \param[in] _wind Reference to the wind.
    /// \param[in] _wind Pointer to an entity at which location the wind
    /// velocity is to be calculated.
    /// \return Wind's velocity at entity's location.
    public: ignition::math::Vector3d LinearVel(
            const physics::Wind *_wind,
            const physics::Entity *_entity);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<WindPluginPrivate> dataPtr;

/*
 * taken from gazebo_wind_field_plugin.h
 */
    private:
    /// \brief Pointer to the update event connection.
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
/*
 * end
 */
} 

#endif
