/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_LIFT_DRAG_WITH_LOOKUP_PLUGIN_HH_
#define _GAZEBO_LIFT_DRAG_WITH_LOOKUP_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>

// KITEPOWER (Xander)
#include "../common.h"
#include "common.h"
#include <common.h>
#include "WindField.pb.h"

#include <iostream>
#include <fstream>
// #include <stdlib.h>     /* atof */
#include <string>
#include <list>
#include <vector>
#include <ignition/math/Vector3.hh>

#include "send_protobuf_msgs.hh"

#include <math.h>

static const std::string kDefaultWindFieldSubTopic = "/wind_field";

using namespace std;

double cl_retrieved;
double cd_retrieved;
double cm_retrieved;
double additional_ratio;

// to do: add it in the initialization function (initialize the vectors with the correct size):
vector<double> alpha_vec;
vector<double> cl_vec;
vector<double> cd_vec;
vector<double> cdp_vec;
vector<double> cm_vec;
vector<double> top_xtr_vec;
vector<double> bot_xtr_vec;
vector<double> top_itr_vec;
vector<double> bot_itr_vec;


namespace gazebo
{
  // KITEPOWER (Xander)
  typedef const boost::shared_ptr<const wind_field_msgs::msgs::WindField> WindFieldPtr;

  /// \brief A plugin that simulates lift and drag.
  class GAZEBO_VISIBLE LiftDragWithLookupPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: LiftDragWithLookupPlugin();

    /// \brief Destructor.
    public: ~LiftDragWithLookupPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief Coefficient of Lift / alpha slope.
    /// Lift = C_L * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    protected: double cla;

    /// \brief Coefficient of Drag / alpha slope.
    /// Drag = C_D * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    protected: double cda;

    /// \brief Coefficient of Moment / alpha slope.
    /// Moment = C_M * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    protected: double cma;

    /// \brief angle of attach when airfoil stalls
    protected: double alphaStall;

    /// \brief Cl-alpha rate after stall
    protected: double claStall;

    /// \brief Cd-alpha rate after stall
    protected: double cdaStall;

    /// \brief Cm-alpha rate after stall
    protected: double cmaStall;

    /// \brief: \TODO: make a stall velocity curve
    protected: double velocityStall;

    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
    protected: double rho;

    protected: double chord_length;

    protected: ignition::math::Vector3d tmp_vector;

    /// \brief if the shape is aerodynamically radially symmetric about
    /// the forward direction. Defaults to false for wing shapes.
    /// If set to true, the upward direction is determined by the
    /// angle of attack.
    protected: bool radialSymmetry;

    /// \brief effective planeform surface area
    protected: double area;

    /// \brief angle of sweep
    protected: double sweep;

    /// \brief initial angle of attack
    protected: double alpha0;

    /// \brief angle of attack
    protected: double alpha;

    /// \brief center of pressure in link local coordinates
    protected: ignition::math::Vector3d cp;

    /// \brief Normally, this is taken as a direction parallel to the chord
    /// of the airfoil in zero angle of attack forward flight.
    protected: ignition::math::Vector3d forward;

    /// \brief A vector in the lift/drag plane, perpendicular to the forward
    /// vector. Inflow velocity orthogonal to forward and upward vectors
    /// is considered flow in the wing sweep direction.
    protected: ignition::math::Vector3d upward;

    /// \brief Smoothed velocity
    protected: ignition::math::Vector3d velSmooth;

    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;

    /// \brief Pointer to a joint that actuates a control surface for
    /// this lifting body
    protected: physics::JointPtr controlJoint;

    /// \brief how much to change CL per radian of control surface joint
    /// value.
    protected: double controlJointRadToCL;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    // KITEPOWER (Xander)
    private:
	  transport::NodePtr node_handle_;
	  std::string wind_field_sub_topic_;
	  transport::SubscriberPtr wind_field_sub_;
	  void WindFieldCallback(WindFieldPtr &wind_field);
	  std::string namespace_;

    // An Additional SubscriberPtr To Subscribe to the test_msg Topic
    transport::SubscriberPtr test_msg_sub_;
    typedef const boost::shared_ptr<const msgs::Vector3d> TestMsgPtr;
    void TestMsgCallback(TestMsgPtr &test_msg);

    // KITEPOWER (Xander)
    protected:
	  double azimuth_wind;	// [rad/s]
	  double vel_wind;	// [m/s]
    double vel_wind_x;	// [m/s]
    double vel_wind_y;	// [m/s]
    double vel_wind_z;	// [m/s]

    // KITEPOWER
    /// \brief set to true to use constant Coefficient of Drag
    protected: bool useConstantDragCoefficient;

    // (Isabelle)
    protected: bool useLookUpTable;
    protected: bool testMsgCallbackUsed;
    protected: double V_N_wind;
    protected: double V_E_wind;
    protected: double V_D_wind;
    protected: double eta_wind;
    protected: ignition::math::Vector3d airspeed_world;
    protected: ignition::math::Vector3d airspeed_body;
    protected: ignition::math::Vector3d groundspeed_NED;
    protected: ignition::math::Vector3d groundspeed_world;
    protected: ignition::math::Vector3d constantWind;

    ///////////////////////////////////////////////////////////////////////////
    protected:
      ////////////////////////////////////////////////////////////////////////////////
      const int MAXLINE=256;
      char oneline[256];
      string line;
      int counter = 0;
      string alpha_str;
      string cl_str;
      string cd_str;
      string cdp_str;
      string cm_str;
      string top_xtr_str;
      string bot_xtr_str;
      string top_itr_str;
      string bot_itr_str;

      // double alpha;
      double cl;
      double cd;
      double cdp;
      double cm;
      double top_xtr;
      double bot_xtr;
      double top_itr;
      double bot_itr;

      string airfoilDatafilePath;

      float binarySearch(vector<double> arr, double x, int l, int r);
      ignition::math::Vector3d retrieve_values(float float_index);

  };
}
#endif
