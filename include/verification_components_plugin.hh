#ifndef _GAZEBO_VERIFICATION_COMPONENTS_PLUGIN_HH_
#define _GAZEBO_VERIFICATION_COMPONENTS_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>

// KITEPOWER (Xander)
// #include "../common.h"
#include "common.h"
#include <common.h>

#include <iostream>
#include <fstream>
// #include <stdlib.h>     /* atof */
#include <string>
#include <list>
#include <vector>
#include <ignition/math/Vector3.hh>

#include <math.h>

#include <gtest/gtest.h>
#include <ignition/math/Matrix3.hh>
#include <gazebo/physics/LinkState.hh>
#include <gazebo/physics/physics.hh>


namespace gazebo
{
  class GAZEBO_VISIBLE VerificationComponentsPlugin : public ModelPlugin
  {


    public: VerificationComponentsPlugin();
    public: ~VerificationComponentsPlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    protected: virtual void OnUpdate();

    protected: physics::ModelPtr model;
    protected: physics::Link_V links;
    protected: event::ConnectionPtr updateConnection;
    protected: physics::LinkPtr currentLink;
    protected: physics::InertialPtr inertiaCurrentLink;
    protected: physics::LinkState currentLinkState(const physics::LinkPtr _link);

    /////////////////////////////////
    // Overall Properties Model /////
    /////////////////////////////////
    protected:
      ignition::math::Vector3d modelCoG;
      ignition::math::Vector3d positionMassProductTotal;
      ignition::math::Matrix3<double> inertiaComponentAboutOverallCoG;
      double modelMass;
      ignition::math::Vector3d modelLinearVel;
      ignition::math::Vector3d modelAngularVel;
      ignition::math::Vector3d modelLinearAccel;
      ignition::math::Vector3d modelAngularAccel;
      ignition::math::Vector3d calculatedModelForce;
      ignition::math::Vector3d calculatedModelMoment;
      ignition::math::Matrix3<double> modelInertia;
      double IzzModel;
      ignition::math::Box currentBoundingBox;
      







  };
}
#endif
