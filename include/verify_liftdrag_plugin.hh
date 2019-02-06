#ifndef _GAZEBO_VERIFY_LIFTDRAG_PLUGIN_HH_
#define _GAZEBO_VERIFY_LIFTDRAG_PLUGIN_HH_

#include <string>
#include <vector>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>

// KITEPOWER (Xander)
#include "common.h"
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
#include <gtest/gtest.h>


#include "send_protobuf_msgs.hh"

#include <math.h>
// #include <process.h>
#include <stdio.h>

#include <stdlib.h>

namespace gazebo
{
 class GAZEBO_VISIBLE VerifyLiftdragPlugin : public ModelPlugin
 {

 public: VerifyLiftdragPlugin();
 public: ~VerifyLiftdragPlugin();

  public:
    int type_of_test;
    int wind_condition_iterator;
    int test_number;

    double start;
    double end;
    double step;
    double second_spawn_arg;
    double third_spawn_arg;

    double force_X;
    double force_Y;
    double force_Z;

    double magnitude;
    double azimuth;
    double elevation;

    double control_variable;

    std::string spawn_args[4];

 /// \brief Pointer to model containing plugin.
 protected: physics::ModelPtr model;

 /// \brief SDF for this plugin;
 protected: sdf::ElementPtr sdf;

 /// \brief Connection to World Update events.
 protected: event::ConnectionPtr updateConnection;

 private: transport::SubscriberPtr force_sub_;

 typedef const boost::shared_ptr<const msgs::WrenchStamped> WrenchStampedMsgPtr;

 typedef const boost::shared_ptr<const msgs::Wrench> WrenchMsgPtr;


 void ForceMsgCallback(WrenchStampedMsgPtr &wrench_stamped_msg);

 public: msgs::Wrench wrench_msg;

 public: msgs::Vector3d force_msg;

 protected: virtual void OnUpdate();
 public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

 //
 public: std::string jointName;
 public: std::string jointNameFull;
 protected: physics::JointPtr joint;

public: physics::JointWrench jointWrench;


};
}

#endif
