
#ifndef _GAZEBO_LIFT_DISTRIBUTION_PLUGIN_HH_
#define _GAZEBO_LIFT_DISTRIBUTION_PLUGIN_HH_


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

#include "send_protobuf_msgs.hh"

#include <math.h>


// Check which variables go into class and which don't
using namespace std;
double spanwise_yn;

// to do: add it in the initialization function (initialize the vectors with the correct size):
vector<double> alpha_vec;
vector<double> cl_vec;
vector<double> cd_vec;
vector<double> cm_vec;
double cl_retrieved;
double cd_retrieved;
double cm_retrieved;
double additional_ratio;

ignition::math::Vector3d vel;
ignition::math::Vector3d velI;

namespace gazebo
{
 class GAZEBO_VISIBLE LiftDistributionPlugin : public ModelPlugin
 {
 // Parameters of the Lift Distribution
 public:
   double Gamma0;
   // Wing parameters
   double span;
   double chord = 0.32; //m
   // The spanwise location
   double spanwise_y;
   double theta;
   double current_y;
   double D = 0.05;
   double rho;
   double saved_component;
   double cosSweepAngle;

   // Number of N_segments
   int N_segments;
   int N_iterations;
   // Circulation Stuff
   double circulation_element;
   std::vector<double> circulation_vector;
   std::vector<double> circulation_vector_new;
   std::vector<double> derivative_circulation_vector;

   ignition::math::Vector3d liftI_arr[100]; // TODO ISABELLE fix this, not hardcoded!

   transport::PublisherPtr force_pub_;

 public: int k;


 /// \brief Constructor.
 public: LiftDistributionPlugin();
 /// \brief Destructor.
 public: ~LiftDistributionPlugin();

 // Documentation Inherited.
 protected: bool radialSymmetry;

 /// \brief Callback for World Update events.
 protected: virtual void OnUpdate();

 protected: ignition::math::Vector3d cp;

 protected: ignition::math::Vector3d current_wing_location;

 protected: ignition::math::Vector3d groundspeed_world;
 // Documentation Inherited.
 public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

 /// \brief Pointer to link currently targeted by mud joint.
 protected: physics::LinkPtr link;

 /// \brief Pointer to model containing plugin.
 protected: physics::ModelPtr model;

 /// \brief SDF for this plugin;
 protected: sdf::ElementPtr sdf;

 std::string namespace_;

 /// \brief Pointer to world.
 protected: physics::WorldPtr world;

 /// \brief Pointer to physics engine.
 protected: physics::PhysicsEnginePtr physics;

 /// \brief Connection to World Update events.
 protected: event::ConnectionPtr updateConnection;

 /// \brief angle of attack
 protected: double alpha;

 /// \brief effective planeform surface area
 protected: double area;

 protected: ignition::math::Vector3d forward;

 protected: ignition::math::Vector3d upward;

 protected: std::vector<double> local_airspeed_components; // the components aee magnitudes

 protected: std::vector<double> local_cl_vector;

 protected:
   double azimuth_wind;	// [rad/s]
   double vel_wind;	// [m/s]

 protected: double V_N_wind;
 protected: double V_E_wind;
 protected: double V_D_wind;
 protected: double eta_wind;

 private:
 // An Additional SubscriberPtr To Subscribe to the test_msg Topic
 transport::NodePtr node_handle_;
 transport::SubscriberPtr test_msg_sub_;
 typedef const boost::shared_ptr<const msgs::Vector3d> TestMsgPtr;
 void TestMsgCallback(TestMsgPtr &test_msg);

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

   // the functions
   float binarySearch(vector<double> arr, double x, int l, int r);
   ignition::math::Vector3d retrieve_values(float float_index);
   std::vector<double> get_local_cl_vector(std::vector<double> AoA_vector);
   std::vector<double> get_induced_AoA_vector();
   std::vector<double> get_geometric_AoA_vector();
   std::vector<double> get_effective_AoA_vector();
   double get_induced_AoA(double spanwise_yn);


};
}
#endif
