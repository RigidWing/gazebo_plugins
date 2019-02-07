#ifndef GAZEBO_PLUGINS_WAYPOINTFOLLOWINGCONTROLPLUGIN_HH_
#define GAZEBO_PLUGINS_WAYPOINTFOLLOWINGCONTROLPLUGIN_HH_

#include <array>
#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include <ignition/transport/Node.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gtest/gtest.h>
#include <gazebo/rendering/rendering.hh>

using namespace gazebo;

class GAZEBO_VISIBLE WaypointFollowingControlPlugin : public ModelPlugin
{
  /// \brief Constructor.
  public: WaypointFollowingControlPlugin();
  /// \brief Destructor.
  public: ~WaypointFollowingControlPlugin();

  // The Ptrs to the joints of the control surfaces
  private: physics::JointPtr rudder_joint;
  private: physics::JointPtr propeller_joint;

  // protected: physics::JointControllerPtr rudderJointController;
  protected: double rudder_pos_target;
  protected: double previous_rudder_pos_target;
  protected: double diff;

  // The ptrs to the PID control for the control surfaces
  private: common::PID rudder_pid;
  private: common::PID propeller_pid;
  private: common::PID thrust_pid;

  // the gains
  private: double rudder_p_gain;
  private: double rudder_i_gain;
  private: double rudder_d_gain;
  private: double propeller_p_gain;
  private: double propeller_i_gain;
  private: double propeller_d_gain;
  private: double thrust_p_gain;
  private: double thrust_i_gain;
  private: double thrust_d_gain;

  // names of the joints
  private: std::string rudderJointName;
  private: std::string rudderJointNameFull;
  private: std::string propellerJointName;
  private: std::string propellerJointNameFull;

  protected: physics::JointControllerPtr rudderJointController;


  // Transport-related variable pointers and variable
  // An Additional SubscriberPtr To Subscribe to the test_msg Topic
  private: transport::NodePtr node;
  private: transport::SubscriberPtr test_msg_sub_;
  private: transport::SubscriberPtr aoa_msg_sub_;
  typedef const boost::shared_ptr<const msgs::Vector3d> TestMsgPtr;
  typedef const boost::shared_ptr<const msgs::Any> AoaMsgPtr;
  void TestMsgCallback(TestMsgPtr &test_msg);
  void AoaMsgCallback(AoaMsgPtr &aoa_msg);

  // Wind variables
  protected: double azimuth_wind;	// [rad/s]
  protected: double vel_wind;	// [m/s]
  protected: double V_N_wind;
  protected: double V_E_wind;
  protected: double V_D_wind;
  protected: double eta_wind;

  //
  protected: ignition::math::Vector3d cp;

  // Model ptr
  private: physics::ModelPtr model;

  protected: physics::LinkPtr link;

  // Time
  private: common::Time lastControllerUpdateTime;
  private: common::Time currentTime;
  private: double _dt;

  // Connection
  protected: event::ConnectionPtr updateConnection;
  protected: sdf::ElementPtr sdf;
  protected: physics::WorldPtr world;

  // Waypoints
  public: ignition::math::Vector3d currentWaypoint;
  public: ignition::math::Vector3d waypointOne;
  public: ignition::math::Vector3d waypointTwo;
  public: ignition::math::Vector3d waypointThree;
  public: ignition::math::Vector3d waypointFour;

  public: ignition::math::Vector3d lineStartingPoint;
  public: ignition::math::Vector3d lineEndPoint;
  public: rendering::DynamicLines dynamicLine;//ScenePtr scene;

  // the constant forwardSpeed
  public: double forwardSpeed;

  // state machine for the waypoints tracking 1,2,3,4 for each waypoint
  public: int followed_waypoint_number;
  public: int currentWaypointInt;
  public: bool rotation_direction; // 0 for clockwise, 1 for counterclockwise
  public: double heading_error;
  public: double speed_error;
  public: double force_to_apply_rudder;
  public: double force_to_apply_to_propeller;
  public: double force_to_apply_to_body;
  public: double threshold;
  public: double alpha;

  public: int updateIterationInt;
  // Functions
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  private: void OnUpdate();





};


#endif
