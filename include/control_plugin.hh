#ifndef GAZEBO_PLUGINS_CONTROLPLUGIN_HH_
#define GAZEBO_PLUGINS_CONTROLPLUGIN_HH_

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


using namespace gazebo;

class GAZEBO_VISIBLE ControlPlugin : public ModelPlugin
{
  /// \brief Constructor.
  public: ControlPlugin();
  /// \brief Destructor.
  public: ~ControlPlugin();

  // The Ptrs to the joints of the control surfaces
  private: physics::JointPtr elevator_joint;
  private: physics::JointPtr rudder_joint;

  // The ptrs to the PID control for the control surfaces
  private: common::PID elevator_pid;
  private: common::PID rudder_pid;
  // Control targets
  private: double elevator_pos_target;
  private: double rudder_pos_target;

  // Transport-related variable pointers and variable
  private: transport::NodePtr node;
  private: transport::PublisherPtr state_pub_;
  private: transport::SubscriberPtr control_target_sub_;
  typedef const boost::shared_ptr<const msgs::Vector3d> ControlTargetMsgPtr;

  // Model ptr
  private: physics::ModelPtr model;

  protected: physics::LinkPtr link;

  // Time
  private: common::Time lastControllerUpdateTime;
  // Connection
  protected: event::ConnectionPtr updateConnection;

  protected: sdf::ElementPtr sdf;

  protected: physics::WorldPtr world;

  // Functions
  public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  private: void OnUpdate();
  private: void GetControlTargets(ControlTargetMsgPtr &control_targets_msg);

};


#endif
