#include <waypoint_following_control_plugin.hh>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(WaypointFollowingControlPlugin)

////////////////////////////////////////////////////////////////////////////////
WaypointFollowingControlPlugin::WaypointFollowingControlPlugin()
{
  // Initialize the PID parameters
  this->rudder_pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);
}

////////////////////////////////////////////////////////////////////////////////
WaypointFollowingControlPlugin::~WaypointFollowingControlPlugin()
{
  this->updateConnection.reset();
}

////////////////////////////////////////////////////////////////////////////////
void ControlPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf){

  GZ_ASSERT(_model, "ControlPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "ControlPlugin _sdf pointer is NULL");
  this->model = _model;
  EXPECT_TRUE(model != NULL);
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Obtain the SDF Parameters
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  // Overload the Rudder PID parameters if available
  if (_sdf->HasElement("rudder_p_gain"))
  {
      this->rudder_p_gain = _sdf->Get<double>("rudder_p_gain");
  }

  if (_sdf->HasElement("rudder_i_gain"))
  {
      this->rudder_i_gain = _sdf->Get<double>("rudder_i_gain");
  }

  if (_sdf->HasElement("rudder_d_gain"))
  {
      this->rudder_d_gain = _sdf->Get<double>("rudder_d_gain");
  }


  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Get the Controller time control.
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  this->lastControllerUpdateTime = this->model->GetWorld()->SimTime();
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Initialize Transport
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  // create a publisher
  this->state_pub_ = this->node->Advertise<msgs::Vector3d>("/visual/vector_component", 50);
  // TO DO ISABELLE TO SUBSCRIBE TO OBTAIN THE CONTROL TARGETS
  this->control_target_sub_ = this->node->Subscribe("/control_targets",&ControlPlugin::GetControlTargets, this);
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Listen to the Uodate event
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////


  if (_sdf->HasElement("rudder_joint_name"))
  {
    this->rudderJointController.reset(new physics::JointController(this->model));

    // Get the full name of the joint
    this->rudderJointName = _sdf->Get<std::string>("rudder_joint_name");
    this->rudderJointNameFull = this->model->GetJoint(rudderJointName)->GetScopedName();

    // get the needed jointptr from the Model (takes as input The name of the joint, specified in the world file)
    this->rudder_joint = this->model->GetJoint(this->rudderJointNameFull);
    EXPECT_TRUE(this->rudder_joint != NULL);

    // use AddJoint which takes as input a JointPtr
    this->rudderJointController->AddJoint(this->rudder_joint);

    // set the position targets
    this->rudder_pos_target = 2.6;
    this->rudderJointController->SetPositionTarget(this->rudderJointNameFull, this->rudder_pos_target);

    // Set the position PID controller (inouts: name of joint + common pid)
    this->rudderJointController->SetPositionPID(this->rudderJointNameFull, common::PID(this->rudder_p_gain,this->rudder_i_gain,this->rudder_d_gain));
  }

  //////////////////////////////////////////////////////////////////////////////
  // Start the OnUpdate Procedure

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ControlPlugin::OnUpdate, this));


  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
}

////////////////////////////////////////////////////////////////////////////////
void ControlPlugin::OnUpdate()
{
    this->rudderJointController->Update();
    ////////////////////////////////////////////////////////////////////////////
    // Get the airspeed vector /////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    // get the groundspeed (get linear velocity at cp in inertial frame)
    #if GAZEBO_MAJOR_VERSION >= 9
      ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp);
    #else
      ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->cp));
    #endif
    // get the constant wind
    ignition::math::Vector3d constantWind(this->V_N_wind,this->V_E_wind,-1 * this->V_D_wind);
    // add it to the groundspeed to get the airspeed
    vel -= constantWind;

    // Normalize it
    vel.Normalize()

    // Get the current pose of the aircraft
    ignition::math::Pose3d currentPose = this->link->WorldCoGPose();

    // Extract the location from the pose

    // Get the vector connecting the current location of the aircraft

    // Normalize this vector as well

    // Update


    if (1)
    {
      gzdbg << "The rudder target position " << this->rudder_pos_target << "\n";
      gzdbg << "rudderJointName " << this->rudderJointName << "\n";
      gzdbg << "rudderJointNameFull " << this->rudderJointNameFull << "\n";
      gzdbg << "rudder Force : " << this->rudder_joint->GetForce(0) << "\n";


    }
    gzdbg << " AFTER UPDATE: this->rudder_joint->Position(0)" << this->rudder_joint->Position(0) << "\n";
}

// Callback of the SubscriberPtr to the test_msg Topic
void LiftDragWithLookupPlugin::TestMsgCallback(TestMsgPtr &test_msg){

  vel_wind = test_msg->x();
  azimuth_wind = test_msg->y();
  eta_wind = test_msg->z();
  // In north east down coordinates VERIFY
  V_N_wind = vel_wind * cos(azimuth_wind) * cos(eta_wind + M_PI);
  V_E_wind = vel_wind * sin(azimuth_wind) * cos(eta_wind + M_PI);
  V_D_wind = vel_wind * sin(eta_wind + M_PI);

}
