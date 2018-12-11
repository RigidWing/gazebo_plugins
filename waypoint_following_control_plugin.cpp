#include <waypoint_following_control_plugin.hh>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(WaypointFollowingControlPlugin)

////////////////////////////////////////////////////////////////////////////////
WaypointFollowingControlPlugin::WaypointFollowingControlPlugin()
{
  this->cp = ignition::math::Vector3d(0, 0, 0);
  // Initialize the PID parameters
  this->rudder_pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);
  // this->propeller_pid.Init(10.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0);

  // the state
  this->reached_waypoint = 0;
  this->followed_waypoint_number = 1;

  // set the initial heading error
  this->heading_error = 0.0;
}

////////////////////////////////////////////////////////////////////////////////
WaypointFollowingControlPlugin::~WaypointFollowingControlPlugin()
{
  // this->updateConnection.reset();
}

////////////////////////////////////////////////////////////////////////////////
void WaypointFollowingControlPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf){

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
      this->rudder_pid.SetPGain(this->rudder_p_gain);
  }

  if (_sdf->HasElement("rudder_i_gain"))
  {
      this->rudder_i_gain = _sdf->Get<double>("rudder_i_gain");
      this->rudder_pid.SetIGain(this->rudder_i_gain);
  }

  if (_sdf->HasElement("rudder_d_gain"))
  {
      this->rudder_d_gain = _sdf->Get<double>("rudder_d_gain");
      this->rudder_pid.SetDGain(this->rudder_d_gain);
  }

  if (_sdf->HasElement("propeller_p_gain"))
  {
    this->propeller_p_gain = _sdf->Get<double>("propeller_p_gain");
    this->propeller_pid.SetPGain(this->propeller_p_gain);
  }

  if (_sdf->HasElement("propeller_i_gain"))
  {
    this->propeller_i_gain = _sdf->Get<double>("propeller_i_gain");
    this->propeller_pid.SetPGain(this->propeller_i_gain);
  }

  if (_sdf->HasElement("propeller_d_gain"))
  {
    this->propeller_d_gain = _sdf->Get<double>("propeller_d_gain");
    this->propeller_pid.SetPGain(this->propeller_d_gain);
  }



  if (_sdf->HasElement("waypointOne"))
  {
    this->waypointOne = _sdf->Get<ignition::math::Vector3d>("waypointOne");
  }

  if (_sdf->HasElement("waypointTwo"))
  {
    this->waypointTwo = _sdf->Get<ignition::math::Vector3d>("waypointTwo");
  }

  if (_sdf->HasElement("waypointThree"))
  {
    this->waypointThree = _sdf->Get<ignition::math::Vector3d>("waypointThree");
  }

  if (_sdf->HasElement("waypointFour"))
  {
    this->waypointFour = _sdf->Get<ignition::math::Vector3d>("waypointFour");
  }

  if (_sdf->HasElement("forwardSpeed"))
  {
    this->forwardSpeed = _sdf->Get<double>("forwardSpeed");
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
  // subscribe to the message containing the information about the windfield
  this->test_msg_sub_ = this->node->Subscribe("/test_topic",&WaypointFollowingControlPlugin::TestMsgCallback, this);
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Listen to the Uodate event
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////


  if (_sdf->HasElement("rudder_joint_name"))
  {
    // Get the full name of the joint
    this->rudderJointName = _sdf->Get<std::string>("rudder_joint_name");
    this->rudderJointNameFull = this->model->GetJoint(this->rudderJointName)->GetScopedName();
    // get the needed jointptr from the Model (takes as input The name of the joint, specified in the world file)
    this->rudder_joint = this->model->GetJoint(this->rudderJointNameFull);
    EXPECT_TRUE(this->rudder_joint != NULL);
  }

  if (_sdf->HasElement("propeller_joint_name"))
  {
    this->propellerJointName =  _sdf->Get<std::string>("propeller_joint_name");
    this->propellerJointNameFull = this->model->GetJoint(this->propellerJointName)->GetScopedName();
    // get the joint
    this->propeller_joint = this->model->GetJoint(this->propellerJointNameFull);
    EXPECT_TRUE(this->propeller_joint != NULL);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Start the OnUpdate Procedure

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    // GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The LiftDragWithLookupPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&WaypointFollowingControlPlugin::OnUpdate, this));
    }
  }

  if(_sdf->HasElement("forwardSpeed"))
  {
    this->forwardSpeed = _sdf->Get<double>("forwardSpeed");
  }



  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
}

////////////////////////////////////////////////////////////////////////////////
void WaypointFollowingControlPlugin::OnUpdate()
{
    ////////////////////////////////////////////////////////////////////////////
    // Get the airspeed vector /////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    // Get the current Time
    this->currentTime = this->model->GetWorld()->SimTime();
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
    ignition::math::Vector3d vel_normalized = vel;
    vel_normalized.Normalize();

    // Get the current pose of the aircraft
    ignition::math::Pose3d currentPose = this->link->WorldCoGPose();

    gzdbg << "Current Pose: " << currentPose << "\n";
    // Extract the location from the pose
    ignition::math::Vector3d currentLocation = currentPose.Pos();
    gzdbg << "Current Position: " << currentLocation << "\n";

    // Get the vector connecting the current location of the aircraft
    ignition::math::Vector3d difference = this->waypointOne.operator-(currentLocation);
    gzdbg << "Waypoint one: " << this->waypointOne << "\n";
    gzdbg << "The difference is: " << difference << "\n";
    // Normalize this vector as well
    difference.Normalize();

    // Get the angle between the difference vector and the vel vector
    double dot_product = difference.Dot(vel_normalized);
    this->heading_error = acos(dot_product); // as desired will be the one less than 180 degrees.

    // Let clockwise be positive and counterclockwise be negative.
    ignition::math::Vector3d cross_product = vel_normalized.Cross(difference);
    // check the z-component
    if (cross_product.Z() > 0) // will be counterclockwise so 1
    {
      this->rotation_direction = 1;
    }
    else
    {
      this->rotation_direction = 0;
      this->heading_error = -1*this->heading_error; // TODO CHECK THIS WHETHER OTHER WAY AROUND
    }

    // Get the speed speed_error
    this->speed_error = this->forwardSpeed - vel.Length();

    // Update
    if (1)
    {
      gzdbg << "rudderJointName " << this->rudderJointName << "\n";
      gzdbg << "rudderJointNameFull " << this->rudderJointNameFull << "\n";
      gzdbg << "propellerJointName " << this->propellerJointName << "\n";
      gzdbg << "propellerJointNameFull " << this->propellerJointNameFull << "\n";
      gzdbg << "rudder Force : " << this->rudder_joint->GetForce(0) << "\n";
    }
    gzdbg << " AFTER UPDATE: this->rudder_joint->Position(0)" << this->rudder_joint->Position(0) << "\n";


    ////////////////////////////////////////////////////////////////////////////
    // Update the state int if the waypoint is reached /////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    if (difference.Length() < this->threshold){
      this->reached_waypoint = 1;
    }

    ////////////////////////////////////////////////////////////////////////////
    // get the dt
    this->_dt = (this->currentTime - this->lastControllerUpdateTime).Double();

    ////////////////////////////////////////////////////////////////////////////
    // Get the force that needs to be applied, from the PID ////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // RUDDER
    this->force_to_apply_rudder = this->rudder_pid.Update(this->heading_error,this->_dt);
    // PROPELLER
    this->force_to_apply_to_propeller = this->propeller_pid.Update(this->speed_error,this->_dt);
    gzdbg << "The speed error " << this->speed_error << "\n";
    gzdbg << "dt " << this->_dt << "\n";
    gzdbg << "the force applied rudder " << this->force_to_apply_rudder << "\n";
    gzdbg << "the force applied propeller " << this->force_to_apply_to_propeller << "\n";


    double propellerRpms = this->propeller_joint->GetVelocity(0)/(2.0*M_PI)*60.0;
    gzdbg << "propellerRpms " << propellerRpms << "\n";
    ////////////////////////////////////////////////////////////////////////////
    // Apply the forces ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // RUDDER
    this->rudder_joint->SetForce(0,this->force_to_apply_rudder);
    // PROPELLER
    this->propeller_joint->SetForce(0, this->force_to_apply_to_propeller);

    ////////////////////////////////////////////////////////////////////////////
    // Get the time for the next update loop ///////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    this->lastControllerUpdateTime = this->model->GetWorld()->SimTime();
}

// Callback of the SubscriberPtr to the test_msg Topic
void WaypointFollowingControlPlugin::TestMsgCallback(TestMsgPtr &test_msg){

  vel_wind = test_msg->x();
  azimuth_wind = test_msg->y();
  eta_wind = test_msg->z();
  // In north east down coordinates VERIFY
  V_N_wind = vel_wind * cos(azimuth_wind) * cos(eta_wind + M_PI);
  V_E_wind = vel_wind * sin(azimuth_wind) * cos(eta_wind + M_PI);
  V_D_wind = vel_wind * sin(eta_wind + M_PI);

}
