#include <waypoint_following_control_plugin.hh>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(WaypointFollowingControlPlugin)

////////////////////////////////////////////////////////////////////////////////
WaypointFollowingControlPlugin::WaypointFollowingControlPlugin()
{
  this->cp = ignition::math::Vector3d(0, 0, 0);
  // Initialize the PID parameters
  this->rudder_pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);

  // the state
  this->followed_waypoint_number = 1;

  // set the initial heading error
  this->heading_error = 0.0;

  this->threshold = 3.0;

  // Create the file needed for logging
  // std::freopen("waypointFollowing.txt","w", stdout);
}

////////////////////////////////////////////////////////////////////////////////
WaypointFollowingControlPlugin::~WaypointFollowingControlPlugin()
{
  // this->updateConnection.reset();
}

////////////////////////////////////////////////////////////////////////////////
void WaypointFollowingControlPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf){

  GZ_ASSERT(_model, "WaypointFollowingControlPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "WaypointFollowingControlPlugin _sdf pointer is NULL");
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

  if (_sdf->HasElement("thrust_p_gain"))
  {
    this->thrust_p_gain = _sdf->Get<double>("thrust_p_gain");
    this->thrust_pid.SetPGain(this->thrust_p_gain);
  }

  if (_sdf->HasElement("thrust_i_gain"))
  {
    this->thrust_i_gain = _sdf->Get<double>("thrust_i_gain");
    this->thrust_pid.SetIGain(this->thrust_i_gain);
  }

  if (_sdf->HasElement("thrust_d_gain"))
  {
    this->thrust_d_gain = _sdf->Get<double>("thrust_d_gain");
    this->thrust_pid.SetDGain(this->thrust_d_gain);
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

  this->currentWaypoint = this->waypointOne;
  this->currentWaypointInt = 1;
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

  this->aoa_msg_sub_ = this->node->Subscribe("/aoa_topic",&WaypointFollowingControlPlugin::AoaMsgCallback, this);
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Listen to the Uodate event
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////


  EXPECT_TRUE(_sdf->HasElement("rudder_joint_name"));
  // Get the full name of the joint
  this->rudderJointName = _sdf->Get<std::string>("rudder_joint_name");
  this->rudderJointNameFull = this->model->GetJoint(this->rudderJointName)->GetScopedName();
  // get the needed jointptr from the Model (takes as input name of the joint, specified in the world file)
  this->rudder_joint = this->model->GetJoint(this->rudderJointNameFull);

  // set the limits for the rudder
  this->rudder_joint->SetUpperLimit(0, 0.17);//0.3 TODO NEED TO ADJUST THE LOOKUP TABLE FIRST!
  this->rudder_joint->SetLowerLimit(0,-0.17); //-0.3 NEED TO CAP IT OFF AT AOA THAT REACHES CLMAX RATHER THAN RUDDER DEFLECTION.

  EXPECT_TRUE(this->rudder_joint != NULL);

  // this->rudderJointController->AddJoint(this->rudder_joint);
  // // set the position targets
  // this->rudder_pos_target = 2.6;
  // this->rudderJointController->SetPositionTarget(this->rudderJointNameFull, this->rudder_pos_target);
  //
  // // Set the position PID controller (inouts: name of joint + common pid)
  // this->rudderJointController->SetPositionPID(this->rudderJointNameFull, common::PID(this->rudder_p_gain,this->rudder_i_gain,this->rudder_d_gain));


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
        << "WaypointFollowingControlPlugin will not generate forces\n";
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

  // get the first point
  this->lineStartingPoint = this->model->WorldPose().Pos();

  // Initialize renderable
  this->dynamicLine.Init(rendering::RENDERING_POINT_LIST,false); // 0 corresponds to RENDERING_POINT_LIST
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  updateIterationInt = 1;
}

////////////////////////////////////////////////////////////////////////////////
void WaypointFollowingControlPlugin::OnUpdate()
{
    gzdbg << "==============================================================\n";
    gzdbg << "**************************************************************\n";
    gzdbg << "OnUpdate Waypoint Following Plugin \n";
    std::cout << "Update iteration integer: " << updateIterationInt << std::endl;
    // Get the main body axis
    ignition::math::Pose3d modelPose = this->model->WorldPose();
    auto modelQuatern = modelPose.Rot();
    double modelRoll = modelQuatern.Roll();
    double modelPitch = modelQuatern.Pitch();
    double modelYaw = modelQuatern.Yaw();
    ignition::math::Vector3d vectorMainAxis(cos(modelYaw)*cos(modelPitch), sin(modelYaw)*cos(modelPitch), -1*sin(modelPitch)); //-1*sin(modelPitch)

    ////////////////////////////////////////////////////////////////////////////
    // Get the airspeed vector /////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    gzdbg << this->currentWaypoint << "\n";
    // Get the current Time
    this->currentTime = this->model->GetWorld()->SimTime();
    // get the groundspeed (get linear velocity at cp in inertial frame)
    #if GAZEBO_MAJOR_VERSION >= 9
      ignition::math::Vector3d groundSpeed = this->link->WorldLinearVel(this->cp);
      groundSpeed = this->link->WorldLinearVel(this->cp);
    #else
      ignition::math::Vector3d groundSpeed = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->cp));
      groundSpeed = this->link->WorldLinearVel(this->cp);
    #endif

    std::cout << "groundspeed " << groundSpeed << std::endl;




    // get the constant wind
    ignition::math::Vector3d constantWind(this->V_N_wind,this->V_E_wind,-1 * this->V_D_wind);
    // add it to the groundspeed to get the airspeed
    ignition::math::Vector3d airSpeed = groundSpeed - constantWind;

    // Normalize it
    ignition::math::Vector3d groundSpeed_normalized = groundSpeed;
    groundSpeed_normalized.Normalize();

    // Get the current pose of the aircraft
    ignition::math::Pose3d currentPose = this->link->WorldCoGPose();

    std::cout << "currentPose of main wing: " << currentPose << std::endl;
    // Extract the location from the pose
    ignition::math::Vector3d currentLocation = currentPose.Pos();
    std::cout << "currentLocation of main wing: " << currentLocation << std::endl;

    // Get the vector connecting the current location of the aircraft
    ignition::math::Vector3d difference = this->currentWaypoint.operator-(currentLocation);
    std::cout << "this->currentWaypoint: " << this->currentWaypoint << std::endl;
    std::cout << "Difference " << difference << std::endl;

    double differenceLength = difference.Length();
    // Normalize this vector as well
    difference.Normalize();

    // Get the angle between the difference vector and the airSpeed vector
    double dot_product = difference.Dot(vectorMainAxis.Normalize()); //difference.Dot(groundSpeed_normalized);
    this->heading_error = acos(dot_product); // as desired will be the one less than 180 degrees.
    gzdbg << "heading error first " << this->heading_error*180/M_PI << "\n";
    // Let clockwise be positive and counterclockwise be negative.
    ignition::math::Vector3d cross_product = vectorMainAxis.Cross(difference);
    // check the z-component
    gzdbg << "cross product Z sign " << cross_product.Z() << " vectorMainAxis " << vectorMainAxis << " difference " << difference << "\n";
    if (cross_product.Z() > 0) // will be counterclockwise so 1
    {
      gzdbg << "HERE will be negative" << "\n";
      this->heading_error = -1*this->heading_error; // Tif z pos, need pos rudder force, so need negative error.
    }


    // Get the speed speed_error
    this->speed_error = -1*(this->forwardSpeed - groundSpeed.Length()); // multiplied by minus 1 because the PID controller in the API takes the error not as ref minus current state.

    std::cout << "groundSpeed: " << groundSpeed << std::endl;

    // Update
    if (1)
    {
      // std::cout << "rudderJointName " << this->rudderJointName << std::endl;
      // std::cout << "rudderJointNameFull " << this->rudderJointNameFull << std::endl;
      // std::cout << "propellerJointName " << this->propellerJointName << std::endl;
      // std::cout << "propellerJointNameFull " << this->propellerJointNameFull << std::endl;

    }


    ////////////////////////////////////////////////////////////////////////////
    // Update the state int if the waypoint is reached /////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    std::cout << "difference.Length() " << differenceLength << std::endl;
    if (differenceLength < this->threshold){
      std::cout << "SWITCHING WAYPOINTS" << std::endl;

      if (this->currentWaypointInt == 1)
      {
        this->currentWaypoint = this->waypointTwo;
      }

      if (this->currentWaypointInt == 2)
      {
        this->currentWaypoint = this->waypointThree;
      }

      if (this->currentWaypointInt == 3)
      {
        this->currentWaypoint = this->waypointFour;
      }

      if (this->currentWaypointInt == 4)
      {
        this->currentWaypoint = this->waypointOne;
      }

      this->currentWaypointInt++;
    }
    std::cout << "the difference is: " << differenceLength << std::endl;
    std::cout << "the current waypoint integer is: " << this->currentWaypointInt << std::endl;
    std::cout << "the current waypoint is: " << this->currentWaypoint << std::endl;
    ////////////////////////////////////////////////////////////////////////////
    // get the dt

    this->_dt = (this->currentTime - this->lastControllerUpdateTime).Double();

    ////////////////////////////////////////////////////////////////////////////
    // Get the force that needs to be applied, from the PID ////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // RUDDER
    this->rudder_pos_target = this->rudder_pid.Update(this->heading_error,this->_dt);
    // if (updateIterationInt == 1) // since no alpha yet
    // {
      if (this->rudder_pos_target > 0.17){
        std::cout << "this->rudder_pos_target was " << this->rudder_pos_target << std::endl;
        this->rudder_pos_target = 0.17; // brute force because set upper limit was not working
      }
      else{
        std::cout << "this->rudder_pos_target was " << this->rudder_pos_target << std::endl;
        this->rudder_pos_target = -0.17;
      }
    // }
    // else{
    //   if (this->alpha > 0.17 || this->alpha < -0.17){
    //
    //     gzdbg << "target will be alterred \n";
    //     if (alpha > 0)
    //     {
    //       this->diff = this->alpha - 0.17;
    //     }
    //     else{
    //       this->diff = -0.17 - this->alpha;
    //     }
    //     gzdbg << "the diff " << this->diff << "\n";
    //     this->rudder_pos_target = this->previous_rudder_pos_target - this->diff;
    //   }
    //   else{
    //     this->rudder_pos_target = this->rudder_pid.Update(this->heading_error,this->_dt);
    //   }
    // }





    // // TODO check the SetUpperLimit functionality why it is not working. lines below circumvent this issue.






    gzdbg << "this->rudder_pos_target " << this->rudder_pos_target << "\n";
    // PROPELLER
    // this->force_to_apply_to_propeller = this->propeller_pid.Update(this->speed_error,this->_dt);
    // THRUST
    this->force_to_apply_to_body =  this->thrust_pid.Update(this->speed_error, this->_dt);


    ////////////////////////////////////////////////////////////////////////////
    // Apply the forces ////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // RUDDER
    this->rudder_joint->SetPosition(0,this->rudder_pos_target);  // this->rudder_joint->SetForce(0,this->force_to_apply_rudder);

    // PROPELLER
    // this->propeller_joint->SetForce(0,this->force_to_apply_to_propeller);

    // APPLY THE FORCE TO THE MAIN BODY
    // multiply the vector by a magnitude
    std::cout << this->force_to_apply_to_body << std::endl;
    // std::cout << "force magnitude applied to the main body " << this->force_to_apply_to_body << std::endl;
    ignition::math::Vector3d forceVectorWorld = this->force_to_apply_to_body * vectorMainAxis;

    // convert to body frame
    ignition::math::Pose3d modelPoseInverse = modelPose.Inverse();
    std::cout << "current time " << this->lastControllerUpdateTime.Double() << "\n";
    gzdbg << "main axis of the model is " << vectorMainAxis << "(computed in WaypointFollowing) \n";
    std::cout << "modelPoseInverse: " << modelPoseInverse << std::endl;

    // rotate forward and upward vectors into inertial frame
    ignition::math::Vector3d forceVector = forceVectorWorld;//modelPoseInverse.Rot().RotateVector(forceVectorWorld);


    gzdbg << "Thrust force: " << forceVector << "\n";

    std::cout << "BEFORE ADDING THE FORCE: this->link->WorldForce(): " << this->link->WorldForce() << std::endl;
    // std::transform(vectorMainAxis.begin(), vectorMainAxis.end(), forceVector.begin(), std::bind1st(std::multiplies<float>(), this->force_to_apply_to_body));

    this->link->AddForceAtWorldPosition(ignition::math::Vector3d(this->force_to_apply_to_body,0,0), this->link->WorldCoGPose().Pos()); // was //AddForce(forceVector)

    std::cout << "this->forwardSpeed "<<  this->forwardSpeed  << " groundSpeed.Length() " << groundSpeed.Length() << "this->speed_error: " << this->speed_error << std::endl;
    std::cout << "this->heading_error: " << this->heading_error*180.0/M_PI << std::endl;
    std::cout << "this->rudder_pos_target: " << this->rudder_pos_target << std::endl;
    std::cout << "this->force_to_apply_to_propeller: " << this->force_to_apply_to_propeller << std::endl;
    std::cout << "this->force_to_apply_to_body: " << this->force_to_apply_to_body << std::endl;
    std::cout << "this->link->WorldForce(): " << this->link->WorldForce() << std::endl;
    std::cout << "this->rudder_joint->Position(): " << this->rudder_joint->Position() << std::endl;

    ////////////////////////////////////////////////////////////////////////////
    // Drawing time ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    this->lineEndPoint = this->model->WorldPose().Pos();
    const std::string nameLine = "trajectory";

    std::cout << "this->lineStartingPoint: " << this->lineStartingPoint << std::endl;
    std::cout << "this->lineEndPoint: " << this->lineEndPoint << std::endl;
    this->dynamicLine.AddPoint(this->lineEndPoint);//this->scene->DrawLine(this->lineStartingPoint, this->lineEndPoint, "nameLine");
    int count = this->dynamicLine.GetPointCount();
    this->dynamicLine.setMaterial("Gazebo/Purple");
    this->dynamicLine.setVisible(true);
    this->dynamicLine.setVisibilityFlags(GZ_VISIBILITY_GUI);
    bool isVisible = this->dynamicLine.getVisible();
    std::cout << "count is: " << count << std::endl;
    std::cout << "isVisible: " << isVisible << std::endl;

    // this->dynamicLine.SetColor(count, ignition::math::Color(1, 1, 0.5, 1.0));
    // this->dynamicLine.Update();

    this->lineStartingPoint = this->lineEndPoint;
    ////////////////////////////////////////////////////////////////////////////
    // Get the time for the next update loop ///////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    this->lastControllerUpdateTime = this->model->GetWorld()->SimTime();

    updateIterationInt++;

    this->previous_rudder_pos_target = this->rudder_pos_target;

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

void WaypointFollowingControlPlugin::AoaMsgCallback(AoaMsgPtr &aoa_msg){
  this->alpha = aoa_msg->double_value();
  gzdbg << "this->alpha " << this->alpha << "\n";

}
