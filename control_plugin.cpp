#include <control_plugin.hh>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

////////////////////////////////////////////////////////////////////////////////
ControlPlugin::ControlPlugin()
{
  // Initialize the PID parameters
  this->elevator_pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);
  this->rudder_pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);

}

////////////////////////////////////////////////////////////////////////////////
ControlPlugin::~ControlPlugin()
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

  // Overload the Elevator PID parameters if available
  if (_sdf->HasElement("elevator_p_gain"))
  {
      this->elevator_p_gain = _sdf->Get<double>("elevator_p_gain");
  }

  if (_sdf->HasElement("elevator_i_gain"))
  {
      this->elevator_i_gain = _sdf->Get<double>("elevator_i_gain");
  }

  if (_sdf->HasElement("elevator_d_gain"))
  {
      this->elevator_d_gain = _sdf->Get<double>("elevator_d_gain");
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


  // TODO ISABELLE: CHECK IF THE CONDITIONAL ON THE LINK-NAME HERE IS NEEDED
  if (_sdf->HasElement("elevator_joint_name"))
  {
    this->elevatorJointController.reset(new physics::JointController(this->model));

    // Get the full name of the joint
    this->elevatorJointName = _sdf->Get<std::string>("elevator_joint_name");
    this->elevatorJointNameFull = this->model->GetJoint(elevatorJointName)->GetScopedName();

    // get the needed jointptr from the Model (takes as input The name of the joint, specified in the world file)
    this->elevator_joint = this->model->GetJoint(this->elevatorJointNameFull); // was elevatorJointName
    EXPECT_TRUE(this->elevator_joint != NULL);


    // use AddJoint which takes as input a JointPtr
    this->elevatorJointController->AddJoint(this->elevator_joint);

    // set the position targets
    this->elevator_pos_target = 0.3;
    this->elevatorJointController->SetPositionTarget(this->elevatorJointNameFull, this->elevator_pos_target); // TODO ISABELLE needs to be moved away

    // Set the position PID controller (inouts: name of joint + common pid)
    this->elevatorJointController->SetPositionPID(this->elevatorJointNameFull, common::PID(this->elevator_p_gain,this->elevator_i_gain,this->elevator_d_gain));
  }

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
  // to be removed
  // this->list_joint_ptrs = this->elevatorJointController->GetJoints();

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
}

////////////////////////////////////////////////////////////////////////////////
void ControlPlugin::OnUpdate()
{
    // for(auto elem : this->list_joint_ptrs)
    // {
    //    gzdbg << elem.first << " " << elem.second << "\n";
    //
    //    physics::JointPtr elevator_joint_ptr = elem.second;
    //
    //    gzdbg << "Position " << elevator_joint_ptr->Position() << "\n";
    // }
    //
    // gzdbg << this->elevator_joint << "\n";


    // gzdbg << "BEFORE UPDATE this->elevator_joint->Position(0)" << this->elevator_joint->Position(0) << "\n";

    this->elevatorJointController->Update();
    this->rudderJointController->Update();

    if (1)
    {
      // gzdbg << "NON SCOPED " << this->elevatorJointName << "\n";
      // gzdbg << "SCOPED " << this->model->GetJoint(elevatorJointName)->GetScopedName() << "\n";
      // gzdbg << "The elevator target position is " << this->elevator_pos_target << "\n";
      gzdbg << "The rudder target position " << this->rudder_pos_target << "\n";
      gzdbg << "rudderJointName " << this->rudderJointName << "\n";
      gzdbg << "rudderJointNameFull " << this->rudderJointNameFull << "\n";
      // gzdbg << " JUST AFTER UPDATE: this->elevator_joint->Position(0)" << this->elevator_joint->Position(0) << "\n";
      // gzdbg << " JUST AFTER UPDATE: this->rudder_joint->Position(0)" << this->rudder_joint->Position(0) << "\n";
      // gzdbg << "elevator Force : " << this->elevator_joint->GetForce(0) << "\n";
      gzdbg << "rudder Force : " << this->rudder_joint->GetForce(0) << "\n";

      // for(auto elem : this->list_joint_ptrs)
      // {
      //    gzdbg << elem.first << " " << elem.second << "\n";
      //
      //    physics::JointPtr elevator_joint_ptr = elem.second;
      //    gzdbg << "get the force " << elevator_joint_ptr->GetForce(0) << "\n";
      //
      //
      // }

    }

    gzdbg << " AFTER UPDATE: this->elevator_joint->Position(0)" << this->elevator_joint->Position(0) << "\n";
    gzdbg << " AFTER UPDATE: this->rudder_joint->Position(0)" << this->rudder_joint->Position(0) << "\n";


}

////////////////////////////////////////////////////////////////////////////////
void ControlPlugin::GetControlTargets(ControlTargetMsgPtr &control_targets_msg)
{
  // todo ISABELLE: make general
  // this->elevator_pos_target = 0.1;//control_targets_msg->x();
  // this->rudder_pos_target = 0.2;//control_targets_msg->y();
}
