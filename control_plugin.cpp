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
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Obtain the SDF Parameters
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  // Overload the Rudder PID parameters if available
  if (_sdf->HasElement("rudder_p_gain"))
  {
      this->rudder_pid.SetPGain(_sdf->Get<double>("rudder_p_gain"));
  }

  if (_sdf->HasElement("rudder_i_gain"))
  {
      this->rudder_pid.SetIGain(_sdf->Get<double>("rudder_i_gain"));
  }

  if (_sdf->HasElement("rudder_d_gain"))
  {
      this->rudder_pid.SetDGain(_sdf->Get<double>("rudder_d_gain"));
  }

  // Overload the Elevator PID parameters if available
  if (_sdf->HasElement("elevator_p_gain"))
  {
      this->elevator_pid.SetPGain(_sdf->Get<double>("elevator_p_gain"));
  }

  if (_sdf->HasElement("elevator_i_gain"))
  {
      this->elevator_pid.SetIGain(_sdf->Get<double>("elevator_i_gain"));
  }

  if (_sdf->HasElement("elevator_d_gain"))
  {
      this->elevator_pid.SetDGain(_sdf->Get<double>("elevator_d_gain"));
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Get the Controller time control.
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  this->lastControllerUpdateTime = this->model->GetWorld()->SimTime();
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Listen to the Uodate event
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  // TODO ISABELLE: CHECK IF THE CONDITIONAL ON THE LINK-NAME HERE IS NEEDED
  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");

    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    //GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. \n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ControlPlugin::OnUpdate, this));
    }
  }

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

}

////////////////////////////////////////////////////////////////////////////////
void ControlPlugin::OnUpdate()
{
  common::Time current_time = this->model->GetWorld()->SimTime();

  if (current_time > this->lastControllerUpdateTime){

    // Get the dt
    double _dt = (current_time - this->lastControllerUpdateTime).Double();
    // Obtain the current position of the control surfaces
    double elevator_pos = this->elevator_joint->Position(0); // why position(0)? axis
    double rudder_pos = this->rudder_joint->Position(0);
    // Compute the error in the position of the control surfaces
    double elevator_pos_error = this->elevator_pos_target - elevator_pos;
    double rudder_pos_error = this->rudder_pos_target - rudder_pos;
    // Use the PID control to calculate the control force
    double elevator_force = this->elevator_pid.Update(elevator_pos_error, _dt);
    double rudder_force = this->rudder_pid.Update(rudder_pos_error, _dt);
    // Apply the force on the control surface joint
    this->elevator_joint->SetForce(0, elevator_force);
    this->rudder_joint->SetForce(0, rudder_force);

  }
  // Update the time to the current time
  this->lastControllerUpdateTime = current_time;
}

////////////////////////////////////////////////////////////////////////////////
void ControlPlugin::GetControlTargets(ControlTargetMsgPtr &control_targets_msg)
{
  // todo ISABELLE: make general
  this->elevator_pos_target = 0.1;//control_targets_msg->x();
  this->rudder_pos_target = 0.2;//control_targets_msg->y();
}
