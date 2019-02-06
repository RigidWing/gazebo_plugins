#include "verify_liftdrag_plugin.hh"
#include <algorithm>
#include <string>
#include <iostream>
#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "common.h"
#include <string>
#include <cstring>
#include <sstream>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(VerifyLiftdragPlugin)

VerifyLiftdragPlugin::VerifyLiftdragPlugin()
{
  this->wind_condition_iterator = 0;
}

VerifyLiftdragPlugin::~VerifyLiftdragPlugin()
{

}

////////////////////////////////////////////////////////////////////////////////

void VerifyLiftdragPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{

  //////////////////////////////////////////////////////////////////////////////
  // Load parameters from the SDF //////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Type of test:
  // 0:Change azimuth
  // 1:Change elevation
  // 2:Change magnitude

  GZ_ASSERT(_sdf->Get<int>("type_of_test"),"The test type should be specified.");



  this->type_of_test = _sdf->Get<int>("type_of_test");

  if(this->type_of_test == 0)
  {
    std::cout << "Azimuth test" << std::endl;
    this->start = _sdf->Get<double>("start_azimuth");
    this->end = _sdf->Get<double>("end_azimuth");
    this->step = _sdf->Get<double>("step_azimuth");

    this->second_spawn_arg = _sdf->Get<double>("elevation");
    this->third_spawn_arg = _sdf->Get<double>("magnitude");
  }
  else if(this->type_of_test == 1)
  {
    std::cout << "Elevation test" << std::endl;
    this->start = _sdf->Get<double>("start_elevation");

    this->end = _sdf->Get<double>("end_elevation");
    this->step = _sdf->Get<double>("step_elevation");
    gzdbg << "Over here " << "\n";

    this->second_spawn_arg = _sdf->Get<double>("azimuth");
    this->third_spawn_arg = _sdf->Get<double>("magnitude");
  }
  else if(this->type_of_test == 2)
  {
    std::cout << "Magnitude test" << std::endl;
    this->start = _sdf->Get<double>("start_magnitude");
    this->end = _sdf->Get<double>("end_magnitude");
    this->step = _sdf->Get<double>("step_magnitude");

    this->second_spawn_arg = _sdf->Get<double>("azimuth");
    this->third_spawn_arg = _sdf->Get<double>("elevation");
  }
  else
  {
    gzerr << "The type of test is incorrect. \n";
  }

  //////////////////////////////////////////////////////////////////////////////
  // Initialize transport //////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  GZ_ASSERT(_model, "VerifyLiftdragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "VerifyLiftdragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  if (_sdf->HasElement("joint_name"))
  {
    // Get the full name of the joint
    this->jointName = _sdf->Get<std::string>("joint_name");
    this->jointNameFull = this->model->GetJoint(jointName)->GetScopedName();

    // get the needed jointptr from the Model (takes as input The name of the joint, specified in the world file)
    this->joint = this->model->GetJoint(this->jointName);
    EXPECT_TRUE(this->joint != NULL);
  }
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();


  // Get the force from the sensor
  // force_sub_ = node_handle_->Subscribe<msgs::WrenchStamped>("/gazebo/default/rigid_wing/rigid_wing/fixed_to_world/main_force_torque/wrench",&VerifyLiftdragPlugin::ForceMsgCallback, this);
  //////////////////////////////////////////////////////////////////////////////
  //update connection ////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  // Create the file needed for logging
  std::freopen("verify_liftdrag.txt","w", stdout);


  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VerifyLiftdragPlugin::OnUpdate, this));

  // Initialize the control variable
  this->control_variable = this->start;
  this->test_number = 0;



}

void VerifyLiftdragPlugin::OnUpdate()
{
  gzdbg << "On update inside \n";

  std::cout << "Time " << common::Time::GetWallTimeAsISOString() << std::endl;

  std::cout << "OnUpdate of Verify Lift Drag Plugin \n";
  std::cout << "The control variable: " << this->control_variable << std::endl;

  std::cout << "Change of control variable iterator" << std::endl;
  if(this->type_of_test == 0) // azimuth test
  {
    this->spawn_args[0] = std::to_string(this->third_spawn_arg); //magnitude
    this->spawn_args[1] = std::to_string(this->control_variable); //azimuth
    this->spawn_args[2] = std::to_string(this->second_spawn_arg); //elevation
  }
  else if(this->type_of_test == 1) // elevation test
  {
    this->spawn_args[0] = std::to_string(this->third_spawn_arg); //magnitude
    this->spawn_args[1] = std::to_string(this->second_spawn_arg); //azimuthf
    this->spawn_args[2] = std::to_string(this->control_variable);//elevation
  }
  else // magnitude test
  {
    this->spawn_args[0] = std::to_string(this->control_variable); //magnitude
    this->spawn_args[1] = std::to_string(this->second_spawn_arg); //azimuth
    this->spawn_args[2] = std::to_string(this->third_spawn_arg); //elevation
  }

  std::string ss;
  ss = "./build/send_commandline_protobuf " + spawn_args[0] + " " + spawn_args[1] + " " + spawn_args[2];

  system(ss.c_str());

  this->jointWrench = this->joint->GetForceTorque(0u);
  this->force_X = this->jointWrench.body1Force.X();
  this->force_Y = this->jointWrench.body1Force.Y();
  this->force_Z = this->jointWrench.body1Force.Z();


  // update the control variable (Note that a wait of two update loops is needed to get to the correct values)
  if (this->wind_condition_iterator == 2){
    this->control_variable = this->control_variable + this->step;

    // Log
    std::cout << "Iterator: " << this->wind_condition_iterator << std::endl;
    std::cout << "Test: " << this->test_number << std::endl;
    std::cout << "Magnitude: " << this->spawn_args[0] << std::endl;
    std::cout << "Azimuth: " << this->spawn_args[1] << std::endl;
    std::cout << "Elevation: " << this->spawn_args[2] << std::endl;
    std::cout << "Force x: " << this->force_X << std::endl;
    std::cout << "Force y: " << this->force_Y << std::endl;
    std::cout << "Force z: " << this->force_Z << std::endl;
    gzdbg << "Something after \n";

    this->wind_condition_iterator = 0;
    this->test_number++;

    if (this->control_variable > this->end)
    {
      this->control_variable = this->start;
      this->test_number = 0;
      std::cout << "New batch. " << std::endl;
    }
  }
  else
  {
    this->wind_condition_iterator++;
  }


}


////////////////////////////////////////////////////////////////////////////////
// Force callback function /////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// void VerifyLiftdragPlugin::ForceMsgCallback(WrenchStampedMsgPtr &wrench_stamped_msg){
//
//   std::cout << "Inside the callback function " << std::endl;
//
//
//
//   this->wrench_msg = wrench_stamped_msg->wrench();//this->joint->GetForce(0);
//   this->force_msg = this->wrench_msg.force();
//
//   this->force_X = this->force_msg.x();
//   this->force_Y = this->force_msg.y();
//   this->force_Z = this->force_msg.z();
//
//   this->test_number = 1;
//
//   // // Log
//   // std::cout << "Iterator: " << this->wind_condition_iterator << std::endl;
//   // std::cout << "Magnitude: " << this->spawn_args[0] << std::endl;
//   // std::cout << "Azimuth: " << this->spawn_args[1] << std::endl;
//   // std::cout << "Elevation: " << this->spawn_args[2] << std::endl;
//   // std::cout << "Force x " << this->force_X << std::endl;
//   // std::cout << "Force y " << this->force_Y << std::endl;
//   // std::cout << "Force z " << this->force_Z << std::endl;
//
// }
