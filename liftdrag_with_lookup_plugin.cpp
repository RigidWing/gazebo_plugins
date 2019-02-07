/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <algorithm>
#include <string>
#include<string.h>
#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "liftdrag_plugin/liftdrag_with_lookup_plugin.h"
// KITEPOWER (Xander)
#include "common.h"

#include <iostream>


bool compute_values = 1;

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragWithLookupPlugin)

/////////////////////////////////////////////////
LiftDragWithLookupPlugin::LiftDragWithLookupPlugin() : cla(1.0), cda(0.01), cma(0.01), rho(1.2041)
{
  this->cp = ignition::math::Vector3d(0, 0, 0);
  this->forward = ignition::math::Vector3d(1, 0, 0); // TODO CHECK IF THIS IS CUMBERSOME
  this->upward = ignition::math::Vector3d(0, 0, 1);
  this->area = 1.0;
  this->alpha0 = 0.0;
  this->alpha = 0.0;
  this->sweep = 0.0;
  this->velocityStall = 0.0;

  // 90 deg stall
  this->alphaStall = 0.5*M_PI;
  this->claStall = 0.0;

  this->radialSymmetry = false;

  /// \TODO: what's flat plate drag?
  this->cdaStall = 1.0;
  this->cmaStall = 0.0;

  /// how much to change CL per every radian of the control joint value
  this->controlJointRadToCL = 4.0;

  /// KITEPOWER
  this->azimuth_wind			= 0.0; // [rad]
  this->vel_wind			= 0.0; // [m/s]
  this->V_N_wind = 0.0;
  this->V_E_wind = 0.0;
  this->V_D_wind = 0.0;
  this->wind_field_sub_topic_		= kDefaultWindFieldSubTopic;
  this->namespace_			= "";
  this->useConstantDragCoefficient	= true;
}

/////////////////////////////////////////////////
LiftDragWithLookupPlugin::~LiftDragWithLookupPlugin()
{
}

/////////////////////////////////////////////////
void LiftDragWithLookupPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragWithLookupPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragWithLookupPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);



  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragWithLookupPlugin world pointer is NULL");

  #if GAZEBO_MAJOR_VERSION >= 9
    this->physics = this->world->Physics();
  #else
    this->physics = this->world->GetPhysicsEngine();
  #endif
  GZ_ASSERT(this->physics, "LiftDragWithLookupPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "LiftDragWithLookupPlugin _sdf pointer is NULL");

  //////////////////////////////////////////////////////////////////////////////
  //(KITEPOWER)
  if (_sdf->HasElement("airfoilDatafilePath")){
    compute_values = 0;
    airfoilDatafilePath = _sdf->Get<std::string>("airfoilDatafilePath");

    // Open the file using the retrieved filepath
    ifstream inFile;
    try {
      inFile.open(airfoilDatafilePath.c_str(), ios_base::in);
    }
    catch (std::ifstream::failure e) {
      std::cerr << "Exception opening file: " << std::strerror(errno) << "\n";
    }
    GZ_ASSERT(inFile.is_open(),"Could not open the airfoil data file!");

    // PARSE THE AIRFOIL DATA FILE
    while (inFile)
    {
      gzdbg << "==============================================================\n";
      gzdbg << "Parsing the airfoil data. \n";
      counter++;
      inFile.getline(oneline, MAXLINE);
      line = oneline;
      // Starting from line 13, the data begins
      if (counter >= 13 && counter < 300)
      {
        alpha_str = line.substr(0,8);
        alpha = atof(alpha_str.c_str());

        if (alpha_str.length() > 0)
        {
          if (counter == 13)
          {
            gzdbg << "First alpha is: " << alpha << "\n";
          }
          else
          {
            gzdbg << "Alpha is " << alpha_str <<"\n";
          }

          cl_str = line.substr(10,7);
          gzdbg << "Cl is " << cl_str << "\n";
          cl = atof(cl_str.c_str());


          cd_str = line.substr(20,7);
          gzdbg << "Cd is " << cd_str << "\n";
          cd = atof(cd_str.c_str());


          cdp_str = line.substr(30,7);
          gzdbg << "Cdp is " << cdp_str << "\n";
          cdp = atof(cdp_str.c_str());


          cm_str = line.substr(39,7);
          gzdbg << "Cm is " << cm_str << "\n";
          cm = atof(cm_str.c_str());


          top_xtr_str = line.substr(49,6);
          top_xtr = atof(top_xtr_str.c_str());

          bot_xtr_str = line.substr(58,6);
          bot_xtr = atof(bot_xtr_str.c_str());

          top_itr_str = line.substr(66,7);
          top_itr = atof(top_itr_str.c_str());


          bot_itr_str = line.substr(74,8);
          bot_itr = atof(bot_itr_str.c_str());

          // APPEND THE VALUES OBTAINED FROM THE FILE TO VECTORS
          alpha_vec.push_back(alpha);
          cl_vec.push_back(cl);
          cd_vec.push_back(cd);
          cm_vec.push_back(cm);
        }
      }
    }
    // CLOSE THE FILE
    inFile.close();
  }

  gzdbg << "==============================================================\n";
  gzdbg << "Values got appended correctly? \n";
  // gzdbg << "Cl vector " << cl_vec << "\n";
  // gzdbg << "Cd vector " << cd_vec << "\n";
  // gzdbg << "Cm vector " << cm_vec << "\n";
  // CONTINUE CHECKING FOR THE OTHER PARAMETERS
  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("a0"))
    this->alpha0 = _sdf->Get<double>("a0");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->cmaStall = _sdf->Get<double>("cma_stall");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
  this->upward.Normalize();

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    gzdbg << "There is the tag link_name. " << "\n";
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzdbg << "link is not obtained so OnUpdate will not be accessed. " << "\n";
      gzerr << "Link with name[" << linkName << "] not found. "
        << "LiftDragWithLookupPlugin will not generate forces\n";
    }
    else
    {
      gzdbg << "link is obtained!. " << "\n";
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&LiftDragWithLookupPlugin::OnUpdate, this));

    }
  }

  if (_sdf->HasElement("control_joint_name"))
  {
    std::string controlJointName = _sdf->Get<std::string>("control_joint_name");
    this->controlJoint = this->model->GetJoint(controlJointName);
    if (!this->controlJoint)
    {
      gzerr << "Joint with name[" << controlJointName << "] does not exist.\n";
    }
  }

  if (_sdf->HasElement("control_joint_rad_to_cl"))
    this->controlJointRadToCL = _sdf->Get<double>("control_joint_rad_to_cl");

  // // KITEPOWER
  // if (_sdf->HasElement("robotNamespace"))
  //   namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  // else
  //   gzerr << "Please specify a robotNamespace.\n";
  // node_handle_ = transport::NodePtr(new transport::Node());
  // node_handle_->Init(namespace_);

  // KITEPOWER
  if (_sdf->HasElement("useConstantDragCoefficient"))
  {
    this->useConstantDragCoefficient = _sdf->Get<bool>("useConstantDragCoefficient");
  }

  // ISABELLE

  if (_sdf->HasElement("link_world_velocity"))
  {
    // this->groundspeed_world = _sdf->Get<ignition::math::Vector3d>("link_world_velocity");
    // this->model->GetLink("rigid_wing::main_wing")->SetLinearVel({groundspeed_world.X(), groundspeed_world.Y(), groundspeed_world.Z()});
  }

  if (_sdf->HasElement("chord"))
  {
    this->chord = _sdf->Get<double>("chord");
  }


  //////////////////////////////////////////////////////////////////////////////
  // check if the link is rudder
  gzdbg << "this->link->GetName(): " << this->link->GetName() << "\n";

  string dummy_var = "rigid_wing::rudder";
  gzdbg << dummy_var.substr(12,6) << "\n";
  string linkName = this->link->GetName().c_str();

  string dummy_var_sub = dummy_var.substr(12,6);
  string linkName_sub = linkName.substr(12,6);

  gzdbg << "dummy_var " << dummy_var << "\n";
  gzdbg << "linkName " << linkName << "\n";

  gzdbg << "__cplusplus " << __cplusplus << "\n";
  gzdbg << "here " << linkName.compare(dummy_var) << "\n";
  // result of zero means they are equal

  // result of zero means they are equal
  if ( linkName.compare(dummy_var) == 0 )
  {
    gzdbg << "Inside the loop" << "\n";
    // string rudderJointNameFull = this->model->GetJoint("rudder_joint")->GetScopedName();
    this->rudderJointName = _sdf->Get<std::string>("rudder_joint_name");
    this->rudderJointNameFull = this->model->GetJoint(this->rudderJointName)->GetScopedName();
    gzdbg << "rudderJointNameFull " << rudderJointNameFull << "\n";
    this->rudder_joint = this->model->GetJoint(rudderJointNameFull);
  }
  //getSdfParam<std::string>(_sdf, "windFieldSubTopic", wind_field_sub_topic_, wind_field_sub_topic_);
  //wind_field_sub_ = node_handle_->Subscribe<wind_field_msgs::msgs::WindField>("~/" + this->model->GetName() + wind_field_sub_topic_, &LiftDragWithLookupPlugin::WindFieldCallback, this);
  wind_field_sub_ = node_handle_->Subscribe<wind_field_msgs::msgs::WindField>(wind_field_sub_topic_, &LiftDragWithLookupPlugin::WindFieldCallback, this);
  // An Additional SubscriberPtr To Subscribe to the test_msg Topic
  test_msg_sub_ = node_handle_->Subscribe<msgs::Vector3d>("/test_topic",&LiftDragWithLookupPlugin::TestMsgCallback, this);

  aoa_pub_ = node_handle_->Advertise<msgs::Any>("/aoa_topic",1);
}

/////////////////////////////////////////////////
void LiftDragWithLookupPlugin::OnUpdate()
{

  cout << "OnUpdate of Lift Drag Plugin " << endl;
  gzdbg << "==============================================================\n";
  gzdbg << "**************************************************************\n";
  gzdbg << "OnUpdate LiftDrag Plugin \n";
  GZ_ASSERT(this->link, "Link was NULL");

  // Get the main body axis
  ignition::math::Pose3d modelPose = this->model->WorldPose();
  auto modelQuatern = modelPose.Rot();
  double modelRoll = modelQuatern.Roll();
  double modelPitch = modelQuatern.Pitch();
  double modelYaw = modelQuatern.Yaw();
  ignition::math::Vector3d vectorMainAxisModel(cos(modelYaw)*cos(modelPitch), sin(modelYaw)*cos(modelPitch), -1*sin(modelPitch));
  gzdbg << "main axis of the model is " << vectorMainAxisModel << "(computed in LiftDrag plugin) \n";
  // CHECK THE LINK NAME
  gzdbg << "this->link->GetName(): " << this->link->GetName() << "\n";

  string dummy_var = "rigid_wing::rudder";
  string linkName = this->link->GetName().c_str();
  // COMPARE THE STRINGS (RESULT OF ZERO MEANS THEY ARE SAME STRING)
  if ( linkName.compare(dummy_var) == 0 )
  {
    gzdbg << "rudder_joint->Position()" << this->rudder_joint->Position() << "\n";
    if (this->rudder_joint->Position() >= 0)
    {
      gzdbg << "Rudder position is positive so flip upward! \n";
      this->upward = ignition::math::Vector3d(0,-1, 0);;//-1*this->upward;
      gzdbg << "this->upward " << this->upward << "\n";
    }

  }

//  // ISABELLE hack the world velocity
// this->model->GetLink("rigid_wing::main_wing")->SetLinearVel({groundspeed_world.X(), groundspeed_world.Y(), groundspeed_world.Z()});

  // get linear velocity at cp in inertial frame
#if GAZEBO_MAJOR_VERSION >= 9 // NOTE KEEP AS MAIN WING OTHERWISE HAS TROUBLE GETTING VELOCITY
ignition::math::Vector3d vel = this->model->GetLink("rigid_wing::main_wing")->WorldLinearVel(this->cp);
#else
ignition::math::Vector3d vel = ignitionFromGazeboMath(this->model->GetLink("rigid_wing::main_wing")->WorldLinearVel(this->cp);
#endif
  ignition::math::Vector3d velI = vel;

  // gzdbg << "GROUNDSPEED " << velI << "\n";

  // FTERO (jonas) & KITEPOWER (Xander)
  // start ---
  //ignition::math::Vector3d constantWind(this->vel_wind*cos(this->azimuth_wind),this->vel_wind*sin(this->azimuth_wind),0);
  ignition::math::Vector3d constantWind(this->V_N_wind,this->V_E_wind,-1 * this->V_D_wind); // minus 1 because NED to world
  vel -= constantWind; // where vel is the ground speed of the vehicle. In which reference frame is vel defined? In the world coordinates.
  //so changed to minus 1, because of a redefinition of the constant wind Vector. Now the resultant vel is the airspeed. Airspeed defined in the world coordinate system.


  // end   ---
  velI.Normalize();
  // smoothing
  // double e = 0.8;
  // this->velSmooth = e*vel + (1.0 - e)*velSmooth;
  // vel = this->velSmooth;

  if (vel.Length() <= 0.01)
    return;

  // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

  // rotate forward and upward vectors into inertial frame
  ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);


  ignition::math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    ignition::math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
  ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;

  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = ignition::math::clamp(
      spanwiseI.Dot(velI), minRatio, maxRatio);

  // get cos from trig identity
  double cosSweepAngle = sqrt(1.0 - sinSweepAngle * sinSweepAngle); //iso sqrt
  this->sweep = asin(sinSweepAngle);
  // truncate sweep to within +/-90 deg
  while (fabs(this->sweep) > 0.5 * M_PI)
    this->sweep = this->sweep > 0 ? this->sweep - M_PI
                                  : this->sweep + M_PI;

  // angle of attack is the angle between velI projected into lift-drag plane
  //  and forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI; //why velI? (Isabelle) changed velI to spanwiseI

  // (ISABELLE)

  // ignition::math::Vector3d velInPlaneAxisOfSymmetry = vel - vel.Dot()*spa

  // get direction of drag
  ignition::math::Vector3d dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
  liftI.Normalize();

  // get direction of moment
  ignition::math::Vector3d momentDirection = spanwiseI;

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Lenghth())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  // gzdbg << constantWind << "\n";
  // gzdbg << "upwardI" << upwardI << "\n";
  // gzdbg << "spanwiseI" << spanwiseI << "\n";
  // gzdbg << "Vel " << vel << "\n";
  // gzdbg << "velInLDPlane" << velInLDPlane << "\n";
  // gzdbg << "liftI which is the direction of lift" << liftI << "\n";
  // gzdbg << liftI.Dot(upwardI) << "\n";



  double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);
  EXPECT_TRUE(isnan(cosAlpha) != 1);
  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  if (liftI.Dot(forwardI) >= 0.0) // changed forwardI to upwardI
    {
      // then alpha will be positive
      gzdbg << "Acute angle " << "\n";
      this->alpha = acos(cosAlpha); // this->alpha0 + acos(cosAlpha);  TODO ISABELLE, SEE IF THE ADDITION OF a0 is necessary? This is for the shift in the curve as made in the plugin
    }
  else
    {// alpha is negative
      gzdbg << "Obtuse angle " << "\n";
      this->alpha = -1 * acos(cosAlpha); //this->alpha0 - acos(cosAlpha); also added the abs there!to get the acute andle between then
    }
  // normalize to within +/-90 deg
  while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;



  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.Length();
  double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// (KITEPOWER)

if (compute_values == 1){
    // compute cl at cp, check for stall, correct for sweep

    // double cl;
    printf("INSIDE THE COMPUTE VALUES PORTION \n");

    // (ISABELLE) Add Alpha0 as required by the plugin.

    this->alpha_shifted = this->alpha + this->alpha0;

    if (this->alpha_shifted > this->alphaStall)
    {
      cl = (this->cla * this->alphaStall +
            this->claStall * (this->alpha_shifted - this->alphaStall))
           * cosSweepAngle;
      // make sure cl is still great than 0
      cl = std::max(0.0, cl);
    }
    else if (this->alpha_shifted < -this->alphaStall)
    {
      cl = (-this->cla * this->alphaStall +
            this->claStall * (this->alpha_shifted + this->alphaStall))
           * cosSweepAngle;
      // make sure cl is still less than 0
      cl = std::min(0.0, cl);
    }
    else
      cl = this->cla * this->alpha_shifted * cosSweepAngle;

      // cout << "Alpha shifted " << this->alpha_shifted << endl;
      // cout << this->alpha0 << endl;
      // cout  << "3 the cl value " << cl << endl;

    // modify cl per control joint value
    if (this->controlJoint)
    {
      #if GAZEBO_MAJOR_VERSION >= 9
          double controlAngle = this->controlJoint->Position(0);
      #else
          double controlAngle = this->controlJoint->GetAngle(0).Radian();
      #endif
          cl = cl + this->controlJointRadToCL * controlAngle;
      /// \TODO: also change cm and cd
      // cout  << "4 the cl value " << cl << endl;

    }

    // cout  << "the cl value " << cl << endl;
    ////////////////////////////////////////////////////////////////////////////
    // compute cd at cp, check for stall, correct for sweep
    // KITEPOWER: if useConstantDragCoefficient is true, computes the drag
    // coefficient based on the cda and cda_stall only without considering
    // an angle of attach of an airfoil

    // double cd;

    if (this->alpha_shifted > this->alphaStall)
    { // KITEPOWER
      if (!this->useConstantDragCoefficient)
        cd = (this->cda * this->alphaStall +
              this->cdaStall * (this->alpha_shifted - this->alphaStall))
             * cosSweepAngle;
      else
        cd = (this->cda + this->cdaStall) * cosSweepAngle;
    }
    else if (this->alpha_shifted < -this->alphaStall)
    { // KITEPOWER
      if (!this->useConstantDragCoefficient)
        cd = (-this->cda * this->alphaStall +
              this->cdaStall * (this->alpha_shifted + this->alphaStall))
             * cosSweepAngle;
      else
        cd = (-this->cda + this->cdaStall) * cosSweepAngle;
    }
    else // KITEPOWER
      if (!this->useConstantDragCoefficient)
        cd = (this->cda * this->alpha_shifted) * cosSweepAngle;
      else
        cd = this->cda * cosSweepAngle;

    // make sure drag is positive
    cd = fabs(cd);

    ////////////////////////////////////////////////////////////////////////
    // compute cm at cp, check for stall, correct for sweep

    // double cm;

    if (this->alpha_shifted > this->alphaStall)
    {
      cm = (this->cma * this->alphaStall +
            this->cmaStall * (this->alpha_shifted - this->alphaStall))
           * cosSweepAngle;
      // make sure cm is still great than 0
      cm = std::max(0.0, cm);
    }
    else if (this->alpha_shifted < -this->alphaStall)
    {
      cm = (-this->cma * this->alphaStall +
            this->cmaStall * (this->alpha_shifted + this->alphaStall))
           * cosSweepAngle;
      // make sure cm is still less than 0
      cm = std::min(0.0, cm);
    }
    else{
      cm = this->cma * this->alpha_shifted * cosSweepAngle;
    }
    }
else{

    // (Isabelle) Lookup
    // cout << "Check alpha here: " << this->alpha << endl;



    // if yes get the joint corresponding

    float binarySearchResult =  binarySearch(alpha_vec,this->alpha * 180.0 / M_PI,0, (int)(alpha_vec.size()-1));//binarySearch(alpha_vec,this->alpha * 180.0 / M_PI, alpha_vec[0], alpha_vec[(int)(alpha_vec.size()-1)]); //
    // GZ_ASSERT((int)binarySearchResult != -1, "Angle of attack is out of range");

    if ((int)binarySearchResult != -1)
    {
      // cout << "the binary search result " << binarySearchResult << endl;
      ignition::math::Vector3d vector_cl_cd_cm = retrieve_values(binarySearchResult);

      cl = vector_cl_cd_cm[0] * cosSweepAngle * this->chord;
      cd = vector_cl_cd_cm[1] * cosSweepAngle * this->chord;
      cm = vector_cl_cd_cm[2] * cosSweepAngle * this->chord * this->chord;
    }
    else{
      // TODO check this - For AoA out of range
      cl = 2*cos(this->alpha)*sin(this->alpha)*sin(this->alpha);
      cd = 2*sin(this->alpha)*sin(this->alpha)*sin(this->alpha);
      cm = -1*sin(this->alpha)/4; // currently not needed TODO revise!
    }

}
// compute lift force at cp
// std::cout << "Cl is " << cl << std::endl;
ignition::math::Vector3d lift = cl * q * this->area * liftI;
// std::cout << "the lift is " << lift << std::endl;
// drag at cp
ignition::math::Vector3d drag = cd * q * this->area * dragDirection;

// compute moment (torque) at cp
ignition::math::Vector3d moment = cm * q * this->area * momentDirection;

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d cog = this->link->GetInertial()->CoG();
#else
  ignition::math::Vector3d cog = ignitionFromGazeboMath(this->link->GetInertial()->GetCoG());
#endif

// moment arm from cg to cp in inertial plane. but isnt cp the distance between cop and cog?
// Get the axis of the link
ignition::math::Pose3d linkPose = this->link->WorldPose();
auto linkQuatern = linkPose.Rot();
double linkRoll = linkQuatern.Roll();
double linkPitch =linkQuatern.Pitch();
double linkYaw = linkQuatern.Yaw();
ignition::math::Vector3d vectorMainAxis(cos(linkYaw)*cos(linkPitch), sin(linkYaw)*cos(linkPitch), -1*sin(linkPitch)); //-1*sin(modelPitch)
// with a minus one since the vector points backwards (from cog to cop) and positive is in direction of nose
ignition::math::Vector3d momentArm = -1*this->cp * vectorMainAxis;//pose.Rot().RotateVector(this->cp - cog);
// gzerr << this->cp << " : " << this->link->GetInertial()->CoG() << "\n";

// force and torque about cg in inertial frame
ignition::math::Vector3d aerodynamicForce = lift + drag;

// + moment.Cross(momentArm);
tmp_vector = momentArm.Cross(aerodynamicForce);
ignition::math::Vector3d torque = moment;  //THIS IS THE LINE!! CHANGING THE EXPRESSION: TORQUE = MOMENT TO TORQUE = TMP_VECTOR
// - lift.Cross(momentArm) - drag.Cross(momentArm);


  if (1)
  {
    gzdbg << "=================================================================\n";
    gzdbg << "================= DEBUGGING AOA CALCULATION ==================== \n";
    gzdbg << "forward " << this->forward << "\n";
    gzdbg << "upward " << this->upward << "\n";
    gzdbg << "*****************************\n";
    gzdbg << "**changing to inertial reference frame \n";
    gzdbg << "forward (inertial): " << forwardI << "\n";
    gzdbg << "upward (inertial): " << upwardI << "\n";
    gzdbg << "*****************************\n";
    gzdbg << "** Get the spanwise direction (forwardI x upwardI) \n";
    gzdbg << "spanwiseI: " << spanwiseI << "\n";
    gzdbg << "*****************************\n";
    gzdbg << "**Obtain the velocity in LD plane (vel - vel along span) \n";
    gzdbg << "spd: [" << vel.Length()
          << "] vel: [" << vel << "]\n";
    gzdbg << "Windspeed " << constantWind << "\n";
    gzdbg << "VelI :" << velI << "\n";
    gzdbg << "LD plane spd: [" << velInLDPlane.Length()
          << "] velInLDPlane : [" << velInLDPlane << "]\n";
    gzdbg << "*****************************\n";
    gzdbg << "** Get the lift direction = spanwiseI.Cross(velInLDPlane) \n";
    gzdbg << "liftI: " << liftI << "\n";
    gzdbg << "*****************************\n";
    gzdbg << "Get the angle of attack angle between liftI and upwardI\n";
    gzdbg << "cosAlpha " << cosAlpha << "\n";
    gzdbg << "Finally, alpha " << this->alpha << "\n";
    gzdbg << "=================================================================\n";
    gzdbg << "**Retrieve the coefficients \n";
    gzdbg << "cl: " << cl << "]\n";
    gzdbg << "cd: " << cd << "]\n";
    gzdbg << "cm: " << cm << "\n";
    gzdbg << "=================================================================\n";
    gzdbg << "computed Lift and Drag and total aerodynamicForce and Moment \n";
    gzdbg << "Lift " << lift << "\n";
    gzdbg << "Drag " << drag << "\n";
    gzdbg << "Aerodynamic force: " << aerodynamicForce << "\n";
    gzdbg << "cog of " << this->link->GetName() << "is " << cog << "\n";
    gzdbg << "main axis of " << linkName << " is " << vectorMainAxis << "\n";
    gzdbg << "this->cp " << this->cp << "\n";
    gzdbg << "momentArm: " << momentArm << "\n";
    gzdbg << "Aerodynamic torque: " << torque << "\n";
    gzdbg << "=================================================================\n";


    gzdbg << "sensor: [" << this->GetHandle() << "]\n";
    gzdbg << "Link: [" << this->link->GetName()
          << "] pose: [" << pose
          << "] dynamic pressure: [" << q << "]\n";





    gzdbg << "sweep: " << this->sweep << "\n";
    gzdbg << "alpha shifted: " << this->alpha_shifted << "\n";
    gzdbg << "lift: " << lift << "\n";
    gzdbg << "drag: " << drag << " cd: "
          << cd << " cda: " << this->cda << "\n";
    gzdbg << "moment: " << moment << "\n";
    gzdbg << "cp momentArm: " << momentArm << "\n";

    gzdbg << "wind: " << constantWind << "\n";
    gzdbg << "airfoilDatafilePath: " << airfoilDatafilePath << "\n";
    gzdbg << "Computing values? " << compute_values << "\n";
    gzdbg << "Wind azimuth is " << azimuth_wind << "\n";
    gzdbg << "Wind eta is " << eta_wind << "\n";

  }



  // Correct for nan or inf
  aerodynamicForce.Correct();
  this->cp.Correct();
  torque.Correct();
  std::cout << "Groundspeed: " << velI << std::endl;
  std::cout << "Airspeed: " << vel << std::endl;
  std::cout << "Area: " << this->area << std::endl;
  std::cout << "Rho: " << this->rho << std::endl;
  gzdbg << "aerodynamicForce is " << aerodynamicForce << "\n";
  std::cout << "Pose: " << pose << std::endl;
  std::cout << "Cl: " << cl << std::endl;
  std::cout << "AoA: " << alpha << std::endl;
  // todo fix this to the cp
  gzdbg << "Cog " << this->link->WorldCoGPose().Pos() << "\n";
  this->link->AddForceAtWorldPosition(aerodynamicForce, this->link->WorldCoGPose().Pos());//AddForceAtRelativePosition(force, this->cp)
  // this->link->AddTorque(torque);


  //Publish the AoA
  aoa_msg = msgs::ConvertAny(this->alpha);
  aoa_pub_->Publish(aoa_msg);
}

// KITEPOWER (Xander)
void LiftDragWithLookupPlugin::WindFieldCallback(WindFieldPtr &wind_field){
  // printf("Inside the WindFieldCallback function \n");
	vel_wind = wind_field->velocity();
	azimuth_wind = wind_field->azimuth();
  // std::cout << "wind velocity is " << vel_wind << std::endl;
}

// Callback of the SubscriberPtr to the test_msg Topic
void LiftDragWithLookupPlugin::TestMsgCallback(TestMsgPtr &test_msg){
  gzdbg << "Wind Message Received! \n";
  vel_wind = test_msg->x();
  azimuth_wind = test_msg->y();
  eta_wind = test_msg->z();
  // In north east down coordinates VERIFY
  this->V_N_wind = vel_wind * cos(azimuth_wind) * cos(eta_wind + M_PI);
  this->V_E_wind = vel_wind * sin(azimuth_wind) * cos(eta_wind + M_PI);
  this->V_D_wind = vel_wind * sin(eta_wind + M_PI);

  // std::cout << "velocity magnitude is " << test_msg->x() << std::endl;
  // std::cout << "azimuth direction is " << test_msg->y() << std::endl;

}

////////////////////////////////////////////////////////////////////////////////

// A recursive binary search function. It returns
// location of x in given array arr[l..r] is present,
// otherwise -1
float LiftDragWithLookupPlugin::binarySearch(vector<double> arr, double x, int l, int r)
{
   // cout << "x is " << x << endl;
   // cout << "l is " << l << endl;
   // cout << "r is " << r << endl;
   if (r >= l)
   {
        int mid = l + (r - l)/2;
        // cout << "mid is " << mid << endl;
        // If the element is present at the middle
        // itself
        gzdbg << "mid " << mid << "\n";
        gzdbg << "arr[mid] " << arr[mid] << "\n";
        if (arr[mid] == x)
            return mid;

        // If element is smaller than mid, then
        // it can only be present in left subarray
        if (arr[mid] > x){

            if (arr[mid - 1] < x){
              additional_ratio = (x - arr[mid - 1])/(arr[mid] - arr[mid - 1]);
              return (mid - 1) +  additional_ratio;
            }
            else{
              return binarySearch(arr, x, l, mid-1);
            }
        }
         // Else the element can only be present
         // in right subarray
        else {
            if (arr[mid + 1] > x){
              // cout << "inside this case 3" << endl;
              additional_ratio = (x - arr[mid])/(arr[mid + 1] - arr[mid]);
              return mid + additional_ratio;
            }
            else{
              return binarySearch(arr, x, mid+1, r);
            }
        }
   }

   // We reach here when element is not
   // present in array
   return -1;
}
////////////////////////////////////////////////////////////////////////////////

// will probably need to use the ignition::math library
ignition::math::Vector3d LiftDragWithLookupPlugin::retrieve_values(float float_index){
    // int ten_times_float = (int)(float_index * 10);
    // int index_as_int = (int)float_index;
    additional_ratio = float_index - floor(float_index);
    cl_retrieved = cl_vec[floor(float_index)] + additional_ratio * (cl_vec[floor(float_index) + 1] - cl_vec[floor(float_index)]);
    cd_retrieved = cd_vec[floor(float_index)] + additional_ratio * (cd_vec[floor(float_index) + 1] - cd_vec[floor(float_index)]);
    cm_retrieved = cm_vec[floor(float_index)] + additional_ratio * (cm_vec[floor(float_index) + 1] - cm_vec[floor(float_index)]);

    gzdbg << "cl_retrieved (for unit chord) " << cl_retrieved << "\n";
    gzdbg << "cd_retrieved (for unit chord) " << cd_retrieved << "\n";
    gzdbg << "cm_retrieved (for unit chord) " << cm_retrieved << "\n";

    ignition::math::Vector3d resultant_vector = ignition::math::Vector3d(cl_retrieved,cd_retrieved,cm_retrieved);
    return resultant_vector;
}
