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
#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "liftdrag_plugin/liftdrag_with_lookup_plugin.h"
// KITEPOWER (Xander)
#include "common.h"

bool compute_values = 1;

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragWithLookupPlugin)

/////////////////////////////////////////////////
LiftDragWithLookupPlugin::LiftDragWithLookupPlugin() : cla(1.0), cda(0.01), cma(0.01), rho(1.2041)
{
  this->cp = ignition::math::Vector3d(0, 0, 0);
  this->forward = ignition::math::Vector3d(1, 0, 0);
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
    printf("INSIDE THE AIRFOIL RETRIEVAL PORTION \n");
    compute_values = 0;
    airfoilDatafilePath = _sdf->Get<std::string>("airfoilDatafilePath");
    cout << typeid(airfoilDatafilePath).name() << endl;

    // TODO ISABELLE GET THE FILEPATH

    ifstream inFile;
    inFile.open("/home/isabelle/gazebo_plugins/e423_data.txt", ios::in | ios::binary); // has to be the full path to the file
    // TODO: change all the print lines to gzdbg lines.


    // PARSE THE AIRFOIL DATA FILE
    while (inFile)
    {   counter++;
        inFile.getline(oneline, MAXLINE);
        // cout << "the counter is " << counter << endl;
        line = oneline;
        // Starting from line 13, the data begins
        if (counter >= 13 && counter < 300){
          // cout << "cd is: " << line.substr(20,7) << endl;

          alpha_str = line.substr(0,8);
          // cout << "Alpha is: " << alpha_str << endl;
          alpha = atof(alpha_str.c_str());
          // cout << "Alpha is: " << alpha << endl;

          cl_str = line.substr(11,6);
          cl = atof(cl_str.c_str());
          // cout << "Cl is: " << cl << endl;

          cd_str = line.substr(20,7);
          cd = atof(cd_str.c_str());
          // cout << "Cd is: " << cd << endl;

          cdp_str = line.substr(30,7);
          cdp = atof(cdp_str.c_str());
          // cout << "Cdp is: " << cdp << endl;

          cm_str = line.substr(39,7);
          cm = atof(cm_str.c_str());
          // cout << "Cm is: " << cm << endl;

          top_xtr_str = line.substr(49,6);
          top_xtr = atof(top_xtr_str.c_str());
          // cout << "top_xtr is: " << top_xtr << endl;

          bot_xtr_str = line.substr(58,6);
          bot_xtr = atof(bot_xtr_str.c_str());
          // cout << "bot_xtr is: " << bot_xtr << endl;

          top_itr_str = line.substr(66,7);
          top_itr = atof(top_itr_str.c_str());
          // cout << "top_itr is: " << top_itr << endl;

          bot_itr_str = line.substr(74,8);
          bot_itr = atof(bot_itr_str.c_str());
          // cout << "bot_itr is: " << bot_itr << endl;

          // APPEND THE VALUES OBTAINED FROM THE FILE TO VECTORS

          alpha_vec.push_back(alpha);
          cl_vec.push_back(cl);
          cd_vec.push_back(cd);
          cm_vec.push_back(cm);
        }
    }

    // CLOSE THE FILE

    inFile.close();
  }

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

  // KITEPOWER
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // KITEPOWER
  if (_sdf->HasElement("useConstantDragCoefficient"))
  {
    this->useConstantDragCoefficient = _sdf->Get<bool>("useConstantDragCoefficient");
  }

  // ISABELLE

  if (_sdf->HasElement("link_world_velocity"))
  {
    this->groundspeed_world = _sdf->Get<ignition::math::Vector3d>("link_world_velocity");
  }

  //getSdfParam<std::string>(_sdf, "windFieldSubTopic", wind_field_sub_topic_, wind_field_sub_topic_);
  //wind_field_sub_ = node_handle_->Subscribe<wind_field_msgs::msgs::WindField>("~/" + this->model->GetName() + wind_field_sub_topic_, &LiftDragWithLookupPlugin::WindFieldCallback, this);
  wind_field_sub_ = node_handle_->Subscribe<wind_field_msgs::msgs::WindField>(wind_field_sub_topic_, &LiftDragWithLookupPlugin::WindFieldCallback, this);
  // An Additional SubscriberPtr To Subscribe to the test_msg Topic
  test_msg_sub_ = node_handle_->Subscribe<msgs::Vector3d>("/test_topic",&LiftDragWithLookupPlugin::TestMsgCallback, this);
}

/////////////////////////////////////////////////
void LiftDragWithLookupPlugin::OnUpdate()
{
  // printf("Inside the OnUpdate function \n");
  GZ_ASSERT(this->link, "Link was NULL");

  // ISABELLE hack the world velocity
this->model->GetLink("rigid_wing::main_wing")->SetLinearVel({groundspeed_world.X(), groundspeed_world.Y(), groundspeed_world.Z()});

  // get linear velocity at cp in inertial frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp);
#else
  ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->cp));
#endif
  ignition::math::Vector3d velI = vel;

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
  ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*velI; //why velI

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
  double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);
  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  if (liftI.Dot(forwardI) >= 0.0)
    this->alpha = acos(cosAlpha); // this->alpha0 + acos(cosAlpha);  TODO ISABELLE, SEE IF THE ADDITION OF a0 is necessary?
  else
    this->alpha = -1 * acos(cosAlpha); //this->alpha0 - acos(cosAlpha);

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
    if (this->alpha > this->alphaStall)
    {
      cl = (this->cla * (this->alphaStall - this->alpha0) +
            this->claStall * (this->alpha - this->alphaStall))
           * cosSweepAngle; // NOTE (ISABELLE) MADE ADJUSTMENT HERE, SUBTRACTED ALPHA0
      // make sure cl is still great than 0
      cl = std::max(0.0, cl);
      cout  << "1 the cl value " << cl << endl;
    }
    else if (this->alpha < -this->alphaStall) // TODO (ISABELLE) CHECK THIS
    {
      cl = (-this->cla * this->alphaStall +
            this->claStall * (this->alpha + this->alphaStall))
           * cosSweepAngle;
      // make sure cl is still less than 0
      cl = std::min(0.0, cl);
      cout  << "2 the cl value " << cl << endl;
    }
    else
      cl = this->cla * (this->alpha - this->alpha0) * cosSweepAngle; // NOTE (ISABELLE) MADE ADJUSTMENT HERE, SUBTRACTED ALPHA0
      cout << this->alpha << endl;
      cout << this->alpha0 << endl;
      cout  << "3 the cl value " << cl << endl;

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
      cout  << "4 the cl value " << cl << endl;

    }

    cout  << "the cl value " << cl << endl;
    ////////////////////////////////////////////////////////////////////////////
    // compute cd at cp, check for stall, correct for sweep
    // KITEPOWER: if useConstantDragCoefficient is true, computes the drag
    // coefficient based on the cda and cda_stall only without considering
    // an angle of attach of an airfoil

    // double cd;

    if (this->alpha > this->alphaStall)
    { // KITEPOWER
      if (!this->useConstantDragCoefficient)
        cd = (this->cda * (this->alphaStall - this->alpha0) +
              this->cdaStall * (this->alpha - this->alphaStall))
             * cosSweepAngle; // NOTE (ISABELLE) MADE ADJUSTMENT HERE, SUBTRACTED ALPHA0
      else
        cd = (this->cda + this->cdaStall) * cosSweepAngle; // TODO (ISABELLE), WHAT IS THIS?
    }
    else if (this->alpha < -this->alphaStall)
    { // KITEPOWER
      if (!this->useConstantDragCoefficient)
        cd = (-this->cda * this->alphaStall +
              this->cdaStall * (this->alpha + this->alphaStall))
             * cosSweepAngle;
      else
        cd = (-this->cda + this->cdaStall) * cosSweepAngle;
    }
    else // KITEPOWER
      if (!this->useConstantDragCoefficient)
        cd = (this->cda * (this->alpha - this->alpha0)) * cosSweepAngle; // NOTE (ISABELLE) MADE ADJUSTMENT HERE, SUBTRACTED ALPHA0
      else
        cd = this->cda * cosSweepAngle; // TODO (ISABELLE), WHAT IS THIS?

    // make sure drag is positive
    cd = fabs(cd);

    ////////////////////////////////////////////////////////////////////////
    // compute cm at cp, check for stall, correct for sweep

    // double cm;

    if (this->alpha > this->alphaStall)
    {
      cm = (this->cma * (this->alphaStall - this->alpha0) +
            this->cmaStall * (this->alpha - this->alphaStall))
           * cosSweepAngle; // NOTE (ISABELLE) MADE ADJUSTMENT HERE, SUBTRACTED ALPHA0
      // make sure cm is still great than 0
      cm = std::max(0.0, cm);
    }
    else if (this->alpha < -this->alphaStall)
    {
      cm = (-this->cma * this->alphaStall +
            this->cmaStall * (this->alpha + this->alphaStall))
           * cosSweepAngle;
      // make sure cm is still less than 0
      cm = std::min(0.0, cm);
    }
    else
      cm = this->cma * (this->alpha - this->alpha0) * cosSweepAngle; // NOTE (ISABELLE) MADE ADJUSTMENT HERE, SUBTRACTED ALPHA0
    }
else{
    float binarySearchResult = binarySearch(alpha_vec,alpha,0, (int)(alpha_vec.size()-1));
    // GZ_ASSERT((int)binarySearchResult != -1, "Angle of attack is out of range");
    ignition::math::Vector3d vector_cl_cd_cm = retrieve_values(binarySearchResult);

    cl = vector_cl_cd_cm[0];
    cd = vector_cl_cd_cm[1];
    cm = vector_cl_cd_cm[2];
}
// compute lift force at cp
std::cout << "Cl is " << cl << std::endl;
ignition::math::Vector3d lift = cl * q * this->area * liftI;
std::cout << "the lift is " << lift << std::endl;
// drag at cp
ignition::math::Vector3d drag = cd * q * this->area * dragDirection;

// compute moment (torque) at cp
ignition::math::Vector3d moment = cm * q * this->area * momentDirection;

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d cog = this->link->GetInertial()->CoG();
#else
  ignition::math::Vector3d cog = ignitionFromGazeboMath(this->link->GetInertial()->GetCoG());
#endif

// moment arm from cg to cp in inertial plane
ignition::math::Vector3d momentArm = pose.Rot().RotateVector(
  this->cp - cog
);
// gzerr << this->cp << " : " << this->link->GetInertial()->CoG() << "\n";

// force and torque about cg in inertial frame
ignition::math::Vector3d force = lift + drag;

// + moment.Cross(momentArm);
tmp_vector = momentArm.Cross(force);
ignition::math::Vector3d torque = moment;  //THIS IS THE LINE!! CHANGING THE EXPRESSION: TORQUE = MOMENT TO TORQUE = TMP_VECTOR
// - lift.Cross(momentArm) - drag.Cross(momentArm);


  if (1)
  {
    gzdbg << "=============================\n";
    gzdbg << "sensor: [" << this->GetHandle() << "]\n";
    gzdbg << "Link: [" << this->link->GetName()
          << "] pose: [" << pose
          << "] dynamic pressure: [" << q << "]\n";
    gzdbg << "spd: [" << vel.Length()
          << "] vel: [" << vel << "]\n";
    gzdbg << "LD plane spd: [" << velInLDPlane.Length()
          << "] vel : [" << velInLDPlane << "]\n";
    gzdbg << "VelI :" << velI << "\n";
    gzdbg << "forward (inertial): " << forwardI << "\n";
    gzdbg << "upward (inertial): " << upwardI << "\n";
    gzdbg << "lift dir (inertial): " << liftI << "\n";
    gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
    gzdbg << "sweep: " << this->sweep << "\n";
    gzdbg << "alpha: " << this->alpha << "\n";
    gzdbg << "lift: " << lift << "\n";
    gzdbg << "drag: " << drag << " cd: "
          << cd << " cda: " << this->cda << "\n";
    gzdbg << "moment: " << moment << "\n";
    gzdbg << "cp momentArm: " << momentArm << "\n";
    gzdbg << "force: " << force << "\n";
    gzdbg << "torque: " << torque << "\n";
    gzdbg << "wind: " << constantWind << "\n";
    gzdbg << "airfoilDatafilePath: " << airfoilDatafilePath << "\n";
    gzdbg << "Computing values? " << compute_values << "\n";
  }

  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  torque.Correct();

  this->link->AddForceAtRelativePosition(force, this->cp);
  this->link->AddTorque(torque);
}

// KITEPOWER (Xander)
void LiftDragWithLookupPlugin::WindFieldCallback(WindFieldPtr &wind_field){
  printf("Inside the WindFieldCallback function \n");
	vel_wind = wind_field->velocity();
	azimuth_wind = wind_field->azimuth();
  std::cout << "The wind velocity is " << vel_wind << std::endl;
}

// Callback of the SubscriberPtr to the test_msg Topic
void LiftDragWithLookupPlugin::TestMsgCallback(TestMsgPtr &test_msg){

  testMsgCallbackUsed = 1;
  printf("Inside the TestMsgCallback function \n");
  vel_wind = test_msg->x();
  azimuth_wind = test_msg->y();
  eta_wind = test_msg->z();

  // In north east down coordinates VERIFY
  V_N_wind = vel_wind * cos(azimuth_wind) * cos(eta_wind + M_PI);
  V_E_wind = vel_wind * sin(azimuth_wind) * cos(eta_wind + M_PI);
  V_D_wind = vel_wind * sin(eta_wind + M_PI);

  std::cout << "The velocity magnitude is " << test_msg->x() << std::endl;
  std::cout << "The azimuth direction is " << test_msg->y() << std::endl;

}

////////////////////////////////////////////////////////////////////////////////

// A recursive binary search function. It returns
// location of x in given array arr[l..r] is present,
// otherwise -1
float LiftDragWithLookupPlugin::binarySearch(vector<double> arr, double x, int l, int r)
{
   if (r >= l)
   {
        int mid = l + (r - l)/2;
        cout << "mid is " << mid << endl;
        // If the element is present at the middle
        // itself
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
              cout << "inside this case 3" << endl;
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

    ignition::math::Vector3d resultant_vector = ignition::math::Vector3d(cl_retrieved,cd_retrieved,cm_retrieved);
    return resultant_vector;
}
