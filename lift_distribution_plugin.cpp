#include <algorithm>
#include <string>
#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "lift_distribution_plugin.h"
#include "common.h"

// for numerical differentiation
#include <boost/math/tools/numerical_differentiation.hpp>

using namespace gazebo;



GZ_REGISTER_MODEL_PLUGIN(LiftDistributionPlugin)


LiftDistributionPlugin::LiftDistributionPlugin()
{
  this->Gamma0 = 1.0; // initial value of Gamma0

}

LiftDistributionPlugin::~LiftDistributionPlugin()
{
}


/////////////////////////////////////////////////
void LiftDistributionPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{

  // Get the wind velocity
  test_msg_sub_ = node_handle_->Subscribe<msgs::Vector3d>("/test_topic",&LiftDragWithLookupPlugin::TestMsgCallback, this);


}

/////////////////////////////////////////////////
void LiftDistributionPlugin::OnUpdate()
{

}


// Function to get the initial circulation
std::vector<double> LiftDistributionPlugin::get_initial_circulation_vector(){
  for ( int i = 0; i <= this->N_segments; i = i + 1){
    this->theta = M_PI/(this->N_segments) * i;
    if (i == 0 | i == this->N_segments){
      this->circulation_element = 0.0;
    }
    else{
      this->circulation_element = this->Gamma0 * sin(this->theta);
    }
    this->circulation_vector.push_back(circulation_element);
  }
  return this->circulation_vector;
}
std::vector<double> LiftDistributionPlugin::get_derivative_circulation_vector(){


  for (int j = 0; j <= this->N_segments; ++j){
      double tmp_variable_derivative_val;
      if (j == 0){
        tmp_variable_derivative_val = (this->circulation_vector[j + 1] - this->circulation_vector[j]) * this->N_segments/(this->span);
      }
      else if (j == N_segments){
        tmp_variable_derivative_val = (this->circulation_vector[j] - this->circulation_vector[j - 1]) * this->N_segments/(this->span);
      }
      else {
        tmp_variable_derivative_val = (this->circulation_vector[j + 1] - this->circulation_vector[j - 1]) * 2 * this->N_segments/(this->span);
      }
      this->derivative_circulation_vector.push_back(tmp_variable_derivative_val);
  }
  return this->derivative_circulation_vector; //note that the circulation vector has N_segments + 1 elements
}

// Step3: Get the induced angle of attack at each station
double LiftDistributionPlugin::get_induced_AoA(double spanwise_yn){
  // Get the vector of the derivative of the circulation with respect to the span.
  // implement sampson's integration method
  double sum_components = 0.0;
  double current_y = -1 * this->span / 2;
  for (int k = 0; k <= this->N_segments; ++k){
    if (k == 0){
      sum_components += (this->derivative_circulation_vector[k])/(spanwise_yn - current_y);
    }
    else if (k % 2 == 0){
      sum_components += 2 * (this->derivative_circulation_vector[k])/(spanwise_yn - current_y);
    }
    else {
      sum_components += 4 * (this->derivative_circulation_vector[k])/(spanwise_yn - current_y);
    }
    current_y += this->span/N_segments;
  }

  induced_AoA = (1/(4 * M_PI * vel)) * sum_components;
  return induced_AoA;
}

//Step4: Get the effective angle of attack
//Step4a: Get local ground speed (Kinematics)
get_local_airspeed_components(){

  ignition::math::Vector3d local_airspeed_components[this->N_segments];
  // get the Pose so then the direction of the axis of symmetry is known
  // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

  //ignition::math::Vector3d constantWind(this->vel_wind*cos(this->azimuth_wind),this->vel_wind*sin(this->azimuth_wind),0);
  ignition::math::Vector3d constantWind(this->V_N_wind,this->V_E_wind,-1 * this->V_D_wind); // minus 1 because NED to world

  // get the GetVelocity of the object
  // get linear velocity at cp in inertial frame
  double current_y = -1 * this->span / 2;
  for (int l = 0; l <= this->N_segments; ++l){

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d groundspeed_component = this->link->WorldLinearVel(this->current_y);
  #else
    ignition::math::Vector3d groundspeed_component = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->current_y));
  #endif

    ignition::math::Vector3d local_airspeed_component = groundspeed_component + constantWind;

    local_airspeed_components[l] = local_airspeed_component;

    current_y += this->span/N_segments;
}

  // Get the angular velocity WorldAngularVel();

  // OR I GUESS I COULD USE this->link->WorldLinearVel(this->cp);
  // Get the wind velocity

  // get the GetCoG

  return local_airspeed_components;
}

//Step4b: Get the local angle of attack Obtain the section angle of attack based on

std::vector<double> LiftDistributionPlugin::get_local_geometric_AoA_values(ignition::math::Vector3d local_airspeed_components[this->N_segments]){

  // The process for obtaining the AoA
  // Taken from the liftDrag plugin
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
  std::vector<double> local_geometric_AoA_values;
  for (int m = 0, m <= N_segments, ++m){

      // step1: Get velInLDPlane
      ignition::math::Vector3d vel = local_airspeed_components[m];
      ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI; //why velI? (Isabelle) changed velI to spanwiseI

      // step2: get direction of lift
      ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
      liftI.Normalize();

      // get cosAlpha
      double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), -1.0, 1.0);
      double current_geometric_alpha;
      if (liftI.Dot(forwardI) >= 0.0) // changed forwardI to upwardI
        // then alpha will be positive
        current_geometric_alpha = acos(cosAlpha); // this->alpha0 + acos(cosAlpha);  TODO ISABELLE, SEE IF THE ADDITION OF a0 is necessary? This is for the shift in the curve as made in the plugin
      else
        // alpha is negative
        current_geometric_alpha = -1 * acos(cosAlpha);

      // normalize to within +/-90 deg
      while (fabs(current_geometric_alpha) > 0.5 * M_PI)
        current_geometric_alpha = current_geometric_alpha > 0 ? current_geometric_alpha - M_PI
                                      : current_geometric_alpha + M_PI;

      local_geometric_AoA_values.push_back(current_geometric_alpha);
  }

  return local_geometric_AoA_values;
}

std::vector<double> LiftDistributionPlugin::get_induced_AoA_vector(){
  std::vector<double> induced_AoA_vector;
  double current_y = -1 * this->span / 2;
  for (int n = 0; n <= N_segments; ++n){
    double current_induced_AoA = get_induced_AoA(current_y);
    induced_AoA_vector.push_back(current_induced_AoA);
    current_y += this->span/N_segments;
  }
  return induced_AoA_vector;
}

//Step4c: Get the difference

std::vector<double> effective_AoA_vector = get_induced_AoA_vector();


//Step5: Obtain the local cl
//Step6: Get the new Gamma values:
//Step7: Update Gamma
// Iterate with Step3 then with Step4c, Step5, Step6 and Step7.

// Get the local lift vector

// Use the AddForceAtRelativePosition  function. or AddForceAtWorldPosition



// Callback of the SubscriberPtr to the test_msg Topic
void LiftDistributionPlugin::TestMsgCallback(TestMsgPtr &test_msg){

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
