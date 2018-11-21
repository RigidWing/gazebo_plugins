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
// get_local_airspeed_components(){
std::vector<double> LiftDistributionPlugin::get_geometric_AoA_vector(){

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

  // return local_airspeed_components;


//Step4b: Get the local angle of attack Obtain the section angle of attack based on

// std::vector<double> LiftDistributionPlugin::get_geometric_AoA_vector(ignition::math::Vector3d local_airspeed_components[this->N_segments]){

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



std::vector<double> get_local_cl_vector(std::vector<double> AoA_vector){
  std::vector<double> local_cl_vector;
  for (o == 0; o <= N_segments; ++o){

    double current_angle = AoA_vector[o];
    float binarySearchResult = binarySearch(alpha_vec,current_angle * 180.0 / M_PI,0, (int)(alpha_vec.size()-1));
    // GZ_ASSERT((int)binarySearchResult != -1, "Angle of attack is out of range");

    cout << "the binary search result " << binarySearchResult << endl;
    ignition::math::Vector3d vector_cl_cd_cm = retrieve_values(binarySearchResult);

    cl = vector_cl_cd_cm[0] * cosSweepAngle;
    cd = vector_cl_cd_cm[1] * cosSweepAngle;
    cm = vector_cl_cd_cm[2] * cosSweepAngle;

    local_cl_vector[o] = cl;
  }

  return local_cl_vector;
}
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

////////////////////////////////////////////////////////////////////////////////

// A recursive binary search function. It returns
// location of x in given array arr[l..r] is present,
// otherwise -1
float LiftDistributionPlugin::binarySearch(vector<double> arr, double x, int l, int r)
{
   cout << "x is " << x << endl;
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
ignition::math::Vector3d LiftDistributionPlugin::retrieve_values(float float_index){
    // int ten_times_float = (int)(float_index * 10);
    // int index_as_int = (int)float_index;
    additional_ratio = float_index - floor(float_index);
    cl_retrieved = cl_vec[floor(float_index)] + additional_ratio * (cl_vec[floor(float_index) + 1] - cl_vec[floor(float_index)]);
    cd_retrieved = cd_vec[floor(float_index)] + additional_ratio * (cd_vec[floor(float_index) + 1] - cd_vec[floor(float_index)]);
    cm_retrieved = cm_vec[floor(float_index)] + additional_ratio * (cm_vec[floor(float_index) + 1] - cm_vec[floor(float_index)]);

    ignition::math::Vector3d resultant_vector = ignition::math::Vector3d(cl_retrieved,cd_retrieved,cm_retrieved);
    return resultant_vector;
}
/////////////////////////////??????//////////////////////////////////////////////////////////////////////////////////////////////////
// After all the functions:

//Step4c: Get the difference

std::vector<double> effective_AoA_vector = get_geometric_AoA_vector() -  get_induced_AoA_vector();


//Step5: Obtain the local cl

std::vector<double> local_cl_vector = get_local_cl_vector(effective_AoA_vector);

//Step6:
this->circulation_vector = 0.5 * this->chord * std::transform( local_airspeed_components.begin()+1, local_airspeed_components.end(),local_cl_vector.begin()+1, local_cl_vector.begin(),std::multiplies<int>() ); 
