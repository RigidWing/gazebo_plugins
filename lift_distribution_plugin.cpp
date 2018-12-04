#include <algorithm>
#include <string>
#include <iostream>
#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "lift_distribution_plugin.h"
#include "common.h"


using namespace gazebo;



GZ_REGISTER_MODEL_PLUGIN(LiftDistributionPlugin)


LiftDistributionPlugin::LiftDistributionPlugin()
{
  this->Gamma0 = 1.0; // initial value of Gamma0
  this->namespace_			= "";
}

LiftDistributionPlugin::~LiftDistributionPlugin()
{
}


/////////////////////////////////////////////////
void LiftDistributionPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{

  // Open a file for logging
  std::freopen("output.txt","w", stdout);

  std::cout << "Start of the Logfile \n" << std::endl;
  //LOG
  std::cout << "=========================================================================================" << std::endl;
  std::cout << "INPUTS" << std::endl;

  GZ_ASSERT(_model, "LiftDistributionPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDistributionPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  second_node_handle_ = transport::NodePtr(new transport::Node());
  second_node_handle_->Init("visual/");

  // Get the wind velocity
  force_pub_ = second_node_handle_->Advertise<msgs::Vector3d>("/visual/vector_component", 50);
  test_msg_sub_ = node_handle_->Subscribe<msgs::Vector3d>("/test_topic",&LiftDistributionPlugin::TestMsgCallback, this);

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDistributionPlugin world pointer is NULL");

  #if GAZEBO_MAJOR_VERSION >= 9
    this->physics = this->world->Physics();
  #else
    this->physics = this->world->GetPhysicsEngine();
  #endif
  GZ_ASSERT(this->physics, "LiftDistributionPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "LiftDistributionPlugin _sdf pointer is NULL");

  //////////////////////////////////////////////////////////////////////////////
  //(KITEPOWER)
  if (_sdf->HasElement("airfoilDatafilePath")){
    airfoilDatafilePath = _sdf->Get<std::string>("airfoilDatafilePath");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

    // TODO ISABELLE GET THE FILEPATH

    ifstream inFile;
    inFile.open("/home/isabelle/gazebo_plugins/e423_data.txt", ios::in | ios::binary); // has to be the full path to the file
    // TODO: change all the print lines to gzdbg lines.


    // PARSE THE AIRFOIL DATA FILE
    while (inFile)
    {   counter++;
        inFile.getline(oneline, MAXLINE);

        line = oneline;
        // Starting from line 13, the data begins
        if (counter >= 13 && counter < 300){


          alpha_str = line.substr(0,8);

          alpha = atof(alpha_str.c_str());


          cl_str = line.substr(11,6);
          cl = atof(cl_str.c_str());


          cd_str = line.substr(20,7);
          cd = atof(cd_str.c_str());


          cdp_str = line.substr(30,7);
          cdp = atof(cdp_str.c_str());


          cm_str = line.substr(39,7);
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

    // CLOSE THE FILE

    inFile.close();
  }
  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");
  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
  this->forward.Normalize();

  //LOG
  std::cout << "Forward: " << this->forward << std::endl;

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
  this->upward.Normalize();

  //LOG
  std::cout << "Upward: " << this->upward << std::endl;


  if(_sdf->HasElement("span"))
    this->span = _sdf->Get<double>("span");

  //LOG
  std::cout << "Span: " << this->span << std::endl;

  // ISABELLE
  if (_sdf->HasElement("link_world_velocity"))
  {
    this->groundspeed_world = _sdf->Get<ignition::math::Vector3d>("link_world_velocity");
  }


  //////////////////////
  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");

    // std::cout << elem << std::endl;
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    //GZ_ASSERT(this->link, "Link was NULL");


    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The LiftDragWithLookupPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDistributionPlugin::OnUpdate, this));
    }
  }



}

///////////////////////////////////////////////
void LiftDistributionPlugin::OnUpdate()
{

  ////////////////////////////////////////////////////////////////////////////
  // Get the Initial Circulation

  //LOG
  std::cout << "=========================================================================================" << std::endl;
  std::cout << "NEW UPDATE" << std::endl;
  std::cout << "N_segments: " << this->N_segments << std::endl;

  std::cout << "***Initial circulation vector***" << std::endl;
  for ( int i = 0; i <= this->N_segments; i = i + 1){
    this->theta = M_PI/(this->N_segments) * i;
    if (i == 0 | i == this->N_segments)
    {
      this->circulation_element = 0.0;
    }
    else{
      this->circulation_element = this->Gamma0 * sin(this->theta);
    }
    this->circulation_vector.push_back(circulation_element);

    //LOG
    // std::cout << "Segment " << i << " circulation: " << this->circulation_element << "\n";
    if (i==this->N_segments){
      std::cout << this->circulation_element << "\n";
    }
    else{
      std::cout << this->circulation_element << ",";
    }

  }

  GZ_ASSERT(this->link, "Link was NULL");
  #if GAZEBO_MAJOR_VERSION >= 9
    vel = this->link->WorldLinearVel(this->cp);
  #else
    vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->cp));
  #endif
    velI = vel;

  // gzdbg << "Obtained the world velocity" << "\n";
  for (int i_iteration = 0; i_iteration <= this->N_iterations; ++i_iteration){
    // Get the circulation derivative
    // gzdbg << "Iteration number: " << i_iteration << "\n";
    for (int j = 0; j <= this->N_segments; ++j){
        double tmp_variable_derivative_val;
        if (j == 0){
          tmp_variable_derivative_val = (this->circulation_vector[j + 1] - this->circulation_vector[j]) * this->N_segments/(this->span);
        }
        else if (j == N_segments){
          tmp_variable_derivative_val = (this->circulation_vector[j] - this->circulation_vector[j - 1]) * this->N_segments/(this->span);
        }
        else {
          tmp_variable_derivative_val = (this->circulation_vector[j + 1] - this->circulation_vector[j - 1]) * 0.5 * this->N_segments/(this->span);
        }
        this->derivative_circulation_vector.push_back(tmp_variable_derivative_val);
    }

    // std::copy(this->circulation_vector.begin(), this->circulation_vector.end(), std::ostream_iterator<double>(std::cout, " "));


    //Step4c: Get the effective angle of attack
    std::vector<double> effective_AoA_vector = get_effective_AoA_vector();//get_geometric_AoA_vector() -  get_induced_AoA_vector();


    //Obtain the local cl


    this->local_cl_vector = get_local_cl_vector(effective_AoA_vector);
    // gzdbg << "Obtained the cl vector " << "\n";
    //Get the new circulation vecctor :
    std::vector<double> elementwise_product_cl_V;
    elementwise_product_cl_V = local_cl_vector;
    std::transform( this->local_airspeed_components.begin(), this->local_airspeed_components.end(),local_cl_vector.begin(), elementwise_product_cl_V.begin(),std::multiplies<float>());
    this->circulation_vector_new = local_cl_vector; // initialize the vector to the required size otherwise transform wont work


    std::transform(elementwise_product_cl_V.begin(), elementwise_product_cl_V.end(), this->circulation_vector_new.begin(), std::bind1st(std::multiplies<double>(), 0.5 * this->chord));//0.5 * this->chord * elementwise_product_cl_V;
    // std::copy(elementwise_product_cl_V.begin(), elementwise_product_cl_V.end(), std::ostream_iterator<double>(std::cout, " "));

    // Update the circulation vector :
    std::vector<double> circulation_diff_vector;
    circulation_diff_vector = local_cl_vector; // just to initialize its size
    std::transform(this->circulation_vector_new.begin(), this->circulation_vector_new.end(), this->circulation_vector.begin(), circulation_diff_vector.begin(), std::minus<float>());
    std::vector<double> circulation_increment_vector;
    circulation_increment_vector = local_cl_vector;
    std::transform(circulation_diff_vector.begin(), circulation_diff_vector.end(), circulation_increment_vector.begin(), std::bind1st(std::multiplies<float>(), this->D));//0.5 * this->chord * elementwise_product_cl_V;
    std::transform(this->circulation_vector.begin(), this->circulation_vector.end(), circulation_increment_vector.begin(), this->circulation_vector.begin(), std::plus<float>());

    if (0){
      gzdbg << "Size of effective_AoA_vector " << effective_AoA_vector.size() << "\n";
      gzdbg << "Size of this->local_airspeed_components " << this->local_airspeed_components.size() << "\n";
      gzdbg << "Size of local_cl_vector " << local_cl_vector.size() << "\n";
      gzdbg << "Size of elementwise_product_cl_V " << elementwise_product_cl_V.size() << "\n";
      gzdbg << "Size of this->local_airspeed_components " << this->local_airspeed_components.size() << "\n";
      gzdbg << "Size of this->circulation_vector_new " << this->circulation_vector_new.size() << "\n";
      gzdbg << "Size of circulation_diff_vector " << circulation_diff_vector.size() << "\n";
      gzdbg << "Size of circulation_increment_vector " << circulation_increment_vector.size() << "\n";
      gzdbg << "Size of this->circulation_vector_new " << this->circulation_vector_new.size() << "\n";
    }

    std::cout << "\n***" << "ITERATION " << i_iteration << " Updated Circulation Vector***" << std::endl;
    std::copy(this->circulation_vector.begin(), this->circulation_vector.end(), std::ostream_iterator<double>(std::cout, ","));

    // effective_AoA_vector.clear();
    this->local_cl_vector.clear();
    elementwise_product_cl_V.clear();
    this->local_airspeed_components.clear();
    this->circulation_vector_new.clear();
    circulation_diff_vector.clear();
    circulation_increment_vector.clear();

  }

   // Update the lift sum_components
   double L_prime_magnitude; // the small lift component at a spanwise location
   ignition::math::Vector3d L_prime_vector;
   double current_spanwise_y = -1 * this->span / 2; // Terrible variable names also redundant TODO ISABELLE
   ignition::math::Vector3d current_spanwise_location(this->cp.X(),current_spanwise_y,0);

   //LOG
   std::cout << "\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
   std::cout << "LIFT" << std::endl;

   for ( int q = 0; q <= this->N_segments; q = q + 1){

     L_prime_magnitude = this->rho * local_airspeed_components[q] * this->circulation_vector[q] * (this->span/this->N_segments);
     L_prime_vector = liftI_arr[q] * L_prime_magnitude;

     if (0){
       gzdbg << "Segment " << q << "\n";
       gzdbg << "This lift array " << this->liftI_arr[q] << "\n";
       gzdbg << "The lift component magnitude " << L_prime_magnitude << "\n";
       gzdbg << "The lift component vector " << L_prime_vector << "\n";
   }
     //LOG
     std::cout << "Segment " << q << " - lift magnitude " << L_prime_magnitude << "\n";

     // Add for force/Moment bookkeeping:
     this->link->AddForceAtRelativePosition(L_prime_vector, current_spanwise_location); // note that those two have to be vectors.
     // publish something onto the visualize topic - anything for now, and then to be adjusted
     msgs::Vector3d force_msg;
     force_msg.set_x(1.0);
     force_msg.set_y(4.0);
     force_msg.set_z(5.0);
     force_pub_->Publish(force_msg);

     current_spanwise_y += this->span/this->N_segments;
     // Update the spanwise location
     current_spanwise_location.Y(current_spanwise_y);

   }





   // std::copy(this->circulation_vector.begin(), this->circulation_vector.end(), std::ostream_iterator<double>(std::cout, " "));

   this->circulation_vector.clear();

}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
double LiftDistributionPlugin::get_induced_AoA(double spanwise_yn){
  // Get the vector of the derivative of the circulation with respect to the span.
  // implement sampson's integration method

  // reset the sum to zero
  double sum_components = 0.0;
  // reset the initial location to -b/2
  double current_y = -1 * this->span / 2;

  double component;
  int k_singularity;
  // std::cout << "N_segments " << N_segments << std::endl;
  std::vector<double> components;
  for (this->k = 0; this->k <= this->N_segments; this->k++){

    if (this->k == 0){
      component = (this->derivative_circulation_vector[this->k]) * (this->span / this->N_segments) / (3 * (spanwise_yn - current_y));
    }
    else if (this->k % 2 == 0){
      // gzdbg << "In case b " << "\n";
      component = 2 * (this->derivative_circulation_vector[this->k]) * (this->span / this->N_segments) / (3 * (spanwise_yn - current_y));
    }
    else {
      // gzdbg << "In case c " << "\n";
      component = 4 * (this->derivative_circulation_vector[this->k]) * (this->span / this->N_segments) / (3 * (spanwise_yn - current_y));
    }

    if (fabs(spanwise_yn - current_y) < 0.0001){
      int k_singularity = k;
      // append something so that we are not left with one less entry
      components.push_back(0.0);
    }
    else{
      components.push_back(component);
    }

    current_y += this->span/this->N_segments;

  }


  //  Sum the components in the deribat
  for (int s = 0; s <= this->N_segments; s++){
    if (s == k_singularity){
      if (k_singularity == 0)
      {
        sum_components+= 0.5*components[s+1];
      }
      else if (k_singularity == this->N_segments)
      {
        sum_components+= 0.5*components[s-1];
      }
      else{
        sum_components+= 0.5*(components[s-1] + components[s+1]);
      }
    }
    else{
      sum_components += components[s];
    }
  }


  double vel_length = std::accumulate(local_airspeed_components.begin(), local_airspeed_components.end(), 0)/local_airspeed_components.size();
  double induced_AoA = (1/(4 * M_PI * vel_length)) * sum_components; //double induced_AoA = (1/(4 * M_PI * vel.Length())) * sum_components;
  return induced_AoA;
}

// get_local_airspeed_components(){
std::vector<double> LiftDistributionPlugin::get_effective_AoA_vector(){

  ignition::math::Vector3d constantWind(this->V_N_wind,this->V_E_wind,-1 * this->V_D_wind); //
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
  this->cosSweepAngle = sqrt(1.0 - sinSweepAngle * sinSweepAngle); //iso sqrt
  std::vector<double> effective_AoA_vector;

  // get the velocity at different spanwise locations
  double current_y = -1 * this->span / 2;

  this->current_wing_location.Set(this->cp.X(), this->current_y,this->cp.Z()); // ATT the x and z values might have to be changed

  this->model->GetLink("rigid_wing::main_wing")->SetLinearVel({groundspeed_world.X(), groundspeed_world.Y(), groundspeed_world.Z()});

  for (int l = 0; l <= this->N_segments; ++l){
    #if GAZEBO_MAJOR_VERSION >= 9
      ignition::math::Vector3d groundspeed_component = this->link->WorldLinearVel(this->current_wing_location); // arg has to be a vector
    #else
      ignition::math::Vector3d groundspeed_component = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->current_wing_location));
    #endif

    ignition::math::Vector3d local_airspeed_component = groundspeed_component - constantWind; // TODO DEBUG THIS GROUNDSPEED - WINDSPEED = AIRSPEED BUT FOR SOME REASON IN PREVIOUS DOC IT WAS PLUS?!
    this->local_airspeed_components.push_back(local_airspeed_component.Length());
    //////////////////////////
    // step1: Get velInLDPlane
    ignition::math::Vector3d vel = local_airspeed_component;
    ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI; //why velI? (Isabelle) changed velI to spanwiseI
    // step2: get direction of lift
    ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
    liftI.Normalize();

    // Append to the array of LiftI vectors
    this->liftI_arr[l] = liftI;

    // step3: get cosAlpha
    double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), -1.0, 1.0);

    double current_geometric_alpha;


    // step4: get alpha
    if (liftI.Dot(forwardI) >= 0.0) // changed forwardI to upwardI
      // then alpha will be positive
      current_geometric_alpha = acos(cosAlpha); // this->alpha0 + acos(cosAlpha);  TODO ISABELLE, SEE IF THE ADDITION OF a0 is necessary? This is for the shift in the curve as made in the plugin
    else
      // alpha is negative
      current_geometric_alpha = -1 * acos(cosAlpha);
    // step5: normalize to within +/-90 deg

    while (fabs(current_geometric_alpha) > 0.5 * M_PI)
      current_geometric_alpha = current_geometric_alpha > 0 ? current_geometric_alpha - M_PI
                                    : current_geometric_alpha + M_PI;

    // step6: append to the vector
    double effective_alpha = current_geometric_alpha - get_induced_AoA(current_y);
    effective_AoA_vector.push_back(effective_alpha);
    //////////////////////////

    current_y += this->span/N_segments;
  }
  return effective_AoA_vector;
}

std::vector<double> LiftDistributionPlugin::get_local_cl_vector(std::vector<double> AoA_vector){
  for (int o = 0; o <= N_segments; ++o){
    double current_angle = AoA_vector[o];
    float binarySearchResult = binarySearch(alpha_vec,current_angle * 180.0 / M_PI,0, (int)(alpha_vec.size()-1));
    ignition::math::Vector3d vector_cl_cd_cm = retrieve_values(binarySearchResult);
    cl = vector_cl_cd_cm[0] * this->cosSweepAngle;
    cd = vector_cl_cd_cm[1] * this->cosSweepAngle;
    cm = vector_cl_cd_cm[2] * this->cosSweepAngle;
    local_cl_vector.push_back(cl);
  }
  return this->local_cl_vector;
}

// Callback of the SubscriberPtr to the test_msg Topic
void LiftDistributionPlugin::TestMsgCallback(TestMsgPtr &test_msg){
  vel_wind = test_msg->x();
  azimuth_wind = test_msg->y();
  eta_wind = test_msg->z();
  // In north east down coordinates VERIFY
  V_N_wind = vel_wind * cos(azimuth_wind) * cos(eta_wind + M_PI);
  V_E_wind = vel_wind * sin(azimuth_wind) * cos(eta_wind + M_PI);
  V_D_wind = vel_wind * sin(eta_wind + M_PI);
}

float LiftDistributionPlugin::binarySearch(vector<double> arr, double x, int l, int r)
{
   // cout << "x is " << x << endl;
   if (r >= l)
   {
        int mid = l + (r - l)/2;
        // cout << "mid is " << mid << endl;
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
