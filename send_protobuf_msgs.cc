#include "send_protobuf_msgs.hh"

// using namespace gazebo;
// void parse_wind_field_msg(WindFieldPtr &wind_field_msg_ptr)
// {
//   new_velocity = wind_field_msg_ptr->velocity();
//   std::cout << "The new velocity is: " << new_velocity << std::endl;
// }


int main(int _argc, char **_argv) { //int main(int argc, char * const argv[]) {


  using namespace gazebo;

  // Load gazebo
  client::setup(_argc, _argv);

  // EXPERIMENTAL Set the namespace_
  // namespace_ = "some_namespace";

  //Establish a node
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Is namespace_ ever specified somewhere?
  std::cout << "Namespace is " << namespace_ << std::endl;

  // // EXPERIMENTAL Maybe a new node handle is needed for subscribing. One for subscription and one for publishing.
  // second_node_handle_ = transport::NodePtr(new transport::Node());
  // second_node_handle_->Init(namespace_);


  // commandline_wind_pub_topic_ = "/wind_field";
  // commandline_wind_pub_ = node_handle_->Advertise<wind_field_msgs::msgs::WindField>(commandline_wind_pub_topic_, 1); // wind_pub_ is the publish pointer

  // publish while misusing the Vector3d message to see if this in principle would work.
  test_pub_topic_ = "/test_topic";
  test_pub_ = node_handle_->Advertise<msgs::Vector3d>(test_pub_topic_, 1);

    while (true) {
        // printf("inside loop \n");
        common::Time::MSleep(100);


        // Convert from ‘char* const’ to ‘double’ using atof
        argv1_double = atof(_argv[1]);
        argv2_double = atof(_argv[2]);
        argv3_double = atof(_argv[3]);
        // //wind_field_msgs::msgs::WindField wind_msg;
        // wind_msg.set_velocity(argv_double);
        // commandline_wind_pub_->Publish(wind_msg);

        test_msg.set_x(argv1_double);
        test_msg.set_y(argv2_double);
        test_msg.set_z(argv3_double);
        // Misusing the Vector3d
        test_pub_->Publish(test_msg);

        // // For Debugging, node that subscribes to the topic
        // commandline_wind_field_sub_topic_ = commandline_wind_pub_topic_;
        // commandline_wind_field_sub_ = node_handle_->Subscribe<wind_field_msgs::msgs::WindField>(commandline_wind_field_sub_topic_, parse_wind_field_msg);

        // For Debugging, get the published content
        // previously_published_message = wind_pub_->GetPrevMsgPtr();
        // std::cout << previously_published_message->DebugString();
        // parse_wind_field_msg(&previously_published_message);
        // bool output = previously_published_message.ParseFromString();
        // std::cout << typeid(previously_published_message).name() << std::endl;
  }
  return 0;

}
