#include "send_protobuf_msgs.hh"

int main(int _argc, char **_argv) {

  using namespace gazebo;

  // Load gazebo
  client::setup(_argc, _argv);

  //Establish a node
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  test_pub_topic_ = "/test_topic";
  test_pub_ = node_handle_->Advertise<msgs::Vector3d>(test_pub_topic_, 1);

    while (true) {
        common::Time::MSleep(100);

        // Convert from ‘char* const’ to ‘double’ using atof
        argv1_double = atof(_argv[1]);
        argv2_double = atof(_argv[2]);
        argv3_double = atof(_argv[3]);

        test_msg.set_x(argv1_double); // the wind velocity
        test_msg.set_y(argv2_double); // the azimuth
        test_msg.set_z(argv3_double);

        // Use the Vector3d to publish
        test_pub_->Publish(test_msg);

  }
  return 0;

}
