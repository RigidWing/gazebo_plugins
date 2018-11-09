#include <stdlib.h>
#include "WindField.pb.h"
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include "common.h"
#include <string>
#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Assert.hh"
#include <typeinfo>

#include <gazebo/gazebo_client.hh>
//#include "liftdrag_plugin/liftdrag_plugin.h"

double argv1_double;
double argv2_double;
double argv3_double;

namespace gazebo{

  std::string namespace_;
  std::string commandline_wind_pub_topic_;
  std::string commandline_wind_field_sub_topic_;
  transport::NodePtr node_handle_;
  transport::PublisherPtr commandline_wind_pub_;
  transport::SubscriberPtr commandline_wind_field_sub_;
  wind_field_msgs::msgs::WindField wind_msg;

  // The Test Topic and Test Message
  std::string test_pub_topic_;
  transport::PublisherPtr test_pub_;
  msgs::Vector3d test_msg;

  // Create a different node that would subscribe to the /wind_field topic (new node possibly to resolve the mutex issue)
  transport::NodePtr second_node_handle_;

  // a Message Ptr
  transport::MessagePtr previously_published_message;

  typedef const boost::shared_ptr<const wind_field_msgs::msgs::WindField> WindFieldPtr;

  double new_velocity;
  // void parse_wind_field_msg(WindFieldPtr &wind_field_msg_ptr);

  void parse_wind_field_msg(WindFieldPtr &wind_field_msg_ptr)
  {
    common::Time::MSleep(100);
    printf("Inside the parse_wind_field_msg function \n");
    new_velocity = wind_field_msg_ptr->velocity();
    std::cout << "The new velocity is: " << new_velocity << std::endl;
  }



}
