#ifndef VISUALIZE_VECTOR_PLUGIN_H
#define VISUALIZE_VECTOR_PLUGIN_H

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include <algorithm>
#include <iostream>
#include <string>
#include <iostream>
#include "common.h"
#include "gazebo/common/Assert.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ignition/math/Vector3.hh>

#include <gazebo/physics/Model.hh>

// #include "Vector3d.pb.h"
#include "gazebo/msgs/msgs.hh"
// if you want some positions of the model use this....
// #include <gazebo_msgs/ModelStates.h>

#include <string>
#include <vector>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>

// KITEPOWER (Xander)
#include "common.h"
#include "common.h"
#include <common.h>
#include "WindField.pb.h"

#include <iostream>
#include <fstream>
// #include <stdlib.h>     /* atof */
#include <string>
#include <list>
#include <vector>
#include <ignition/math/Vector3.hh>

#include "send_protobuf_msgs.hh"

#include <math.h>

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

namespace gazebo
{
  namespace rendering
  {
    class VisualizeVectorPlugin : public VisualPlugin
    {
      public:
        /// \brief Constructor
        VisualizeVectorPlugin();

        /// \brief Destructor
        virtual ~VisualizeVectorPlugin();

        /// \brief Load the visual force plugin tags
        /// \param node XML config node
        void Load( VisualPtr _parent, sdf::ElementPtr _sdf );


      protected:
        /// \brief Update the visual plugin
        virtual void UpdateChild();
        double vector_x;
        double vector_y;
        double vector_z;

      private:
        // /// \brief pointer to ros node
        // ros::NodeHandle* rosnode_
        transport::NodePtr node_handle_;

        /// \brief store model name
        std::string model_name_;

        /// \brief topic name
        std::string topic_name_;

        std::string namespace_;

        // /// \brief The visual pointer used to visualize the force.
        VisualPtr visual_;

        // /// \brief The scene pointer.
        ScenePtr scene_;

        /// \brief For example a line to visualize the force
        DynamicLines *line;

        /// \brief for setting ROS name space
        std::string visual_namespace_;

        // /// \Subscribe to some force
        // ros::Subscriber force_sub_;
        transport::SubscriberPtr force_sub_;

        /// \brief Visualize the force
        typedef const boost::shared_ptr<const msgs::Vector3d> PointConstPtr; // typedef const boost::shared_ptr<msgs::Vector3d> PointConstPtr;
        void VectorMsgCallback(PointConstPtr &vector_msg);
        void VisualizeForceOnLink(PointConstPtr &force_ms);

        // Pointer to the update event connection
        event::ConnectionPtr update_connection_;
    };
  }
}

#endif
