#include <visualize_vector_plugin.hh>

namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    VisualizeVectorPlugin::VisualizeVectorPlugin(): line(NULL)
    {

    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    VisualizeVectorPlugin::~VisualizeVectorPlugin()
    {
      // Finalize the visualizer
      // this->node_handle_->shutdown();
      // delete this->node_handle_;
      this->namespace_			= "";
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the plugin
    void VisualizeVectorPlugin::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
    {
      this->visual_ = _parent;

      this->visual_namespace_ = "visual/";

      // // start ros node
      // if (!ros::isInitialized())
      // {
      //   int argc = 0;
      //   char** argv = NULL;
      //   ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      // }
      //
      // this->node_handle_ = new ros::NodeHandle(this->visited_visual_namespace_);
      // this->force_sub_ = this->node_handle_->subscribe("/some_force", 1000, &VisualizeVectorPlugin::VisualizeForceOnLink, this);

      this->node_handle_ = transport::NodePtr(new transport::Node());
      this->node_handle_->Init(this->visual_namespace_);

      force_sub_ = node_handle_->Subscribe<msgs::Vector3d>("/vector_component",&VisualizeVectorPlugin::VectorMsgCallback, this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->update_connection_ = event::Events::ConnectRender(
          boost::bind(&VisualizeVectorPlugin::UpdateChild, this));
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void VisualizeVectorPlugin::UpdateChild()
    {
      // ros::spinOnce();
    }

    //////////////////////////////////////////////////////////////////////////////////
    // VisualizeForceOnLink
    void VisualizeVectorPlugin::VisualizeForceOnLink(PointConstPtr &force_msg)
    {
      this->line = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

      // //TODO: Get the current link position
      // link_pose = CurrentLinkPose();
      // //TODO: Get the current end position
      // endpoint = CalculateEndpointOfForceVector(link_pose, force_msg);

      // Add two points to a connecting line strip from link_pose to endpoint
      this->line->AddPoint(
        ignition::math::Vector3d(
          1.0,
          2.0,
          3.0
          // link_pose.position.x,
          // link_pose.position.y,
          // link_pose.position.z
          )
        );
      this->line->AddPoint(ignition::math::Vector3d(1.5,2.5,3.5)); //endpoint.x, endpoint.y, endpoint.z
      // set the Material of the line, in this case to purple
      this->line->setMaterial("Gazebo/Purple");
      this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
      this->visual_->SetVisible(true);
    }

    // Callback of the SubscriberPtr to the test_msg Topic
    void VisualizeVectorPlugin::VectorMsgCallback(PointConstPtr &vector_msg){
      vector_x = vector_msg->x();
      vector_y = vector_msg->y();
      vector_z = vector_msg->z();
    }


    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(VisualizeVectorPlugin)
  }


}
