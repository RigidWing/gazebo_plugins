#ifndef GAZEBO_PLUGINS_CONTROLINPUTPLUGIN_HH_
#define GAZEBO_PLUGINS_CONTROLINPUTPLUGIN_HH_

#include <mutex>

#include <ignition/math/Angle.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include "common.h"
#include <string>
#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Assert.hh"
#include <typeinfo>
#include <functional>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>

namespace gazebo
{

class GAZEBO_VISIBLE ControlInputPlugin : public GUIPlugin
{

  Q_OBJECT

  public: ControlInputPlugin();

  public: virtual ~ControlInputPlugin();

  private: double elevatorAngleStep;

  private: double rudderAngleStep;

  private slots: void OnIncreaseElevator();

  private slots: void OnDecreaseElevator();

  private slots: void OnIncreaseRudder();

  private slots: void OnDecreaseRudder();

  // Transport variables
  private: transport::NodePtr node;
  private: transport::PublisherPtr controlInputPub;
  private: msgs::Vector3d control_input_msg;

  // mutex
  private: std::mutex mutex;
};

}

#endif
