#include <algorithm>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/Actions.hh>
#include "control_input_plugin.hh"


using namespace gazebo;


GZ_REGISTER_GUI_PLUGIN(ControlInputPlugin)


ControlInputPlugin::ControlInputPlugin()
  : GUIPlugin()
{
  this->elevatorAngleStep = 0.0174533; // 1 degree in radians
  this->rudderAngleStep = 0.0174533; // 1 degree in radians

  //////////////////////////////////////////////////////////////////////////////
  //Initialize transport
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->controlInputPub = this->node->Advertise<msgs::Vector3d>("/control_inputs", 1);
  //////////////////////////////////////////////////////////////////////////////
  //Setup the hotkeys

  // QShortcut *increaseThrust = new QShortcut(QKeySequence("w"), this);
  // QObject::connect(increaseThrust, SIGNAL(activated()), this,
  //     SLOT(OnIncreaseThrust()));
  //
  // QShortcut *decreaseThrust = new QShortcut(QKeySequence("s"), this);
  // QObject::connect(decreaseThrust, SIGNAL(activated()), this,
  //     SLOT(OnDecreaseThrust()));

  QShortcut *increaseElevator =
    new QShortcut(QKeySequence(Qt::Key_Down), this);
  QObject::connect(increaseElevator, SIGNAL(activated()), this,
      SLOT(OnIncreaseElevator()));

  QShortcut *decreaseElevator = new QShortcut(QKeySequence(Qt::Key_Up), this);
  QObject::connect(decreaseElevator, SIGNAL(activated()), this,
      SLOT(OnDecreaseElevator()));

  QShortcut *increaseRudder = new QShortcut(QKeySequence("d"), this);
  QObject::connect(increaseRudder, SIGNAL(activated()), this,
      SLOT(OnIncreaseRudder()));

  QShortcut *decreaseRudder = new QShortcut(QKeySequence("a"), this);
  QObject::connect(decreaseRudder, SIGNAL(activated()), this,
      SLOT(OnDecreaseRudder()));

  // TODO: Add other functionalities such as for particular preset maneuvers
}


////////////////////////////////////////////////////////////////////////////////
// Deconstructor
ControlInputPlugin::~ControlInputPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// The functions

// void ControlInputPlugin::OnIncreaseThrust()
//
// void ControlInputPlugin::OnDecreaseThrust()

void ControlInputPlugin::OnIncreaseElevator()
{
  gzdbg << "Entered OnIncreaseElevator function .. " << "\n";
  control_input_msg.set_x(this->elevatorAngleStep);
  this->controlInputPub->Publish(control_input_msg);
}

void ControlInputPlugin::OnDecreaseElevator()
{
  gzdbg << "Entered OnDecreaseElevator function .. " << "\n";
  control_input_msg.set_x(-1*this->elevatorAngleStep);
  this->controlInputPub->Publish(control_input_msg);
}

void ControlInputPlugin::OnIncreaseRudder()
{
  gzdbg << "Entered OnIncreaseRudder function .. " << "\n";
  control_input_msg.set_y(this->rudderAngleStep);
  this->controlInputPub->Publish(control_input_msg);
}

void ControlInputPlugin::OnDecreaseRudder()
{
  gzdbg << "Entered OnDecreaseRudder function .. " << "\n";
  control_input_msg.set_y(-1*this->rudderAngleStep);
  this->controlInputPub->Publish(control_input_msg);
}
