#include "verification_components_plugin.hh"

#include <algorithm>
#include <string>
#include<string.h>
#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
// KITEPOWER (Xander)
#include "common.h"

#include <iostream>

using namespace std;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(VerificationComponentsPlugin)


VerificationComponentsPlugin::VerificationComponentsPlugin()
{
}


VerificationComponentsPlugin::~VerificationComponentsPlugin()
{
}


void VerificationComponentsPlugin::Load(physics::ModelPtr _model,sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragWithLookupPlugin _model pointer is NULL");
  this->model = _model;
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
  boost::bind(&VerificationComponentsPlugin::OnUpdate, this));
}

void VerificationComponentsPlugin::OnUpdate()
{
  gzdbg << "==============================================================\n";
  gzdbg << "**************************************************************\n";
  gzdbg << "OnUpdate Verification Components Plugin \n";

  this->links = this->model->GetLinks();

  auto size = this->links.size();
  gzdbg << "Size of links is " << size;

  // Reset the values to zero
  this->modelMass = 0;
  this->positionMassProductTotal = 0;
  for (int i=0; i<this->links.size(); i++)
  {
    this->currentLink = this->links[i];
    gzdbg << "******************************\n";
    gzdbg << "Name of link " << this->currentLink->GetName() << "\n";
    gzdbg << "Center of gravity " << this->currentLink->WorldCoGPose().Pos() << "\n";
    gzdbg << "Yet again CoG " << this->currentLink->GetInertial()->CoG() << "\n";
    gzdbg << "Experienced force " << this->currentLink->WorldForce() << "\n";
    gzdbg << "Experienced moment " << this->currentLink->WorldTorque() << "\n";
    gzdbg << "Velocity " << this->currentLink->WorldLinearVel() << "\n";
    gzdbg << "Angular velocity " << this->currentLink->WorldAngularVel() << "\n";
    gzdbg << "Mass " << this->currentLink->GetInertial()->Mass() << "\n";
    gzdbg << "Izz " << this->currentLink->GetInertial()->IZZ() << "\n";

    this->modelMass+= this->currentLink->GetInertial()->Mass();
    this->positionMassProductTotal+= this->currentLink->GetInertial()->CoG() * this->currentLink->GetInertial()->Mass();
    this->currentBoundingBox = this->currentLink->BoundingBox();
    auto currentBoxSize = this->currentBoundingBox.Size();
    gzdbg << "Size of bounding box of link " << currentBoxSize << "\n";
    gzdbg << "With the x component being " << this->currentBoundingBox.XLength() << "\n";
    gzdbg << "With the y component being " << this->currentBoundingBox.YLength() << "\n";
    gzdbg << "With the z component being " << this->currentBoundingBox.ZLength() << "\n";
  }
  gzdbg << "==============================================================\n";
  gzdbg << "==============================================================\n";

  this->modelCoG = this->positionMassProductTotal/this->modelMass;
  this->modelLinearVel = this->model->WorldLinearVel();
  this->modelAngularVel = this->model->WorldAngularVel();
  this->modelLinearAccel = this->model->WorldLinearAccel();
  this->modelAngularAccel = this->model->WorldAngularAccel();
  this->calculatedModelForce = this->modelLinearAccel * this->modelMass;

  // this->modelInertia.Set(0,0,0,0,0,0,0,0,0);
  this->IzzModel=0;
  for (int i=0; i<this->links.size(); i++)
  {
    this->currentLink = this->links[i];
    ignition::math::Vector3d r = this->modelCoG - this->currentLink->GetInertial()->CoG();
    double perpDistanceSquared = r.X()*r.X() + r.Y()*r.Y();
    // steiner theorm
    this->IzzModel+= this->currentLink->GetInertial()->IZZ() + perpDistanceSquared *  this->currentLink->GetInertial()->Mass();

    // this->inertiaComponentAboutOverallCoG = r.Dot(r.transpose()) * this->currentLink->GetInertial()->Mass() + this->currentLink->GetInertial();
    // this->modelInertia.operator+(this->inertiaComponentAboutOverallCoG);

    // gzdbg << "Obtaining the Collision State \n";
    // physics::LinkState this->currentLinkState(this->currentLink);
    // gzdbg << "Collision States " << this->currentLinkState->GetCollisionStates() << "\n";

    gzdbg << "Self Collide status of "<< this->currentLink->GetName() << " is " << this->currentLink->GetSelfCollide() << "\n";
  }

  gzdbg << "IzzModel is " << this->IzzModel << "\n";

  this->calculatedModelMoment = this->modelAngularAccel * this->modelInertia;


  gzdbg << "Mass of model " << this->modelMass << "\n";
  gzdbg << "Position mass product " << this->positionMassProductTotal << "\n";
  gzdbg << "Model cog " << this->modelCoG << "\n";
  gzdbg << "Model linear Vel " << this->modelLinearVel << "\n";
  gzdbg << "Model Angular Vel " << this->modelAngularVel << "\n";
  gzdbg << "Model Linear Acc " << this->modelLinearAccel << "\n";
  gzdbg << "Model Angular Acc " << this->modelAngularAccel << "\n";
  gzdbg << "Moment about the z axis " << this->modelAngularAccel.Z()*this->IzzModel << "\n";
}
