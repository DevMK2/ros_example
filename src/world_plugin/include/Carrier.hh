#ifndef __CARRIER_HH__
#define __CARRIER_HH__

#include "lms_type.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>

namespace gazebo {

class Carrier {
private:
    physics::WorldPtr controller;
    physics::ModelPtr carrier;

public:
    Carrier(const physics::WorldPtr& world, const physics::ModelPtr& carrier)
    : carrier(carrier), controller(world) {
    }

    void MoveTo(const float& x, const float& y, const float& z) {
        math::Pose pose = carrier->GetWorldPose();
        pose.pos.x = x;
        pose.pos.y = y;
        pose.pos.z = z;
        carrier->SetWorldPose(pose);
    }
};

using CarrierPtr = std::unique_ptr<Carrier>;

};

#endif
