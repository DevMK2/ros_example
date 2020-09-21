#ifndef __ELEVATOR_HH__
#define __ELEVATOR_HH__

#include "lms_type.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>

namespace gazebo {

class Elevator {
private:
    ID_T id;
    unsigned int bottom, top;
    physics::WorldPtr controller;
    physics::ModelPtr elevator;
    physics::LinkPtr coilLink;

public:
    Elevator(const physics::WorldPtr& world, const physics::ModelPtr& elevator)
    : controller(world), elevator(elevator){
        auto links = elevator->GetLinks();

        for(auto link : links) {
            if(link->GetName().find("coil") != std::string::npos) {
                this->coilLink = link;
            }
        }
    }

    void MoveTo(const float& x, const float& y, const float& z) {
        math::Pose pose = this->coilLink->GetWorldPose();
        //pose.pos.x = x;
        //pose.pos.y = y;
        pose.pos.z = z;
        this->coilLink->SetWorldPose(pose);
    }
};

using ElevatorPtr = std::unique_ptr<Elevator>;

};

#endif
