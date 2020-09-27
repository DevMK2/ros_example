#include "SimulationWorld.hh"
#include <iostream>

using namespace gazebo;

void SimulationWorld::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    this->world = _world;
    this->worldName = _world->GetName();

    this->initRos();

    std::cout << "===================================" << std::endl;
    std::cout << "[World Plugin] Load" << std::endl;
    std::cout << "Worldname : " << this->worldName << std::endl;
    std::cout << "===================================" << std::endl;
}

void SimulationWorld::initRos() {
    if(!ros::isInitialized()) { 
        int argc = 0; char **argv = NULL;
        std::string nodeName = this->worldName+ "_node";

        ros::init(argc, argv, nodeName, ros::init_options::NoRosout); 
        this->nh.reset(new ros::NodeHandle(nodeName));
    }
    else {
        this->nh.reset(new ros::NodeHandle(ros::this_node::getName()));
    }


    this->subscriberIsAttached = this->nh->subscribe(
        "/example/is_attached", 3, &SimulationWorld::callbackIsAttached, this
    );
}


void SimulationWorld::callbackIsAttached(const ros_msgs::IsAttached::ConstPtr&) {
    std::cout << "Worldname : " << this->worldName << std::endl;
}
