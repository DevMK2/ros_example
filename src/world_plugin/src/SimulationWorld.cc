#include "SimulationWorld.hh"
#include <iostream>

using namespace gazebo;

void SimulationWorld::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    this->world = _world;
    this->worldName = _world->GetName();

    std::cout << "===================================" << std::endl;
    std::cout << "[World Plugin] Load" << std::endl;
    std::cout << "Worldname : " << this->worldName << std::endl;
    std::cout << "===================================" << std::endl;
}
