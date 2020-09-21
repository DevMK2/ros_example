#ifndef __SIMULATION_WORLD_HH__
#define __SIMULATION_WORLD_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>
#include <ros/ros.h>

namespace gazebo{
class SimulationWorld : public WorldPlugin {
private:
    physics::WorldPtr world;
    std::string worldName;

public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override final;
};

GZ_REGISTER_WORLD_PLUGIN(SimulationWorld)
} 

#endif
