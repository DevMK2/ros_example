#ifndef __SIMULATION_WORLD_HH__
#define __SIMULATION_WORLD_HH__

//\ for Gazebo plugin, Gazebo physics, Gazebo math
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

/**
 * gazebo world의 플러그인입니다.
 * 빌드하여 shared library인 lib_simulation_world.so 를 생성하며,
 * ros_example/src/gazebo/world/simulation.world 파일에 plugin 태그에서 사용합니다.
 */
namespace gazebo{
class SimulationWorld : public WorldPlugin {
    private:
        //\ simulation world의 이름
        std::string worldName;

    public:
        //\ Plugin을 상속한 객체는 추상 메서드인 Load를 구현해야 합니다. 
        //  Load는 객체가 world에 처음 소환될 때 호출되는 메서드입니다.
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override final;
};

GZ_REGISTER_WORLD_PLUGIN(SimulationWorld)
} 

#endif
