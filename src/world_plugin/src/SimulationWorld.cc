#include "SimulationWorld.hh"
#include <iostream>

using namespace gazebo;

/**
 * Plugin을 상속한 객체는 추상 메서드인 Load를 구현해야 합니다. 
 * Load는 객체가 world에 처음 소환될 때 호출되는 메서드입니다.
 *
 * physics::WorldPtr _world : 이 플러그인이 삽입된 world 의 포인터
 */
void SimulationWorld::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    //\ gazebo world 의 포인터로부터 world의 이름을 얻어옵니다.
    this->worldName = _world->GetName();

    //\ world가 처음 시뮬레이션 상에 나타난 시점에 콘솔에 출력합니다.
    std::cout << "===================================" << std::endl;
    std::cout << "[World Plugin] Load" << std::endl;
    std::cout << "Worldname : " << this->worldName << std::endl;
    std::cout << "===================================" << std::endl;
}
