#ifndef __MAGNETICSPHERE_HH__
#define __MAGNETICSPHERE_HH__

//\ for ROS
#include "ros/ros.h"

//\ for Gazebo plugin, Gazebo physics, Gazebo math
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

//\ for Attach service 
#include "ros_msgs/Attach.h"
#include "ros_msgs/AttachRequest.h"
#include "ros_msgs/AttachResponse.h"

//\ for Detach service 
#include "ros_msgs/Detach.h"
#include "ros_msgs/DetachRequest.h"
#include "ros_msgs/DetachResponse.h"

/**
 * 빨간 공의 자석 효과를 구현하는 플러그인입니다.
 * 빌드하여 shared library인 lib_magnetic_sphere.so 를 생성하며,
 * ros_example/src/gazebo/models/magnetic_sphere/model.sdf 파일에 plugin 태그에서 사용합니다.
 */
namespace gazebo{
class MagneticSphere : public ModelPlugin {
    private:
        //\ ROS node 
        std::unique_ptr<ros::NodeHandle> nh;
        //\ 빨간공 gazebo 모델의 이름
        std::string modelName;
        //\ gazebo world 객체의 포인터
        physics::WorldPtr world;
        //\ 빨간공 gazebo model 객체의 포인터
        physics::ModelPtr model;
        //\ 회색공 gazebo model 객체의 포인터
        physics::ModelPtr otherSphere;
        //\ 빨간공과 회색공을 연결할 joint의 포인터
        physics::JointPtr attachJoint;

    public:
        MagneticSphere() : ModelPlugin() {}
        //\ Plugin을 상속한 객체는 추상 메서드인 Load를 구현해야 합니다. 
        //  Load는 객체가 world에 처음 소환될 때 호출되는 메서드입니다.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override final;

    //\ ROS node 및 ROS service를 초기화합니다.
    private: void initRos();

    //\ 회색공을 빨간공 쪽으로 이동시킨 뒤 부착시키는 서비스.
    private: ros::ServiceServer attachServer;
    //\ 서비스가 호출됐을때의 동작을 정의하는 콜백메서드
    private: bool attachService(ros_msgs::Attach::Request&, ros_msgs::Attach::Response&);

    //\ 회색공과 빨간공을 탈착하는 서비스.
    private: ros::ServiceServer detachServer;
    //\ 서비스가 호출됐을때의 동작을 정의하는 콜백메서드
    private: bool detachService(ros_msgs::Detach::Request&, ros_msgs::Detach::Response&);
};

GZ_REGISTER_MODEL_PLUGIN(MagneticSphere)
}

#endif
