#include "MagneticSphere.hh"

using namespace gazebo;

/**
 * Plugin을 상속한 객체는 추상 메서드인 Load를 구현해야 합니다. 
 * Load는 객체가 world에 처음 소환될 때 호출되는 메서드입니다.
 *
 * physics::ModelPtr model : 이 플러그인이 삽입된 gazebo 모델 객체의 포인터
 */
void MagneticSphere::Load(physics::ModelPtr model, sdf::ElementPtr _sdf) 
{
    //\ gazebo world 객체의 포인터를 얻어서 초기화.
    this->world = model->GetWorld();
    //\ 빨간공 gazebo model 객체의 포인터로부터 모델 이름을 얻어서 초기화.
    this->modelName = model->GetName();
    //\ 빨간공 gazebo model 객체의 포인터를 얻어서 초기화.
    this->model= model;
    //\ gazebo world 객체 포인터로부터 world 에 존재하는 모든 객체를 얻음.
    auto models = this->world->GetModels();
    for(auto model : models) {
        //\ world에 존재하는 모든 객체중에 
        //  빨간공 객체와 이름이 다르면서 이름에 'sphere'를 포함하는 객체를 찾음.
        std::string name = model->GetName();
        if(name != this->modelName && name.find("sphere") != std::string::npos) {
            //\ 찾은 객체가 회색공 객체이므로 회색공 gazebo model 객체를 초기화.
            this->otherSphere = model;
        }
    }
    //\ 빨간공 모델 객체에 새로운 fixed 조인트를 생성하고
    //  생성한 joint 포인터를 attachJoint에 초기화.
    this->attachJoint = this->world->GetPhysicsEngine()->CreateJoint("fixed", this->model);
    
    //\ ROS node 및 ROS service를 초기화.
    this->initRos();

    std::cout << "===================================" << std::endl;
    std::cout << "[Model Plugin] Load" << std::endl;
    std::cout << "Modelname : " << this->modelName<< std::endl;
    std::cout << "===================================" << std::endl;
}

/**
 * ROS node 및 ROS service를 초기화.
 */
void MagneticSphere::initRos() {
    //\ ROS node 초기화.
    if(!ros::isInitialized()) { 
        //\ ros::init이 호출되지 않은 상태면 모델 이름으로 ROS를 node를 초기화하고
        //  해당 노드의 handle을 생성하여 this->nh에 초기화.
        int argc = 0; char **argv = NULL;
        std::string nodeName = this->modelName + "_node";

        ros::init(argc, argv, nodeName, ros::init_options::NoRosout); 
        this->nh.reset(new ros::NodeHandle(nodeName));
    }
    else {
        //\ ros::init이 다른 프로세스에서 호출 되어있는 상태면
        //  ros node를 새로 초기화하지 않고,
        //  존재하는 노드의 handle을 생성하여 this->nh에 초기화.
        this->nh.reset(new ros::NodeHandle(ros::this_node::getName()));
    }

    //\ attach ROS service를 발행
    this->attachServer = nh->advertiseService(
        "/example/attach", /*서비스 이름*/
        &MagneticSphere::attachService, /*서비스 콜백*/
        this
    );

    //\ detach ROS service를 발행
    this->detachServer = nh->advertiseService(
        "/example/detach", /*서비스 이름*/
        &MagneticSphere::detachService, /*서비스 콜백*/
        this
    );
}

/**
 * 두 gazebo 모델간의 거리의 절대값을 구함.
 */
inline static double distAbs(physics::ModelPtr& from, physics::ModelPtr& to) {
    return std::abs(to->GetWorldPose().pos.Distance(from->GetWorldPose().pos));
}

/**
 * from 으로부터 to 까지의 vector를 구함.
 */
inline static math::Vector3 distVector(physics::ModelPtr& from, physics::ModelPtr& to) {
    return to->GetWorldPose().pos - from->GetWorldPose().pos;
}

/**
 * /example/attach ROS 서비스가 호출됐을때의 동작을 정의하는 콜백.
 * 회색공을 빨간공을 향해 이동시키고,
 * 빨간공과 회색공을 fixed joint로 부착시킨다.
 */
bool MagneticSphere::attachService(
    ros_msgs::Attach::Request& req,
    ros_msgs::Attach::Response& res
) {
    //\ 빨간공과 회색공의 링크의 포인터를 획득
    auto thisSphereLink = this->model->GetLinks()[0];
    auto otherSphereLink = this->otherSphere->GetLinks()[0];

    //\ 빨간공과 회색공의 거리가 1 이하가 될 때까지
    while(distAbs(this->otherSphere, this->model) > 1) {
        //\ 회색공에게 회색공->빨간공 방향으로 힘을 가한다.
        otherSphereLink->AddForce(distVector(this->otherSphere, this->model)/1000);
    }
    //\ 거리가 1이하가 되면 회색공과 빨간공에 가해진 물리상태(힘)을 초기화한다.
    this->model->ResetPhysicsStates();
    this->otherSphere->ResetPhysicsStates();

    //\ 빨간공과 회색공을 attachJoint로 부착시킨다.
    this->attachJoint->Attach(this->model->GetLinks()[0], this->otherSphere->GetLinks()[0]);

    std::cout << "attach!" << std::endl;
    return res.ok = true;
}

/**
 * /example/detach ROS 서비스가 호출됐을때의 동작을 정의하는 콜백.
 */
bool MagneticSphere::detachService(
    ros_msgs::Detach::Request& req, 
    ros_msgs::Detach::Response& res
) {
    //\ attachJoint로 부착된 두 링크를 탈착한다.
    this->attachJoint->Detach();

    //\ 빨간공과 회색공의 physics state를 초기화한다
    this->model->ResetPhysicsStates();
    this->otherSphere->ResetPhysicsStates();

    std::cout << "detach!" << std::endl;
    return res.ok = true;
}
