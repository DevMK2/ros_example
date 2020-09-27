#include "MagneticSphere.hh"

using namespace gazebo;

void MagneticSphere::Load(physics::ModelPtr model, sdf::ElementPtr _sdf) 
{
    this->world = model->GetWorld();
    this->modelName = model->GetName();
    this->model= model;
    auto models = this->world->GetModels();
    for(auto model : models) {
        std::string name = model->GetName();
        if(name != this->modelName && name.find("sphere") != std::string::npos) {
            this->otherSphere = model;
        }
    }
    this->attachJoint = this->world->GetPhysicsEngine()->CreateJoint("fixed", this->model);
    
    this->initRos();

    std::cout << "===================================" << std::endl;
    std::cout << "[Model Plugin] Load" << std::endl;
    std::cout << "Modelname : " << this->modelName<< std::endl;
    std::cout << "===================================" << std::endl;
}

void MagneticSphere::initRos() {
    if(!ros::isInitialized()) { 
        int argc = 0; char **argv = NULL;
        std::string nodeName = this->modelName + "_node";

        ros::init(argc, argv, nodeName, ros::init_options::NoRosout); 
        this->nh.reset(new ros::NodeHandle(nodeName));
    }
    else {
        this->nh.reset(new ros::NodeHandle(ros::this_node::getName()));
    }

    this->attachServer = nh->advertiseService(
        "/example/attach", /*서비스 이름*/
        &MagneticSphere::attachService, /*서비스 콜백*/
        this
    );

    this->detachServer = nh->advertiseService(
        "/example/detach", /*서비스 이름*/
        &MagneticSphere::detachService, /*서비스 콜백*/
        this
    );
}

inline static double distAbs(physics::ModelPtr& from, physics::ModelPtr& to) {
    return std::abs(to->GetWorldPose().pos.Distance(from->GetWorldPose().pos));
}

inline static math::Vector3 distVector(physics::ModelPtr& from, physics::ModelPtr& to) {
    return to->GetWorldPose().pos - from->GetWorldPose().pos;
}

bool MagneticSphere::attachService(
    ros_msgs::Attach::Request& req,
    ros_msgs::Attach::Response& res
) {
    auto thisSphereLink = this->model->GetLinks()[0];
    auto otherSphereLink = this->otherSphere->GetLinks()[0];


    while(distAbs(this->otherSphere, this->model) > 1) {
        otherSphereLink->AddForce(distVector(this->otherSphere, this->model)/1000);
    }

    thisSphereLink->SetForce(math::Vector3::Zero);
    otherSphereLink->SetForce(math::Vector3::Zero);
    this->model->ResetPhysicsStates();
    this->otherSphere->ResetPhysicsStates();

    this->attachJoint->Attach(this->model->GetLinks()[0], this->otherSphere->GetLinks()[0]);
    this->attachJoint->Init();
    std::cout << "attach!" << std::endl;
    return res.ok = true;
}

bool MagneticSphere::detachService(
    ros_msgs::Detach::Request& req, 
    ros_msgs::Detach::Response& res
) {
    this->attachJoint->Detach();
    this->model->ResetPhysicsStates();
    this->otherSphere->ResetPhysicsStates();

    std::cout << "detach!" << std::endl;
    return res.ok = true;
}
