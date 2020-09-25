#include "MagneticBox.hh"

using namespace gazebo;

void MagneticBox::Load(physics::ModelPtr model, sdf::ElementPtr _sdf) 
{
    this->world = model->GetWorld();
    this->modelName = model->GetName();
    
    this->initRos();

    std::cout << "===================================" << std::endl;
    std::cout << "[Model Plugin] Load" << std::endl;
    std::cout << "Modelname : " << this->modelName<< std::endl;
    std::cout << "===================================" << std::endl;
}

void MagneticBox::initRos() {
    if(!ros::isInitialized()) { 
        int argc = 0; char **argv = NULL;
        std::string nodeName = this->modelName + "_node";

        ros::init(argc, argv, nodeName, ros::init_options::NoRosout); 
        this->nh.reset(new ros::NodeHandle(nodeName));
    }
    else {
        this->nh.reset(new ros::NodeHandle(ros::this_node::getName()));
    }
}
