#ifndef __MAGNETICSPHERE_HH__
#define __MAGNETICSPHERE_HH__

#include "ros/ros.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// for Attach service 
#include "ros_msgs/Attach.h"
#include "ros_msgs/AttachRequest.h"
#include "ros_msgs/AttachResponse.h"
// for Detach service 
#include "ros_msgs/Detach.h"
#include "ros_msgs/DetachRequest.h"
#include "ros_msgs/DetachResponse.h"

namespace gazebo{
class MagneticSphere : public ModelPlugin {
    private:
        std::unique_ptr<ros::NodeHandle> nh;
        std::string modelName;
        physics::WorldPtr world;
        physics::ModelPtr model;
        physics::ModelPtr otherSphere;
        physics::JointPtr attachJoint;

    public:
        MagneticSphere() : ModelPlugin() {}
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override final;

    private:
        void initRos();

    private: ros::ServiceServer attachServer;
    private: bool attachService(ros_msgs::Attach::Request&, ros_msgs::Attach::Response&);

    private: ros::ServiceServer detachServer;
    private: bool detachService(ros_msgs::Detach::Request&, ros_msgs::Detach::Response&);
};

GZ_REGISTER_MODEL_PLUGIN(MagneticSphere)
}

#endif
