#ifndef __MAGNETICBOX_HH__
#define __MAGNETICBOX_HH__

#include "ros/ros.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo{
class MagneticBox : public ModelPlugin {
    private:
        std::unique_ptr<ros::NodeHandle> nh;
        std::string modelName;
        physics::WorldPtr world;

    public:
        MagneticBox() : ModelPlugin() {}
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override final;

    private:
        void initRos();
};

GZ_REGISTER_MODEL_PLUGIN(MagneticBox)
}

#endif
