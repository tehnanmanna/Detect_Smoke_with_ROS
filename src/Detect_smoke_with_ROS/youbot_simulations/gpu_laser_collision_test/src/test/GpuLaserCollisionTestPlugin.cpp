#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <math.h>
#include "GpuLaserCollisionTestPlugin.h"
#include <vector>
#include <boost/algorithm/string/replace.hpp>

namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(GpuLaserSensorParticlePlugin)
  
  GpuLaserCollisionTestPlugin::GpuLaserCollisionTestPlugin()
  {
      this->sensor_update_time_ = 0.0;
      this->last_sensor_update_time_ = common::Time(0);
  }
  
  GpuLaserCollisionTestPlugin::~GpuLaserCollisionTestPlugin()
  {
    this->nh_->shutdown();
  }
  
  void GpuLaserCollisionTestPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
      if (!ros::isInitialized())
      {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
          return;
      }
      this->nh_.reset(new ros::NodeHandle("~"));
      // Store the model pointer for convenience.
      this->parent_ = std::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);
      this->sdf_ = _sdf;
# if GAZEBO_MAJOR_VERSION >= 7
      std::string worldName = parent_->WorldName();
# else
      std::string worldName = parent_->GetWorldName();
# endif
     this->world_ = physics::get_world(worldName);
     GazeboRosLaser::Load(_parent,this->sdf_);
     
     
  }
  
  void GpuLaserCollisionTestPlugin::OnUpdate()
  {
      if(this->timeToSensorUpdate())
      {
          // TODO
      }
  }
  
  bool GpuLaserCollisionTestPlugin::serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res)
  {}
  
  bool GpuLaserCollisionTestPlugin::serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
  {}
  
  bool GpuLaserCollisionTestPlugin::timeToSensorUpdate()
  {}
  
}
