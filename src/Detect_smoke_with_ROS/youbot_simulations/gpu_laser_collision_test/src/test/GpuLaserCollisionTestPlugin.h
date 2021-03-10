#ifndef GPU_LASER_SENSOR_COLLISION_PLUGIN_HH
#define GPU_LASER_SENSOR_COLLISION_PLUGIN_HH

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
// ros message stuff
#include <std_srvs/Empty.h>
// gazebo stuff
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorTypes.hh>
// #include <gazebo/plugins/DepthCameraPlugin.hh>
// #include <gazebo_plugins/gazebo_ros_gpu_laser.h>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
// dynamic reconfigure stuff
//#include <gazebo_plugins/GazeboRosCameraConfig.h>
//#include <dynamic_reconfigure/server.h>

// boost stuff
#include <boost/thread/mutex.hpp>
#include <ignition/math/Frustum.hh>

namespace gazebo
{
  class GpuLaserCollisionTestPlugin : public GazeboRosLaser //, DepthCameraPlugin, GazeboRosCameraUtils
  {
    public:
      GpuLaserCollisionTestPlugin();
      ~GpuLaserCollisionTestPlugin();
      void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate();
      bool serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
      bool serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
    private:
      physics::WorldPtr world_;
      sensors::SensorPtr parent_;
      sdf::ElementPtr sdf_;

      //ROS Stuff
      boost::shared_ptr<ros::NodeHandle> nh_;
      ros::ServiceServer enable_service_;
      ros::ServiceServer disable_service_;
      
      common::Time last_sensor_update_time_;
      double sensor_update_time_;
      event::ConnectionPtr updateConnection;
      bool timeToSensorUpdate();
      
      ignition::math::Frustum frustum;
  }
}

#endif
