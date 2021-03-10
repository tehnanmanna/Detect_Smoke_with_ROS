#ifndef GPU_LASER_SENSOR_Particle_PLUGIN_HH
#define GPU_LASER_SENSOR_Particle_PLUGIN_HH

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
// ros message stuff
#include <std_srvs/Empty.h>
#include "simulation_msgs/LaserParticle.h"
// gazebo stuff
#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_gpu_laser.h>
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderTypes.hh"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>

// dynamic reconfigure stuff
//#include <gazebo_plugins/GazeboRosCameraConfig.h>
//#include <dynamic_reconfigure/server.h>

// boost stuff
#include <boost/thread/mutex.hpp>


namespace gazebo
{
  class GpuLaserSensorParticlePlugin : public GazeboRosLaser //, DepthCameraPlugin, GazeboRosCameraUtils
  {
    public:
      GpuLaserSensorParticlePlugin();
      ~GpuLaserSensorParticlePlugin();
      void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate();
      bool serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
      bool serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
      bool changeParticlesService(simulation_msgs::LaserParticle::Request &req,
                                simulation_msgs::LaserParticle::Response &res);

      std::string pose3dToString(ignition::math::Pose3d pose);
    private:
      event::ConnectionPtr updateConnection;
      //sensors::SensorPtr parentSensor;
      bool timeToSensorUpdate();
      bool timeToParticleUpdate();
      bool valuesHasChanged(int num_particle,
                            double particle_size,
                            double horizontal_angle_min,
                            double horizontal_angle_max);
      bool valNotZero(int val);
      bool valNotZero(double val);
      void loadDefaultParticleParams();
      void spawnParticles();
      void respawnParticles();
      void updateParticles();
      void parentSafeName();
      void moveParticles();

      common::Time last_sensor_update_time_;
      common::Time last_particle_update_time_;
      common::Time particle_update_time_;
      double sensor_update_time_;

      //particle values
      sdf::ElementPtr model_;
      physics::LinkPtr parent_link_;
      physics::ModelPtr parent_model_;
      physics::ModelPtr particle_model_;
      double p_range;
      double p_size;
      double p_move_range;
      double p_hor_angle_min_;
      double p_hor_angle_max_;
      double p_ver_angle_min_;
      double p_ver_angle_max_;
      double p_transparency_;
      int num_particles;
      std::vector<double> p_angels;
      ignition::math::Pose3d sensor_pose;
      ignition::math::Pose3d link_pose;
      std::string parent_link_scoped_name;
      std::string parent_save_name_;
      std::string particle_link_name_;

      bool enabled_;
      bool new_particles;
      ignition::math::Pose3d pos_far_away;

      std::string sensor_parent_link_;

      physics::WorldPtr world_;
      sensors::SensorPtr parent_;
      sdf::ElementPtr sdf_;

      //ROS Stuff
      boost::shared_ptr<ros::NodeHandle> nh_;
      ros::ServiceServer enable_service_;
      ros::ServiceServer disable_service_;
      ros::ServiceServer change_service_;
      std::string topic_name;

      //random number generator
      ignition::math::Rand ng;
      // lock for service
      bool lock;
      // Scene Pointer
      rendering::ScenePtr scene;
      /*ros::Publisher pub_;
      void SubConnect();
      void SubDisconnect();
      int connect_count_;
      bool subscriber_;
      void publish_scan();
      common::Time time_;*/
  };// Class End
}
#endif
