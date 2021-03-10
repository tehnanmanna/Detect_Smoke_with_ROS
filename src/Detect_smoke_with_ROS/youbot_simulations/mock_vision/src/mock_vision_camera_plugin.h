#ifndef MOCK_VISION_CAMERA_PLUGIN_HH
#define MOCK_VISION_CAMERA_PLUGIN_HH

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
// ros messages stuff
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>
// gazebo stuff
#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>

#include "observer/ObserverLinkStates.h"
// dynamic reconfigure stuff
//#include <gazebo_plugins/GazeboRosCameraConfig.h>
//#include <dynamic_reconfigure/server.h>

// boost stuff
#include <boost/thread/mutex.hpp>

// camera stuff
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{
  class MockVisionCameraPlugin : public SensorPlugin //DepthCameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public:
      MockVisionCameraPlugin();
      ~MockVisionCameraPlugin();
      void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
      void OnUpdate();
      bool serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
      bool serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
    private:
      event::ConnectionPtr updateConnection;
      //sensors::SensorPtr parentSensor;
      void fillCameraInfo();
      void computeObjectPoses();
      void checkForNewObjects();
      void publishCameraInfo();
      bool nameMatch(std::string name);
      bool timeToSensorUpdate();
      bool timeToObjectUpdate();

      common::Time last_update_time_;
      common::Time last_info_update_time_;
      common::Time last_object_update_time_;
      //common::Time last_update_time_;
      unsigned int height_;
      unsigned int width_;
      unsigned int object_count_;
      unsigned int active_object_count_;
      float clip_near_;
      float clip_far_;
      float horizontal_opening_angle_;
      float vertical_opening_angle_;
      float custom_object_update_time_;
      float sensor_update_time_;
      bool use6D_;
      bool enabled_;
      bool use_costum_reference_frame_;
      std::string camera_parent_link_;
      std::string costumReferenceFrameName_;
      std::string image_topic_name_;
      std::string camera_info_topic_name_;
      std::string distortion_model_;
      std::vector<std::string> object_names_;
      std::vector<std::string> names_;
      std::vector<double> distortion_params_;
      std::vector<double> camera_params_;
      sdf::ElementPtr objects_;
      physics::WorldPtr world_;
      sensors::SensorPtr parent_;
      sdf::ElementPtr sdf_;
      sensor_msgs::CameraInfo camera_info_;
      //ROS Stuff
      boost::shared_ptr<ros::NodeHandle> nh_;
      ros::Publisher     pub_link_states_;
      ros::Publisher     pub_camera_info_;
      ros::ServiceServer enable_service_;
      ros::ServiceServer disable_service_;
  };// Class End
}
#endif









