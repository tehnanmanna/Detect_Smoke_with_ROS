
#ifndef SIMPLE_LINE_FOLLOWER_HH
#define SIMPLE_LINE_FOLLOWER_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class SimpleLineFollower : public ModelPlugin {

    public:
      SimpleLineFollower();
      ~SimpleLineFollower();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:

      physics::WorldPtr world_;
      physics::ModelPtr parent_;
      physics::LinkPtr base_link_;
      event::ConnectionPtr update_connection_;

      // ROS STUFF
      ros::NodeHandle* rosnode_;
      ros::Publisher odometry_publisher_;
      ros::Subscriber cmd_vel_subscriber_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string robot_base_frame_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      double x_vel_;
      double rot_vel_;
      bool alive_;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

      // other stuff
      std::vector<math::Vector3> points_;
      math::Vector3 active_point_;
      int active_point_ind_;
      double velocity_;

  };

}

#endif
