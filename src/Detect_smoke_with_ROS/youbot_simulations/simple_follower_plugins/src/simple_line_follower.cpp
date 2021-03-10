#include <algorithm>
#include <assert.h>
#include <random>

#include <simple_follower_plugins/simple_line_follower.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <gazebo/gazebo_config.h>

namespace gazebo {


  SimpleLineFollower::SimpleLineFollower() {}

  // Destructor
  SimpleLineFollower::~SimpleLineFollower() {
    delete rosnode_;
  }

  // Load the controller
  void SimpleLineFollower::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

    this->parent_ = _parent;
    this->world_ = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO("SimpleLineFollower Plugin missing <robotNamespace>, defaults to \"%s\"", 
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ = 
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    this->robot_base_frame_ = "base_link";
    if (!_sdf->HasElement("robotBaseFrame")) {
      ROS_WARN("SimpleLineFollower Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
      this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }
    
    try{
        base_link_ = this->parent_->GetChildLink(this->robot_base_frame_);
    }catch(...){
        ROS_ERROR("SimpleLineFollower Plugin cant get robot_base_frame Link %s",robot_base_frame_.c_str());
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN("SimpleLineFollower Plugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }

    this->velocity_ = 1.0;
    if (!_sdf->HasElement("Velocity")) {
      ROS_WARN("SimpleLineFollower Plugin using 1.0 m/s for velocity");
    } else {
      this->velocity_ = _sdf->GetElement("Velocity")->Get<double>();
    }
    
    std::string temp = "";
    if (!_sdf->HasElement("pointList"))
        {
            ROS_FATAL_STREAM("No Points are given -> nothing will happen");
            return;
        }
    else
        temp = _sdf->GetElement("pointList")->Get<std::string>();

    // Nasty string parsing into vector begins
    std::string delimiter = ",";            // the pointList should look like this:
    std::vector<std::string> temp_points;   // <pointList>0 1 0,1 1 1,2.1 2.2 1.1, ... </pointList>
    size_t pos = 0;
    while ((pos = temp.find(delimiter)) != std::string::npos) {
        temp_points.push_back(temp.substr(0, pos));
        temp.erase(0, pos + delimiter.length());
    }
    delimiter = " ";
    math::Vector3 vec;
    for (std::vector<int>::size_type i = 0; i != temp_points.size(); i++){
        temp = temp_points[i];

        pos = 0;
        int ind = 0;
        while ((pos = temp.find(delimiter)) != std::string::npos) {
            if (ind == 0)
                vec.x = math::parseFloat(temp.substr(0, pos));
            else if (ind == 1)
                vec.y = math::parseFloat(temp.substr(0, pos));
            else if (ind == 2)
                vec.z = math::parseFloat(temp.substr(0, pos));

            temp.erase(0, pos + delimiter.length());
            ind++;
        }
        
        points_.push_back(vec);
    }
    // Nasty string parsing ends

    // Initialize update rate
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
    last_update_time_ = this->world_->GetSimTime();
    ROS_INFO("## Start Pose ## X: %f  Y: %f  Z: %f", points_[0].x, points_[0].y, points_[0].z);
    alive_ = true;

    active_point_ = points_[0];
    this->active_point_ind_ = 0;
    math::Pose pose;

    math::Vector3 diff_vec = points_[1] - active_point_;
    double angle = atan2( diff_vec[1], diff_vec[0]);

    pose.pos = active_point_;
    pose.pos.z = 0.3;
    pose.rot = math::Quaternion(0.0, 0.0, angle);

    base_link_->SetWorldPose(pose);
    active_point_ = points_[1];
    active_point_ind_ = 1;

    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ROS_INFO("Starting SimpleLineFollower Plugin (ns = %s)!", this->robot_namespace_.c_str());

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&SimpleLineFollower::UpdateChild, this));
  }

  // Update the controller
  void SimpleLineFollower::UpdateChild() {
    common::Time current_time = this->world_->GetSimTime();
    double seconds_since_last_update = (current_time - last_update_time_).Double();

    if (seconds_since_last_update > update_period_)
    {
        //ROS_INFO("Update");
        math::Pose pose = base_link_->GetWorldPose();
        if(pose.pos.z <= 0.2)
        {
            pose.pos.z = 0.3;
            base_link_->SetWorldPose(pose);
        }
        math::Vector3 diff_vec = active_point_ - pose.pos;
        diff_vec.z = 0.0;
        if (diff_vec.GetLength() < 0.1)
        {
            active_point_ind_+=1;
            if (active_point_ind_>points_.size())
            {
                active_point_ind_ = 0;
            }
            active_point_ = points_[active_point_ind_];
            
        }
        //double angle = atan2( diff_vec[1], diff_vec[0]);
        double curr_angle = atan2( diff_vec[1], diff_vec[0]) - pose.rot.GetAsEuler()[2];
        math::Vector3 ang_vel_vec;
        ang_vel_vec.Set(0.0, 0.0, curr_angle/abs(curr_angle));
        math::Vector3 vel_vec = diff_vec.Normalize();
        vel_vec*= velocity_;
        //ROS_INFO("Setting Velocity X: %f  Y: %f  Z: %f  ", vel_vec.x, vel_vec.y, vel_vec.z);
        base_link_->SetWorldTwist(vel_vec, ang_vel_vec);


        last_update_time_+= common::Time(update_period_);

    }
  }

  // Finalize the controller
  void SimpleLineFollower::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void SimpleLineFollower::QueueThread() {
    static const double timeout = 0.01;

    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(SimpleLineFollower)
}
