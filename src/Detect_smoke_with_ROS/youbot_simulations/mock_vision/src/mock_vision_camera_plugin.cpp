
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <math.h>
#include "mock_vision_camera_plugin.h"


namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(MockVisionCameraPlugin)
    //Constructor
    MockVisionCameraPlugin::MockVisionCameraPlugin() : SensorPlugin()
    {
        this->last_update_time_ = common::Time(0);
        this->last_info_update_time_ = common::Time(0);
        this->last_object_update_time_ = common::Time(0);
        this->height_ = 0;
        this->width_ = 0;
        this->object_count_ = 0;
        this->active_object_count_ = 0;
    }
    //Destructor
    MockVisionCameraPlugin::~MockVisionCameraPlugin()
    {
        this->nh_->shutdown();
        //delete this->nh_;
    }

    void MockVisionCameraPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }
        this->nh_.reset(new ros::NodeHandle("~"));
        // Store the model pointer for convenience.
        this->parent_ = std::dynamic_pointer_cast<sensors::Sensor>(_parent);
        this->sdf_ = _sdf;

        if (!_sdf->HasElement("imageTopicName"))
            this->image_topic_name_ = "mockVision/objects";
        else
            this->image_topic_name_ = _sdf->GetElement("imageTopicName")->Get<std::string>();
        if (!_sdf->HasElement("cameraInfoTopicName"))
            this->camera_info_topic_name_ = "mockVision/camera_info";
        else
            this->camera_info_topic_name_ = _sdf->GetElement("camerInfoTopicName")->Get<std::string>();
        if (!_sdf->HasElement("clipNear"))
            this->clip_near_ = 0.1;
        else
            this->clip_near_ = _sdf->GetElement("clip_near")->Get<double>();
        if (!_sdf->HasElement("clipFar"))
            this->clip_far_ = 5.0;
        else
            this->clip_far_ = _sdf->GetElement("clip_far")->Get<double>();
        if (!_sdf->HasElement("frameRate"))
            this->sensor_update_time_ = 1.0/50.0;
        else
            this->sensor_update_time_ = 1.0/(_sdf->GetElement("frameRate")->Get<double>());
        if (!_sdf->HasElement("horizontalOpeningAngle"))
            this->horizontal_opening_angle_ = 3.14;
        else
            this->horizontal_opening_angle_  = _sdf->GetElement("horizontalOpeningAngle")->Get<double>();
        if (!_sdf->HasElement("verticalOpeningAngle"))
            this->vertical_opening_angle_ = 3.14;
        else
            this->vertical_opening_angle_  = _sdf->GetElement("verticalOpeningAngle")->Get<double>();
        if(!_sdf->HasElement("publish6DPose"))
            this->use6D_ = false;
        else
            this->use6D_ = _sdf->GetElement("publish6DPose")->Get<bool>();
        if(!_sdf->HasElement("startEnabled"))
            this->enabled_ = false;
        else
            this->enabled_ = _sdf->GetElement("startEnabled")->Get<bool>();
        if(!_sdf ->HasElement("customObjectUpdateTime"))
            this->custom_object_update_time_ = 10.0;
        else
            this->custom_object_update_time_ = _sdf->GetElement("customObjectUpdateTime")->Get<double>();
        if(!_sdf ->HasElement("height"))
            this->height_ = 480;
        else
            this->height_ = _sdf->GetElement("height")->Get<int>();
        if(!_sdf ->HasElement("width"))
            this->width_ = 600;
        else
            this->width_ = _sdf->GetElement("width")->Get<int>();
        if(!_sdf ->HasElement("distortion_model"))
            this->distortion_model_ = "plumb_bob";
        else
            this->distortion_model_ = _sdf->GetElement("distortion_model")->Get<std::string>();
        if(!_sdf ->HasElement("use_distortion")){
            this->distortion_params_.push_back(0.00000001);
            this->distortion_params_.push_back(0.00000001);
            this->distortion_params_.push_back(0.00000001);
            this->distortion_params_.push_back(0.00000001);
            this->distortion_params_.push_back(0.00000001);}
        else{
            this->distortion_params_.push_back(_sdf->GetElement("distortionK1")->Get<double>());
            this->distortion_params_.push_back(_sdf->GetElement("distortionK2")->Get<double>());
            this->distortion_params_.push_back(_sdf->GetElement("distortionK3")->Get<double>());
            this->distortion_params_.push_back(_sdf->GetElement("distortionT1")->Get<double>());
            this->distortion_params_.push_back(_sdf->GetElement("distortionT2")->Get<double>());}
        if(!_sdf ->HasElement("use_cam_params")){
            this->camera_params_.push_back(500.0);
            this->camera_params_.push_back(0.0);
            this->camera_params_.push_back(300.0);
            this->camera_params_.push_back(0.0);
            this->camera_params_.push_back(500.0);
            this->camera_params_.push_back(220.0);
            this->camera_params_.push_back(0.0);
            this->camera_params_.push_back(0.0);
            this->camera_params_.push_back(1.0);}
        else{
            this->camera_params_.push_back(_sdf->GetElement("cam_paramK1")->Get<double>());
            this->camera_params_.push_back(_sdf->GetElement("cam_paramK2")->Get<double>());
            this->camera_params_.push_back(_sdf->GetElement("cam_paramK3")->Get<double>());
            this->camera_params_.push_back(_sdf->GetElement("cam_paramK4")->Get<double>());
            this->camera_params_.push_back(_sdf->GetElement("cam_paramK5")->Get<double>());
            this->camera_params_.push_back(_sdf->GetElement("cam_paramK6")->Get<double>());
            this->camera_params_.push_back(_sdf->GetElement("cam_paramK7")->Get<double>());
            this->camera_params_.push_back(_sdf->GetElement("cam_paramK8")->Get<double>());
            this->camera_params_.push_back(_sdf->GetElement("cam_paramK9")->Get<double>());}
        if(!_sdf->HasElement("cameraParentLink"))
        {
            this->camera_parent_link_ = _parent->ScopedName();
            ROS_WARN("Camera Parent Link %s",this->camera_parent_link_.c_str());
        }
        else
            this->camera_parent_link_ = _sdf->GetElement("cameraParentLink")->Get<std::string>();
        if(!_sdf->HasElement("useCostumReferenceFrame"))
            {this->use_costum_reference_frame_ = false;}
        else
            {
                this->use_costum_reference_frame_ = _sdf->GetElement("useCostumReferenceFrame")->Get<bool>();
                if(this->use_costum_reference_frame_ && _sdf->HasElement("costumReferenceFrameName"))
                    {this->costumReferenceFrameName_ = _sdf->GetElement("costumReferenceFrameName")->Get<std::string>();}
                else
                {ROS_INFO("No Custom Reference Frame given, will use Camera_Parent as RefFrame");
                this->use_costum_reference_frame_ = false;
                }

            }
        if (!_sdf->HasElement("objectList"))
            {
                ROS_FATAL_STREAM("No Objects given -> nothing will be published");
                return;
            }
        else
            this->objects_ = _sdf->GetElement("objectList");
        while(objects_->HasElement("object")){
            std::string object = objects_->GetFirstElement()->Get<std::string>();
            ROS_INFO("Object %s",object.c_str());
            this->object_names_.push_back(object);
            object_count_++;
            objects_->RemoveChild(objects_->GetFirstElement());
        }
        //for (int i = 0; i < object_count_; i++){ROS_INFO("Object %s in our List",object_names_[i].c_str());}

        // TODO: camera_info publisher
        this->pub_link_states_ =  this->nh_->advertise<observer::ObserverLinkStates>(this->image_topic_name_, 1);
        this->pub_camera_info_ =  this->nh_->advertise<sensor_msgs::CameraInfo>(this->camera_info_topic_name_, 1);

        this->enable_service_ = this->nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
                                                            ("/mockVision/enable",
                                                            boost::bind(&MockVisionCameraPlugin::serviceEnable,this,_1,_2));
        this->disable_service_ = this->nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
                                                            ("/mockVision/disable",
                                                            boost::bind(&MockVisionCameraPlugin::serviceDisable,this,_1,_2));
     # if GAZEBO_MAJOR_VERSION >= 7
        std::string worldName = parent_->WorldName();
     # else
        std::string worldName = parent_->GetWorldName();
     # endif
       this->world_ = physics::get_world(worldName);
       this->last_update_time_ = this->world_->SimTime();
       this->last_object_update_time_ = this->world_->SimTime();
       this->last_info_update_time_ = this->world_->SimTime();
       this->fillCameraInfo();
       // Connect to the sensor update event.
       this->updateConnection = this->parent_->ConnectUpdated(
      std::bind(&MockVisionCameraPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
       this->parent_->SetActive(true);
       ROS_INFO("MockVisionCameraPlugin Loaded");
     }// Load() Ende
    
    void MockVisionCameraPlugin::OnUpdate()
    {

        if(this->timeToObjectUpdate()){
            this->checkForNewObjects(); //all unique names of Manipulation-Objects are in vector this->names_
            //ROS_INFO("Checking for new Objects");
        }
        publishCameraInfo();

        if (this->enabled_ && this->timeToSensorUpdate())
        {
            ignition::math::Pose3d link_pose;
            unsigned int found_objects = 0;
            physics::LinkPtr link;
            observer::ObserverLinkStates model_states; //use observer/ModelStates.msg since it has time stamp

            if(this->use_costum_reference_frame_)
            {
                link_pose = boost::dynamic_pointer_cast<gazebo::physics::Link>
                        (world_->EntityByName(this->costumReferenceFrameName_))->WorldPose();
            }
            else
            {
                link_pose = boost::dynamic_pointer_cast<gazebo::physics::Link>
                        (world_->EntityByName(this->camera_parent_link_))->WorldPose();
            }

            ignition::math::Pose3d pose;
            for(unsigned int i = 0; i < this->active_object_count_;i++)
            {
                //ROS_INFO("Looking for Object: %s",names_[i].c_str());
                link = this->world_->ModelByName(names_[i])->GetLink("link");
                pose = link->WorldPose();
                //compute relative pose to the camera link
                pose.Pos() = pose.Pos() - link_pose.Pos();
                pose.Pos() = link_pose.Rot().RotateVectorReverse(pose.Pos());
                pose.Rot() *= link_pose.Rot().Inverse();

                //TODO:
                // we have pos+rot of camera parent + pos of object
                // problem dont know rot of camera -> use sensor_pose = this->parent_->Pose(); to get the pose of the sensor
                // (in youbot.urdf: yz = horizontal ; xz = vertical, z-axis = optical axis (0, 0, 1) ) 

                float lenght;
                lenght = sqrt(std::pow(pose.Pos().X(),2) + std::pow(pose.Pos().Y(),2) +std::pow(pose.Pos().Z(),2));
                ignition::math::Vector3d xz_vec(pose.Pos().X(), 0, pose.Pos().Z()); // vertical
                ignition::math::Vector3d yz_vec(0, pose.Pos().Y(), pose.Pos().Z()); // horizontal
                double vertical_angle   = std::acos((xz_vec.Z())/(xz_vec.Length()));
                double horizontal_angle = std::acos((yz_vec.Z())/(yz_vec.Length()));
                //ROS_INFO("Horizontal Angle/Opening Angle:    %f / %f ",horizontal_angle,horizontal_opening_angle_);

                if(lenght > this->clip_near_ && lenght < this->clip_far_ &&
                 vertical_angle <= (vertical_opening_angle_/2) &&
                 horizontal_angle <= (horizontal_opening_angle_/2))
                {
                    found_objects++;
                    // fill in infos for publisher
                    model_states.name.push_back(names_[i]);
                    geometry_msgs::Pose pose_g;
                    ignition::math::Pose3d  body_pose = pose; //
                    ignition::math::Vector3d pos = body_pose.Pos();
                    ignition::math::Quaterniond rot = body_pose.Rot();
                    pose_g.position.x = pos.X();
                    pose_g.position.y = pos.Y();
                    pose_g.position.z = pos.Z();
                    pose_g.orientation.w = rot.W();
                    pose_g.orientation.x = rot.X();
                    pose_g.orientation.y = rot.Y();
                    pose_g.orientation.z = rot.Z();
                    model_states.pose.push_back(pose_g);
                    // without twist for now
                    // may be easily possible to get the velocity in world frame
                    /*
                            gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(j));
                            ignition::math::Vector3d linear_vel  = body->GetWorldLinearVel().Ign();
                            ignition::math::Vector3d angular_vel = body->GetWorldAngularVel().Ign();
                    */
                    geometry_msgs::Twist twist;
                    model_states.twist.push_back(twist);
                }
            }

            if (found_objects > 0)
            {
                common::Time currentTime = world_->SimTime();
                ros::Time ros_time_;
                ros_time_.fromSec(currentTime.Double());
                model_states.header.stamp = ros_time_;
                camera_info_.header.stamp = ros_time_;
                this->pub_link_states_.publish(model_states);
            }
        }
    }

void MockVisionCameraPlugin::publishCameraInfo(){
      common::Time cur_time = this->world_->SimTime();
      if ((cur_time - this->last_info_update_time_).Double() >= 1.0)
      {
        this->last_info_update_time_ = cur_time;
        ros::Time ros_time_;
        ros_time_.fromSec(cur_time.Double());
        camera_info_.header.stamp = ros_time_;
        this->pub_camera_info_.publish(camera_info_);
      }
}
//header: 
//  seq: 7
//  stamp: 
//    secs: 68
//    nsecs: 587000000
//  frame_id: "camera_depth_optical_frame"
    
    void MockVisionCameraPlugin::fillCameraInfo(){
        float d[5];
        //std_msgs::Float64 k[9];
        
        camera_info_.header.frame_id = camera_parent_link_;
        camera_info_.height = height_;
        camera_info_.width = width_;
        camera_info_.distortion_model = distortion_model_;
        camera_info_.binning_x = 0;
        camera_info_.binning_y = 0;
        ROS_INFO("Writing Camera Info Message");
        /*for(std::vector<int>::size_type i = 0; i != distortion_params_.size(); i++) {
            d[i] = distortion_params_[i];
        }
        ROS_INFO("###   1   ###");
        for(std::vector<int>::size_type i = 0; i != camera_params_.size(); i++) {
            camera_info_.K[i] = camera_params_[i];
        }*/

        camera_info_.D = {0.1,0.1,0.1,0.1,0.1};
        camera_info_.K = {589.366, 0.0, 320.5, 0.0, 589.366, 240.5, 0.0, 0.0, 1.0};
        camera_info_.R = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0, 1.0};
        camera_info_.P = {589.366, 0.0, 320.5, -0.0, 0.0, 589.366, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0};

//roi: 
//  x_offset: 0
//  y_offset: 0
//  height: 0
//  width: 0
//  do_rectify: False
    }
    
    bool MockVisionCameraPlugin::serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res){
        this->enabled_ = true;
        return true;
    }

    bool MockVisionCameraPlugin::serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res){
        this->enabled_ = false;
        return true;
    }

    void MockVisionCameraPlugin::checkForNewObjects()
    {
          unsigned int count = 0;
          std::vector<std::string> names;
          for (unsigned int i = 0; i < world_->ModelCount(); i ++)
          {
            
            std::string temp_name = world_->Models().at(i)->GetName();
            //ROS_INFO("Found Model: %s",temp_name.c_str());
            if(nameMatch(temp_name))
            {
                count++;
                //ROS_INFO("Match");
                names.push_back(temp_name);
            }//else{ROS_INFO("Nope");}
          }
          this->names_=names;
          this->active_object_count_ = count;
    }
    
    //helper
    bool MockVisionCameraPlugin::nameMatch(std::string name)
    {
        bool match = false;
        std::string temp;
        //unsigned int num = sizeof(&object_names_) / sizeof(&object_names_[0]);
        //ROS_INFO("%i Object Names in our List",num);
        for(unsigned int a = 0; a < object_count_;a++)
        {
            match = false;
            temp = object_names_[a];
            int i = temp.compare(0,temp.size(),name,0,temp.size());
            //ROS_INFO("Comparing %s with %s to the Value of %i",name.c_str(),temp.c_str(),i);
            if(i == 0)
            {
                return true;
            }
            else
            {
                match = false;
            }
        }
        return match;
    }

    bool MockVisionCameraPlugin::timeToSensorUpdate()
    {
      common::Time cur_time = this->world_->SimTime();
      if ((cur_time - this->last_update_time_).Double() >= this->sensor_update_time_)
      {
        this->last_update_time_ = cur_time;
        return true;
      }
      else
      {
        return false;
      }
    }

    bool MockVisionCameraPlugin::timeToObjectUpdate()
    {
      common::Time cur_time = this->world_->SimTime();
      if ((cur_time - this->last_object_update_time_).Double() >= this->custom_object_update_time_)
      {
        this->last_object_update_time_ = cur_time;
        return true;
      }
      else
      {
        return false;
      }
    }     
}

