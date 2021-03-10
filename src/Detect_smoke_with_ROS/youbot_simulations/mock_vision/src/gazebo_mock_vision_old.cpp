#include "ros/ros.h"
#include <algorithm>
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"
#include <std_srvs/Empty.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <imes_3d_vision/msg_Objects.h>
#include <imes_3d_vision/msg_Object.h>
#include <moving_object_tracker/MovingObject.h>
#include <moving_object_tracker/MovingObjectArray.h>
#include <imes_3d_vision/srv_Track.h>
#include <imes_3d_vision/srv_GetTrackedObjectPose.h>
#include <luh_youbot_gripper/GripCheck.h>
#include <std_msgs/Float32.h>
class GazeboMockVision{
     public:
      float clip_far;
      float clip_near;
      ros::NodeHandle *node_;
      ros::Subscriber sub_obj;
      ros::Subscriber sub_cam_range;
      ros::Subscriber conveyor_belt_vel;
      std::string ObjectNames[7] = {"M20","M20_100"}; //, "M30", "S40_40", "V20", "R20", "F20_20"};
      gazebo_msgs::ModelStates Objects_in_World;
      std::vector<std::string> names;
      ros::Publisher obj_pub;
      ros::Publisher imes_obj_pub;
      ros::Publisher obj_pub2;
      ros::Subscriber vel_sub;
      double old;
      double x,y,z;
      geometry_msgs::Twist temp_twist;
      int count;
      int seq;
      ros::Publisher cam_pub;
      bool enabled;
      ros::ServiceClient get_model_state_client;
      std::string camera_parent;
      float RotMat[3][3];
      float max_angle_hor;
      float max_angle_ver;
      float velocity;
      geometry_msgs::PoseStamped obj_pose;
      ros::ServiceServer mock_tracker_switch_service_1;
      ros::ServiceServer service1;
      ros::ServiceServer mock_tracker_switch_service_3;
      ros::ServiceServer service2;
      ros::ServiceServer mock_tracker_switch_service_2;
      ros::ServiceServer mock_tracker_switch_service_4;
      ros::ServiceServer mock_tracker_switch_service_5;
      ros::ServiceServer mock_tracker_switch_service_6;
      ros::ServiceServer mock_tracker_switch_service_7;
      ros::ServiceServer mock_tracker_switch_service_8;
      ros::ServiceServer mock_tracker_switch_service_9; 
      ros::ServiceServer mock_tracker_switch_service_10;
      ros::ServiceServer mock_tracker_switch_service_11;
      ros::ServiceServer mock_tracker_switch_service_12;
      GazeboMockVision(ros::NodeHandle &nh):node_(&nh) {
          seq = 0;
          max_angle_hor = 3.14;
          max_angle_ver = 3.14;
          clip_near = 0.05;
          clip_far = 5.0;
          count = 0;
          camera_parent = "youbot::arm_link_0";
          enabled = false;
          get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
          // ObjectNames = {"M20","M20_100","M30","S40_40","V20","R20","F20_20"};
          ROS_INFO("Starting GazeboMockVision");
          sub_obj = node_->subscribe("/gazebo/model_states", 10, &GazeboMockVision::CallbackOnStates, this);
          ROS_INFO("Subscribed to ModelStates");
          vel_sub = node_->subscribe("/conveyor_belt/vel",10,&GazeboMockVision::velCallback, this);
          obj_pub = nh.advertise<geometry_msgs::PoseStamped>("/point_detection/pose", 10);
          imes_obj_pub = nh.advertise<imes_3d_vision::msg_Objects>("/tracker/objects_transformed", 10);
          //posePublisher = nh.advertise<geometry_msgs::PoseStamped>("tracker/object", 1);
          cam_pub = nh.advertise<gazebo_msgs::ModelStates>("/gazebo/ObjectInCameraRange/Objects", 10);
          obj_pub2 = nh.advertise<moving_object_tracker::MovingObjectArray>("/moving_object_tracker/objects", 10);

          service1 =nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("camera/enable", boost::bind(&GazeboMockVision::enable,this,_1,_2));
          ROS_INFO("ServiceServer initialised");
          mock_tracker_switch_service_1 =nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("tracker/switch_to_object_detection", boost::bind(&GazeboMockVision::mock1,this,_1,_2));
          service2 =nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("/camera/disable", boost::bind(&GazeboMockVision::disable,this,_1,_2));
          
          mock_tracker_switch_service_2 =nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("tracker/switch_to_hole_detection", boost::bind(&GazeboMockVision::mock1,this,_1,_2));
          
          
          mock_tracker_switch_service_3 =nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("tracker/start", boost::bind(&GazeboMockVision::mock1,this,_1,_2));
          mock_tracker_switch_service_4 =nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("tracker/stop", boost::bind(&GazeboMockVision::mock1,this,_1,_2));
          mock_tracker_switch_service_5 =nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("luh_atwork_vision/enable", boost::bind(&GazeboMockVision::mock1,this,_1,_2));
          mock_tracker_switch_service_6 =nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("luh_atwork_vision/disable", boost::bind(&GazeboMockVision::mock1,this,_1,_2));
          mock_tracker_switch_service_7 =nh.advertiseService<imes_3d_vision::srv_Track::Request, imes_3d_vision::srv_Track::Response>("tracker/track", boost::bind(&GazeboMockVision::mock2,this,_1,_2)); //imes_3d_vision::srv_Track
          mock_tracker_switch_service_8 =nh.advertiseService<imes_3d_vision::srv_GetTrackedObjectPose::Request, imes_3d_vision::srv_GetTrackedObjectPose::Response>("tracker/get_tracked_object_pose", boost::bind(&GazeboMockVision::mock3,this,_1,_2));//imes_3d_vision::srv_GetTrackedObjectPose
          //mock_tracker_switch_service_9 =nh.advertiseService("tracker/get_tracked_object_pose", &GazeboMockVision::mock3,this);
          //mock_tracker_switch_service_10 =nh.advertiseService("gripper/check", &GazeboMockVision::mock4,this);//luh_youbot_gripper::GripCheck//????????
          mock_tracker_switch_service_11 =nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("moving_object_tracker/enable",  boost::bind(&GazeboMockVision::mock1,this,_1,_2));
          ros::ServiceServer mock_tracker_switch_service_12 =nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("moving_object_tracker/disable", boost::bind(&GazeboMockVision::mock1,this,_1,_2));
          velocity = 0.1;
      }

      ~GazeboMockVision() {
      }
      
      void CallbackOnStates(const gazebo_msgs::ModelStatesConstPtr &msg) {
       size_t modelCount = msg->name.size();
       // Filter all Models in the World to get the ManipulationObjects
       // TODO: need to be more simply (REGEX)
          for (size_t modelInd = 0; modelInd < modelCount; modelInd++) {
              std::string name = msg->name[modelInd];
              bool Match = false;
              for (unsigned int a = 0; a < sizeof(ObjectNames) / sizeof(ObjectNames[0]);a++){
                  std::string temp = ObjectNames[a];
                  for (unsigned int b = 0; b < temp.length(); b++) {
                      if (temp[b] != name[b]) {
                          Match = false;
                          break;
                      } else {
                          Match = true;
                      }
                  }
                  if (Match) 
                  {
                  
                      //if(names.count(name) == 0)
                      if (std::find(names.begin(), names.end(), name) == names.end())
                      {
                        names.push_back(name);
                        //Objects_in_World.name.push_back(name);
                        //Objects_in_World.pose.push_back(msg->pose[modelInd]);
                        //Objects_in_World.twist.push_back(msg->twist[modelInd]);
                        break;
                      }
                      else
                      {
                        break; 
                      }
                        
                  }
                }
            
            }
        if(enabled == false){// if not enabled simply publish the models we are interested in
            //cam_pub.publish(Objects_in_World);
            ros::Duration(0.1).sleep();
        }
        if (enabled == true)
        {
          gazebo_msgs::GetModelState srv;
          //ROS_INFO("Waiting for get_model_state service");
          if(get_model_state_client.exists())
          {
            //ROS_INFO("Camera Parent: %s",camera_parent.c_str());
            srv.request.relative_entity_name = camera_parent;
            //ROS_INFO("Iterating over: %i Models",names.size());
            for(size_t ind = 0; ind < names.size(); ind++)
            {
                ROS_INFO("Name: %s",names[ind].c_str());
                srv.request.model_name = names[ind];
                //ROS_INFO("Called get_model_state service for %s",names[ind].c_str());
                get_model_state_client.call(srv);
                
                geometry_msgs::Pose pose = srv.response.pose;
                //ROS_INFO("Got Pose x: %f y: %f z: %f",pose.position.x,pose.position.y,pose.position.z);
                //float x = pose.position.x;
                //float y = pose.position.y;
                //float z = pose.position.z;
                //float dist = sqrt(x*x +y*y + z*z);
                //ROS_INFO("%s Is in dist %f",names[ind].c_str(),dist);
                        moving_object_tracker::MovingObject mov_obj;
                        moving_object_tracker::MovingObjectArray mov_obj_arr;
                        imes_3d_vision::msg_Objects objects;
                        imes_3d_vision::msg_Object object;
                        geometry_msgs::PoseStamped temp_2;
                        
                        temp_2.header.stamp = ros::Time::now();
                        temp_2.header.seq = seq;
                        seq++;
                        temp_2.header.frame_id = "arm_link_0";
                        temp_2.pose = srv.response.pose;
                        object.name = "M20_100";
                        object.pose = temp_2;
                        object.is_hole = false;
                        object.rating = 1.0;
                        objects.objects.push_back(object);
                        obj_pose = temp_2;
                        
                        imes_obj_pub.publish(objects);
                        obj_pub.publish(temp_2);
                        
                        mov_obj.id = seq;
                        mov_obj.pose = srv.response.pose;
                        mov_obj.movementType = 1;
                        mov_obj_arr.header = temp_2.header;
                        //========================compute vel======================
                        /*if (count == 0){
                            count = 1;
                            temp_twist = srv.response.twist;
                            old = ros::Time::now().toSec();
                            x = srv.response.pose.position.x;
                            y = srv.response.pose.position.y;
                            z = 0.0;
                        }
                        else{
                            double dt = ros::Time::now().toSec() - old;
                            old = ros::Time::now().toSec();
                            double dx = srv.response.pose.position.x - x;
                            double dy = srv.response.pose.position.y - y;
                            double dz = 0.0;
                            ROS_WARN("DX:   %f  DY: %f  DT: %f ",dx,dy,dt);
                            temp_twist.linear.x = dx/dt;
                            temp_twist.linear.y = dy/dt;
                            temp_twist.linear.z = 0.0;
                            temp_twist.angular.x = 0.0;
                            temp_twist.angular.y = 0.0;
                            temp_twist.angular.z = 0.0;
                            x = srv.response.pose.position.x;
                            y = srv.response.pose.position.y;
                            z = srv.response.pose.position.z;
                        }*/
                        float xn = 0.544639035*velocity;
                        float yn = 0.838670568*velocity;
                        mov_obj.twist.linear.x = xn;
                        mov_obj.twist.linear.y = yn;
                        mov_obj.twist.linear.z = 0.0;
                        mov_obj.twist.angular.x = 0.0;
                        mov_obj.twist.angular.y = 0.0;
                        mov_obj.twist.angular.z = 0.0;
                        //mov_obj.twist = temp_twist;
                        //mov_obj.twist = srv.response.twist;
                        mov_obj_arr.objects.push_back(mov_obj);
                        obj_pub2.publish(mov_obj_arr);

                        count++;
               /* if(dist <= clip_far && dist>=clip_near)
                {
                    
                    float alpha = atan2(z,y);
                    float beta  = atan2(z,x);
                    if(abs(alpha) <= max_angle_hor && abs(beta) <= max_angle_ver)
                    {
                        imes_3d_vision::msg_Objects objects;
                        imes_3d_vision::msg_Object object;
                        geometry_msgs::PoseStamped temp_2;
                        temp_2.header.stamp = ros::Time::now();
                        temp_2.header.seq = seq;
                        seq++;
                        temp_2.header.frame_id = "arm_link_0";
                        temp_2.pose = srv.response.pose;
                        object.name = "M20_100";
                        object.pose = temp_2;
                        object.is_hole = false;
                        object.rating = 1.0;
                        objects.objects.push_back(object);
                        obj_pose = temp_2;
                        imes_obj_pub.publish(objects);
                        obj_pub.publish(temp_2);
                    } 
                }
              }*/
            }
          }
          else
          {
            ROS_WARN("Client to get ModelStates dont exists");
          }
        }
      }
      
      /*void transformationMatrix(geometry_msgs::Pose cam_pos)
      {
        geometry_msgs::Point point = cam_pos.position;
        geometry_msgs::Quaternion quat = cam_pos.orientation;
        float x = quat.x;float y = quat.y;float z = quat.z;float w = quat.w;
        // not sure if i have to use q or q^-1
        //float abs = x*x+y*y+z*z+w*w
        //x= -1*x/abs; y= -1*y/abs; z= -1*z/abs; w = w/abs
        RotMat[0][0] = w*w + x*x - y*y - z*z;
        RotMat[0][1] = 2*x*y + 2*w*z;
        RotMat[0][2] = 2*x*z - 2*w*y;
        RotMat[1][0] = 2*x*y - 2*w*z;
        RotMat[1][1] = w*w - x*x + y*y - z*z;
        RotMat[1][2] = 2*y*z + 2*w*x;
        RotMat[2][0] = 2*x*z + 2*w*y;
        RotMat[2][1] = 2*y*z - 2*w*x;
        RotMat[2][2] = w*w - x*x - y*y + z*z;
      }*/
     void velCallback(const std_msgs::Float32 data)
     {
         ROS_INFO("Set Vel in Mock_Vision to: %f",data.data );
         velocity = data.data;
     }
     bool mock1(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        //enabled = true;
        return true;
      }       
      bool enable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        enabled = true;
        return true;
      }
      bool disable(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        enabled = false;
        return true;
      }
      /*
      bool getTrackedObjectPoseService(imes_3d_vision::srv_GetTrackedObjectPose::Request &req,
                                 imes_3d_vision::srv_GetTrackedObjectPose::Response &res)
        {
            res.object_pose = tracker_ptr->getTrackedObjectPose();

            return true;
        }*/
      bool mock3(imes_3d_vision::srv_GetTrackedObjectPose::Request &req,
                                 imes_3d_vision::srv_GetTrackedObjectPose::Response &res){
        res.object_pose = obj_pose;
        return true;
      }
      
      
//############## TRACK SERVICE CALLBACK ################################################################################
/*bool Tracker::trackService(imes_3d_vision::srv_Track::Request &req, imes_3d_vision::srv_Track::Response &res)
{
//    service_mutex.lock();
    boost::mutex::scoped_lock scopedLock(service_mutex);

    if(req.name.empty())
    {
        res.topic = "";
        if(training)
        {
            tracking_mode = TRAINING;
            ROS_INFO("Returned to training mode.");
        }
        else
        {
            tracking_mode = IDLE;
            ROS_INFO("Returned to idle mode.");
        }
//        service_mutex.unlock();
        return true;
    }

    if(!classifier_loaded)
    {
//        service_mutex.unlock();
        return false;
    }
    object_name = req.name;

    int index;
    if(find_holes)
        index = hole_classifier.getIndex(req.name);
    else
        index = object_classifier.getIndex(req.name);

    if(index < 0)
    {
        ROS_WARN("Requested object '%s' does not exist.", req.name.c_str());
        res.topic = "";
//        service_mutex.unlock();
        return false;
    }

    ROS_INFO("Now tracking '%s'", req.name.c_str());
    res.topic = posePublisher.getTopic();
    tracking_mode = TRACK;

//    service_mutex.unlock();
    return true;
}*/
      bool mock2(imes_3d_vision::srv_Track::Request &req, imes_3d_vision::srv_Track::Response &res){
        res.topic = "point_detection/pose";
        return true;
      }
      
/*bool YoubotGripperController::gripCheckCallback(luh_youbot_gripper::GripCheck::Request &req,
                                                luh_youbot_gripper::GripCheck::Response &res)
{
    boost::mutex::scoped_lock scoped_lock(lock);

    res.has_object = !(left_pos_reached_ && right_pos_reached_);
    return true;
}*/
      bool mock4(luh_youbot_gripper::GripCheck::Request &req, luh_youbot_gripper::GripCheck::Response &res){
        res.has_object = true;
        return true;
      }
};  // Class Ende

int main(int argc, char **argv) {
  ros::init(argc, argv, "GazeboMockVision");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  while(ros::Time::isValid()==0)
  {
    ros::Duration(0.5).sleep();
    ROS_WARN("WAITING FOR TIME TO BECOME VALID");
  }
  GazeboMockVision m(nh);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  //ros::spin();

  return 0;
}
