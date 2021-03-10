/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_GPU_LASER_COLLISION_PLUGIN_HH_
#define _GAZEBO_GPU_LASER_COLLISION_PLUGIN_HH_

#include <string>

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
// ros message stuff
#include <std_srvs/Empty.h>
//#include "simulation_msgs/LaserParticle.h"
#include "simulation_msgs/Material.h"
#include "simulation_msgs/DelMaterial.h"
#include "simulation_msgs/Scanner.h"
#include <sensor_msgs/LaserScan.h>


#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/PhysicsTypes.hh"
//#include <ignition/math/Frustum.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"

#include <boost/thread/mutex.hpp>

#include "simulation_msgs/SetDoubleVal.h"

//#include <ignition/math/Frustum.hh>

namespace gazebo
{
  class GpuRayCollisionPlugin : public SensorPlugin
  {

    public: struct Material{
        std::string name;
        double reflectivity;
        double transmission;
        double absorption;
        double angular_factor;
    };
    // reflectivity is the probability of reflecting the laser beam
    // transmission the probability that the laser beam will go thruw 
    // the object
    // discard absorption it could be either 1 - reflectivity; or 1 - transmission

    public: struct RaysInfo{
        std::vector<double> distances;
        std::vector<double> angle_offset;
        std::vector<std::string> names;
        std::vector<Material> material;
        int rayIndex;
    };

    public: RaysInfo cur_rayInfos;
    public: std::vector<RaysInfo> rayInfos;

    public: std::vector<Material> materials;

    public: GpuRayCollisionPlugin();

    public: ~GpuRayCollisionPlugin();
    
    public: void OnUpdate();

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private: msgs::LaserScanStamped laserMsg;

    private: transport::PublisherPtr scanPub;

    private: transport::NodePtr gznode_;

    private: physics::RayShapePtr testRay;


//    public: virtual void OnNewLaserFrame(const float *_image,
//                unsigned int _width, unsigned int _height,
//                unsigned int _depth, const std::string &_format);


    protected: unsigned int width, height/*, depth*/;

    protected: sensors::GpuRaySensorPtr parentSensor;

    private: std::string getParentLinkSafeName();
    private: std::string parent_save_name_;
    private: std::string topic_name_;

    private: event::ConnectionPtr newUpdateConnection;

    private: physics::WorldPtr world_;

    private: physics::LinkPtr parent_link_;


    private: void initLaserScanMsg();
    
    private: void CheckForModels(ignition::math::Pose3d &_curPose,
                                 const physics::Model_V &_models);

    private: double applyNoiseFunction(GpuRayCollisionPlugin::RaysInfo ray, int pre);

    private: double mapValues(double val, double input_start, double input_end, double output_start, double output_end);

    private: ignition::math::Angle angle_res;
    private: ignition::math::Angle angle_max;
    private: ignition::math::Angle angle_min;
    private: double max_range;
    private: double min_range;
    private: rendering::ScenePtr scene;
    private: physics::CollisionPtr coll;
    private: msgs::LaserScan *scan;
    private: msgs::LaserScanStamped scan_stamped;

    private: void loadMaterials(std::string path);
    private: void loadDefaultMaterial();
    private: Material defaultMaterial;
    private: bool getRayInfo(ignition::math::Vector3d start,
    ignition::math::Vector3d end, ignition::math::Vector3d dir, int i);
    private: void castRay(
                          ignition::math::Vector3d start,
                          ignition::math::Vector3d end,
                          double &dist,
                          std::string &name);
    private: std::string previous_name;
    private: std::string fixNoneHit(double dist, double prev, double next, int i);
    private: double calcRange(
        std::string name, double dist, double range, int i,
        ignition::math::Vector3d startPoint, ignition::math::Vector3d end,
        ignition::math::Vector3d dir);
    private: ignition::math::Rand ng;
    private: double getClosest(double dist, int old_idx, double offset);
    private: double computeAdjustedRange(RaysInfo ray);
    private: Material getMaterial(std::string name);
    private: Material getMaterialInfo(std::string name);
    private: double computeAngleOffset(double dist, std::string name, int index);
    private: double fixLastRange(int index);
    private: bool enabled;

    private: std::string frame;
    //ROS Stuff
    private: boost::shared_ptr<ros::NodeHandle> nh_;
    private: ros::ServiceServer enable_service_;
    private: ros::ServiceServer disable_service_;
    private: ros::ServiceServer change_material_service_;
    private: ros::ServiceServer change_scanner_service_;
    private: ros::ServiceServer add_material_service_;
    private: ros::ServiceServer delete_material_service_;
    private: ros::ServiceServer change_percent_orig_scan_;
    private: void startROSCommunication();
    public: bool serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
    public: bool serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
    public: bool serviceDelMat(simulation_msgs::DelMaterial::Request &req,
                               simulation_msgs::DelMaterial::Response &res);
    public: bool serviceAddMat(simulation_msgs::Material::Request &req,
                               simulation_msgs::Material::Response &res);
    public: bool serviceChangeMat(simulation_msgs::Material::Request &req,
                                  simulation_msgs::Material::Response &res);
    public: bool serviceChangePercent(simulation_msgs::SetDoubleVal::Request &req,
                                  simulation_msgs::SetDoubleVal::Response &res);
    public: bool serviceChangeScanner(simulation_msgs::Scanner::Request &req,
                                      simulation_msgs::Scanner::Response &res);
    private: ros::Publisher pub_;
    private: void SubConnect();
    private: void SubDisconnect();
    private: int connect_count_;
    private: bool subscriber_;
    private: void publish_scan();
    private: common::Time time_;

    private: double max_absorb_mean;
    private: double max_absorb_stddev;
    private: double range_scale_fac;
    private: double trans_scale_mean;
    private: double trans_scale_stddev;
    private: int mean_err_pre;
    private: double use_orig_scan_val_;
    private: double mean;
    private: double std_dev;
    private: double refl_scale;
    private: double angular_scale_std;
    private: double angular_scale_mean;
    private: bool always_hit;
    private: bool multi_echo;

    //bool changeParticlesService(simulation_msgs::LaserParticle::Request &req,
    //                            simulation_msgs::LaserParticle::Response &res);
/*
    private: event::ConnectionPtr newLaserFrameConnection;
    
    private: ignition::math::Frustum frustum;
    
    private: double aspect_ratio;
    
    private: physics::WorldPtr world_;
    
    private: physics::Link parent_link_;
    
    private: physics::Model_V models_;
    
    private: void CheckForModels(
        ignition::math::Pose3d &_curPose, const physics::Model_V &_models)
        
*/
  };
}
#endif
