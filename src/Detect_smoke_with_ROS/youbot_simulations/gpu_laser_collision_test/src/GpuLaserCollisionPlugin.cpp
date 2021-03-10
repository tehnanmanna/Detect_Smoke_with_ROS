/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <yaml-cpp/yaml.h>
#include <functional>
#include "GpuLaserCollisionPlugin.h"
#include "gazebo/sensors/GpuRaySensor.hh"
#include <ignition/math/Helpers.hh>
#include <map>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>


#define PI_2 1.57079632679489661923


//#include "GpuRayCollisionPlugin.hh"
#include "gazebo/sensors/GpuRaySensor.hh"


using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(GpuRayCollisionPlugin)

/////////////////////////////////////////////////
GpuRayCollisionPlugin::GpuRayCollisionPlugin()
: SensorPlugin()
{
  gzwarn << "Start of GpuRayCollisionPlugin "<< std::endl;
}

GpuRayCollisionPlugin::~GpuRayCollisionPlugin()
{
  //this->gznode_.Shutdown();
  this->testRay.reset();
  this->nh_->shutdown();

}

/////////////////////////////////////////////////
void GpuRayCollisionPlugin::Load(sensors::SensorPtr _sensor,
        sdf::ElementPtr _sdf)
{
  this->enabled = false;
  this->connect_count_ = 0;
  this->subscriber_ = false;
  gzwarn << "Start loading GpuRayCollisionPlugin "<< std::endl;
  this->parentSensor =
  std::dynamic_pointer_cast<sensors::GpuRaySensor>(_sensor);

# if GAZEBO_MAJOR_VERSION >= 7
  std::string worldName = parentSensor->WorldName();
# else
  std::string worldName = parentSensor->GetWorldName();
# endif
  this->world_ = physics::get_world(worldName);

  this->parent_link_ = boost::dynamic_pointer_cast<physics::Link>(
    this->world_->EntityByName(this->parentSensor->ParentName()));

  unsigned int seed;
  common::Time cur = this->world_->SimTime();
  seed = (unsigned int)(cur.nsec + cur.sec);
  this->ng.Seed(seed);

  if(!this->parentSensor)
    this->parentSensor =
        std::dynamic_pointer_cast<sensors::GpuRaySensor>(_sensor);

  if(!_sdf->HasElement("path"))
  {
    gzwarn << "No path given, load default config"<<std::endl;
    this->loadDefaultMaterial();
  }else{
    std::string path;
    path = _sdf->Get<std::string>("path");
    this->loadMaterials(path);
  }
  if(!_sdf->HasElement("MeanErr")){
    this->mean = 0.0;
  }else{
    this->mean = _sdf->Get<double>("MeanErr");
  }
  if(!_sdf->HasElement("Stddev")){
    this->std_dev = 0.0;
  }else{
    this->std_dev = _sdf->Get<double>("Stddev");
  }
  if(!_sdf->HasElement("AlwaysGetHit")){
    this->always_hit = false;
  }else{
    this->always_hit = _sdf->Get<bool>("AlwaysGetHit");
  }
  if(!_sdf->HasElement("UseOriginalScanPercentage")){
    this->use_orig_scan_val_ = 0.0;
  }else{
    this->use_orig_scan_val_ = _sdf->Get<double>("UseOriginalScanPercentage");
  }
  //TODO MultiEcho - not implemented
  if(!_sdf->HasElement("MultiEcho")){
    this->multi_echo = true;
  }else{
    this->multi_echo = _sdf->Get<bool>("MultiEcho");
  }

  if(!_sdf->HasElement("MeanErrPrefix"))
    this->mean_err_pre = 0;
  else{
    this->mean_err_pre = _sdf->Get<int>("MeanErrPrefix");
  }

  if(!_sdf->HasElement("AngularScaleStdDev")){
    this->angular_scale_std = 0.1;
  }else{
    this->angular_scale_std = _sdf->Get<double>("AngularScaleStdDev");
  }

  if(!_sdf->HasElement("AngularScaleMean")){
    this->angular_scale_mean = -0.02;
  }else{
    this->angular_scale_mean = _sdf->Get<double>("AngularScaleMean");
  }

  if(!_sdf->HasElement("ReflectionScale")){
    this->refl_scale = 0.1;
  }else{
    this->refl_scale = _sdf->Get<double>("ReflectionScale");
  }

  if(!_sdf->HasElement("MaxAbsorptionStdDev")){
    this->max_absorb_stddev = 0.05;
  }else{
    this->max_absorb_stddev = _sdf->Get<double>("MaxAbsorptionStdDev");
  }

  if(!_sdf->HasElement("MaxAbsorptionMeanErr")){
    this->max_absorb_mean = -0.05;
  }else{
    this->max_absorb_mean = _sdf->Get<double>("MaxAbsorptionMeanErr");
  }

  if(!_sdf->HasElement("ScaleRangeErr")){
    this->range_scale_fac = -0.05;
  }else{
    this->range_scale_fac = _sdf->Get<double>("ScaleRangeErr");
  }

  if(!_sdf->HasElement("TransmissionScaleStdDev")){
    this->trans_scale_stddev = 0.02;
  }else{
    this->trans_scale_stddev = _sdf->Get<double>("TransmissionScaleStdDev");
  }

  if(!_sdf->HasElement("TransmissionScaleMeanErr")){
    this->trans_scale_mean = -0.02;
  }else{
    this->trans_scale_mean = _sdf->Get<double>("TransmissionScaleMeanErr");
  }

  if(!_sdf->HasElement("Frame")){
    this->frame = "map";
  }else{
    this->frame = _sdf->Get<std::string>("Frame");
  }

  std::string topic_name;
  topic_name = "";
  if(!_sdf->HasElement("TopicName")){
    topic_name = this->getParentLinkSafeName()+"/scan";
  }else{
    topic_name = _sdf->Get<std::string>("TopicName");
    if(topic_name == "")
        topic_name = this->getParentLinkSafeName()+"/scan";
  }
  gzwarn<<"Topic: "<<topic_name<<std::endl;
  this->topic_name_ = topic_name;

  // init ros
  gzwarn<< "Starting ROS"<<std::endl;
  std::string namespace_ = "";
  if (_sdf->HasElement("Namespace"))
    namespace_ = _sdf->GetElement("Namespace")->Get<std::string>() + "/";
  //TODO unique Name to prevent namespace conflicts
  this->nh_.reset(new ros::NodeHandle(namespace_));
  this->startROSCommunication();


  // maybe use a collision pointer class var to get infos of rays collision
  this->coll = physics::CollisionPtr();
  this->testRay = boost::dynamic_pointer_cast<physics::RayShape>(
    this->world_->Physics()->CreateShape(
    "ray", this->coll));


  this->width = this->parentSensor->RangeCount();
  this->height = this->parentSensor->VerticalRangeCount();


  std::string topic;
  topic = "~/"+this->parentSensor->ParentName()+"/scan/modified";
  boost::replace_all(topic, "::", "/");
  this->gznode_ = transport::NodePtr(new transport::Node());
  this->gznode_->Init(worldName);
  this->scanPub =
    this->gznode_->Advertise<msgs::LaserScanStamped>(topic, 50);


  this->newUpdateConnection = this->parentSensor->ConnectUpdated(
    std::bind(&GpuRayCollisionPlugin::OnUpdate, this));

  double angle_temp = this->parentSensor->AngleResolution();
  this->angle_res = ignition::math::Angle(angle_temp);
  this->angle_max = this->parentSensor->AngleMax();
  this->angle_min = this->parentSensor->AngleMin();
  this->max_range = this->parentSensor->RangeMax();
  this->min_range = this->parentSensor->RangeMin();
  
  this->scene = rendering::get_scene();
  this->parentSensor->SetActive(true);
  this->scan = this->scan_stamped.mutable_scan();
  this->initLaserScanMsg();
  gzwarn << "Finished loading GpuRayCollisionPlugin "<< std::endl;
  this->enabled = true;
/* OLD CODE NOT IN USE
  // all double values
  this->frustum.SetNear(this->parentSensor->RangeMin());
  this->frustum.SetFar(this->parentSensor->RangeMax());
  this->frustum.SetFOV(this->parentSensor->HorzFOV());
  // Set the aspect ratio, which is the width divided by height
  // of the near or far planes.
  
  this->width = this->parentSensor->RangeCount();
  this->height = this->parentSensor->VerticalRangeCount();
  
  this->aspect_ratio = width / height;
  this->frustum.SetAspectRatio(aspect_ratio);

  this->newLaserFrameConnection = this->parentSensor->ConnectNewLaserFrame(
      std::bind(&GpuRayCollisionPlugin::OnNewLaserFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  this->parentSensor->SetActive(true);
*/
}

//////////////////////////////////////////////////
void GpuRayCollisionPlugin::initLaserScanMsg()
{
  this->scan->set_angle_min(this->parentSensor->AngleMin().Radian());
  this->scan->set_angle_max(this->parentSensor->AngleMax().Radian());
  this->scan->set_angle_step(this->angle_res.Radian());
  this->scan->set_count(this->parentSensor->RayCount());
  this->scan->set_vertical_angle_min(
    this->parentSensor->VerticalAngleMin().Radian());
  this->scan->set_vertical_angle_max(
    this->parentSensor->VerticalAngleMax().Radian());
  this->scan->set_vertical_angle_step(
    this->parentSensor->VerticalAngleResolution());
  this->scan->set_vertical_count(this->parentSensor->VerticalRayCount());
  this->scan->set_range_min(this->min_range);
  this->scan->set_range_max(this->max_range);

  const int numRays = this->parentSensor->RayCount() 
    * this->parentSensor->VerticalRayCount();
  if (scan->ranges_size() != numRays)
  {
    // gzdbg << "Size mismatch; allocating memory\n";
    scan->clear_ranges();
    scan->clear_intensities();
    for (int i = 0; i < numRays; ++i)
    {
      scan->add_ranges(ignition::math::NAN_F);
      scan->add_intensities(ignition::math::NAN_F);
    }
  }
  this->scan_stamped.mutable_scan()->set_frame(
    this->parentSensor->ParentName());
}

/////////////////////////////////////////////////
void GpuRayCollisionPlugin::loadDefaultMaterial()
{
    GpuRayCollisionPlugin::Material mat;
    mat.name = "Default";
    mat.reflectivity = 0.1;
    mat.transmission = 0.99;
    mat.absorption = 0.1;
    this->materials.push_back(mat);
    this->defaultMaterial = mat;
}

/////////////////////////////////////////////////
void GpuRayCollisionPlugin::OnUpdate()
{
if(this->subscriber_)
{
  this->time_ = this->world_->SimTime();
  this->world_->SetPaused(true);
  this->rayInfos.clear();
  if(this->enabled){
  this->parentSensor->SetActive(false);
  ignition::math::Pose3d curPose = this->parentSensor->Pose() +
    this->parent_link_->WorldPose();

  msgs::Set(this->scan->mutable_world_pose(), curPose);
  msgs::Set(this->scan_stamped.mutable_time(), this->world_->SimTime());

  ignition::math::Vector3d start = curPose.Pos();
  ignition::math::Angle cur_angl;
  cur_angl = ignition::math::Angle(curPose.Rot().Yaw()) + this->angle_min
                                   - this->angle_res;

  ignition::math::Angle prev_angl, next_angle;
  ignition::math::Vector3d dir;
  
  bool fix_last = false;
  double cur_range = 0.0;
  bool infos_recieved;
  int pre;
  if(this->mean_err_pre != 0){
    pre = this->mean_err_pre;
  }else{
    pre = ng.DblUniform(0.0, 1.0) < 0.5 ? -1 : 1;
  }

  // TODO Problem is now doing all in one for loop
  // if split the applyNoiseFunction apart in other
  // for loop we could compute better values :–?
  for(unsigned int i = 0; i < this->width; ++i)
  {
    //setup for actual ray
    this->cur_rayInfos.distances.clear();
    this->cur_rayInfos.angle_offset.clear();
    this->cur_rayInfos.material.clear();
    this->cur_rayInfos.names.clear();
    prev_angl = cur_angl;
    cur_angl += this->angle_res;
    next_angle = cur_angl + this->angle_res;
    cur_range = this->parentSensor->Range(i);

    // fix_last flag if previous ray didn't hit
    if(fix_last)
      {
        // TODO better way to correct the value??
        this->scan->set_ranges(i-1, this->parentSensor->Range(i-1));
        fix_last = false;
      }

    //gzwarn << "Cur Range: " << cur_range << std::endl;
    // maybe > this->max_range && < this->min_range???
    if (cur_range != ignition::math::INF_D &&
        cur_range != -ignition::math::INF_D &&
        this->use_orig_scan_val_ != 1.0) // if val is 1 then always use orig value
    {
      dir.Set(std::cos(cur_angl.Radian()),
              std::sin(cur_angl.Radian()), 0.0);
      ignition::math::Vector3d end = start + dir * this->max_range;

      infos_recieved = this->getRayInfo(start, end, dir, i);

      if(this->ng.DblUniform(0.0, 1.0) < this->use_orig_scan_val_)
      {
        this->scan->set_ranges(i, cur_range);
        continue;
      }

      if(!this->cur_rayInfos.distances.empty() && infos_recieved)
      {

        this->scan->set_ranges(
                               i,
                               this->applyNoiseFunction(this->cur_rayInfos, pre)
                               );
      }
      else if(!infos_recieved)
      {
        fix_last = true;
      }
    }
    else // the value is INF
    {
      this->scan->set_ranges(i, cur_range);
    }

  this->rayInfos.push_back(this->cur_rayInfos);
  } // end for

  //gzwarn << "Here1" << std::endl;
  //this->scanPub->Publish(this->scan_stamped);
  this->publish_scan();
  //gzwarn << "Here2" << std::endl;
  this->rayInfos.clear();
  //gzwarn << "Here3" << std::endl;
  this->parentSensor->SetActive(true);
  //gzwarn << "Here4" << std::endl;
  }
  else
  {
    //not enabled
    this->parentSensor->SetActive(false);
    ignition::math::Pose3d curPose = this->parentSensor->Pose() +
                                     this->parent_link_->WorldPose();
    msgs::Set(this->scan->mutable_world_pose(), curPose);
    msgs::Set(this->scan_stamped.mutable_time(), this->world_->SimTime());
    for(unsigned int i = 0; i < this->width; ++i)
    {
        this->scan->set_ranges(i, this->parentSensor->Range(i));
    }
    this->scanPub->Publish(this->scan_stamped);
    this->parentSensor->SetActive(true);
  }
  this->world_->SetPaused(false);
}else{
    //no subscriber
}

}
/////////////////////////////////////////////////////////////////
void GpuRayCollisionPlugin::publish_scan()
{
  sensor_msgs::LaserScan msg;

  msg.header.stamp = ros::Time(this->time_.sec,
                               this->time_.nsec);
  //msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());//
  msg.header.frame_id = this->frame;// this->parentSensor->ParentName();
  msg.angle_min = this->parentSensor->AngleMin().Radian();
  msg.angle_max = this->parentSensor->AngleMax().Radian();
  msg.angle_increment = this->angle_res.Radian();
  msg.time_increment = 0;
  msg.scan_time = 0;  //not sure whether this is correct
  msg.range_min = this->min_range;
  msg.range_max = this->max_range;
  msg.ranges.resize(this->parentSensor->RayCount() 
                    * this->parentSensor->VerticalRayCount());
  std::copy(this->scan->ranges().begin(),
            this->scan->ranges().end(),
            msg.ranges.begin());

  msg.intensities.resize(this->scan->intensities_size());
  std::copy(this->scan->intensities().begin(),
            this->scan->intensities().end(),
            msg.intensities.begin());
  this->pub_.publish(msg);
}
/////////////////////////////////////////////////////////////////
double GpuRayCollisionPlugin::applyNoiseFunction(
    GpuRayCollisionPlugin::RaysInfo ray, int pre)
{
  // errors come with
  //  angle:    middle noise increasing with angle
  // range:     increasing std_dev
  // angle:     increasing std_dev
  // refl:      increasing std_dev + mean
  // absorp:    decrease std_dev, mean
  // trans:     
  // material harsh vs. even
  // harsh:     decrease std_dev, increasing mean
  // even:      crazy effects, increasing std_dev, strange things

/*
    easy model for now: only account reflection, transmission and angle
    mat.name = "Default";
    mat.reflectivity = 0.8;
    mat.transmission = 0.1;
    mat.absorption = 0.1;
*/
  double trans, refl, angl, angl_fac, absorp;
  double range = 0.0;
  double std_dev = this->std_dev;
  double mean = this->mean;
  //gzwarn<<"start mean: "<<mean<<std::endl;
  for(int i = 0; i < ray.distances.size(); i++)
  {
    range = ray.distances[i];
    angl = ray.angle_offset[i]; // 0 good; +-PI/2 bad
    trans = ray.material[i].transmission;
    refl = ray.material[i].reflectivity;
    angl_fac = ray.material[i].angular_factor;
    absorp = ray.material[i].absorption;


    //TODO compute and set scan intensities

    if(this->ng.DblNormal(0.5, 0.15) <= refl)
    {
        //  0 <= angl < 1.57
        //  0.0 <= angl_fac < 3.14
        if (angl_fac == 0.0)
            angl_fac = 0.001;
        
        //angular values
        std_dev += (angl / angl_fac) * this->angular_scale_std;
        mean += (angl / angl_fac) * this->angular_scale_mean;
        // reflection values
        std_dev *= (this->refl_scale*(1/refl - 1));
        mean += this->refl_scale*refl;
        //absorption values
        if(absorp <= 0.5){
          double val = this->mapValues(absorp, 0.0, 0.5, 0.0, 1.0);
          std_dev += val*this->max_absorb_stddev;
          mean += val*this->max_absorb_mean;
        }else{
          double val = this->mapValues(absorp, 0.5, 1.0, 1.0, 0.0);
          std_dev += val*this->max_absorb_stddev;
          mean += val*this->max_absorb_mean;
        }
        //range
        mean += range * this->range_scale_fac;
        //transmission
        std_dev += trans*this->trans_scale_stddev;
        mean += trans*this->trans_scale_mean;
        //mean += pre * this->ng.DblNormal(trans/2, range/50) / 10;
        range += this->ng.DblNormal(mean, std_dev);
        break;
    }
    else if(this->always_hit)
    {
        if(i == ray.distances.size() - 1)
        {
            //gzwarn<<"Last Range on: "<<i<<" Range:"<< range << std::endl;
            // last range, need to pick one
            if (angl_fac == 0.0)
                angl_fac = 0.001;
            //angular values
            std_dev += (angl / angl_fac) * this->angular_scale_std;
            mean += (angl / angl_fac) * this->angular_scale_mean;
            // reflection values
            std_dev *= (this->refl_scale*(1/refl - 1));
            mean += this->refl_scale*refl;
            //absorption values
            if(absorp <= 0.5){
              double val = this->mapValues(absorp, 0.0, 0.5, 0.0, 1.0);
              std_dev += val*this->max_absorb_stddev;
              mean += val*this->max_absorb_mean;
            }else{
              double val = this->mapValues(absorp, 0.5, 1.0, 1.0, 0.0);
              std_dev += val*this->max_absorb_stddev;
              mean += val*this->max_absorb_mean;
            }
            //range
            mean += range * this->range_scale_fac;
            //transmission
            std_dev += trans*this->trans_scale_stddev;
            mean += trans*this->trans_scale_mean;
            //mean += pre * this->ng.DblNormal(trans/2, range/50) / 10;
            range += this->ng.DblNormal(mean, std_dev * std_dev);
            break;
        }
        else if(this->ng.DblUniform(0.0, 1.0) > trans)
        {
            // no transmission, we have to take the actual range
            //gzwarn<<"Last Range on: "<<i<<" Range:"<< range << std::endl;
            // last range, need to pick one
            if (angl_fac == 0.0)
                angl_fac = 0.001;
            //angular values
            std_dev += (angl / angl_fac) * this->angular_scale_std;
            mean += (angl / angl_fac) * this->angular_scale_mean;
            // reflection values
            std_dev *= (this->refl_scale*(1/refl - 1));
            mean += this->refl_scale*refl;
            //absorption values
            if(absorp <= 0.5){
              double val = this->mapValues(absorp, 0.0, 0.5, 0.0, 1.0);
              std_dev += val*this->max_absorb_stddev;
              mean += val*this->max_absorb_mean;
            }else{
              double val = this->mapValues(absorp, 0.5, 1.0, 1.0, 0.0);
              std_dev += val*this->max_absorb_stddev;
              mean += val*this->max_absorb_mean;
            }
            //range
            mean += range * this->range_scale_fac;
            //transmission
            std_dev += trans*this->trans_scale_stddev;
            mean += trans*this->trans_scale_mean;
            //mean += pre * this->ng.DblNormal(trans/2, range/50) / 10;
            range += this->ng.DblNormal(mean, std_dev * std_dev);
            break;
        }
    }
    else{
        range = ignition::math::INF_D;
    }
  }

  return range;
}

double GpuRayCollisionPlugin::mapValues(double val, double input_start, double input_end, double output_start, double output_end)
{
  double slope = 1.0 * (output_end - output_start) / (input_end - input_start);
  return (output_start + slope * (val - input_start));
}

//////////////////////////////////////////////////////////////
bool GpuRayCollisionPlugin::getRayInfo(ignition::math::Vector3d start,
    ignition::math::Vector3d end, ignition::math::Vector3d dir, int index)
{
    //GpuRayCollisionPlugin::RaysInfo ray;
    double dist;
    double angleOff = 0.0;
    double totalDist = 0.0;
    double totalAbsorption = 0.0;
    //double reflectivity = 0.0;
    GpuRayCollisionPlugin::Material material;
    std::string name;//, link_name;
    this->cur_rayInfos.rayIndex = index;
    while(totalDist < this->max_range && totalAbsorption < 1.0)
    {
        name = "";

        // need adjust start_point or the ray will hit the sensor
        this->castRay(start + this->min_range * dir, end, dist, name);

        //starting the ray a bit away from the sensor, need to add the min_range
        dist = dist + this->min_range;

        // did not hit an object
        if(dist > 999.0 && totalDist == 0.0)
            return false;

        //gzwarn << "Dist Ray:" << dist << " Name:" << name << std::endl;
        totalDist += dist;

        if(name != "")
        {
          this->cur_rayInfos.distances.push_back(totalDist);
          // return a material struct (default script if not in config)
          GpuRayCollisionPlugin::Material material = this->getMaterial(name);

          //reflectivity = material.reflectivity;
          angleOff = this->computeAngleOffset(totalDist, material.name, index);
          //gzwarn << "Angle Offset: "<<angleOff<<std::endl;
          this->cur_rayInfos.material.push_back(material);
          this->cur_rayInfos.angle_offset.push_back(angleOff);
          totalAbsorption += material.absorption;
        }
        else if (name == "" && !this->cur_rayInfos.distances.empty())
            return true;

        if(totalAbsorption > 1.0)// || reflectivity == 1.0)
            break;

        start = start + dir * dist;
    }
    return true;

}

////////////////////////////////////////////////////////////////
void GpuRayCollisionPlugin::castRay(
    ignition::math::Vector3d start,
    ignition::math::Vector3d end,
    double &dist, std::string &name)
{
  this->world_->Physics()->InitForThread();
  this->testRay->SetPoints(start, end);
  this->testRay->GetIntersection(dist, name);
  //this->cur_rayInfos.distances.push_back(dist);
  this->cur_rayInfos.names.push_back(name);
}

/////////////////////////////////////////////////////////
double GpuRayCollisionPlugin::computeAngleOffset(double dist,
    std::string name, int index)
{
  double old_dist = 0.0;
  double angle = 0.0;

  // first ray is an exception
  // only compute angle if ray has a neighbour
  if(index == 0 || (index - this->rayInfos[this->rayInfos.size()-1].rayIndex) != 1)
  {
    return 0.0;
  }

   int old_idx = this->rayInfos.size()-1;
    /*gzwarn << "Dist: " << dist << std::endl;
    gzwarn << "Index: " << index << std::endl;
    GpuRayCollisionPlugin::RaysInfo ray = this->rayInfos[this->rayInfos.size()-1];
    for (int i = 0; i<ray.distances.size();i++){
      gzwarn << "Old Dist: " << ray.distances[i]<< std::endl;
      gzwarn << "Old Mat: " << ray.material[i].name<< std::endl;
    }
    gzwarn << " Old Ray Index " << ray.rayIndex << std::endl;*/

  old_dist = this->getClosest(
    dist, old_idx, 0.2);

  double y = 0.0;
  double x = 0.0;

  if(old_dist != 0.0)
  {
    y = std::sin(this->angle_res.Radian()) * (-1*old_dist);
    x = -1*dist + old_dist*std::cos(this->angle_res.Radian());
    angle = std::abs(std::atan2(y ,x));
  }
  else
  {
    return 0.0;
  }
  //gzwarn << "Old Dist is 0" << std::endl;
  //gzwarn << " Angle: "<< std::setprecision (5) << angle << std::endl;
  return std::abs(PI_2 - angle);
}

double GpuRayCollisionPlugin::getClosest(double dist,
    int old_idx, double offset)
{

    double min_dist = 10000.0;
    double min_diff = 100.0;
    double cur_diff = 100.0;

    for(int i = 0; i < this->rayInfos[old_idx].distances.size(); i++)
    {
        cur_diff = std::abs(dist - this->rayInfos[old_idx].distances[i]);
        if(cur_diff < min_diff)
        {
            min_dist = this->rayInfos[old_idx].distances[i];
            min_diff = cur_diff;
        }
    }

    if(min_diff < offset)
        return min_dist;

    return 0.0;
}

/*
name: name of Gazebo Entity e.g. model1::Link1::Collision
the material should not be a mesh!!!
*/
GpuRayCollisionPlugin::Material GpuRayCollisionPlugin::getMaterial(
    std::string name)
{
    std::vector<std::string> string_vec;
    physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>
                            (this->world_->EntityByName(name)->GetParent());
    //physics::ModelPtr mod = (boost::dynamic_pointer_cast<physics::Link>
    //                        (this->world_->GetEntity(name)->GetParent()))->GetParentModel();
    if(!link){
    //if(!mod){
        gzerr<< "Couldn't get Parent"<<std::endl;
        return this->defaultMaterial;
    }

    unsigned int id;
    std::string mat = "";
    bool got_info = false;

    try{
    got_info = link->VisualId(link->GetScopedName()+"::visual", id);
    }catch(...){
    // FIXME works if the link has only 1 visual
    // with a default visuals name (link::visual)
    // could better work with VisualAt(CameraPtr _camera,
    //      const ignition::math::Vector2i
    //      &_mousePos, std::string &_mod)
    }

/*
    try{
      got_info = mod->GetChildLink("link")->VisualId(mod->GetChildLink("link")->GetScopedName()+"::visual", id);
    }catch(...){
    
    }
*/
    if (got_info)
    {
      // GetMaterialName return the name of the visual applied to this visual
      // e.g. bookshelf::link::visual_MATERIAL_Gazebo/Wood
      mat = this->scene->GetVisual(id)->GetMaterialName();
      //gzwarn<< "Material: "<<mat<<std::endl;
      boost::split(string_vec,mat,boost::is_any_of("MATERIAL"));
      mat = string_vec[string_vec.size() - 1];
      boost::replace_all(mat, "_", "");
      if(mat != ""){
        return this->getMaterialInfo(mat);
      }else{
        return this->defaultMaterial;
      }
    }
    return this->defaultMaterial;
}

/////////////////////////////////////////////////////////////////
double GpuRayCollisionPlugin::fixLastRange(int index)
{
  /*
  //TODO
  std::string name = "";
  double delta1 = 10000.0;
  double delta2 = 10000.0;
  if(i > 0)
    delta1 = std::abs(dist - prev);
  if(i < this->parentSensor->RangeCount() -1)
    delta2 = std::abs(dist - next);

  if(delta1 < delta2)
  {
    name = this->previous_name;
  }else
  {
    //TODO ???
    //fix_last = true
  }*/
  return 0.0;
}

//////////////////////////////////////////////////////////////////
GpuRayCollisionPlugin::Material GpuRayCollisionPlugin::getMaterialInfo(
    std::string name)
  {
    for(int i = 0; i < this->materials.size(); i++)
    {
      if(name == this->materials[i].name)
      {
        return this->materials[i];
      }
    }
    return this->defaultMaterial;
}

void GpuRayCollisionPlugin::loadMaterials(
        std::string path)
{
    //std::string p = "/home/alex/wentz_catkin_ws/src/youbot_simulation/gpu_laser_collision_test/src/material.config";
    std::string STRING;
    int count = 0;
    std::ifstream fin;
    fin.open(path);

    std::vector<std::string> strings;
    bool new_mat = false;
    GpuRayCollisionPlugin::Material mat;
    while(!fin.eof()) // To get you all the lines.
    {
        getline(fin,STRING);
        boost::replace_all(STRING, "\n", "");
        boost::replace_all(STRING, " ", "");
        strings.clear();
        if(STRING == "")
        {
            continue;
        }
        boost::split(strings,STRING,boost::is_any_of(":"));

        if(new_mat)
        {
            new_mat = false;
            count = 0;
        }

        if(strings[0] == "Material")
        {
            new_mat = true;
        }
        else
        {
            if(strings[0] == "name")
            {
                mat.name = strings[1];
                gzwarn << strings[1] << std::endl;
                count++;
            }
            else if(strings[0] == "reflectivity")
            {
                mat.reflectivity = std::stod(strings[1]);
                gzwarn << strings[1] << std::endl;
                count++;
            }
            else if(strings[0] == "transmission")
            {
                mat.transmission = std::stod(strings[1]);
                gzwarn << strings[1] << std::endl;
                count++;
            }
            else if(strings[0] == "absorption")
            {
                mat.absorption = std::stod(strings[1]);
                gzwarn << strings[1] << std::endl;
                count++;
            }
            else if(strings[0] == "angular_factor"){
                mat.angular_factor = std::stod(strings[1]);
                gzwarn << strings[1] << std::endl;
                count++;
            }

            if(count == 5)
            {
                gzwarn << "Saving Material: "<< mat.name <<std::endl;
                this->materials.push_back(mat);
                if(mat.name == "Default") this->defaultMaterial = mat;
            }
            //gzerr << "Mat_name: "<< mat.name<<" Reflect: "
            //    <<std::to_string(mat.reflectivity)<<std::endl;
        }
    }
    gzwarn<<"Material loaded"<<std::endl;
    fin.close();
}

void GpuRayCollisionPlugin::startROSCommunication()
{
  this->enable_service_ = this->nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
                                                (this->parent_save_name_+"/enable",
                                                boost::bind(&GpuRayCollisionPlugin::serviceEnable, this,_1,_2));
  this->disable_service_ = this->nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
                                                (this->parent_save_name_+"/disable",
                                                boost::bind(&GpuRayCollisionPlugin::serviceDisable, this,_1,_2));

  this->change_material_service_ = this->nh_->advertiseService<simulation_msgs::Material::Request,
                                                               simulation_msgs::Material::Response>
                                                (this->parent_save_name_+"/change_material",
                                                boost::bind(&GpuRayCollisionPlugin::serviceChangeMat, this,_1,_2));

  this->change_scanner_service_ = this->nh_->advertiseService<simulation_msgs::Scanner::Request,
                                                               simulation_msgs::Scanner::Response>
                                                (this->parent_save_name_+"/change_scanner_values",
                                                boost::bind(&GpuRayCollisionPlugin::serviceChangeScanner, this,_1,_2));

  this->add_material_service_ = this->nh_->advertiseService<simulation_msgs::Material::Request,
                                                            simulation_msgs::Material::Response>
                                                (this->parent_save_name_+"/add_material",
                                                boost::bind(&GpuRayCollisionPlugin::serviceAddMat, this,_1,_2));

  this->delete_material_service_ = this->nh_->advertiseService<simulation_msgs::DelMaterial::Request,
                                                              simulation_msgs::DelMaterial::Response>
                                                (this->parent_save_name_+"/delete_material",
                                                boost::bind(&GpuRayCollisionPlugin::serviceDelMat, this,_1,_2));

  this->change_percent_orig_scan_ = this->nh_->advertiseService<simulation_msgs::SetDoubleVal::Request,
                                                              simulation_msgs::SetDoubleVal::Response>
                                                (this->parent_save_name_+"/change_scan_percent",
                                                boost::bind(&GpuRayCollisionPlugin::serviceChangePercent, this,_1,_2));

  ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->topic_name_, 1,
      boost::bind(&GpuRayCollisionPlugin::SubConnect, this),
      boost::bind(&GpuRayCollisionPlugin::SubDisconnect, this),
      ros::VoidPtr(), NULL);
  this->pub_ = this->nh_->advertise(ao);

//  this->change_service_ = this->nh_->advertiseService<simulation_msgs::LaserCollision::Request,
//                                                simulation_msgs::LaserCollision::Response>
//                                                ("/change_service",
//                                                boost::bind(&GpuRayCollisionPlugin::changeService,
//                                                this,_1,_2));
}

bool GpuRayCollisionPlugin::serviceDelMat(simulation_msgs::DelMaterial::Request &req,
                                          simulation_msgs::DelMaterial::Response &res)
{
    int index = 0;
    std::vector<GpuRayCollisionPlugin::Material>::iterator it;
    for(it = this->materials.begin(); it != this->materials.end(); it++, index++)
    {
        if(it->name == req.name)
            break;
    }
    this->materials.erase(this->materials.begin() + index);
    return true;
}

bool GpuRayCollisionPlugin::serviceAddMat(simulation_msgs::Material::Request &req,
                                          simulation_msgs::Material::Response &res)
{
    GpuRayCollisionPlugin::Material mat;
    mat.name = req.name;
    mat.reflectivity = req.reflectivity;
    mat.transmission = req.transmission;
    mat.absorption = req.absorption;
    mat.angular_factor = req.angular_factor;
    this->materials.push_back(mat);
    return true;
}

bool GpuRayCollisionPlugin::serviceChangeMat(simulation_msgs::Material::Request &req,
                                          simulation_msgs::Material::Response &res)
{
    std::vector<GpuRayCollisionPlugin::Material>::iterator it;
    for(it = this->materials.begin(); it != this->materials.end(); it++)
    {
        if(it->name == req.name)
        {
            it->reflectivity = req.reflectivity;
            it->transmission = req.transmission;
            it->absorption = req.absorption;
            it->angular_factor = req.angular_factor;
            break;
        }
    }
    return true;
}

bool GpuRayCollisionPlugin::serviceChangeScanner(simulation_msgs::Scanner::Request &req,
                                          simulation_msgs::Scanner::Response &res)
{
    this->refl_scale = req.refl_scale;
    this->angular_scale_mean = req.angular_scale_mean;
    this->angular_scale_std = req.angular_scale_std;
    this->max_absorb_mean = req.max_absorb_mean;
    this->max_absorb_stddev = req.max_absorb_stddev;
    this->range_scale_fac = req.range_scale_fac;
    this->trans_scale_mean = req.trans_scale_mean;
    this->trans_scale_stddev = req.trans_scale_stddev;
    return true;
}

bool GpuRayCollisionPlugin::serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res){
    this->enabled = false;
    return true;
}

bool GpuRayCollisionPlugin::serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res){
    this->enabled = true;
    return true;
}

std::string GpuRayCollisionPlugin::getParentLinkSafeName()
{
    gzwarn<<this->parent_link_->GetScopedName()<<std::endl;
    std::string temp = this->parent_link_->GetScopedName();
    boost::replace_all(temp, "::", "_");
    //boost::replace_all(temp, "/", "");
    this->parent_save_name_ = temp;
    return temp;
}

void GpuRayCollisionPlugin::SubConnect()
{
  this->connect_count_++;
  if(this->connect_count_ == 1)
      this->subscriber_ = true;
}

void GpuRayCollisionPlugin::SubDisconnect()
{
  this->connect_count_--;
  if (this->connect_count_ == 0)
    this->subscriber_ = false;
}

bool GpuRayCollisionPlugin::serviceChangePercent(simulation_msgs::SetDoubleVal::Request &req,
                                                 simulation_msgs::SetDoubleVal::Response &res)
{
    if(req.value <= 1.0 && req.value >= 0.0)
        this->use_orig_scan_val_ = req.value;
    return true;
}




/*
//////////////////////////////////////////////////
void GpuRayCollisionPlugin::CheckForModels(
    ignition::math::Pose3d &_curPose, const physics::Model_V &_models)
{
  for (auto const &model : _models)
  {
    auto const &scopedName = model->GetScopedName();
    auto const aabb = model->BoundingBox();

    if (this->modelName != scopedName && this->frustum.Contains(aabb))
    {
        this->models.insert(model);
    }

    CheckForModels(_myPose, model->NestedModels());
  }
}
*/
/////////////////////////////////////////////////
//void GpuRayCollisionPlugin::OnNewLaserFrame(const float * /*_image*/,
//    unsigned int /*_width*/, unsigned int /*_height*/,
//    unsigned int /*_depth*/, const std::string &/*_format*/)
/*{
    ignition::math::Pose3d curPose = this->parentSensor->Pose() +
      this->parent_link_->WorldPose();
    this->frustum.SetPose(curPose);
    
    //iterate all models and check if frustum contains them
    this->models_.clear();
    this->CheckForModels(curPose, this->World->Models());
    
    // now check all scan points, need to know the model they are colliding
    
}
*/
