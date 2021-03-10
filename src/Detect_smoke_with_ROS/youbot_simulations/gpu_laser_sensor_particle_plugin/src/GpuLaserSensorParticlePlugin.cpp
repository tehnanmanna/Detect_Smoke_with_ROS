
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <math.h>
#include "GpuLaserSensorParticlePlugin.h"
#include <vector>
#include <boost/algorithm/string/replace.hpp>


namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(GpuLaserSensorParticlePlugin)
    //Constructor
    GpuLaserSensorParticlePlugin::GpuLaserSensorParticlePlugin()
    {
        std::cout << "Starting the Particle Plugin for Laser type Sensors" << std::endl;
        this->particle_update_time_ = 0.0;
        this->sensor_update_time_ = 0.0;
        this->last_sensor_update_time_ = common::Time(0);
        this->last_particle_update_time_ = common::Time(0);
    }
    //Destructor
    GpuLaserSensorParticlePlugin::~GpuLaserSensorParticlePlugin()
    {
        this->nh_->shutdown();
        //delete this->nh_;
    }

    void GpuLaserSensorParticlePlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        //std::cout << "Starting the Particle Plugin for Laser type Sensors" << std::endl;
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }
        std::string namespace_ = "";
        if (_sdf->HasElement("Namespace"))
            namespace_ = _sdf->GetElement("Namespace")->Get<std::string>() + "/";
        this->nh_.reset(new ros::NodeHandle(namespace_));
        // Store the model pointer for convenience.
        this->parent_ = std::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);
        this->sdf_ = _sdf;

        if(_sdf->HasElement("topicName")){
            this->topic_name = _sdf->GetElement("topicName")->Get<std::string>();
        }else{
            this->topic_name = "";
            gzwarn<<"topicName missing, only one sensor can be used in the simulation"<<std::endl;
        }


# if GAZEBO_MAJOR_VERSION >= 7
        std::string worldName = parent_->WorldName();
# else
        std::string worldName = parent_->GetWorldName();
# endif
       this->world_ = physics::get_world(worldName);
       GazeboRosLaser::Load(_parent,this->sdf_);
       this->scene = rendering::get_scene();

        sdf::ElementPtr sensorElem = this->sdf_->GetParent();
        sdf::ElementPtr scanElem = sensorElem->GetElement("ray")->GetElement("scan");
        if(!scanElem->HasElement("horizontal")){
            ROS_WARN("Sdf missing horizontal Element. Loading default value for horizontal angle");
            this->p_hor_angle_min_ = -3.14;
            this->p_hor_angle_max_ = 3.14;
        }else{
            sdf::ElementPtr horizontal = scanElem->GetElement("horizontal");
            try{
                this->p_hor_angle_min_ = horizontal->Get<double>("min_angle");
                this->p_hor_angle_max_ = horizontal->Get<double>("max_angle");
            }catch(...){
                ROS_WARN("Couldnt read horizontal angles from sdf. Loading default value for horizontal angle");
                this->p_hor_angle_min_ = -3.14;
                this->p_hor_angle_max_ = 3.14;
            }
        }

/*      TODO: For velodyne type laser sensors
        if(!scanElem->HasElement("vertical")){
            ROS_WARN("Sdf missing vertical Element. Loading default value for vertical angle");
            this->p_ver_angle_min_ = -0.3;
            this->p_ver_angle_max_ = 0.3;
        }else{
            sdf::ElementPtr vertical = scanElem->GetElement("vertical");
            try{
                this->p_ver_angle_min_ = horizontal->Get<double>("min_angle");
                this->p_ver_angle_max_ = horizontal->Get<double>("max_angle");
            }catch(...){
                ROS_WARN("Couldnt read vertical angles from sdf. Loading default value for vertical angle")
                this->p_ver_angle_min_ = -0.3;
                this->p_ver_angle_max_ = 0.3;
            }
        }
*/

        //get parameter from the sdf
        if (!this->sdf_->HasElement("particles"))
        {
            ROS_INFO_NAMED("gpu_laser", "GpuLaserSensorParticle Plugin missing <particles>, loading default");
            this->loadDefaultParticleParams();
        }
        else
        {
            sdf::ElementPtr particles = this->sdf_->GetElement("particles");
            if(!particles->HasElement("p_range")){ // range in meter
                this->p_range = 1.0;
            }else{
                this->p_range = particles->Get<double>("p_range");
            }
            if(!particles->HasElement("p_size")){ // size in meter (quadratic particles)
                this->p_size = 0.05;
            }else{
                this->p_size = particles->Get<double>("p_size");
            }
            if(!particles->HasElement("num_particles")){
                this->num_particles = 200;
            }else{
                this->num_particles = particles->Get<int>("num_particles");
            }
            double particle_time = 0.5;
            if(!particles->HasElement("particle_update_time_")){
                this->particle_update_time_.Set(particle_time);
            }else{
                particle_time = particles->Get<double>("particle_update_time_");
                this->particle_update_time_.Set(particle_time);
            }
            if(!particles->HasElement("p_move_range")){
                this->p_move_range = 0.2;
            }else{
                this->p_move_range = particles->Get<double>("p_move_range");
            }
        }

        this->lock = false;

        unsigned int seed;
        seed = (unsigned int)ros::Time::now().toSec();
        this->ng.Seed(seed);

        //get the pose of the sensor within the model
        this->sensor_pose = this->parent_->Pose();


        this->parent_link_scoped_name = this->parent_->ParentName();
        this->parent_link_ = boost::dynamic_pointer_cast<gazebo::physics::Link>(this->world_->EntityByName(this->parent_link_scoped_name));
        this->parent_model_ = parent_link_->GetParentModel();
        this->link_pose = parent_link_->WorldPose();
        this->parentSafeName(); //need to convert a scoped name for later (before: "name::name::name"  ->  after: "name_name_name")
        this->particle_link_name_ = "link_"+this->parent_save_name_;

        ROS_INFO("Loading Particles");
        this->spawnParticles();


        this->enabled_ = true;
        ignition::math::Pose3d p(100.0, 100.0, 0.5, 0.0, 0.0, 0.0);
        this->pos_far_away = p;
        this->enable_service_ = this->nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
                                                            ("/"+this->topic_name+"/enable_Particle",
                                                            boost::bind(&GpuLaserSensorParticlePlugin::serviceEnable,this,_1,_2));
        this->disable_service_ = this->nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
                                                            ("/"+this->topic_name+"/disable_Particle",
                                                            boost::bind(&GpuLaserSensorParticlePlugin::serviceDisable,this,_1,_2));


        this->change_service_ = this->nh_->advertiseService<simulation_msgs::LaserParticle::Request,
                                                            simulation_msgs::LaserParticle::Response>
                                                            ("/"+this->topic_name+"/change_Particle",
                                                            boost::bind(&GpuLaserSensorParticlePlugin::changeParticlesService,
                                                            this,_1,_2));

       this->last_sensor_update_time_ = this->world_->SimTime();
       this->last_particle_update_time_ = this->world_->SimTime();

       // Connect to the sensor update event.
       this->updateConnection = this->parent_->ConnectUpdated(
      std::bind(&GpuLaserSensorParticlePlugin::OnUpdate, this));

      // Make sure the parent sensor is active.
       this->parent_->SetActive(true);
       ROS_INFO("GpuLaserSensorParticlePlugin Loaded");
     }// Load() Ende
    
    void GpuLaserSensorParticlePlugin::OnUpdate()
    {
        if(this->timeToParticleUpdate() && !this->lock && !this->new_particles)
        {
            this->updateParticles();
        }else if(this->new_particles)
        {
            common::Time::MSleep(100);
            this->particle_model_ = this->world_->ModelByName("Particle_"+this->parent_save_name_+this->topic_name);
            if (this->particle_model_ != NULL)
            {
                gzwarn<<"loaded"<<std::endl;
                this->new_particles = false;
                //math::Pose p = math::Pose(this->link_pose);
                this->particle_model_ = this->world_->ModelByName("Particle_"+this->parent_save_name_+this->topic_name);
                this->particle_model_->SetLinkWorldPose(this->link_pose, this->particle_link_name_);
            }
        }
    }

    void GpuLaserSensorParticlePlugin::updateParticles(){
        if(this->enabled_)
        {
            //this->link_pose = parent_link_->GetWorldPose().Ign();
            this->link_pose = parent_link_->WorldPose();
            // FIXME I am assuming that ther isn't a transformation between
            // parent_link and the sensor pose
            //math::Pose p = math::Pose(this->link_pose);
            this->particle_model_->SetLinkWorldPose(this->link_pose, this->particle_link_name_);
            //this->particle_model_->SetLinkWorldPose(this->link_pose, this->particle_link_name_);
            this->moveParticles();
        }
        else
        {
            this->particle_model_->SetLinkWorldPose(this->pos_far_away, this->particle_link_name_);
        }
    }

    void GpuLaserSensorParticlePlugin::moveParticles(){
        double cur_angl;
        uint32_t id;
        for(int i = 0; i <= this->num_particles; i++)
        {
            cur_angl = this->p_angels.at(i);

            ignition::math::Pose3d p;
            p.Rot().Set(ng.DblUniform(0.0,1.0),ng.DblUniform(0.0,1.0),ng.DblUniform(0.0,1.0),ng.DblUniform(0.0,1.0));
            double range = this->ng.DblUniform(p_range-p_move_range,p_range+p_move_range);
            p.Pos().Set(std::cos(cur_angl)*range, std::sin(cur_angl)*range, this->sensor_pose.Pos().Z());

            try{
            if(this->particle_model_->GetLink(this->particle_link_name_)->VisualId("visual_"+std::to_string(i),id)){
                this->particle_model_->GetLink(this->particle_link_name_)->SetVisualPose(id, p);
            }else{
                ROS_WARN("GpuLaserSensorParticlePlugin failed to get visual: visual_%s     ID: %d",std::to_string(i).c_str(),id);
            }}
            catch(...){
            
            }
        }
    }


    /*bool GpuLaserSensorParticlePlugin::timeToSensorUpdate()
    {
      common::Time cur_time = this->world_->SimTime();
      if ((cur_time - this->last_sensor_update_time_).Double() >= this->sensor_update_time_)
      {
        this->last_sensor_update_time_ = cur_time;
        return true;
      }
      else
      {
        return false;
      }
    }*/

    bool GpuLaserSensorParticlePlugin::timeToParticleUpdate()
    {
      common::Time cur_time = this->world_->SimTime();
      if ((cur_time - this->last_particle_update_time_) >= this->particle_update_time_)
      {
        this->last_particle_update_time_ = cur_time;
        return true;
      }
      else
      {
        return false;
      }
    }

    void GpuLaserSensorParticlePlugin::loadDefaultParticleParams(){
        this->p_range = 0.7;
        this->num_particles = 200;
        this->p_size = 0.03;
        this->p_move_range = 0.4;
        this->particle_update_time_.Set(0.2);
        this->p_hor_angle_min_ = -3.14;
        this->p_hor_angle_max_ = 3.14;
        this->p_transparency_ = 0.5;
        /* TODO
        this->p_ver_angle_min_ = -0.3;
        this->p_ver_angle_max_ = 0.3;
        */
    }

    bool GpuLaserSensorParticlePlugin::changeParticlesService(
        simulation_msgs::LaserParticle::Request &req,
        simulation_msgs::LaserParticle::Response &res)
    {
        this->lock = true;
        // make sure there is not a update in process
        common::Time::MSleep(50);

        if(req.mean_range != 0.0)
            this->p_range = req.mean_range;
        if(req.move_range != 0.0)
            this->p_move_range = req.move_range;
        if(req.particle_update_time != 0.0)
            this->particle_update_time_.Set(req.particle_update_time);

        //num_particle, particle_size, horizontal_angle_min, horizontal_angle_max
        // need a re-initialize 
        if(this->valuesHasChanged(req.num_particle,
                                req.particle_size,
                                req.horizontal_angle_min,
                                req.horizontal_angle_max))
        {
            bool modified = false;
            //some sanity checks
            if(this->valNotZero(req.num_particle)){
                this->num_particles = req.num_particle;
                modified = true;
            }
            if(this->valNotZero(req.particle_size)){
                this->p_size = req.particle_size;
                modified = true;
            }
            if(req.horizontal_angle_min > -3.14 && this->valNotZero(req.horizontal_angle_min)){
                this->p_hor_angle_min_ = req.horizontal_angle_min;
                modified = true;
            }
            if(req.horizontal_angle_max < 3.14 && this->valNotZero(req.horizontal_angle_max)){
                this->p_hor_angle_max_ = req.horizontal_angle_max;
                modified = true;
            }
            //TODO Debug me
            if(modified){
                this->respawnParticles();
            }
        }

        // adjust transparency
        if(req.transparency != 0.0 && (this->p_transparency_ != req.transparency))
        {
            this->p_transparency_ = req.transparency;
            uint32_t id;
            for(int i = 0; i <= this->num_particles; i++)
            {
                if(this->particle_model_->GetLink(this->particle_link_name_)->VisualId("visual_"+std::to_string(i),id)){
                    rendering::VisualPtr vis = scene->GetVisual(id);
                    vis->SetTransparency(this->p_transparency_);
                }else{
                    ROS_WARN("GpuLaserSensorParticlePlugin failed to get visual: visual_%s     ID: %d",std::to_string(i).c_str(),id);
                }
            }
        }
        common::Time::MSleep(50);
        // check if our link exist
        bool success = true;
        try{
            if(this->world_->ModelByName("Particle_"+this->parent_save_name_+this->topic_name)->GetLink(this->particle_link_name_)->GetName() == this->particle_link_name_){
                
            }
            ROS_INFO("Change Particle Service was successfull");
        }
        catch(...){
            success = false;
        }
        res.success = success;
        this->lock = false;
        return true;
    }

    bool GpuLaserSensorParticlePlugin::valNotZero(int val)
    {
        if(val != 0)
            return true;
        else
            return false;
    }

    bool GpuLaserSensorParticlePlugin::valNotZero(double val)
    {
        if(val != 0.0)
            return true;
        else
            return false;
    }

    bool GpuLaserSensorParticlePlugin::valuesHasChanged(int num_particle,
                                         double particle_size,
                                         double horizontal_angle_min,
                                         double horizontal_angle_max)
    {
        if(this->num_particles != num_particle)
            return true;
        if(this->p_size !=particle_size)
            return true;
        if(this->p_hor_angle_max_ != horizontal_angle_max)
            return true;
        if(this->p_hor_angle_min_ != horizontal_angle_min)
            return true;
        return false;
    }

    void GpuLaserSensorParticlePlugin::spawnParticles(){

        //ignition::math::Pose3d absolut_pose = this->link_pose + this->sensor_pose;
        std::string sensor_pose_as = pose3dToString(this->sensor_pose);
        sdf::SDF modelSDF;
        modelSDF.SetFromString(
        "<sdf version='1.6'>\
        <model name='Particle_"+this->parent_save_name_+this->topic_name+"'>\
        <link name='"+this->particle_link_name_+"'>\
          <pose frame='"+this->parent_link_scoped_name+"'>"+sensor_pose_as+"</pose>\
          <gravity>0</gravity>\
          <self_collide>0</self_collide>\
          <kinematic>0</kinematic>\
          <visual name='visual_0'>\
            <pose>"+std::to_string(this->p_range)+" 0 0 0 -0 0</pose>\
            <geometry>\
              <box>\
                <size>"+std::to_string(p_size)+" "+std::to_string(p_size)+" 0.001</size>\
              </box>\
            </geometry>\
            <material>\
              <lighting>0</lighting>\
              <script>\
                <uri>file://media/materials/scripts/gazebo.material</uri>\
                <name>Gazebo/Grey</name>\
              </script>\
            </material>\
            <transparency>"+std::to_string(this->p_transparency_)+"</transparency>\
            <cast_shadows>0</cast_shadows>\
          </visual>\
        </link>\
        <static>0</static>\
        </model>\
        </sdf>");
        sdf::ElementPtr model = modelSDF.Root()->GetElement("model");
        this->p_angels.clear();
        this->p_angels.push_back(0.0); //first element is already in place
        for(int i=1; i<=this->num_particles; i++)
        {
            sdf::ElementPtr visElem = model->GetElement("link")->GetElement("visual")->Clone();
            std::string s = std::to_string(i);

            double angle = ng.DblUniform(this->p_hor_angle_min_,this->p_hor_angle_max_);
            this->p_angels.push_back(angle);
            double range = this->ng.DblUniform(p_range-p_move_range,p_range+p_move_range);
            double x     = std::cos(angle)*range;
            double y     = std::sin(angle)*range ;
            std::string p = std::to_string(x)+" "+std::to_string(y)+" "+std::to_string(this->sensor_pose.Pos().Z())
                            +" "+std::to_string(ng.DblUniform(0.0,1.0))+" "+std::to_string(ng.DblUniform(0.0,1.0))
                            +" "+std::to_string(ng.DblUniform(0.0,1.0));
            visElem->GetAttribute("name")->SetFromString("visual_"+s);
            visElem->GetElement("pose")->GetValue()->SetFromString(p);

            model->GetElement("link")->InsertElement(visElem);
            //this->parent_link_->InsertElement(visElem);
        }

        //physics::LinkPtr _particle = this->parent_model_->CreateLink("link_"+this->parent_save_name_);
        //_particle->Load(model->GetElement("link"));
        //this->parent_link_->AddChild(_particle);
        this->model_ = model;
        this->world_->InsertModelSDF(modelSDF);

        this->particle_model_ = NULL;
        this->new_particles = true;

        // tail ?
        //common::Time::MSleep(1000);
        //math::Pose p = math::Pose(this->link_pose);
        //this->particle_model_ = this->world_->ModelByName("Particle_"+this->parent_save_name_);
        //this->particle_model_->SetLinkWorldPose(this->link_pose, this->particle_link_name_);

    }

    void GpuLaserSensorParticlePlugin::parentSafeName(){
        size_t index = 0;
        std::string temp = this->parent_link_scoped_name;
        boost::replace_all(temp, "::", "_");
        //ROS_WARN("%s",temp.c_str());
        this->parent_save_name_ = temp;
    }

    std::string GpuLaserSensorParticlePlugin::pose3dToString(ignition::math::Pose3d pose){
        std::string temp = std::to_string(pose.Pos().X()) +" "+ std::to_string(pose.Pos().Y()) +" "+ std::to_string(pose.Pos().Z()) +" "+std::to_string(pose.Rot().Euler().X()) +" "+ std::to_string(pose.Rot().Euler().Y()) +" "+ std::to_string(pose.Rot().Euler().Z());
        return temp;
    }

    bool GpuLaserSensorParticlePlugin::serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res){
        this->enabled_ = true;
        return true;
    }

    bool GpuLaserSensorParticlePlugin::serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res){
        this->enabled_ = false;
        return true;
    }


    void GpuLaserSensorParticlePlugin::respawnParticles()
    {
        std::string sensor_pose_as = pose3dToString(this->sensor_pose);
        sdf::SDF modelSDF;
        modelSDF.SetFromString(
        "<sdf version='1.6'>\
        <model name='Particle_"+this->parent_save_name_+this->topic_name+"'>\
        <link name='"+this->particle_link_name_+"'>\
          <pose frame='"+this->parent_link_scoped_name+"'>"+sensor_pose_as+"</pose>\
          <gravity>0</gravity>\
          <self_collide>0</self_collide>\
          <kinematic>0</kinematic>\
          <visual name='visual_0'>\
            <pose>"+std::to_string(this->p_range)+" 0 0 0 -0 0</pose>\
            <geometry>\
              <box>\
                <size>"+std::to_string(p_size)+" "+std::to_string(p_size)+" 0.001</size>\
              </box>\
            </geometry>\
            <material>\
              <lighting>0</lighting>\
              <script>\
                <uri>file://media/materials/scripts/gazebo.material</uri>\
                <name>Gazebo/Grey</name>\
              </script>\
            </material>\
            <transparency>"+std::to_string(this->p_transparency_)+"</transparency>\
            <cast_shadows>0</cast_shadows>\
          </visual>\
        </link>\
        <static>0</static>\
        </model>\
        </sdf>");
        sdf::ElementPtr model = modelSDF.Root()->GetElement("model");

        this->p_angels.push_back(0.0); //first element is already in place
        for(int i=1; i<=this->num_particles; i++)
        {
            sdf::ElementPtr visElem = model->GetElement("link")->GetElement("visual")->Clone();
            std::string s = std::to_string(i);

            double angle = ng.DblUniform(this->p_hor_angle_min_,this->p_hor_angle_max_);
            this->p_angels.push_back(angle);
            double range = this->ng.DblUniform(p_range-p_move_range,p_range+p_move_range);
            double x     = std::cos(angle)*range;
            double y     = std::sin(angle)*range ;
            std::string p = std::to_string(x)+" "+std::to_string(y)+" "+std::to_string(this->sensor_pose.Pos().Z())
                            +" "+std::to_string(ng.DblUniform(0.0,1.0))+" "+std::to_string(ng.DblUniform(0.0,1.0))
                            +" "+std::to_string(ng.DblUniform(0.0,1.0));
            visElem->GetAttribute("name")->SetFromString("visual_"+s);
            visElem->GetElement("pose")->GetValue()->SetFromString(p);

            model->GetElement("link")->InsertElement(visElem);
        }

        this->world_->RemoveModel(this->particle_model_);
        common::Time::MSleep(500);
        this->model_ = model;
        this->world_->InsertModelSDF(modelSDF);
        this->particle_model_ = NULL;
        this->new_particles = true;
        //common::Time::MSleep(700);
        //math::Pose p = math::Pose(this->link_pose);
        //this->particle_model_ = this->world_->GetModel("Particle_"+this->parent_save_name_);
        //this->particle_model_->SetLinkWorldPose(p, this->particle_link_name_);
    }
}
