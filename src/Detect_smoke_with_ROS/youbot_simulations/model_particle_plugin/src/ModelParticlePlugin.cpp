#include <math.h>
#include "ModelParticlePlugin.h"
#include <boost/algorithm/string/replace.hpp>


namespace gazebo
{

    //Constructor
    ParticlePlugin::ParticlePlugin()
    {
        std::cout << "Starting the Particle Plugin for Models" << std::endl;
        this->particle_update_time_ = 0.0;
        this->last_particle_update_time_ = common::Time(0);
    }
    //Destructor
    ParticlePlugin::~ParticlePlugin()
    {
        this->nh_->shutdown();
        //delete this->nh_;
    }

    void ParticlePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
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


        this->parent_ = _parent;


        this->sdf_ = _sdf;

        // Error message if the model couldn't be found
        if (!_parent)
        {
            ROS_ERROR("parent model is NULL");
            return;
        }


       this->world_ = this->parent_->GetWorld();
       ROS_INFO("World: %s",this->world_->Name().c_str());
       
       this->scene = rendering::get_scene();

        //get parameter from the sdf
        if (!this->sdf_->HasElement("particles"))
        {
            this->loadDefaultParticleParams();
            ROS_INFO("Loading default parameter for particles");
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

        if(this->parent_->GetWorld()->Running()){
            ROS_INFO("World is upand running");
        }
        this->lock = false;

        unsigned int seed;
        seed = (unsigned int)ros::Time::now().toSec();
        this->ng.Seed(seed);

        this->model_scoped_name = this->parent_->GetName();
        this->parent_link_ = this->parent_->GetLink("link");

        this->link_pose = parent_link_->WorldPose();
        this->parentSafeName(); //need to convert a scoped name for later (before: "name::name::name"  ->  after: "name_name_name")
        this->particle_link_name_ = "link_"+this->parent_save_name_;

        ROS_INFO("Loading Particles");
        this->spawnParticles();


        this->enabled_ = true;
        ignition::math::Pose3d p(100.0, 100.0, 0.5, 0.0, 0.0, 0.0);
        this->pos_far_away = p;
        ROS_INFO("Starting Services");
        this->enable_service_ = this->nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
                                                            ("/"+this->parent_save_name_+"/enable_Particle",
                                                            boost::bind(&ParticlePlugin::serviceEnable,this,_1,_2));
        this->disable_service_ = this->nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
                                                            ("/"+this->parent_save_name_+"/disable_Particle",
                                                            boost::bind(&ParticlePlugin::serviceDisable,this,_1,_2));

        this->change_service_ = this->nh_->advertiseService<simulation_msgs::LaserParticle::Request,
                                                            simulation_msgs::LaserParticle::Response>
                                                            ("/"+this->parent_save_name_+"/change_Particle",
                                                            boost::bind(&ParticlePlugin::changeParticlesService,
                                                            this,_1,_2));

       this->last_particle_update_time_ = this->parent_->GetWorld()->SimTime();
       // Connect to the sensor update event.
       this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ParticlePlugin::OnUpdate, this));

      // Make sure the parent sensor is active.

       ROS_INFO("ParticlePlugin Loaded");
     }// Load() Ende
    
    void ParticlePlugin::OnUpdate()
    {
        if(this->timeToParticleUpdate() && !this->lock && !this->new_particles)
        {
            this->updateParticles();
        }else if(this->new_particles)
        {
            //this->world_->SetPaused(true);
            //unsigned int steps = 100;
            //this->world_->Run(steps);
            //ROS_INFO("Stepping complete");
            //this->world_->SetPaused(false);

            //ignition::math::Pose3d p = math::Pose(this->link_pose);
            //ROS_INFO("1");
            this->particle_model_ = this->world_->ModelByName("Particle_"+this->parent_save_name_);
            if(this->particle_model_ != NULL)
            {
                this->particle_model_->SetLinkWorldPose(this->link_pose, this->particle_link_name_);
                this->new_particles = false;
            }
        }
    }

    void ParticlePlugin::updateParticles(){
        if(this->enabled_)
        {
            this->link_pose = parent_link_->WorldPose();
            // FIXME I am assuming that ther isn't a transformation between
            // parent_link and the sensor pose
            //ignition::math::Pose3d p = math::Pose(this->link_pose);
            this->particle_model_->SetLinkWorldPose(this->link_pose, this->particle_link_name_);
            this->moveParticles();
        }
        else
        {
            this->particle_model_->SetLinkWorldPose(this->pos_far_away, this->particle_link_name_);
        }
    }

    void ParticlePlugin::moveParticles(){

        uint32_t id;
        for(int i = 0; i <= this->num_particles; i++)
        {

            ignition::math::Pose3d p;
            p.Rot().Set(ng.DblUniform(0.0,1.0),ng.DblUniform(0.0,1.0),ng.DblUniform(0.0,1.0),ng.DblUniform(0.0,1.0));
            double range = this->ng.DblUniform(p_range-p_move_range,p_range+p_move_range);
            p.Pos().Set(ng.DblUniform(-range,range), ng.DblUniform(-range,range), ng.DblUniform(0.0,range));

            try{
            if(this->particle_model_->GetLink(this->particle_link_name_)->VisualId("visual_"+std::to_string(i),id)){
                this->particle_model_->GetLink(this->particle_link_name_)->SetVisualPose(id, p);
            }else{
                ROS_WARN("ParticlePlugin failed to get visual: visual_%s     ID: %d",std::to_string(i).c_str(),id);
            }}
            catch(...){
            
            }
        }
    }

    bool ParticlePlugin::timeToParticleUpdate()
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

    void ParticlePlugin::loadDefaultParticleParams(){
        this->p_range = 0.9;
        this->num_particles = 200;
        this->p_size = 0.4;
        this->p_move_range = 0.2;
        this->particle_update_time_.Set(0.3);
        this->p_hor_angle_min_ = -3.14;
        this->p_hor_angle_max_ = 3.14;
        this->p_transparency_ = 0.5;
    }

    bool ParticlePlugin::changeParticlesService(
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
                    ROS_WARN("ParticlePlugin failed to get visual: visual_%s     ID: %d",std::to_string(i).c_str(),id);
                }
            }
        }
        common::Time::MSleep(50);
        // check if our link exist
        bool success = true;
        try{
            if(this->world_->ModelByName("Particle_"+this->parent_save_name_)->GetLink(this->particle_link_name_)->GetName() == this->particle_link_name_){
                
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

    bool ParticlePlugin::valNotZero(int val)
    {
        if(val != 0)
            return true;
        else
            return false;
    }

    bool ParticlePlugin::valNotZero(double val)
    {
        if(val != 0.0)
            return true;
        else
            return false;
    }

    bool ParticlePlugin::valuesHasChanged(int num_particle,
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

    void ParticlePlugin::spawnParticles(){

        //ignition::math::Pose3d absolut_pose = this->link_pose + this->sensor_pose;
        std::string sensor_pose_as = pose3dToString(this->link_pose);
        sdf::SDF modelSDF;
        ROS_WARN("Parent %s",this->parent_save_name_.c_str());
        ROS_WARN("Link %s",this->particle_link_name_.c_str());
        ROS_WARN("Scoped %s",this->model_scoped_name.c_str());
        ROS_WARN("POSE: %s", sensor_pose_as.c_str());
        modelSDF.SetFromString(
        "<sdf version='1.6'>\
        <model name='Particle_"+this->parent_save_name_+"'>\
        <link name='"+this->particle_link_name_+"'>\
          <pose>"+sensor_pose_as+"</pose>\
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
        
        for(int i=1; i<=this->num_particles; i++)
        {
            sdf::ElementPtr visElem = model->GetElement("link")->GetElement("visual")->Clone();
            std::string s = std::to_string(i);

            double angle = ng.DblUniform(-3, 3);

            double range = this->ng.DblUniform(p_range-p_move_range,p_range+p_move_range);
            double x     = std::cos(angle)*range;
            double y     = std::sin(angle)*range ;
            std::string p = std::to_string(x)+" "+std::to_string(y)+" "+std::to_string(0.2)
                            +" "+std::to_string(ng.DblUniform(0.0,1.0))+" "+std::to_string(ng.DblUniform(0.0,1.0))
                            +" "+std::to_string(ng.DblUniform(0.0,1.0));
            visElem->GetAttribute("name")->SetFromString("visual_"+s);
            visElem->GetElement("pose")->GetValue()->SetFromString(p);

            model->GetElement("link")->InsertElement(visElem);
            //this->parent_link_->InsertElement(visElem);
        }
        //ROS_WARN("%s", modelSDF.ToString().c_str());
        ROS_INFO("Insert Model into World");
        this->model_ = model;

        
        this->world_->InsertModelSDF(modelSDF);
        //this->parent_->GetWorld()->InsertModelString(modelSDF.ToString());

        //common::Time::MSleep(1000);
        this->new_particles = true;
        this->particle_model_ = NULL;
    }

    void ParticlePlugin::parentSafeName(){
        size_t index = 0;
        std::string temp = this->model_scoped_name;
        boost::replace_all(temp, "::", "_");
        //ROS_WARN("%s",temp.c_str());
        this->parent_save_name_ = temp;
    }

    std::string ParticlePlugin::pose3dToString(ignition::math::Pose3d pose){
        std::string temp = std::to_string(pose.Pos().X()) +" "+ std::to_string(pose.Pos().Y()) +" "+ std::to_string(pose.Pos().Z()) +" "+std::to_string(pose.Rot().Euler().X()) +" "+ std::to_string(pose.Rot().Euler().Y()) +" "+ std::to_string(pose.Rot().Euler().Z());
        return temp;
    }

    bool ParticlePlugin::serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res){
        this->enabled_ = true;
        return true;
    }

    bool ParticlePlugin::serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res){
        this->enabled_ = false;
        return true;
    }


    void ParticlePlugin::respawnParticles()
    {
        //std::string sensor_pose_as = pose3dToString(this->sensor_pose);
        sdf::SDF modelSDF;
        modelSDF.SetFromString(
        "<sdf version='1.6'>\
        <model name='Particle_"+this->parent_save_name_+"'>\
        <link name='"+this->particle_link_name_+"'>\
          <pose frame='"+this->model_scoped_name+"'>0 0 0 0 0 0</pose>\
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

        for(int i=1; i<=this->num_particles; i++)
        {
            sdf::ElementPtr visElem = model->GetElement("link")->GetElement("visual")->Clone();
            std::string s = std::to_string(i);

            double angle = ng.DblUniform(this->p_hor_angle_min_,this->p_hor_angle_max_);
            double range = this->ng.DblUniform(p_range-p_move_range,p_range+p_move_range);
            double x     = std::cos(angle)*range;
            double y     = std::sin(angle)*range ;
            std::string p = std::to_string(x)+" "+std::to_string(y)+" "+std::to_string(0.5)
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
        common::Time::MSleep(700);
        ignition::math::Pose3d p = ignition::math::Pose3d(this->link_pose);
        this->particle_model_ = this->world_->ModelByName("Particle_"+this->parent_save_name_);
        this->particle_model_->SetLinkWorldPose(p, this->particle_link_name_);
    }

GZ_REGISTER_MODEL_PLUGIN(ParticlePlugin);
}
