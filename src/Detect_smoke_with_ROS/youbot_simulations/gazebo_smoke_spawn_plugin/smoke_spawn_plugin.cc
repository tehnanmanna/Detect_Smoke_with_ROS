#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <string>

namespace gazebo
{
class Factory : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Insert a model from string
    sdf::SDF modelSDF;
    modelSDF.SetFromString(
    "<sdf version='1.6'>\
    <model name='SmokeParticle'>\
    <link name='link_0'>\
      <pose frame=''>-0 -0 0.1 0 -0 0</pose>\
      <gravity>0</gravity>\
      <self_collide>0</self_collide>\
      <kinematic>0</kinematic>\
      <visual name='visual'>\
        <pose frame=''>0 0 0.1 0 -0 0</pose>\
        <geometry>\
          <box>\
            <size>0.01 0.01 0.001</size>\
          </box>\
        </geometry>\
        <material>\
          <lighting>0</lighting>\
          <script>\
            <uri>file://media/materials/scripts/gazebo.material</uri>\
            <name>Gazebo/Grey</name>\
          </script>\
        </material>\
        <transparency>0</transparency>\
        <cast_shadows>0</cast_shadows>\
      </visual>\
    </link>\
    <static>1</static>\
    </model>\
    </sdf>");
    sdf::ElementPtr model = modelSDF.root->GetElement("model");
    unsigned int seed;
    seed = 5;
    ignition::math::Rand ng;
    ng.Seed(seed);
    // Demonstrate using a custom model name.
    /*for(int i=0; i<=3000; i++)
    {
        std::string s = std::to_string(i);
        std::string p = std::to_string(ng.DblUniform(-1.0,1.0))+" "+std::to_string(ng.DblUniform(-1.0,1.0))+" "+ 
                        std::to_string(ng.DblUniform(0.0,1.0))+" "+std::to_string(ng.DblUniform(0.0,1.0))+" "+ 
                        std::to_string(ng.DblUniform(0.0,1.0))+" "+std::to_string(ng.DblUniform(0.0,1.0));
        model->GetAttribute("name")->SetFromString(s);
        model->GetElement("link")->GetElement("pose")->GetValue()->SetFromString(p);
        _parent->InsertModelSDF(modelSDF);
    }*/
    sdf::ElementPtr link = model->GetElement("link");
    sdf::SDF visSDF;
        visSDF.SetFromString(
        "<sdf version='1.6'>\
        <model name='SmokeParticle'>\
        <link name='link_0'>\
        <collision name='collision'>\
        <pose frame=''>0 0 0.1 0 -0 0</pose>\
        <geometry>\
          <box>\
            <size>0.01 0.01 0.001</size>\
          </box>\
        </geometry>\
        <max_contacts>0</max_contacts>\
        <surface>\
            <contact>\
            <collide_without_contact> 1 </collide_without_contact>\
            <ode/>\
            </contact>\
            <friction>\
              <ode/>\
            </friction>\
          </surface>\
        </collision>\
        <visual name='visual'>\
        <pose frame=''>0 0 0.1 0 -0 0</pose>\
        <geometry>\
          <box>\
            <size>0.01 0.01 0.001</size>\
          </box>\
        </geometry>\
        <material>\
          <lighting>0</lighting>\
          <script>\
            <uri>file://media/materials/scripts/gazebo.material</uri>\
            <name>Gazebo/Grey</name>\
          </script>\
        </material>\
        <transparency>0</transparency>\
        <cast_shadows>0</cast_shadows>\
      </visual>\
      </link>\
      </model>\
      </sdf>");
    for(int i=0; i<=1000; i++)
    {
        sdf::ElementPtr visElem = visSDF.root->GetElement("model")->GetElement("link")->GetElement("visual")->Clone();
        //sdf::ElementPtr colElem = visSDF.root->GetElement("model")->GetElement("link")->GetElement("collision")->Clone();
        std::string s = std::to_string(i);
        std::string p = std::to_string(ng.DblUniform(-1.0,1.0))+" "+std::to_string(ng.DblUniform(-1.0,1.0))+" "+ 
                        std::to_string(ng.DblUniform(0.045,0.057))+" "+std::to_string(ng.DblUniform(0.0,1.0))+" "+ 
                        std::to_string(ng.DblUniform(0.0,1.0))+" "+std::to_string(ng.DblUniform(0.0,1.0));
        visElem->GetAttribute("name")->SetFromString("visual_"+s);
        visElem->GetElement("pose")->GetValue()->SetFromString(p);
        //colElem->GetAttribute("name")->SetFromString("collision_"+s);
        //colElem->GetElement("pose")->GetValue()->SetFromString(p);
        //model->GetAttribute("name")->SetFromString(s);
        //model->GetElement("link")->GetElement("pose")->GetValue()->SetFromString(p);
        
        link->InsertElement(visElem);
        //link->InsertElement(colElem);
    }
    _parent->InsertModelSDF(modelSDF);
      /*
      // Option 3: Insert model from file via message passing.
    
      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->GetName());

      // Create a publisher on the ~/factory topic
      transport::PublisherPtr factoryPub =
      node->Advertise<msgs::Factory>("~/factory");

      // Create the message
      msgs::Factory msg;
      unsigned int seed;
      seed = 5;
      ignition::math::Rand ng;
      ng.Seed(seed);
      // Model file to load
      msg.set_sdf_filename("model://SmokeParticle");
      for(int i=0; i<=300; i++){
      // Pose to initialize the model to
        
        msgs::Set(msg.mutable_pose(),
          ignition::math::Pose3d(
            ignition::math::Vector3d(ng.DblUniform(-1.0,1.0), ng.DblUniform(-1.0,1.0), ng.DblUniform(0.0,1.0)),
            ignition::math::Quaterniond(0, 0, 0)));

      // Send the message
        factoryPub->Publish(msg);
      }*/
    
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}
