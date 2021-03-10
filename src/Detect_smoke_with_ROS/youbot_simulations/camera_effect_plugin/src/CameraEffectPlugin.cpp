#include "CameraEffectPlugin.h"
#include "gazebo/sensors/DepthCameraSensor.hh"
#include <ignition/math/Helpers.hh>
#include <map>
#include <boost/algorithm/string.hpp>


using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CameraEffectPlugin)

/////////////////////////////////////////////////
CameraEffectPlugin::CameraEffectPlugin()
: SensorPlugin(),rosnode_(ros::NodeHandle()), it_(rosnode_)//, width(0), height(0), depth(0)
{
  gzwarn << "Start of CameraEffectPlugin "<< std::endl;
}

CameraEffectPlugin::~CameraEffectPlugin()
{
  this->queue_.clear();
  this->queue_.disable();

  this->callback_queue_thread_.join();
  this->rosnode_.shutdown();
  //delete this->rosnode_;
}

/////////////////////////////////////////////////
void CameraEffectPlugin::Load(sensors::SensorPtr _sensor,
        sdf::ElementPtr _sdf)
{
  this->motion_blur_enabled = false;
  this->queue_filled = false;
  gzwarn << "Start loading CameraEffectPlugin "<< std::endl;
  this->parentSensor =
  std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

# if GAZEBO_MAJOR_VERSION >= 7
  std::string worldName = parentSensor->WorldName();
# else
  std::string worldName = parentSensor->GetWorldName();
# endif
/*
  this->world_ = physics::get_world(worldName);

  //this->parent_link_ = boost::dynamic_pointer_cast<physics::Link>(
  //  this->world_->GetEntity(this->parentSensor->ParentName()));

  unsigned int seed;
  common::Time cur = this->world_->GetSimTime();
  seed = (unsigned int)(cur.nsec + cur.sec);
  this->ng.Seed(seed);
*/
  std::string topic_;

  if(!this->parentSensor)
  {
    gzerr << "CameraEffectPlugin can not load CameraSensor"<<std::endl;
    return;
  }

  if(!_sdf->HasElement("TopicName"))
  {
    gzwarn<<"Using a default Topic name"<<std::endl;
    topic_ = "/"+this->parentSensor->ParentName()+"/camera/modified";
    boost::replace_all(topic_, "::", "/");
    this->topic = topic_;
  }else{
    this->topic = _sdf->GetElement("TopicName")->Get<std::string>();
  }
  if(!_sdf->HasElement("MotionBlurStartEnabled")){
    gzwarn << "Motion Blur on start disabled by default"<<std::endl;
    this->motion_blur_enabled = false;
  }else{
    this->motion_blur_enabled = _sdf->GetElement("MotionBlurStartEnabled")->Get<bool>();
  }
  if(!_sdf->HasElement("MotionBlurStrenght")){
    this->queueMaxSize = 6;
  }else{
    this->queueMaxSize = _sdf->GetElement("MotionBlurStrenght")->Get<int>();
  }
  if(!_sdf->HasElement("GaussianBlurStartEnabled")){
    this->gaussianBlurEnabled = false;
  }else{
    this->gaussianBlurEnabled = _sdf->GetElement("GaussianBlurStartEnabled")->Get<bool>();
  }
  if(!_sdf->HasElement("GaussKernelSize")){
    this->gKernelSize = 7;
  }else{
    // kernel size have to be an odd number
    this->gKernelSize = _sdf->GetElement("GaussKernelSize")->Get<int>();
    if(!(this->gKernelSize % 2 == 1))
        this->gKernelSize += 1;
  }
  if(!_sdf->HasElement("GaussSigma")){
    this->gSigma = 1.5;
  }else{
    this->gSigma = _sdf->GetElement("GaussSigma")->Get<double>();
  }
  if(!_sdf->HasElement("RandomNoiseStartEnabled")){
    this->randomNoise = false;
  }else{
    this->randomNoise = _sdf->GetElement("RandomNoiseStartEnabled")->Get<bool>();
  }
  if(!_sdf->HasElement("RandomNoiseStrength")){
    this->randNoiseStrength = 0.0;
  }else{
    this->randNoiseStrength = _sdf->GetElement("RandomNoiseStrength")->Get<double>();
  }
  if(!_sdf->HasElement("ColorSwapStartEnabled")){
    this->colorSwap = false;
  }else{
    this->colorSwap = _sdf->GetElement("ColorSwapStartEnabled")->Get<bool>();
  }
  if(!_sdf->HasElement("ColorSwapRed")){
    this->colorSwapRed = 0;
  }else{
    this->colorSwapRed = _sdf->GetElement("ColorSwapStartEnabled")->Get<bool>();
  }
  if(!_sdf->HasElement("ColorSwapGreen")){
    this->colorSwapGreen = 0;
  }else{
    this->colorSwapGreen = _sdf->GetElement("ColorSwapStartEnabled")->Get<bool>();
  }
  if(!_sdf->HasElement("ColorSwapBlue")){
    this->colorSwapBlue = 0;
  }else{
    this->colorSwapBlue = _sdf->GetElement("ColorSwapStartEnabled")->Get<bool>();
  }

  std::string namespace_ = "";
  if (_sdf->HasElement("Namespace"))
    namespace_ = _sdf->GetElement("Namespace")->Get<std::string>() + "/";

  //this->rosnode_ = new ros::NodeHandle(namespace_);
  //this->it_(*this->rosnode_);
  this->it_pub_ = this->it_.advertise(this->topic,1);
  this->connection_count = 0;

  this->gznode_ = transport::NodePtr(new transport::Node());
  this->gznode_->Init(worldName);
  this->scanPub =
    this->gznode_->Advertise<msgs::ImageStamped>(this->topic, 30);

  this->advertiseServices();

  //this->newUpdateConnection = this->parentSensor->ConnectUpdated(
  //  std::bind(&CameraEffectPlugin::OnUpdate, this));

  callback_queue_thread_ = boost::thread(boost::bind(&CameraEffectPlugin::QueueThread,this));

  this->camera = this->parentSensor->Camera();

  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      std::bind(&CameraEffectPlugin::OnNewFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  this->scene = rendering::get_scene();
  this->parentSensor->SetActive(false);

  this->rows = this->camera->ImageHeight();
  this->cols = this->camera->ImageWidth();
  
  this->GetImageFormat(this->camera->ImageFormat());
  
  this->img_queue_.resize(this->queueMaxSize);
  this->time = this->scene->SimTime();
  //this->fillMsgQueue();
  //this->enabled = true;
}

void CameraEffectPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_.ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

/////////////////////////////////////////////////
//void CameraEffectPlugin::OnUpdate()
//{
  
  //if(this->enabled){
  //this->parentSensor->SetActive(true);

  //this->parentSensor->SetActive(true);
  //gzwarn << "Here Update" << std::endl;
  //}
//}

/////////////////////////////////////////////////
void CameraEffectPlugin::OnNewFrame(const unsigned char *_image,
                              unsigned int /*_width*/,
                              unsigned int /*_height*/,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{
if(this->connection_count > 0){
  cv::Mat img = cv::Mat(this->rows, this->cols, CV_8UC3,
                       (void*)_image, this->step);
  this->time = this->scene->SimTime();

  if(this->queue_filled){
    this->img_queue_.pop_back();
    this->img_queue_.push_front(img.clone());
  }

  // adding motion blur
  if(this->motion_blur_enabled && this->queue_filled)
  {
    cv::Mat cur_img(img.size(), img.type());
    for(int i = 1; i < this->queueMaxSize -1; i++)
    {
        cv::Mat last = this->img_queue_.at(i);
        cv::addWeighted(img, 0.5, last, 0.5, 10.0, cur_img);
        //cv::scaleAdd(work_img, 0.5, last, cur_img);
        img = cur_img.clone();
    }
  }
  else if(!this->queue_filled)
  {
    this->fillMsgQueue();
    gzerr << "Size: " << this->img_queue_.size()<<std::endl;
    this->queue_filled = true;
  }

  if(this->gaussianBlurEnabled){
    try{
      cv::GaussianBlur(img, img,
              cv::Size(this->gKernelSize, this->gKernelSize),
              this->gSigma, this->gSigma);
    }
    catch(...){
    //
    }
  }

  if(this->randomNoise){
    cv::Mat noise(img.size(), img.type());
    cv::randu(noise, 0, 255);
    cv::addWeighted(img, 1.0 - this->randNoiseStrength, noise, this->randNoiseStrength, 0.0, img);
  }

  if(this->colorSwap){
    for(int i = 0; i < img.rows; i++){
      for(int j = 0; j < img.cols; j++){
        img.at<cv::Vec3b>(i,j)[0] += this->colorSwapRed;
        img.at<cv::Vec3b>(i,j)[1] += this->colorSwapGreen;
        img.at<cv::Vec3b>(i,j)[2] += this->colorSwapBlue;
      }
    }
  }
  this->publish(img);
}// if no connections do nothing

}

// for specific update frequenzy
bool CameraEffectPlugin::newImg()
{
    if ((this->scene->SimTime() - this->time) >0.1){
        this->time = this->scene->SimTime();
        return true;
    }
    return false;
}

void CameraEffectPlugin::publish(cv::Mat img)
{
    /*if (this->scanPub)
    {
      msgs::ImageStamped msg;
      msgs::Set(msg.mutable_time(), simTime);
      msg.mutable_image()->set_width(this->camera->ImageWidth());
      msg.mutable_image()->set_height(this->camera->ImageHeight());
      msg.mutable_image()->set_pixel_format(common::Image::ConvertPixelFormat(
            this->camera->ImageFormat()));

      msg.mutable_image()->set_step(this->camera->ImageWidth() *
          this->camera->ImageDepth());
      msg.mutable_image()->set_data(img.data,
          msg.image().width() * this->camera->ImageDepth() *
          msg.image().height());
      this->scanPub->Publish(msg);
    }*/
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    this->it_pub_.publish(msg);
}

//////////////////////////////////////////////
void CameraEffectPlugin::fillMsgQueue()
{
    const unsigned char *arg = NULL;

    while(this->img_queue_.size()<2*this->queueMaxSize)
    {
        usleep(100000);
        arg = this->camera->ImageData();

        if (arg){
        try{
          cv::Mat img = cv::Mat(this->rows, this->cols, CV_8UC3,
                              (void*)arg, this->step);
          for (int i = 0; i <= this->queueMaxSize; i++){
            this->img_queue_.push_front(img.clone());
          }
        }
        catch(cv::Exception e){
          gzwarn<< "Error";
        }
        }
    }
    // sometimes first ImageData is empty thus
    // write 2 times the images and then delete 
    // the older ones
    while(this->img_queue_.size() > (this->queueMaxSize)){
        this->img_queue_.pop_back();
    }
    gzwarn<< std::endl;
}

////////////////////////////////////////////////////////
void CameraEffectPlugin::GetImageFormat(const std::string format)
{
  if (format == "L8")
  {
    this->encoding = sensor_msgs::image_encodings::MONO8;
    this->skip = 1;
  }
  else if (format == "R8G8B8")
  {
    this->encoding = sensor_msgs::image_encodings::RGB8;
    this->skip = 3;
  }
  else if (format == "B8G8R8")
  {
    this->encoding = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }
  else if (format == "BAYER_RGGB8")
  {
    this->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip = 1;
  }
  else if (format == "BAYER_BGGR8")
  {
    this->encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip = 1;
  }
  else if (format == "BAYER_GBRG8")
  {
    this->encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip = 1;
  }
  else if (format == "BAYER_GRBG8")
  {
    this->encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
    this->skip = 1;
  }
  else
  {
    gzerr<<"CameraEffectPlugin: Unsupported Gazebo ImageFormat"<<std::endl;
    this->encoding = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }
  this->step = this->skip * this->cols;
}

/************************
########################
### Helper Functions ###
########################
************************/

/*void CameraEffectPlugin::SensorImageToCvArray()
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(this->image_,
            sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      gzerr<<"cv_bridge exception "<< e.what()<<std::endl;
      return;
    }
    this->img_queue_.push(cv_ptr->Image);
}*/

////////////////////////////////////////////////////////////
/*void CameraEffectPlugin::OgreImageToSensorImage(sensor_msgs::Image& image,
//                  const std::string& encoding_arg,
//                  uint32_t rows_arg,
//                  uint32_t cols_arg,
//                  uint32_t step_arg,
                  const void* arg)
{
     image.encoding = this->encoding;
     image.height   = this->rows;
     image.width    = this->cols;
     image.step     = this->skip;
     //size_t st0 = (this->step * this->cols);
     image.data.resize(this->step);
     memcpy(&image.data[0], arg, this->step);
     image.is_bigendian = 0;
}*/

void CameraEffectPlugin::advertiseServices()
{
  std::string enable_blur_service_name("enable_motion_blur");
  ros::AdvertiseServiceOptions enable_blur_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          enable_blur_service_name,
                                                          boost::bind(&CameraEffectPlugin::enableMotionBlur,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->enable_blur_service_ = this->rosnode_.advertiseService(enable_blur_aso);

  std::string disable_blur_service_name("disable_motion_blur");
  ros::AdvertiseServiceOptions disable_blur_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          disable_blur_service_name,
                                                          boost::bind(&CameraEffectPlugin::disableMotionBlur,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->disable_blur_service_ = this->rosnode_.advertiseService(disable_blur_aso);

  std::string change_blur_service_name("change_motion_blur");
  ros::AdvertiseServiceOptions change_blur_aso =
    ros::AdvertiseServiceOptions::create<camera_effect_plugin::SetIntVal>(
                                                          change_blur_service_name,
                                                          boost::bind(&CameraEffectPlugin::changeMotionBlurStrength,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->change_blur_service_ = this->rosnode_.advertiseService(change_blur_aso);

  std::string enable_gaussian_service_name("enable_gaussian_blur");
  ros::AdvertiseServiceOptions enable_gaussion_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          enable_gaussian_service_name,
                                                          boost::bind(&CameraEffectPlugin::enableGaussianBlur,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->enable_gaussian_service_ = this->rosnode_.advertiseService(enable_gaussion_aso);

  std::string disable_gaussian_service_name("disable_gaussian_blur");
  ros::AdvertiseServiceOptions disable_gaussion_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          disable_gaussian_service_name,
                                                          boost::bind(&CameraEffectPlugin::disableGaussianBlur,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->disable_gaussian_service_ = this->rosnode_.advertiseService(disable_gaussion_aso);

  std::string change_gauss_kernel_name("change_gauss_kernel_size");
  ros::AdvertiseServiceOptions change_kernel_aso =
    ros::AdvertiseServiceOptions::create<camera_effect_plugin::SetIntVal>(
                                                          change_gauss_kernel_name,
                                                          boost::bind(&CameraEffectPlugin::changeGaussKernel,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->change_gauss_kernel_service_ = this->rosnode_.advertiseService(change_kernel_aso);

  std::string change_gauss_sigma_name("change_gauss_sigma");
  ros::AdvertiseServiceOptions change_sigma_aso =
    ros::AdvertiseServiceOptions::create<camera_effect_plugin::SetDoubleVal>(
                                                          change_gauss_sigma_name,
                                                          boost::bind(&CameraEffectPlugin::changeGaussSigma,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->change_gauss_sigma_service_ = this->rosnode_.advertiseService(change_sigma_aso);

  std::string enable_random_service_name("enable_random_noise");
  ros::AdvertiseServiceOptions enable_random_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          enable_random_service_name,
                                                          boost::bind(&CameraEffectPlugin::enableRandomNoise,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->enable_random_service_ = this->rosnode_.advertiseService(enable_random_aso);

  std::string disable_random_service_name("disable_random_noise");
  ros::AdvertiseServiceOptions disable_random_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          disable_random_service_name,
                                                          boost::bind(&CameraEffectPlugin::disableRandomNoise,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->disable_random_service_ = this->rosnode_.advertiseService(disable_random_aso);

  std::string change_random_noise_name("change_random_noise_strength");
  ros::AdvertiseServiceOptions change_random_noise_aso =
    ros::AdvertiseServiceOptions::create<camera_effect_plugin::SetDoubleVal>(
                                                          change_random_noise_name,
                                                          boost::bind(&CameraEffectPlugin::changeRandomNoise,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->change_random_noise_service_ = this->rosnode_.advertiseService(change_random_noise_aso);

  std::string enable_color_swap_service_name("enable_color_swap");
  ros::AdvertiseServiceOptions enable_color_swap_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          enable_color_swap_service_name,
                                                          boost::bind(&CameraEffectPlugin::enableColorSwap,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->enable_color_swap_service_ = this->rosnode_.advertiseService(enable_color_swap_aso);

  std::string disable_color_swap_service_name("disable_color_swap_noise");
  ros::AdvertiseServiceOptions disable_color_swap_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                                                          disable_color_swap_service_name,
                                                          boost::bind(&CameraEffectPlugin::disableColorSwap,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->disable_color_swap_service_ = this->rosnode_.advertiseService(disable_color_swap_aso);

  std::string change_color_swap_name("change_color_swap");
  ros::AdvertiseServiceOptions change_color_swap_aso =
    ros::AdvertiseServiceOptions::create<camera_effect_plugin::ColorSwap>(
                                                          change_color_swap_name,
                                                          boost::bind(&CameraEffectPlugin::changeColorSwap,this,_1,_2),
                                                          ros::VoidPtr(), &queue_);
  this->change_color_swap_service_ = this->rosnode_.advertiseService(change_color_swap_aso);

  ros::AdvertiseOptions image_ao =
    ros::AdvertiseOptions::create<sensor_msgs::Image>(
      this->topic,1,
      boost::bind( &CameraEffectPlugin::OnSubscriberConnect,this),
      boost::bind( &CameraEffectPlugin::OnSubscriberDisconnect,this),
      ros::VoidPtr(), &this->queue_);
  this->image_pub_ = this->rosnode_.advertise(image_ao);
}

void CameraEffectPlugin::OnSubscriberConnect(){
    if(this->connection_count < 0)
        this->connection_count = 0;
    this->connection_count++;
    if (this->connection_count > 0)
        this->parentSensor->SetActive(true);
}

void CameraEffectPlugin::OnSubscriberDisconnect(){
    this->connection_count--;
    if(this->connection_count < 0)
        this->connection_count = 0;
    if(this->connection_count == 0)
        this->parentSensor->SetActive(false);
}

bool CameraEffectPlugin::changeColorSwap(
        camera_effect_plugin::ColorSwap::Request &req,
        camera_effect_plugin::ColorSwap::Response &res)
{
  if(req.red > 255){
    this->colorSwapRed = 255;
  }else{
    this->colorSwapRed = req.red;
  }
  if(req.green > 255){
    this->colorSwapGreen = 255;
  }else{
    this->colorSwapGreen = req.green;
  }
  if(req.blue > 255){
    this->colorSwapBlue = 255;
  }else{
    this->colorSwapBlue = req.blue;
  }
  return true;
}

bool CameraEffectPlugin::changeRandomNoise(
        camera_effect_plugin::SetDoubleVal::Request &req,
        camera_effect_plugin::SetDoubleVal::Response &res)
{
  if(req.value > 1.0){
    this->randNoiseStrength = 1.0;
  }else if(req.value < 0.0){
    this->randNoiseStrength = 0.0;
  }else{
    this->randNoiseStrength = req.value;
  }
  return true;
}

bool CameraEffectPlugin::changeGaussKernel(
        camera_effect_plugin::SetIntVal::Request &req,
        camera_effect_plugin::SetIntVal::Response &res)
{
  // kernel size have to be an odd number
  if(!(req.value % 2 == 0))
    this->gKernelSize = req.value;
  else
    this->gKernelSize = req.value + 1;
  return true;
}

bool CameraEffectPlugin::changeGaussSigma(
        camera_effect_plugin::SetDoubleVal::Request &req,
        camera_effect_plugin::SetDoubleVal::Response &res)
{
  this->gSigma = req.value;
  return true;
}

bool CameraEffectPlugin::changeMotionBlurStrength(
        camera_effect_plugin::SetIntVal::Request &req,
        camera_effect_plugin::SetIntVal::Response &res)
{
  bool old = this->motion_blur_enabled;
  this->motion_blur_enabled = false;
  this->queueMaxSize = req.value;
  this->fillMsgQueue();
  this->motion_blur_enabled = old;
  return true;
}

bool CameraEffectPlugin::enableColorSwap(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  this->colorSwap = true;
  return true;
}

bool CameraEffectPlugin::disableColorSwap(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  this->colorSwap = false;
  return true;
}


bool CameraEffectPlugin::enableGaussianBlur(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  this->gaussianBlurEnabled = true;
  return true;
}

bool CameraEffectPlugin::disableGaussianBlur(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  this->gaussianBlurEnabled = false;
  return true;
}

bool CameraEffectPlugin::enableRandomNoise(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  this->randomNoise = true;
  return true;
}

bool CameraEffectPlugin::disableRandomNoise(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  this->randomNoise = false;
  return true;
}

bool CameraEffectPlugin::enableMotionBlur(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  this->motion_blur_enabled = true;
  return true;
}

bool CameraEffectPlugin::disableMotionBlur(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  this->motion_blur_enabled = false;
  return true;
}

