#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "camera_image_modulation.h"

CameraImageModulation::CameraImageModulation()
{
    this->ready_ = false;
}

CameraImageModulation::~CameraImageModulation()
{
    this->nh_->shutdown();
    this->queue_.clear();
    this->queue_.disable();
    //delete *this->nh_;
}

void CameraImageModulation::Load()
{
    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "camera_image_modulation_client",
          ros::init_options::NoSigintHandler);
    }

    this->nh_.reset(new ros::NodeHandle("~"));

    this->blur_num_factor_ = 5; // use 5 msg to create a blured image
    this->enabled_ = true;
    this->avg_time_stamp_delay_ = 0.02; // in sec
    this->time_stamp_jitter_ = 0.01;    // in sec
    this->data_loss_percent_ = 0.1; // 10 Percent
    this->input_topic_name_ = "/camera/image";
    this->output_topic_name_ = "/camera/image/modified";
    
    // TODO: read params from param server

    if (!this->connection_count_)
        this->connection_count_ = boost::shared_ptr<int>(new int(0));
    if (!this->lock_)
        this->lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);

    ros::AdvertiseOptions too =
      ros::AdvertiseOptions::create<sensor_msgs::Image>(
        this->output_topic_name_, 2,
        boost::bind(&CameraImageModulation::Connect, this),
        boost::bind(&CameraImageModulation::Disconnect, this),
        ros::VoidPtr(), &this->queue_);
    this->pub_ = this->nh_->advertise(too);

    ros::SubscribeOptions imso =
      ros::SubscribeOptions::create<sensor_msgs::Image>(
        this->input_topic_name_, 10,
        boost::bind(&CameraImageModulation::subCallback, this, _1),
        ros::VoidPtr(), &this->queue_);
    this->sub_ = nh_->subscribe(imso);

    unsigned int seed;
    seed = (unsigned int)ros::Time::now().toSec();
    this->ng_.Seed(seed);
    
    this->enable_service_ = this->nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
                                                    ("/camera_image_modulation/enable",
                                                    boost::bind(&CameraImageModulation::serviceEnable,this,_1,_2));
    this->disable_service_ = this->nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
                                                    ("/camera_image_modulation/disable",
                                                    boost::bind(&CameraImageModulation::serviceDisable,this,_1,_2));
    this->callback_queue_thread_ = boost::thread(
        boost::bind(&CameraImageModulation::QueueThread, this));
    
    this->timer_ = this->nh_->createTimer(
        ros::Duration(0.1),
        boost::bind(&CameraImageModulation::timerCallback,this,_1));
    
    this->last_image_ = sensor_msgs::Image();
    this->ready_ = true;
}

void CameraImageModulation::timerCallback(const ros::TimerEvent&)
{
    if(this->enabled_ && this->ready_ && this->pub_.getNumSubscribers() > 0)
    {
        this->curr_image_ = this->msg_queue_.front();

        this->modulateHeaders();
        this->modulateImageData();
        this->publish();
    }
}

void CameraImageModulation::modulateHeaders()
{
    this->stampToDouble();
    double jitter = this->ng_.DblUniform(this->avg_time_stamp_delay_
                                         -this->time_stamp_jitter_,
                                         this->avg_time_stamp_delay_
                                         +this->time_stamp_jitter_);
    this->double_time_ += jitter;
    this->doubleToStamp();
    std_msgs::Header head;
    head.frame_id = this->curr_image_->header.frame_id;
    head.seq = this->curr_image_->header.seq;
    head.stamp = this->stamp_;
    // TODO
    //this->curr_image_->header = head;
}

void CameraImageModulation::stampToDouble()
{
    this->double_time_ = double(this->curr_image_->header.stamp.sec)
        + double(this->curr_image_->header.stamp.nsec)/1000000000.0;
}

void CameraImageModulation::doubleToStamp()
{
    ros::Time stamp;
    stamp.fromSec(this->double_time_);
    this->stamp_ = stamp;
    //
}

void CameraImageModulation::modulateImageData()
{

}

void CameraImageModulation::publish()
{
    this->pub_.publish(this->last_image_);
}

bool CameraImageModulation::serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res)
{
    this->enabled_ = true;
    return true;
}

bool CameraImageModulation::serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res)
{
    this->enabled_ = false;
    return true;
}

void CameraImageModulation::Connect()
{
    boost::mutex::scoped_lock lock(*this->lock_);
    (*this->connection_count_)++;
}

void CameraImageModulation::Disconnect()
{
    boost::mutex::scoped_lock lock(*this->lock_);
    (*this->connection_count_)--;
    if ((*this->connection_count_) < 0)
    {
        (*this->connection_count_) = 0;
    }
}

void CameraImageModulation::subCallback(const sensor_msgs::ImageConstPtr& msg)
{
    this->msg_queue_.push(msg); 
}

void CameraImageModulation::QueueThread()
{
  static const double timeout = 0.001;

  while (this->nh_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
