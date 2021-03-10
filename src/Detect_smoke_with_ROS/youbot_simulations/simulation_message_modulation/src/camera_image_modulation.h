#ifndef CAMERA_IMAGE_MODULATION_HH
#define CAMERA_IMAGE_MODULATION_HH

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
// ros message stuff
//#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
//#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Header.h>

// boost stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
// other stuff
#include <ignition/math/Rand.hh>
#include <queue>

class CameraImageModulation
{
    public:
      CameraImageModulation();
      ~CameraImageModulation();
      void Load();
      bool serviceEnable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
      bool serviceDisable(std_srvs::Empty::Request req, std_srvs::Empty::Response res);
    private:
      void Connect();
      void Disconnect();
      void QueueThread();
      void modulateHeaders();
      void modulateImageData();
      void publish();
      void doubleToStamp();
      void stampToDouble();

      bool enabled_;
      bool ready_;
      //int connection_count_;
      std::string input_topic_name_;
      std::string output_topic_name_;
      
      boost::shared_ptr<ros::NodeHandle> nh_;
      ros::ServiceServer enable_service_;
      ros::ServiceServer disable_service_;
      ros::Publisher pub_;
      ros::Subscriber sub_;

      int blur_num_factor_;
      float avg_time_stamp_delay_;
      float time_stamp_jitter_;
      float data_loss_percent_;
      
      ignition::math::Rand ng_;
      std::queue<sensor_msgs::ImageConstPtr> msg_queue_;
      sensor_msgs::Image last_image_;
      sensor_msgs::ImageConstPtr curr_image_;
      double double_time_;
      ros::Time stamp_;

      ros::Timer timer_;
      void timerCallback(const ros::TimerEvent&);
      void subCallback(const sensor_msgs::ImageConstPtr& msg);
    protected: 
        ros::CallbackQueue queue_;
        boost::shared_ptr<int> connection_count_;
        boost::thread callback_queue_thread_;
        void CameraQueueThread();
        boost::shared_ptr<boost::mutex> lock_;
};
#endif