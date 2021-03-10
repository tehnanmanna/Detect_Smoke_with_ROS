#ifndef _GAZEBO_GPU_LASER_COLLISION_PLUGIN_HH_
#define _GAZEBO_GPU_LASER_COLLISION_PLUGIN_HH_

#include <string>

#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include <deque>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include "std_srvs/Empty.h"
#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>
#include "camera_effect_plugin/SetIntVal.h"
#include "camera_effect_plugin/SetDoubleVal.h"
#include "camera_effect_plugin/ColorSwap.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace gazebo
{
  class CameraEffectPlugin : public SensorPlugin
  {

    public: CameraEffectPlugin();

    public: ~CameraEffectPlugin();

    public: bool enableMotionBlur(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    public: bool disableMotionBlur(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    public: bool enableGaussianBlur(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    public: bool disableGaussianBlur(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    public: bool enableRandomNoise(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    public: bool disableRandomNoise(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    public: bool enableColorSwap(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    public: bool disableColorSwap(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    public: bool changeColorSwap(
        camera_effect_plugin::ColorSwap::Request &req,
        camera_effect_plugin::ColorSwap::Response &res);

    public: bool changeGaussKernel(
        camera_effect_plugin::SetIntVal::Request &req,
        camera_effect_plugin::SetIntVal::Response &res);

    public: bool changeGaussSigma(
        camera_effect_plugin::SetDoubleVal::Request &req,
        camera_effect_plugin::SetDoubleVal::Response &res);

    public: bool changeMotionBlurStrength(
        camera_effect_plugin::SetIntVal::Request &req,
        camera_effect_plugin::SetIntVal::Response &res);

    public: bool changeRandomNoise(
        camera_effect_plugin::SetDoubleVal::Request &req,
        camera_effect_plugin::SetDoubleVal::Response &res);
    //public: void OnUpdate();

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    private: transport::PublisherPtr scanPub;

    private: transport::NodePtr gznode_;

    private: bool newImg();
    private: void advertiseServices();

//    protected: unsigned int width, height, depth;

    protected: sensors::CameraSensorPtr parentSensor;
    protected: rendering::CameraPtr camera;

    //private: event::ConnectionPtr newUpdateConnection;
    private: event::ConnectionPtr newFrameConnection;

    //private: physics::WorldPtr world_;
    
    //private: physics::LinkPtr parent_link_;

    private: rendering::ScenePtr scene;

    //private: ignition::math::Rand ng;

    private: bool motion_blur_enabled;
    
    private: std::deque<cv::Mat> img_queue_;

    private: void fillMsgQueue();

    private: int queueMaxSize;

    //private: sensor_msgs::Image image_;

    private: void GetImageFormat(const std::string format);
    /*private: void SensorImageToCvArray();*/
    /*private: void OgreImageToSensorImage(
                                    sensor_msgs::Image& image,
                                    const void* arg);*/

    private: std::string encoding;
    private: int rows;
    private: int cols;
    private: size_t step;
    private: int skip;
    private: common::Time time;
    private: bool queue_filled;
    private: bool gaussianBlurEnabled;
    private: int gKernelSize;
    private: double gSigma;
    private: bool randomNoise;
    private: double randNoiseStrength;
    private: bool colorSwap;
    private: int colorSwapBlue;
    private: int colorSwapGreen;
    private: int colorSwapRed;

    private: void publish(cv::Mat img);

    private: ros::NodeHandle rosnode_;
    private: ros::Publisher pub_;

    private: ros::ServiceServer enable_blur_service_;
    private: ros::ServiceServer disable_blur_service_;
    private: ros::ServiceServer change_blur_service_;
    private: ros::ServiceServer disable_gaussian_service_;
    private: ros::ServiceServer enable_gaussian_service_;
    private: ros::ServiceServer change_gauss_kernel_service_;
    private: ros::ServiceServer change_gauss_sigma_service_;
    private: ros::ServiceServer change_random_noise_service_;
    private: ros::ServiceServer disable_random_service_;
    private: ros::ServiceServer enable_random_service_;
    private: ros::ServiceServer enable_color_swap_service_;
    private: ros::ServiceServer disable_color_swap_service_;
    private: ros::ServiceServer change_color_swap_service_;
    private: ros::Publisher image_pub_;
    private: void OnSubscriberConnect();
    private: void OnSubscriberDisconnect();
    private: int connection_count;
    private: std::string topic;
    private: image_transport::ImageTransport it_;
    private: image_transport::Publisher it_pub_;

    private: void QueueThread();
    private: ros::CallbackQueue queue_;
    private: boost::thread callback_queue_thread_;
  };
}
#endif
