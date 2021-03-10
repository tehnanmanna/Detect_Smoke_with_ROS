#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub;
//ros::Publisher pub1;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Read in the cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
 // pcl::PCDReader reader;
// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    // pcl::PointCloud<pcl::PointXYZ> cloud;
     pcl::fromROSMsg(*input, *cloud);
 

 // reader.read ("table_scene_lms400.pcd", *cloud);
  //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm :  <sensor_msgs::PointCloud2>
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  vg.setInputCloud (cloud);
 // vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.setLeafSize (0.05f, 0.05f, 0.05f); //10cm
  vg.filter (*cloud_filtered);
  //std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.2);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  if (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      //std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
     // break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
   // extract.setNegative (true);
  //  extract.filter (*cloud_f);
  //  *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  //pcl::search::KdTree<pcl::PointXYZ> tree;
if(cloud_filtered->size()>0){
  tree->setInputCloud (cloud_filtered);
   // std::cout << "here in kd tree" << std::endl;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.2); // 2cm
  ec.setMinClusterSize (3);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
    std::cout << " " << cluster_indices.size() << std::endl;
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
   // std::cout << "in for" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "cluster="<< j <<"PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    //ss << "/home/mohamed/catkin_ws/pcl_cpp_tutorial/src/cloud_cluster_" << j << ".pcd";
   // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
    msg.data = j;
  }

}
  //sensor_msgs::PointCloud2 output;
  std_msgs::Int16 msg;
  
  // Publish the data
  //pub.publish (output);
  pub.publish(msg);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  //ros::init (argc, argv, "pcl_cpp_tutorial");
  ros::init (argc, argv, "HMM_model");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("/laser_pointcloud", 1, cloud_cb);

  pub = nh.advertise<std_msgs::Int16>("/data", 1);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("/output_cloud", 1);


  // Spin
  ros::spin ();
}
