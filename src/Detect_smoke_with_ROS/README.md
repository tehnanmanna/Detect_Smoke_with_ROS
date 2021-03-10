# Detect_smoke_with_ROS
Description:
This project is about Anamoly detection of unusual behavior in the enviroment such as smoke detection using indirect observing method as HMM ( Hidden MArkov model) which is based on the first order of the markov chain.

how it works?
in the gazeb environment there is a husky robot which has a LiDar which publishes laser vector
1) converting the laser beam vector into a point cloud is essential to start processing so that we created laser2pc.py that listens to /scan topic and converts to pointcloud3 and publishes /laser_pointcloud

2)after that point cloud processing is needed:
* Filteration of the point cloud using vox method , filter grid is applied with paramters of 5 cm in every axis
* Segementation is needed using RANSAC Algorithm , 100 max iteration
* KdTree is applied for Nearest Neighbor Search with distancec threshold of 2cm (can be changed)
*Clustering using Eculdiean clustering algorithm : where every cluster has minimum number of points of 3(can be changed) and maximum of 25000 (can be changed)

3) printing the results of how many clusters in normal and abnormal conditions also how many points in each cluster is also printed
