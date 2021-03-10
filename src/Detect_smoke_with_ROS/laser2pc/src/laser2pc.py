#! /usr/bin/env python
#pip install cython==0.25.2 && pip install numpy
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

#tutorial important
#https://pcl.gitbook.io/tutorial/part-2/part02-chapter01/part02-chapter01-euclidean-pcl-python
class laser2pc():
	def __init__(self):
		self.laserProj=LaserProjection()
		self.pcPub=rospy.Publisher("/laser_pointcloud",pc2,queue_size=1)
		self.lasersub=rospy.Subscriber("/scan",LaserScan,self.laserCallback)
		


	def laserCallback(self,data):
		cloud_out=self.laserProj.projectLaser(data)
		#convert from point cloud 2 to PCL
		#pc = ros_numpy.numpify(cloud_out)
    		#points=np.zeros((pc.shape[0],3))
    		#points[:,0]=pc['x']
   		#points[:,1]=pc['y']
   		#points[:,2]=pc['z']
    		#p = pcl.PointCloud(np.array(points, dtype=np.float32))
		self.pcPub.publish(cloud_out)
"""
		white_cloud= XYZRGB_to_XYZ(p)
		#make filter
		fil = white_cloud.make_passthrough_filter()
   		fil.set_filter_field_name("z")
  		fil.set_filter_limits(0, 1.5)
		cloud_filtered=fil.filter()
		#self.pcPub.publish(cloud_filtered)
		#save inliers to a file	
		#fil.filter().to_file("inliers.pcd")
		#Segmentation
		seg = cloud_filtered.make_segmenter_normals(ksearch=50)
  		seg.set_optimize_coefficients(True)
   		seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
                seg.set_normal_distance_weight(0.1)
    		seg.set_method_type(pcl.SAC_RANSAC)
    		seg.set_max_iterations(100)
    		seg.set_distance_threshold(0.03)
   		indices, model = seg.segment()
		print(model)
		#cloud_plane = cloud_filtered.extract(indices, negative=False)
   		#NG : const char* not str
   		#cloud_plane.to_file('table_scene_mug_stereo_textured_plane.pcd')
   	        #pcl.save(cloud_plane, 'smoke_plane.pcd')
		#Clustering


		tree = cloud_filtered.make_kdtree_flann() #make_kdtree()

		ec = cloud_filtered.make_EuclideanClusterExtraction()
   		ec.set_ClusterTolerance(0.02)
   		ec.set_MinClusterSize(100)
   		ec.set_MaxClusterSize(25000)
 		ec.set_SearchMethod(tree)
  	        cluster_indices = ec.Extract()

   		print('cluster_indices : ' + str(cluster_indices.count) + " count.")
"""




if __name__ == '__main__':

	rospy.init_node("laser2pc")
	l2pc=laser2pc()	
	rospy.spin()
