#!/usr/bin/env python

# Import modules
from pcl_helper import *
import pcl

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    
    leaf_size = 0.01
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    
    cloud_filtered = vox.filter()

    # PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    
    ax_min = 0.77
    ax_max = 1.1
    passthrough.set_filter_limits(ax_min, ax_max)
    
    cloud_filtered = passthrough.filter()

    # RANSAC Plane Segmentation
    segmenter = cloud_filtered.make_segmenter()
    
    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    
    max_dist = 0.01
    segmenter.set_distance_threshold(max_dist)
    
    inliers, coeff = segmenter.segment()

    # Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    kd_tree = white_cloud.make_kdtree()
    
    ec = white_cloud.make_EuclideanClusterExtraction() # Cluster extraction object.
    
    ec.set_ClusterTolerance(0.05) 	# distance threshold.
    ec.set_MinClusterSize(200) 		# Minimum cluster size.
    ec.set_MaxClusterSize(1250)		# Maximum cluster size.
    
    ec.set_SearchMethod(kd_tree)	# Search our tree.
    
    cluster_indices = ec.Extract()
    
    cluster_color = get_color_list(len(cluster_indices))
    
    color_cluster_point_list = []
    
    for j, indices in enumerate(cluster_indices):		# Assign a color to each segmented object in the scene.
    	for i, index in enumerate(indices):
    		color_cluster_point_list.append([white_cloud[index][0],
    										 white_cloud[index][1],
    										 white_cloud[index][2],
    										 rgb_to_float(cluster_color[j])])
    
    cluster_cloud = pcl.PointCloud_PointXYZRGB()		# New cloud!
    cluster_cloud.from_list(color_cluster_point_list)	# Put in all the clusters, each in a different color.

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy .is_shutdown():
    	rospy.spin()
