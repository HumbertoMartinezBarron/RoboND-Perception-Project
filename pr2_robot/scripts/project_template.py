#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    ros_msg = ros_to_pcl(pcl_msg)
    
    # Statistical Outlier Filtering
    outlier_filter = ros_msg.make_statistical_outlier_filter()
    
    outlier_filter.set_mean_k(10)
    
    tsf = 0.01 # Threshold scale factor.
    outlier_filter.set_std_dev_mul_thresh(tsf)
    
    cloud = outlier_filter.filter()

    # Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    
    leaf_size = 0.01
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    
    cloud_filtered = vox.filter()

    # PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    
    ax_min = 0.6
    ax_max = 0.8
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
    ec.set_MinClusterSize(115) 		# Minimum cluster size.
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

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_cloud = pcl.PointCloud_PointXYZRGB()		# New cloud!
    cluster_cloud.from_list(color_cluster_point_list)	# Put in all the clusters, each in a different color.

    # Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        # Compute the associated feature vector
        ros_cluster = pcl_to_ros(pcl_cluster)
        
        color_histograms = compute_color_histograms(ros_cluster)
        normals = get_normals(ros_cluster)
        normal_histograms = compute_normal_histograms(normals)
        feature = np.concatenate((color_histograms, normal_histograms))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_position = list(white_cloud[pts_list[0]])
        label_position[2] += 0.4
        object_markers_pub.publish(make_label(label, label_position, index))

        # Add the detected object to the list of detected objects.
        det_obj = DetectedObject()
        det_obj.label = label
        det_obj.cloud = ros_cluster
        detected_objects.append(det_obj)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    TEST_SCENE_NUM = Int32()
    OBJECT_NAME = String()
    WHICH_ARM = String()
    PICK_POSE = Pose()
    PLACE_POSE = Pose()

    # Get/Read parameters
    pick_list = rospy.get_param('/object_list')
    dropbox = rospy.get_param('/dropbox')

    # Parse parameters into individual variables
    centroids = []
    
    labels = []
    
    for obj in object_list:
    	labels.append(obj.label)
    	points_arr = ros_to_pcl(obj.cloud).to_array()
    	centroids.append(\
        	   [np.asscalar(np.mean(points_arr, axis=0)[0]), \
        		np.asscalar(np.mean(points_arr, axis=0)[1]), \
        		np.asscalar(np.mean(points_arr, axis=0)[2])])

    # Loop through the pick list
    
    TEST_SCENE_NUM.data = 1
    
    dictionary_list = []
    
    i = 0
    
    # A while loop makes it easier to loop through the objects in the list while 
    #allowing us to skip undetected objects.
    while i < len(pick_list):
    	
        # Get the PointCloud for a given object and obtain its centroid
        OBJECT_NAME.data = pick_list[i]['name']
        
        # In case we did not detect all the objects, we will keep iterating through the list.
        while not OBJECT_NAME.data in labels:
        	if i < len(pick_list) - 1:
        		i += 1
        		print("PR2 did not detect "+str(OBJECT_NAME.data)+". Moving on to next object.")
        		OBJECT_NAME.data = pick_list[i]['name']
        	else:
        		send_to_yaml('output_1.yaml', dictionary_list)
        		raise ValueError("PR2 did not detect the last object: "+OBJECT_NAME.data+". Saved yaml file.")

        # Create 'place_pose' for the object
        # Assign the arm to be used for pick_place
        object_group = pick_list[i]['group']
        position = Point() 							# save the position in a Point object to build the pose.
        a = object_group == dropbox[1]['group']
        if a:
        	WHICH_ARM.data = 'right'
        else:
        	WHICH_ARM.data = 'left'
        position.x, position.y, position.z = dropbox[a]['position']
        PLACE_POSE.position = position
        
        point = Point()
        j = labels.index(OBJECT_NAME.data)	# Find the corresponding centroid by cross-referencing the two lists.
        point.x, point.y, point.z = centroids[j]
        PICK_POSE.position = point

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(TEST_SCENE_NUM, WHICH_ARM, OBJECT_NAME, PICK_POSE, PLACE_POSE)
        dictionary_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        i += 1

    # Output your request parameters into output yaml file
    send_to_yaml('output_1.yaml', dictionary_list)



if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('recognition_node')

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy .is_shutdown():
    	rospy.spin()
