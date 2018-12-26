# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')


# Voxel Grid filter
# Create a VoxelGrid filter object for our input point cloud
vox = cloud.make_voxel_grid_filter()

# Choose a voxel (also known as leaf) size
# Note: this (1) is a poor choice of leaf size   
# Experiment and find the appropriate size!
LEAF_SIZE = 0.01

# Set the voxel (or leaf) size  
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

# Call the filter function to obtain the resultant downsampled point cloud
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)

# PassThrough filter
# Make a Passthrough filter object.
passthrough = cloud_filtered.make_passthrough_filter()

# Assign axis and range to the passthrough filter object.
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)

# Finally use the filter funciton to obtain the resultant pointcloud.
cloud_filtered = passthrough.filter()
filename = "pass_through_filtered.pcd"
pcl.save(cloud_filtered, filename)

# RANSAC plane segmentation
# Create the segmentation object.
seg = cloud_filtered.make_segmenter()

# Set the model you wish to fit it.
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

# Max distance for a point to be consider fitting the model.
# Experiment with multiple values for max_distance
# for segmenting the table.
max_distance = 0.01
seg.set_distance_threshold(max_distance)

# Call the segment function to obtain the set of inliner indices and model coefficients.
inliers, coefficients = seg.segment()

# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative=False)

# Save pcd for table
# pcl.save(cloud, filename)
filename = "extracted_inliers.pcd"
pcl.save(extracted_inliers, filename)


# Extract outliers
extracted_outliers = cloud_filtered.extract(inliers, negative=True)

# Save pcd for tabletop objects
filename = "extracted_outliers.pcd"
pcl.save(extracted_outliers, filename)

# Filter out noise.
# Create filter object.
#outlier_filter = cloud_filtered.make_statistical_outlier_filter()

# Set the number of neighboring points to analyze for any given point.
#outlier_filter.set_mean_k(50)

# Set threshold scale factor.
#x = 1.0

# Any point with a mean distance larger than global (mean+std_dev * x) will be considered an outlier.
#outlier_filter.set_std_drv_mul_thresh(x)

#cloud_filter = outlier_filter.filter()


