# 3D Perception
This [Udacity clone project](https://github.com/udacity/RoboND-Perception-Project) is modeled after [Amazon Robotics Challenge](https://www.amazonrobotics.com/#/roboticschallenge/results). Giving a robot the ability to locate an object in a cluttered environment, pick it up and then move it to some other location is not just an interesting problem to try to solve,
it's a challenge at the forefront of the robotics industry today. 

A perception pipeline is created base on Exercises 1, 2 and 3  from [RoboND Percetion Excercises](https://github.com/fouliex/RoboND-Perception-Exercises) o identify target objects from a so-called “Pick-List” in that particular order, pick up those objects and place them in corresponding dropboxes.

# Perception Pipeline
## Tabletop Segmentation
The Table Segmetation is about applying some filtering techniques and use RANSAC plane fitting to segment a table in a point cloud.

### Downsample your point cloud by applying a Voxel Grid Filter
When running computation on full resolution point cloud can be slow and may not achieve any improvement on results obtained using a more widely apart sampled point cloud.
Therefore, in many cases, it is advantageous to downsample the data.

VoxelGrid is use to Downsampling Filter to drive a point cloud that has fewer points but should  do a good job of representing the input point cloud as a whole. As the word "pixel" is short for "picture element", the word
"voxel" is short for "volume element". Just as we can divided a 2d image into a regular grid of aread element, we can also divided up a 3D point cloud, into a regualr 3D grid of volume elements. Each individual cell in
 grid is now voxel and the 3D grid is known as "Voxel Grid".

###### Voxel Grid Code
```python
def do_voxel_grid_downssampling(pcl_data,leaf_size):
    '''
    Create a VoxelGrid filter object for a input point cloud
    :param pcl_data: point cloud data subscriber
    :param leaf_size: voxel(or leaf) size
    :return: Voxel grid downsampling on point cloud
    '''
    vox = pcl_data.make_voxel_grid_filter()
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    return  vox.filter()
    
 # Convert ROS msg to PCL data
cloud = ros_to_pcl(pcl_msg)

# Voxel Grid Downsampling
LEAF_SIZE = 0.01
cloud = do_voxel_grid_downssampling(cloud,LEAF_SIZE)
```

#### 2D image into a regular grid of area elements(Left picture) and 3D grid volume elements(Right picture)
![Voxel Grid](https://github.com/fouliex/RoboND-Perception-Exercises/blob/master/misc_images/VoxelGrid.png)


###### [**Voxel Downsampling Sample code**](https://github.com/fouliex/RoboND-Perception-Exercises/blob/master/Exercise-1/voxel_grid_downsampling.py)

### Apply Statistical Outlier Filtering

Statistical Outlier Filtering is use to remove outlieres using number of neighboring points of 10 and standard deviation
threshold of 0.001
###### Statistical Outlier Filtering Code
```python
def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    '''
    :param pcl_data: point could data subscriber
    :param mean_k:  number of neighboring points to analyze for any given point
    :param tresh:   Any point with a mean distance larger than global will be considered outlier
    :return: Statistical outlier filtered point cloud data
    '''
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()

# Convert ROS msg to PCL data
cloud = ros_to_pcl(pcl_msg)

# Statistical Outlier Filtering
cloud = do_statistical_outlier_filtering(cloud,10,0.001)
```

### Apply a Pass Through Filter to isolate the table and objects.
When we have prior information about the location of a target in  the scene, we can apply a Pass Through Filter to remove useless data from our point cloud.
The Pass Through Filter  works just like a cropping tool,which allows us to crop any given 3D point cloud by specifing an axis with cut-off values along that axis.
The region that we allow to pass through is referred as **region of interest**.

By applying a Pass Through filter along  z axis (the height with respect to the ground) to our tabletop scene in the
range 0.1 to 0.8 gives us the table and filtered out all of the objects.

![Pass through Table](https://github.com/fouliex/RoboND-Perception-Exercises/blob/master/misc_images/passthroughTable.png)

By applying a Pass Through filter  with the following range 0.6 and 1.1 isolate our region of interest containing the
table and the objects on the table

![Pass through Object](https://github.com/fouliex/RoboND-Perception-Exercises/blob/master/misc_images/passthroughTable.png)

###### PassThrouh Filter Code
```python
def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    '''
    Create a PassThrough  object and assigns a filter axis and range.
    :param pcl_data: point could data subscriber
    :param filter_axis: filter axis
    :param axis_min: Minimum  axis to the passthrough filter object
    :param axis_max: Maximum axis to the passthrough filter object
    :return: passthrough on point cloud
    '''
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()
    
# Convert ROS msg to PCL data
cloud = ros_to_pcl(pcl_msg)

# PassThrough Filter
filter_axis ='z'
axis_min = 0.44
axis_max =0.85
cloud = do_passthrough(cloud,filter_axis,axis_min,axis_max)

filter_axis = 'x'
axis_min = 0.33
axis_max = 1.0
cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)    
```

###### [**Pass Through Filter Sample code**](https://github.com/fouliex/RoboND-Perception-Exercises/blob/master/Exercise-1/pass_through_filtering.py)

### Perform RANSAC plane filtering to identify the table.

To Remove the table completely from the scene we can use a  popular technique known as **Random Sample Consensus**(RANSAC). RANSAC is an algorithm which is use to identify points in out dataset that belong to a particular model.
In the  3D scene, the model can be a plane a cylinder, a box or any other common shape.

The algorithm assumes that all of the data in a dataset is composed of both **inliers** and **outliers**.
* Inliers can be defined by a particular model with a specific set of parameters.
* Outliers if that model does not fit then it get discarded.

By modeling the table as a plane, we can remove it from the point cloud.

![Pass through Object](https://github.com/fouliex/RoboND-Perception-Exercises/blob/master/misc_images/RANSAC.png)

###### RANSAC Plane Filtering Code
```python
def do_ransac_plane_segmentation(pcl_data,pcl_sac_model_plane,pcl_sac_ransac,max_distance):
    '''
    Create the segmentation object
    :param pcl_data: point could data subscriber
    :param pcl_sac_model_plane: use to determine plane models
    :param pcl_sac_ransac: RANdom SAmple Consensus
    :param max_distance: Max distance for apoint to be considered fitting the model
    :return: segmentation object
    '''
    seg = pcl_data.make_segmenter()
    seg.set_model_type(pcl_sac_model_plane)
    seg.set_method_type(pcl_sac_ransac)
    seg.set_distance_threshold(max_distance)
    return seg
# Convert ROS msg to PCL data
cloud = ros_to_pcl(pcl_msg)
    
# RANSAC Plane Segmentation
ransac_segmentation = do_ransac_plane_segmentation(cloud,pcl.SACMODEL_PLANE,pcl.SAC_RANSAC,0.01)

# Extract inliers and outliers
cloud_table,cloud_objects= extract_cloud_objects_and_cloud_table(cloud,ransac_segmentation )
```

## Euclidean Clustering with ROS and PCL
To perform  Euclidean Clustering, a [k-d tree](http://pointclouds.org/documentation/tutorials/kdtree_search.php) from the 'cloud_objects' point cloud needs to be constructed.

The k-d tree data structure is used in the Euclidian Clustering algorithm to decrease the computational burden of 
searching for neighboring points. While other  efficient algorithms/data structures for nearest neighbor search exist,PCL's
Euclidian Clustering algorithm only supports k-d trees.

![Euclidean Cluster Extraction](https://github.com/fouliex/RoboND-Perception-Exercises/blob/master/misc_images/EuclideanClusterExtraction.png)

###### Eucliean Cluster Extraction Code
```python
def do_euclidean_clustering(white_cloud):
    '''
    :param cloud_objects:
    :return: cluster cloud and cluster indices
    '''
    tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(20000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud,cluster_indices
    
#TODO: Euclidean Clustering
white_cloud= XYZRGB_to_XYZ(cloud_objects)
cluster_cloud,cluster_indices = do_euclidean_clustering(white_cloud)
```

# Project Setup
For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the src directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Perception-Project.git
```
### Note: If you have the Kinematics Pick and Place project in the same ROS Workspace as this project, please remove the 'gazebo_grasp_plugin' directory from the `RoboND-Perception-Project/` directory otherwise ignore this note. 

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```
Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboticPerception/pr2_robot/models:$GAZEBO_MODEL_PATH
```

If you haven’t already, following line can be added to your .bashrc to auto-source all new terminals
```
source ~/catkin_ws/devel/setup.bash
```

To run the demo:
```sh
$ cd ~/catkin_ws/src/RoboticPerception/pr2_robot/scripts
$ chmod u+x pr2_safe_spawner.sh
$ ./pr2_safe_spawner.sh
```
![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)



Once Gazebo is up and running, make sure you see following in the gazebo world:
- Robot

- Table arrangement

- Three target objects on the table

- Dropboxes on either sides of the robot


If any of these items are missing, please report as an issue on [the waffle board](https://waffle.io/udacity/robotics-nanodegree-issues).

In your RViz window, you should see the robot and a partial collision map displayed:

![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Proceed through the demo by pressing the ‘Next’ button on the RViz window when a prompt appears in your active terminal

The demo ends when the robot has successfully picked and placed all objects into respective dropboxes (though sometimes the robot gets excited and throws objects across the room!)

Close all active terminal windows using **ctrl+c** before restarting the demo.

You can launch the project scenario like this:
```sh
$ roslaunch pr2_robot pick_place_project.launch
```
# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  See the example `output.yaml` for details on what the output should look like.  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

For all the step-by-step details on how to complete this project see the [RoboND 3D Perception Project Lesson](https://classroom.udacity.com/nanodegrees/nd209/parts/586e8e81-fc68-4f71-9cab-98ccd4766cfe/modules/e5bfcfbd-3f7d-43fe-8248-0c65d910345a/lessons/e3e5fd8e-2f76-4169-a5bc-5a128d380155/concepts/802deabb-7dbb-46be-bf21-6cb0a39a1961)
Note: The robot is a bit moody at times and might leave objects on the table or fling them across the room :D
As long as your pipeline performs succesful recognition, your project will be considered successful even if the robot feels otherwise!
