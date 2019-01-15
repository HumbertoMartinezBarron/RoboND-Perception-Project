# PR2 project: 3D Perception
By: Humberto MartinezBarron

## Parts of the Project :)

1. Writeup (duh!)
2. Exercises 1, 2, and 3 pipeline implementation
3. Pick and place setup

NOTE: please view the SETUP.md file in this repository to check out Udacity's project requirements, as this project was part of the Robotics Software Engineer Nanodegree Program.

## Part 1.
### Writeup!

### Right here!
This project was great! I think it gave me several tools to perform object recognition in robotics, which is an extremely cool thing to implement in my future projects!

## Part 2.
### Pipeline implementation!

There were three exercises implemented in the project:
a) Filtering
b) Clustering
c) Object Recognition!

#### Filtering
The big challenge with exercise 1 consisted in finding the right parameters to do two things:
- Eliminate noise
- Segment the table

Basically, trial and error is a good bet to find the right values.

Here's a look at my noisy data:

![](https://github.com/HumbertoMartinezBarron/RoboND-Perception-Project/blob/master/imgs/noise_points.png)

In order to fix this problem, I chose the following parameter:
* tsf = 0.01

Where tsf stands for threshold scale factor.

This parameter is passed to the method ```set_std_dev_mul_thresh()``` in order to perform the statistical outlier filtering.

Now, I need to segment the table in order to see the objects on the tabletop! Once again, I had to try different values before I got this working! I should mention this task was rather exhausting, given I am running all of this on a Macbook Air which won't allow me to assign more than 3300 MB and 2 cores to the VM! This makes each run painstakingly slow.

However, I was able to discover that the needed parameters in order to successfully filter out the table were the following:
 ```leaf_size = 0.01
filter_axis = 'z' # param or set_filter_field_name()
ax_min = 0.6 # lower bound param for set_filter_limits()
ax_max = 0.8 # upper bound param for set_filter_limits()```

#### Clustering!

Now, it's time to perform the DBSCAN algorithm in order to separate my point into clusters for later using object recognition! Again, the challenge was tuning the parameters! In this case, the pertinent parameters were:
- Cluster tolerance --> the radius around which I should find more points to make up a cluster.
- Minimum cluster size --> the minimum amount of points that can be used to make a cluster.
- Maximum cluster size --> the maximum amount of points that can be used to make a cluster.

Once more, good ol' trial and error was performed to find the right values! Here's why finding these parameters was highly nontrivial:
- The tolerance can make the algorithm either not detect any clusters, or make it detect all of the points as making up a single cluster! In either of these scenarios it would be impossible to recognize the objects!
- The max and min cluster sizes could make the difference between detecting unwanted objects and detecting the right ones!
	- About detecting the wrong objects, the Pr2's RGB-D camera would detect the corners of the boxes at its sides as part of the scene! This means that the clustering algorithm could end up passing more clusters than needed to the recognition segment of the project. Therefore, controlling the minimum cluster size was extremely important!

In the end, I chose the following parameters after trying over and over:
* Distance tolerance: 0.05
* Minimum cluster size: 115
* Maximum cluster size: 1250

Even so, the minimum cluster size remains a problem; it will (rarely) count the edge of the red box as a cluster! However, pushing it up will make it unable to detect the glue in scenes 2 and 3.

#### Object Recognition!

The trick in the object recognition part was finding the type of data I would need in order to make the ```.yaml``` files. Everything else was taken care of by the code implemented in the lessons - all I had to do was train the SVM the same way I did in Exercise 3! Well, except for 1 detail:
- Turns out, there is a trick to exercise 3: the glue is placed __behind__ the book! This makes the recognition fail (in order to fix it, I could do two things: bring down the minimum cluster size and hope the algorithm succeeds in finding a cluster for the glue, or try to make the ```pr2_mover()``` function call the ```pcl_callback()``` function after each movement. However, I don't know what to send as a parameter!).
	- What I decided to do (to prevent the code from crashing) is to change the ```for``` loop for a ```while``` loop and check first if the object in the pick list is in the detected objects list. If not, increment the index and move on until we find one that is.

### Pick & Place Setup!

In order to make the required files for the project submission, it was necessary to to check out the type of inputs the ```make_yaml_dict()``` method needed.

The lesson goes through the way to assign the values of ```Int32()``` and ```String()``` objects (all I had to do was assign the desired values to the ```data``` attribute).

However, the lesson did not go through how to make the position! turns out, the ```Pose()``` object has 2 attributes: ```position``` and ```orientation```. I care about the position attribute, which turns out to be, in turn, a ```Point()``` object with three attributes: ```x```, ```y```, and ```z``` (all ```float64```'s).

Thus, I wrote the line
```PICK_POSE.position.x, PICK_POSE.position.y, PICK_POSE.position.z = centroids[j]```
where ```j``` is the index where the object's name matches an element in the labels list.

As for the ```PLACE_POSE```, I had to load the ```dropbox.yaml``` file and extract the position to assign it to the position attribute.

```PLACE_POSE.position.x, PLACE_POSE.position.y, PLACE_POSE.position.z = dropbox[a]['position']```
- Where ```a``` is the index corresponding to the box in which the object must be placed.

Confusion matrices!

![](https://github.com/HumbertoMartinezBarron/RoboND-Perception-Project/blob/master/imgs/confusion_matrix_1.png)

![](https://github.com/HumbertoMartinezBarron/RoboND-Perception-Project/blob/master/imgs/confusion_matrix_2.png)

![](https://github.com/HumbertoMartinezBarron/RoboND-Perception-Project/blob/master/imgs/confusion_matrix.png)

## Results

In my opinion, my results were rather good! Here's what happened when running the code:
- 100% (3/3) of objects in ```test1.world``` were recognized correctly.
- 100% (5/5) objects in ```test2.world``` were recognized correctly.
- 87.5% (7/8) of objects ```test3.world``` were recognized correctly.

Object recognition step implemented (3/3 objects detected!)

![](https://github.com/HumbertoMartinezBarron/RoboND-Perception-Project/blob/master/imgs/object_recognition_1.png)

Object recognition step implemented (5/5 objects detected!)

![](https://github.com/HumbertoMartinezBarron/RoboND-Perception-Project/blob/master/imgs/object_recognition_2.png)

Object recognition step implemented (7/8 objects detected!)

![](https://github.com/HumbertoMartinezBarron/RoboND-Perception-Project/blob/master/imgs/object_recognition.png)

## Possible Improvements!

Okay, so the main problem here was the last test. I realized that sometimes the arrangement of the objects might make it difficult for the robot to know where each object is! Therefore, the most significant possible improvement is adding functionality which allows the robot to perform clustering object recognition sequentially after each action!

Also, the motion could definitely be smarter! Making the robot aware of its surroundings in the motion planning step could help increase the success of its movements!

Finally, in order to recognize all the elements in a scene, despite of how complicated it might be, it could be helpful to increase the calculations made while training the SVM! This could boost the recognition accuracy!

## Conclusion!

This project was great! It gave me an awesome overview of how object recognition works! This knowledge can be taken as far as I want to make robots that can interact with their environment.

Thanks for reviewing my project! Any tips on how to make the robot detect the glue in the third scene would be really helpful! :)

Thanks! :)

![](https://github.com/HumbertoMartinezBarron/RoboND-Perception-Project/blob/master/imgs/terminator.jpg)