# Perception V1

![](pipeline.png)

**Note** The red lines indicate rostopic names


#### Run Instructions
First download rosbag(simulation_v1.bag) from [here]()

```
git checkout perception_v1
catkin_make
rosbag play <path to rosbag> -r 0.1
roslaunch pcl_filter cloud_pub.launch
```
