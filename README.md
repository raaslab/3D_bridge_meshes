# Perception V1

![](pipeline.png)

**Note** The red lines indicate rostopic names


#### Run Instructions
First download rosbag(simulation_v1.bag) from [here](https://drive.google.com/file/d/1f-OXbnUOXiB1iEFGs9W5oXcsnhm74dib/view?usp=sharing)

```
git checkout perception_v1
catkin_make
rosbag play <path to rosbag> -r 0.1
roslaunch pcl_filter cloud_pub.launch
```
