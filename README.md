# Perception V1



### Package setups

1. ORB-SLAMV2 [Setup](ORB/ORB_SLAM2) | [Original Repo](https://github.com/raulmur/ORB_SLAM2)
2. LOAM-SLAM [Setup](LOAM) | [Original Repo](https://github.com/laboshinl/loam_velodyne)



| SLAM/Mapping package  | RGBD  | Stereo  | Lidar  | Fusion available |
|---                    |---    |---      |---     |---               |
| ORB-SLAMV2            | yes   | yes     | no     |   unknown        |
|  Laser Odometry and Mapping (Loam) |  No | No  | Yes  | Yes(with IMU)  |
|   |   |   |   |   |


### Datasets

| Dataset | Comments | RGBD?  | Stereo?  | Lidar? | IMU | T265 | Location |  Best Mapping| Snapshots/sample| 
|---         |---    |---      |---     |---      |--   |---   |----   |--- | --- |
| wysor_1.bag | Wysor bridge pass | yes   | yes     | no |  yes  | |  Google Drive   |   LOAM  | |
| t265p3.bag | Looping data around lab  | no   | no     | yes |  yes  | yes |  Google Drive   |   LOAM (fused with IMU)| |
| labarea.bag | Data about lab's walls  | no   | no     | yes |  yes  | no |  Google Drive   |   LOAM & Open mapping| |
| iribepart1.bag  | Iribe ground floor mapping  | no  | no  | yes | yes| no | Google Drive|  LOAM| |


