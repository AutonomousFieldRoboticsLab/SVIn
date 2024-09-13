SVIN is a tightly coupled Sonar-Visual-Inertial-Depth formulation of Simultaneous Localization and Mapping (SLAM)
algorithm for real-time underwater navigation. The package contains two modules:

1. okvis_ros: Adaption of OKVIS (<https://github.com/ethz-asl/okvis_ros>) to fuse sonar,depth information in the tightly
   coupled formulation.
2. pose_graph:  Loop-closing module to enable real-time loop detection and pose-graph optimization based on the
   bag-of-binary-words library DBoW2.

### !!! Note !!! ##

**The main branch now uses ROS 2. Please use the ros1 branch if you need to work with ROS1. We are in process of
figuring out how to use old sonar and data in ROS2 as they are custom topic types. Sonar and depth modes will not work
and are disabled by default.**

### Setup Instructions ###

The setup instructions are tested on Ubuntu 24.04 with ROS Jazzy.

**Prerequisites**

- [Glog](http://rpg.ifi.uzh.ch/docs/glog.html), [Gflags](https://gflags.github.io/gflags/)
- [BLAS](https://www.netlib.org/blas/), [LAPACK](https://www.netlib.org/lapack/)
- [OpenCV](https://github.com/opencv/opencv) >= 3.4
- [Suitesparse](https://people.engr.tamu.edu/davis/suitesparse.html)
- [ceres-solver](https://github.com/ceres-solver/ceres-solver/tree/master)
- [Brisk](https://ieeexplore.ieee.org/document/6126542)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html)

Please follow [installation page](install.md) for detailed instructions on building SVIN.

### Running the project ###

Running it on our publicly available datasets: <https://www.afrl.ai/resources>. If you follow "Datasets for
Visual-Inertial-Based State Estimation Algorithms" link you will be directed to a google drive directory, under the '
Bus' and 'Cave' you will find ROS bagfile with Sonar topic named as '/imagenex831l/range' and  '
/imagenex831l/range_raw'.

### !!!Note !!!: Any changes in config/launch files are not reflected unless you build the repo again. All the config/launch files are saved inside install folder and will be updated as part of build. ###

To build again use

```bash
colcon build  --event-handlers console_direct+
```

## Converting between ROS2 and ROS1 bags

The easiest way to convert between ROS1 and ROS2 bag is using
rosbags-convert [rosbags](https://gitlab.com/ternaris/rosbags).

To install

```bash
sudo apt install pipx
pipx install rosbags
```

To convert ROS1 bag to ROS2 use

```bash
rosbags-convert --src <ros1 bag> --dst <ros2_bag_folder>
```

## Running with GoPro Dataset ## 

Run the launch file for Cave:

```bash
source install/setup.bash
ros2 launch okvis_ros svin_gopro_uw.xml
```

## Running on AFRL Datasets ##

Run the launch file for Cave:

```bash
source install/setup.bash
ros2 launch okvis_ros svin_stereorig_v2.xml
```

Or, run the launch file for Bus:

```bash
source install/setup.bash
roslaunch okvis_ros svin_stereorig_v1.xml
```

In different terminal, run the bag file

```bash
ros2 bag play bagfile_name --clock
```

### Ground Truth ###

The pseudo ground truth trajectories obtained using COLMAP are in colmap_groundtruth folder. These trajectories are only
accurate up to scale and evaluation should be done after scaling only.

Note: We plan to release the scale accurate trajectory using rig constraints soon.
