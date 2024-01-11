SVIn2 is a tightly coupled Sonar-Visual-Inertial-Depth formulation of Simultaneous Localization and Mapping (SLAM) algorithm for real-time Underwater navigation. The package contains two modules:

1. okvis_ros: Adaption of OKVIS (<https://github.com/ethz-asl/okvis_ros>) to fuse Sonar and Depth information in the tightly coupled formulation.
2. pose_graph:  Loop-closing module to enable real-time loop detection and pose-graph optimization based on the bag-of-binary-words library DBoW2.

### How do I get set up? ###

This is a catkin package that wraps the pure CMake project.
You will need to install the following dependencies,

* CMake,

        sudo apt install cmake

* ROS (currently tested in: melodic and noetic). Read the instructions to install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). You will need the additional package pcl-ros and tf2-sensor-msgs.

        sudo apt install ros-noetic-pcl-ros
        sudo apt install ros-noetic-tf2-sensor-msgs

* google-glog + gflags, BLAS & LAPACK, Eigen3

        sudo apt install libgoogle-glog-dev libatlas-base-dev libeigen3-dev

* SuiteSparse, CXSparse, OpenCV and Boost

        sudo apt install libsuitesparse-dev libopencv-dev
        sudo apt install libboost-dev libboost-filesystem-dev

* ceres-solver

        git clone https://github.com/ceres-solver/ceres-solver.git
        cd ceres-solver
        git checkout 1.14.x
        mkdir build
        cd build
        cmake -DCMAKE_BUILD_TYPE=Release ..
        make -j8
        sudo make install
        cd ../..

* brisk

        wget https://www.doc.ic.ac.uk/~sleutene/software/brisk-2.0.8.zip
        unzip brisk-2.0.8.zip
        cd brisk
        mkdir build
        cd build
        cmake -DCMAKE_BUILD_TYPE=Release ..
        make -j8
        sudo make install
        cd ../..

### Building the project ###

* Create ros workspace. Download SVIn and sonar driver. Then build catkin project:

        mkdir -p ~/svin_ws/src
        cd ~/svin_ws/src
        git clone --branch v0.1 https://github.com/AutonomousFieldRoboticsLab/SVIn.git
        git clone https://github.com/AutonomousFieldRoboticsLab/imagenex831l.git
        
        # For Ubuntu 18.04/20.04 (ROS Noetic)
        git clone --branch ros-noetic git@github.com:AutonomousFieldRoboticsLab/sonar_rviz_plugin.git
        
        #For Ubuntu 16.04
        git clone git@github.com:AutonomousFieldRoboticsLab/sonar_rviz_plugin.git
        
        cd ..
        catkin_make

### Running the project ###

Running it on our publicly available datasets: <https://afrl.cse.sc.edu/afrl/resources/datasets/>. If you follow "Datasets for Visual-Inertial-Based State Estimation Algorithms" link you will be directed to a google drive directory,  under the 'Bus' and 'Cave' you will find ROS bagfile with Sonar topic named as '/imagenex831l/range' and  '/imagenex831l/range_raw'.

Run the launch file for Cave:

        source ~/svin_ws/devel/setup.bash
        roslaunch okvis_ros svin_stereorig_v2.launch

Or, run the launch file for Bus:

        source ~/svin_ws/devel/setup.bash
        roslaunch okvis_ros svin_stereorig_v1.launch

In different terminal, run the bag file

        rosbag play bagfile_name --clock -r 0.8


### Ground Truth ###
The pseudo ground truth trajectories obtained using COLMAP are in colmap_groundtruth folder. These trajectories are only accurate up to scale and evaluation should be done after scaling only.

Note: We plan to release the scale accurate trajectory using rig constraints soon.


### Dockerfile
        docker build -t svin .

        BAGS=/path/to/your/bag/files

        docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "$BAGS:/data" svin