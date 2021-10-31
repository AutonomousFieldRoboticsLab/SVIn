SVIn2 is a tightly coupled Sonar-Visual-Inertial-Depth formulation of Simultaneous Localization and Mapping (SLAM) algorithm for real-time Underwater navigation. The package contains two modules:
	1. okvis_ros: Adaption of OKVIS (https://github.com/ethz-asl/okvis_ros) to fuse Sonar and Depth information in the tightly coupled formulation.
	2. pose_graph:  Loop-closing module to enable real-time loop detection and pose-graph optimization based on the bag-of-binary-words library DBoW2.

1. Installation:

    -  Dependencies:

        -- ROS kinetic
        
        -- ceres-solver. See http://ceres-solver.org/installation.html for installation.
        
        -- The followings should be already installed through ROS or when installing ceres-solver:
        
             # Cmake:  sudo apt-get install cmake
             
             # google-glog + gflags: sudo apt-get install libgoogle-glog-dev
             
             # BLAS & LAPACK: sudo apt-get install libatlas-base-dev
             
             # Eigen3: sudo apt-get install libeigen3-dev
             
             # SuiteSparse and CXSparse: sudo apt-get install  libsuitesparse-dev 
             
             # Boost: sudo apt-get install libboost-dev libboost-filesystem-dev

        -- You may need to install some ros packages which can be installed by: sudo apt-get install ros-kinetic-PACKAGE-NAME. For example, to install tf2-sensor-msgs: 
        
            sudo apt-get install ros-kinetic-tf2-sensor-msgs
        
        
        -- Driver for Sonar: git clone https://github.com/AutonomousFieldRoboticsLab/imagenex831l.
        
        -- Driver for Depth Sensor: aquacore has the driver for depth. If you want to install SVIn2 on your laptop/pc make sure you have the depth driver. 
        
        -- BRISK feature detector for pose-graph.
             # Download https://www.doc.ic.ac.uk/~sleutene/software/brisk-2.0.3.zip and Extract
            $ mkdir build
            $ cd build 
            $ cmake ../
            $ make 
            $ sudo make install  
            
            
    -  Building the project:
        Once you have the above installed, from the catkin_ws root, type
        $ catkin_make
        




2. Running it on our publicly available datasets: https://afrl.cse.sc.edu/afrl/resources/datasets/. If you follow "Datasets for Visual-Inertial-Based State Estimation Algorithms" link you will be directed to a google drive directory,  under the 'Bus' and 'Cave' you will find ROS bagfile with Sonar topic named as '/imagenex831l/range' and  '/imagenex831l/range_raw'. Run the launch file for Cave: 

        roslaunch okvis_ros svin_node_stereoRig2_water.launch
        
Or, run the launch file for Bus: 

        roslaunch okvis_ros svin_node_stereo_rig_water.launch

You can use rviz for visulalization -- you can load rviz config from .../SVIn2/pose-graph/rviz/svin2.rviz.       



3. Calibration guidelines:
    In general, both Camera calibration and Camera-IMU calibration are very important for accurate state estimation. Errors in them could severly mess up with the results. The calibration package from ETH Zurich https://github.com/ethz-asl/kalibr is a good tool for calibration.