Current instructions are tested in Ubuntu 24.04 with ROS Jazzy. It should work with Ubuntu 22.04 and ROS Humble.

**PCL**

Install pcl libraries

```bash
sudo apt install libpcl-dev
``` 

**Glog, Gflags, BLAS & LAPACK, Eigen3**

Install Glog, Gflags, BLAS & LAPACK, and Eigen3

```bash
sudo apt install libgoogle-glog-dev libgflags-dev libblas-dev liblapack-dev libatlas-base-dev libeigen3-dev
```

**SuiteSparse and OpenCV**

```bash
sudo apt install libsuitesparse-dev libopencv-dev
``` 

**ROS Jazzy**

Please follow installation instructions for ROS Jazzy. Follow the [ROS Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) and install ROS Jazzy full desktop.

You will need the additional package pcl-ros, cv-bridge and tf2-sensor-msgs.

```bash
sudo apt install ros-jazzy-pcl-ros ros-jazzy-tf2-sensor-msgs 
```

**Brisk**

Install brisk features

```bash
wget https://www.doc.ic.ac.uk/~sleutene/software/brisk-2.0.8.zip
unzip brisk-2.0.8.zip
cd brisk
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
sudo make install
```

### Build SVIN ###

Create ros workspace and then build project:

```bash
   mkdir -p ~/svin_ws/src
   cd ~/svin_ws/src
   git clone https://github.com/AutonomousFieldRoboticsLab/SVIn.git     
   cd ..
   colcon build  --event-handlers console_direct+
```
You can just use
```bash
colcon build
```
for silent build. 
