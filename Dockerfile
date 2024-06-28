FROM 18r441m/ros:noetic-desktop-full

ARG CERES_VERSION=2.2.0
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
ENV brisk_DIR=/usr/local/lib/CMake/brisk
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-10 \
    g++-10 \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    build-essential \
    cmake \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libopencv-dev \
    libboost-dev \
    libboost-filesystem-dev && \
    rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100

RUN git clone https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && \
    git checkout ${CERES_VERSION} && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf ceres-solver

RUN git clone --branch 2.0.8 https://github.com/18r441m/ethzasl_brisk.git && \
    cd ethzasl_brisk && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && \
    rm -rf ethzasl_brisk

RUN git clone https://github.com/laurentkneip/opengv.git && \
    cd opengv && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

RUN mkdir -p /svin_ws/src && \
    cd /svin_ws/src && \
    git clone https://github.com/AutonomousFieldRoboticsLab/SVIn.git && \
    git clone https://github.com/AutonomousFieldRoboticsLab/imagenex831l.git && \
    git clone --branch ros-noetic https://github.com/AutonomousFieldRoboticsLab/sonar_rviz_plugin.git

WORKDIR /svin_ws

RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; catkin_make'
