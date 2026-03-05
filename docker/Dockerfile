FROM ros:noetic-perception

ENV CERES_VERSION="1.14.0"
ENV CATKIN_WS=/root/catkin_ws

# set up thread number for building
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      cmake \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      libboost-all-dev \
      python3-catkin-tools \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf \
      git \
      sudo && \
      rm -rf /var/lib/apt/lists/* && \
      # Build and install Ceres
      git clone https://ceres-solver.googlesource.com/ceres-solver && \
      cd ceres-solver && \
      git checkout tags/${CERES_VERSION} && \
      mkdir build && cd build && \
      cmake .. && \
      make -j$(USE_PROC) install && \
      rm -rf ../../ceres-solver && \
      mkdir -p $CATKIN_WS/src/VINS-Mono/

# Copy VINS-Mono project and the GTSAM install script
COPY ./ $CATKIN_WS/src/VINS-Mono/

# Install GTSAM from source inside the container
RUN cd $CATKIN_WS/src/VINS-Mono && \
      chmod +x install_gtsam.sh && \
      ./install_gtsam.sh

# Build VINS-Mono
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release && \
      catkin build && \
      sed -i '/exec "$@"/i \
      source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh
