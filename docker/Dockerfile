# FROM franc0r/ros2-base:humble
FROM eduartrobotik/eduart-ros-base:humble-1.2.1

# Setup environment
ENV ROS_DISTRO humble
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV USER user

SHELL ["/bin/bash", "-c"]

USER root

########################################################
# Install dependencies
RUN \
    # Build and install libobviously.
    ## Install dependencies.
    apt-get update \
    && apt-get install -q -y --no-install-recommends \
    libgsl-dev \
    libgslcblas0 \
    libann-dev \
    libflann-dev \
#    libvtk7-dev \
    libxml++2.6-dev \
    liblua5.1-dev \
    libopenni-dev \
    libudev-dev \
    freeglut3-dev \
    libboost-thread-dev \
    ros-$ROS_DISTRO-fastrtps \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ## Get sources.
    && cd /tmp \
    && git clone --branch master https://github.com/autonohm/obviously.git \
    && cd obviously/build/release \
    ## Only build needed modules.
    && sed -i 's+ADD_SUBDIRECTORY(../../obdevice obdevice)+#ADD_SUBDIRECTORY(../../obdevice obdevice)+g' CMakeLists.txt \
    && sed -i 's+ADD_SUBDIRECTORY(../../applications applications)+#ADD_SUBDIRECTORY(../../applications applications)+g' CMakeLists.txt \
    && sed -i 's+ADD_SUBDIRECTORY(../../obgraphic obgraphic)+#ADD_SUBDIRECTORY(../../obgraphic obgraphic)+g' CMakeLists.txt \    
    && export OBVIOUSLY_ROOT=/tmp/obviously \
    && cmake . -DCMAKE_BUILD_TYPE=Release \
    && make -j8 \
    ## Install library
    && make install \
    && cp ../../obcore/base/System.inl /usr/local/usr/include/obviously/obcore/base/ \
########################################################
# Clean up
    && apt-get clean \
    && rm /tmp/obviously -rf

USER $USER
COPY ./fastrtps_profile.xml /home/$USER/

RUN \
    # Build slam package.
    cd \
    && mkdir -p ros2/src \
    && cd ros2/src \
    && git clone --branch feature/ros2 https://github.com/franc0r/ohm_tsd_slam.git \
    && cd .. \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release \
########################################################
# Add line for sourcing ROS environment.
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/$USER/.bashrc \
    # && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastrtps_profile.xml" >> /home/$USER/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc \
    && echo "source ~/ros2/install/setup.bash" >> /home/$USER/.bashrc

# Setup workdir
WORKDIR /home/$USER

CMD export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    # && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastrtps_profile.xml \
    && source /home/$USER/ros2/install/setup.bash \
    && ros2 launch ohm_tsd_slam slam.launch.py
