# ROS Noetic
FROM ros:noetic

# Prevent console from interacting with the user
ARG DEBIAN_FRONTEND=noninteractive

# This is required else apt-get update throws Hash mismatch error
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# Install catkin_tools for catkin build, RViz and Gazebo
RUN apt-get install --no-install-recommends -yqqq \
    python3-catkin-tools \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-gazebo-ros

# Python Dependencies
RUN apt-get install --no-install-recommends -yqqq \
    python3-pip

# Repository setup
#--------------
# Make directory for catkin workspace
RUN mkdir -p /root/catkin_ws/src/gazebo-ros-actor-plugin

# Copy package.xml only to the workspace (prevents unnecessary rosdep triggers on file changes in the workspace)
COPY package.xml /root/catkin_ws/src/gazebo-ros-actor-plugin/package.xml

# Install dependencies using rosdep
RUN rosdep install --from-paths /root/catkin_ws/src/ --ignore-src -r -y

# Copy the pip requirements file to the workspace
COPY requirements.txt /root/catkin_ws/src/gazebo-ros-actor-plugin/requirements.txt

# Install python dependencies
RUN pip3 install -r /root/catkin_ws/src/gazebo-ros-actor-plugin/requirements.txt

# Copy the source code to the workspace
COPY . /root/catkin_ws/src/gazebo-ros-actor-plugin

# Using shell to use bash commands like source
SHELL ["/bin/bash", "-c"]

# Build the workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /root/catkin_ws && \
    catkin build gazebo_ros_actor_plugin

# Source workspace
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
