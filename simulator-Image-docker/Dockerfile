FROM ros:noetic-ros-core

MAINTAINER Sen Wang <sen.wang@imperial.ac.uk>

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Basic tools and Python
RUN apt-get update && \
    apt-get install -y curl git python3-pip && \
    pip3 install -U --no-cache-dir supervisor supervisor_twiddler rosdep && \
    rosdep init && \
    apt-get clean

# PyTorch
RUN pip3 install torch && \
    apt-get clean

# OSRF distribution for Gazebo 11
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    curl -L http://packages.osrfoundation.org/gazebo.key | apt-key add -

# Retry logic for apt-get update
RUN set -eux; \
for i in $(seq 1 5); do \
    if apt-get update; then \
        break; \
    else \
        echo "Error: apt-get failed (attempt $i of 5). Retrying..." >&2; \
        sleep 1; \
    fi; \
done; \
if [ "$i" = 5 ]; then \
    echo "Error: apt-get failed after 5 attempts." >&2; \
    exit 1; \
fi

# Install ROS Noetic packages including Gazebo plugins
RUN set -eux; \
for i in $(seq 1 5); do \
    if apt-get install -y ros-noetic-gazebo-plugins ros-noetic-xacro \
                          ros-noetic-controller-manager ros-noetic-robot-state-publisher; then \
        break; \
    else \
        echo "Error: apt-get failed (attempt $i of 5). Retrying..." >&2; \
        sleep 1; \
    fi; \
done; \
if [ "$i" = 5 ]; then \
    echo "Error: apt-get failed after 5 attempts." >&2; \
    exit 1; \
fi

# Gazebo models
RUN git clone --depth 1 https://github.com/osrf/gazebo_models.git /tmp/gazebo_models && \
    cp -r /tmp/gazebo_models/cafe_table /usr/share/gazebo-11/models/ && \
    cp -r /tmp/gazebo_models/first_2015_trash_can /usr/share/gazebo-11/models/ && \
    cp -r /tmp/gazebo_models/mailbox /usr/share/gazebo-11/models/ && \
    cp -r /tmp/gazebo_models/table_marble /usr/share/gazebo-11/models/ && \
    rm -r /tmp/gazebo_models

RUN mkdir /src

ADD src /src

# Catkin workspace setup with Noetic
RUN source /opt/ros/noetic/setup.bash && \
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && \
    catkin_init_workspace && \
    git clone --depth 1 -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone --depth 1 -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    cp -r /src . && \
    cd .. && \
    rosdep update && rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install

RUN apt-get clean && rm -r ~/catkin_ws

ADD supervisord.conf /etc/supervisor/supervisord.conf

VOLUME /opt/ros/noetic/share/turtlebot3_description

RUN echo "export ROBOT_HOST=hiwonder LIDAR_TYPE=G4" >> /opt/ros/$ROS_DISTRO/setup.bash

CMD ["/usr/local/bin/supervisord", "-n", "-c", "/etc/supervisor/supervisord.conf"]