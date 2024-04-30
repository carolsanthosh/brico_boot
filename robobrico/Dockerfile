# Use the arm64v8/ubuntu:20.04 base image
FROM arm64v8/ubuntu:20.04
ARG DEBIAN_FRONTEND=noninteractive
# Install necessary packages for ROS Noetic
RUN apt-get update && apt-get install -y \
    curl \
    nano \
    gnupg2 \
    lsb-release

# Add ROS Noetic repository
RUN curl -sSL http://packages.ros.org/ros.key | apt-key add -
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Install ROS Noetic
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

RUN apt-get install -y \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-joy \
    ros-noetic-rplidar-ros \
    ros-noetic-slam-gmapping \
    ros-noetic-rosserial-arduino \
    ros-noetic-laser-scan-matcher \
    jstest-gtk \
    joystick


# Set up ROS environment
ENV ROS_DISTRO noetic
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV PATH=$ROS_ROOT/bin:$PATH
ENV LD_LIBRARY_PATH=$ROS_ROOT/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=$ROS_ROOT/lib/python3/dist-packages:$PYTHONPATH

# Create a workspace (optional)
WORKDIR /ros_ws
RUN mkdir -p src
COPY brico_boot /ros_ws/src/brico_boot
COPY brico_startup.sh /ros_ws
# Source ROS setup.bash
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Clean up
RUN rm -rf /var/lib/apt/lists/*



RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /ros_ws; catkin_make'

# Add the workspace setup.bash to the .bashrc
RUN echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc
# Set the entrypoint


CMD ["bash"]

# Build and run the Docker image
# $ docker build -t my_ros_noetic .
# docker build --progress plain -t bricobot .
# $ docker run -it my_ros_noetic
