# This is a template for a Dockerfile to build a docker image for your ROS2 package.
# The main purpose of this file is to install dependencies for your package.

FROM ros:jazzy-ros-base-noble

ENV ROS_DISTRO=jazzy

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# Set up workspace
RUN mkdir -p /ws/src

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Package apt dependencies
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    # EXAMPLE: \
    # libssl-dev \
    # libffi-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws


# Installing of pip dependencies
#RUN pip3 install \
#     # EXAMPLE: \
#     # torch \
#     # torchvision \
#     # tensorboardX \
#     # opencv-python \
#     # scikit-image \
#     # scikit-learn \


# Build dependencies from source
#RUN git clone https://github.com/your-github-username/your-repo.git
#WORKDIR your-repo
#RUN make
