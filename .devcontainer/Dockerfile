# Use the specified Ubuntu 22.04 base image
FROM osrf/ros:jazzy-desktop-full

# Set a default environment variable to avoid frontend interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install basic dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    wget \
    vim \
    git \
    software-properties-common

# Set the working directory
WORKDIR /app

## Set Locale # check for UTF-8
RUN locale \ 
    && apt update && apt install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && locale

RUN apt update && sudo apt install -y \
    python3-flake8-blind-except \
    python3-flake8-class-newline \
    python3-flake8-deprecated \
    python3-mypy \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-mock \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-runner \
    python3-pytest-timeout \
    ros-dev-tools \
    gfortran \
    libopenblas-dev \
    libgeographiclib-dev \
    python3-rosinstall-generator \
    gnupg \
    lsb-release \
    mesa-utils

# # Install GZ Harmonic
# RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# RUN apt-get update && apt-get install -y \ 
#     gz-harmonic

# preamptive Install ROS dep packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-ament-cmake-pycodestyle \
    ros-jazzy-test-msgs \
    ros-jazzy-tf-transformations \
    ros-jazzy-rviz-satellite \
    ros-jazzy-joy-teleop \
    ros-jazzy-flexbe-core

# Update all packages
RUN apt-get update && apt-get upgrade -y

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN if id -u $USERNAME >/dev/null 2>&1; then \
        existing_uid=$(id -u $USERNAME) && \
        existing_gid=$(id -g $USERNAME) && \
        if [ "$existing_uid" != "$USER_UID" ] || [ "$existing_gid" != "$USER_GID" ]; then \
            userdel $USERNAME && \
            groupdel $USERNAME; \
        fi \
    fi && \
    if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# ## speedup rosdep
# # Copy the src directory from the build context
# COPY src /temp/src

# # Install dependencies using the copied files
# RUN . /opt/ros/jazzy/setup.sh && \
#     rosdep update && \
#     rosdep install --from-paths /temp/src --ignore-src --rosdistro jazzy -r -y && \
#     sudo rm -rf /temp/src  
    
# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# FIX gazebo crashing issues by ensuring proper permissions
RUN sudo apt-get update && \
    sudo apt-get install -y libgl1-mesa-dri libegl-mesa0 && \
    sudo mkdir -p /tmp/runtime-vscode && \
    sudo chown $USERNAME:$USERNAME /tmp/runtime-vscode && \
    sudo mkdir -p ~/.gz && \
    sudo chown -R $USERNAME:$USERNAME ~/.gz && \
    sudo chmod 0700 /tmp/runtime-vscode

# Define the entrypoint or command (adjust as needed)
CMD ["/bin/bash"]
