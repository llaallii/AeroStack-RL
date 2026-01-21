FROM osrf/ros:jazzy-desktop

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG WARN_ON_LICENSE=echo

# Install system dependencies for GUI and basic tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    vulkan-tools \
    x11-apps \
    git \
    nano \
    htop \
    wget \
    curl \
    tmux \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Setup python environment via Virtual Environment to avoid system conflicts
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

COPY requirements-dev.txt /tmp/requirements-dev.txt
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r /tmp/requirements-dev.txt

# Sim-RL Profile Layer
COPY requirements-rl.txt /tmp/requirements-rl.txt
RUN pip install --no-cache-dir -r /tmp/requirements-rl.txt

# Ensure permissions for the non-root user
RUN chmod -R a+rx $VIRTUAL_ENV

# Setup Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Setup user
# Setup user
# Check if the user/group already exists (e.g. 'ubuntu' on strict base images) and delete if so, to ensure fresh creation
RUN if getent passwd $USER_UID; then userdel -r $(getent passwd $USER_UID | cut -d: -f1); fi \
    && if getent group $USER_GID; then groupdel $(getent group $USER_GID | cut -d: -f1); fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME/workspace

# Source ROS 2 and Setup Display for XServer
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "export DISPLAY=\$(cat /etc/resolv.conf | grep nameserver | awk '{print \$2}'):0" >> ~/.bashrc \
    && echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc
