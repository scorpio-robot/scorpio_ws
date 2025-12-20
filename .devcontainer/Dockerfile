FROM osrf/ros:humble-desktop-full

# Change apt sources to USTC mirror for faster downloads in China
# Support both traditional format (Ubuntu 22.04 and earlier) and DEB822 format (Ubuntu 24.04+)
RUN sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list 2>/dev/null || true && \
    sed -i 's@//.*archive.ubuntu.com@//mirrors.ustc.edu.cn@g' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || true

# Set up dev packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get -y install --no-install-recommends \
    zsh \
    wget \
    gdb \
    gettext-base \
    git-core \
    openssh-client \
    clangd \
    python3-pip \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcs2l \
    sudo \
    # For GUI
    libx11-xcb1 libxcb1 libxcb-render0 libxcb-shm0 libxcb-xfixes0 libxcb-shape0 \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 libxcb-util1 \
    libxcb-xinerama0 libxcb-xkb1 libxkbcommon0 libxkbcommon-x11-0 \
    libglu1-mesa libxi6 libsm6 libxrender1 libfontconfig1 \
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

### For Windows, uncomment the following lines to install WSLg dependencies and utilize vGPU acceleration
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install \
#           vainfo \
#           mesa-va-drivers \
#           mesa-utils \
#    # Clean up
#         && apt-get autoremove -y \
#         && apt-get clean -y \
#         && rm -rf /var/lib/apt/lists/*
# ENV LIBVA_DRIVER_NAME=d3d12
# ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
# CMD vainfo --display drm --device /dev/dri/card0
# ENV DEBIAN_FRONTEND=dialog

################
# uncomment below for nvidia support
# Expose the nvidia driver to allow opengl
# Dependencies for glvnd and X11.
################
# RUN apt-get update \
#  && apt-get install -y -qq --no-install-recommends \
#   libglvnd0 \
#   libgl1 \
#   libglx0 \
#   libegl1 \
#   libxext6 \
#   libx11-6 \
# && rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
#ENV NVIDIA_VISIBLE_DEVICES all
#ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

#ENV QT_X11_NO_MITSHM=1

# ** [Optional] Uncomment this section to install additional packages. **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Find and replace any user with matching UID
RUN set -eux; \
    existing_user=$(getent passwd "$USER_UID" | cut -d: -f1 || true); \
    if [ -n "$existing_user" ]; then \
        usermod -l $USERNAME -d /home/$USERNAME -m $existing_user && \
        groupmod -n $USERNAME $existing_user && \
        echo "Renamed $existing_user to $USERNAME"; \
    else \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/zsh --uid $USER_UID --gid $USER_GID -m $USERNAME && \
        echo "Created new user $USERNAME"; \
    fi

# Add sudo support for the non-root user
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Copy .zshrc to user's home
COPY --chown=$USERNAME:$USERNAME .zshrc /home/$USERNAME/.zshrc

# Set up autocompletion for user
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.zsh ]; then source /opt/ros/${ROS_DISTRO}/setup.zsh; fi" >> /home/$USERNAME/.zshrc \
    && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh; fi" >> /home/$USERNAME/.zshrc \
    && echo "if [ -f /usr/share/vcs2l-completion/vcs.zsh ]; then source /usr/share/vcs2l-completion/vcs.zsh; fi" >> /home/$USERNAME/.zshrc \
    # Set up auto-source of workspace for user
    && echo "if [ -f ${WORKSPACE}/install/setup.zsh ]; then source ${WORKSPACE}/install/setup.zsh; fi" >> /home/$USERNAME/.zshrc

# Set up python symlinks for vscode paths
RUN PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info[0]}.{sys.version_info[1]}')") \
    && tgt="/opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION}/site-packages" \
    && [ -d "$tgt" ] && ln -s "$tgt" "/opt/ros/${ROS_DISTRO}/lib/ros_site_packages"
