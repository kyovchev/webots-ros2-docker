# ROS 2 Jazzy + Webots (GUI over noVNC, software OpenGL, no host X server needed)
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
# Webots release matching webots_ros2 for Jazzy (amd64 only)
ARG WEBOTS_VERSION=R2025a
ARG WEBOTS_DEB=webots_2025a_amd64.deb
ARG USERNAME=ros
ARG UID=1000
ARG GID=1000

# Virtual desktop + VNC + noVNC + ROS 2 / Webots integration
RUN apt-get update && apt-get install -y --no-install-recommends \
        xvfb x11vnc x11-utils openbox tint2 xterm websockify supervisor \
        mesa-utils libgl1-mesa-dri libglu1-mesa libegl1 libxkbcommon-x11-0 \
        fonts-dejavu-core wget ca-certificates git nano sudo net-tools netcat-openbsd \
        iputils-ping python3-colcon-common-extensions python3-rosdep python3-pil \
        ros-jazzy-webots-ros2 ros-jazzy-rviz2 ros-jazzy-rqt-image-view ros-jazzy-moveit \
        ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2controlcli \
        ros-jazzy-web-video-server python3-uvicorn python3-fastapi \
    && rm -rf /var/lib/apt/lists/*

# noVNC pinned from the upstream release, so the HTML and JS always match
ARG NOVNC_VERSION=1.5.0
RUN mkdir -p /opt/novnc \
    && wget -qO- "https://github.com/novnc/noVNC/archive/refs/tags/v${NOVNC_VERSION}.tar.gz" \
       | tar xz --strip-components=1 -C /opt/novnc \
    && ln -sf vnc.html /opt/novnc/index.html

# Webots itself
RUN wget -q "https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}/${WEBOTS_DEB}" -O /tmp/webots.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends /tmp/webots.deb \
    && rm -rf /tmp/webots.deb /var/lib/apt/lists/*

# Non-root user (the base image ships "ubuntu" with UID 1000)
RUN (userdel -r ubuntu 2>/dev/null || true) \
    && groupadd -g ${GID} ${USERNAME} \
    && useradd -m -s /bin/bash -u ${UID} -g ${GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix

COPY supervisord.conf /etc/supervisor/conf.d/desktop.conf
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
COPY with-x.sh /usr/local/bin/with-x
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/with-x \
    && mkdir -p /etc/xdg/openbox \
    && printf 'tint2 &\nxterm -geometry 120x35+20+20 -fa Monospace -fs 11 &\n' > /etc/xdg/openbox/autostart

ENV DISPLAY=:1 \
    RESOLUTION=1600x900 \
    WEBOTS_HOME=/usr/local/webots \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe \
    QT_X11_NO_MITSHM=1 \
    ROS_DOMAIN_ID=0

ENV USER=${USERNAME} \
    USERNAME=${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> ~/.bashrc \
    && (rosdep update || true)

EXPOSE 6080
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["supervisord", "-n", "-c", "/etc/supervisor/conf.d/desktop.conf"]